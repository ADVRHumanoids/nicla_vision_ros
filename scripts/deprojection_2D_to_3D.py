#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
import message_filters
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped, Pose


bridge = CvBridge()


class Deprojection(object):

    def __init__(self):

        self.rate = rospy.Rate(30)

        self.fx = 439.5975502749937
        self.fy = 439.5975502749937
        self.cx = 160.5
        self.cy = 120.5

        # Intrinsic matrix
        # fx = 439.5975502749937
        # fy = 439.5975502749937
        # cx = 160.5
        # cy = 120.5
        # self.K = np.array([[fx, 0, cx],
        #             [0, fy, cy],
        #             [0, 0, 1]])

        # Extrinsic matrix (transformation matrix T)
        # T = np.array([[R11, R12, R13, Tx],
        #             [R21, R22, R23, Ty],
        #             [R31, R32, R33, Tz],
        #             [0, 0, 0, 1]])
        self.T = self.get_transformation("nicla_camera", "relax_arm1_link1")

    def run(self):

        while not rospy.is_shutdown():
            # Subscribe to the ROS topic
            image_sub = message_filters.Subscriber(
                "/relax/nicla/camera/image_raw", Image
            )
            depth_sub = message_filters.Subscriber("/relax/nicla/tof", LaserScan)

            # ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1)
            ts = message_filters.ApproximateTimeSynchronizer(
                [image_sub, depth_sub], 10, 0.1
            )  # , allow_headerless=True
            ts.registerCallback(self.callback)

            rospy.spin()

    def callback(self, msg_rgb, msg_depth):
        # Solve all of perception here...
        self.cv2_img = bridge.imgmsg_to_cv2(msg_rgb, "bgr8")

        self.img_shape = self.cv2_img.shape

        # To visualize the received image
        # cv2.imshow("Received img", self.cv2_img)
        # cv2.waitKey(1)

        self.depth = msg_depth.ranges[0]
        # print("Received depth: ", self.depth)

        self.central_pixel_to_3d(self.depth)

        print("INFO: Received frame and depth.")

    def central_pixel_to_3d(self, depth):
        """
        Convert central pixel coordinates to 3D coordinates using camera intrinsics and extrinsics.
        Note: the central pixel is assumed to be the one for which the laser gives the depth info.

        Parameters:
        - depth: Depth value of the pixel

        Returns:
        - Point in 3D space (x, y, z)
        """

        # u, v: Pixel coordinates in the image
        u = self.img_shape[0] // 2
        v = self.img_shape[1] // 2

        # Compute normalized coordinates in camera space
        cam_point = [0, 0, 1.0]
        cam_point[0] = (u - self.cx) / self.fx
        cam_point[1] = (v - self.cy) / self.fy

        print(cam_point)
        cam_point = [value * depth for value in cam_point]

        print(cam_point)
        x = cam_point[2]
        y = cam_point[0]
        z = cam_point[1]

        # Transform camera coordinates to world coordinates
        point_wrt_source = Point(x, y, z)
        point_wrt_target = self.transform_point(self.T, point_wrt_source)

        print(point_wrt_target)

        # return world_point.flatten()

    def DEPRECATED_pixel_to_3d(self, u, v, depth, K, T):
        """
        Convert pixel coordinates to 3D coordinates using camera intrinsics and extrinsics.

        Parameters:
        - u, v: Pixel coordinates in the image
        - depth: Depth value of the pixel
        - K: Intrinsic camera matrix (3x3)
        - T: Extrinsic camera matrix (4x4)

        Returns:
        - Point in 3D space (x, y, z)
        """

        # Convert pixel coordinates to homogeneous coordinates
        pixel_homogeneous = np.array([[u], [v], [1]])

        # Compute inverse of intrinsic matrix
        K_inv = np.linalg.inv(K)

        # Compute normalized coordinates in camera space
        cam_point = np.dot(K_inv, pixel_homogeneous)

        print(cam_point.flatten())
        cam_point *= depth

        print(cam_point.flatten())

        # Transform camera coordinates to world coordinates
        world_point = np.dot(T[:3, :3], cam_point) + T[:3, 3:]

        return world_point.flatten()

    def transform_point(self, transformation, point_wrt_source):
        point_wrt_target = tf2_geometry_msgs.do_transform_point(
            PointStamped(point=point_wrt_source), transformation
        ).point
        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

    def get_transformation(self, source_frame, target_frame, tf_cache_duration=2.0):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
        tf2_ros.TransformListener(tf_buffer)

        # get the tf at first available time
        try:
            transformation = tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0)
            )  # 0.1
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr(
                "Unable to find the transformation from %s to %s" % source_frame,
                target_frame,
            )
        return transformation


def main(args=None):
    # Initialize ROS node
    rospy.init_node("deproject_2D_to_3D", anonymous=False)

    recorder = Deprojection()
    recorder.run()


if __name__ == "__main__":
    main()
