#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from tf2_ros import TransformStamped, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_matrix
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def project_point(pt, cam_info:CameraInfo):
    """Project a 3D point (camera coords) into 2D pixel using intrinsics."""
    fx = cam_info.K[0]
    fy = cam_info.K[4]
    cx = cam_info.K[2]
    cy = cam_info.K[5]

    u = fx * pt[0] / pt[2] + cx
    v = fy * pt[1] / pt[2] + cy
    return int(round(u)), int(round(v))


def draw_tf_on_image(cv_image:np.ndarray, cam_T_target:TransformStamped, cam_info:CameraInfo):

    target_origin = np.array([cam_T_target.transform.translation.x, 
                              cam_T_target.transform.translation.y, 
                              cam_T_target.transform.translation.z])
    target_R = quaternion_matrix(
        [cam_T_target.transform.rotation.x,
        cam_T_target.transform.rotation.y,
        cam_T_target.transform.rotation.z,
        cam_T_target.transform.rotation.w] 
    )[:3, :3]

    x_axis_cam = target_R.dot(np.array([1, 0, 0]))
    y_axis_cam = target_R.dot(np.array([0, 1, 0]))
    z_axis_cam = target_R.dot(np.array([0, 0, 1]))

    ax_length = 0.1  # Length of the axes in meters
    x_axis_end = target_origin + x_axis_cam * ax_length
    y_axis_end = target_origin + y_axis_cam * ax_length
    z_axis_end = target_origin + z_axis_cam * ax_length

    p0 = project_point(target_origin, cam_info)
    px_end = project_point(x_axis_end, cam_info)
    py_end = project_point(y_axis_end, cam_info)
    pz_end = project_point(z_axis_end, cam_info)
    
    if (0 <= p0[0] < cv_image.shape[1]) and (0 <= p0[1] < cv_image.shape[0]):
        cv2.circle(cv_image, p0, 6, (0, 0, 0), -1)  # black dot
    else:
        rospy.logwarn("Origin out of bounds: %s, from 3D %s", p0, target_origin)
    
    cv2.line(cv_image, p0, px_end, (0, 0, 255), 4)  # Red X-axis, color in BGR
    cv2.line(cv_image, p0, py_end, (0, 255, 0), 4)  # Green Y-axis color in BGR
    cv2.line(cv_image, p0, pz_end, (255, 0, 0), 4)  # Blue Z-axis color in BGR

    caption = cam_T_target.child_frame_id
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    color = (0, 0, 255)  # Red
    thickness = 1
    text_size, _ = cv2.getTextSize(caption, font, font_scale, thickness)
    text_x = p0[0] - text_size[0] // 2
    text_y = p0[1] - text_size[1] +5  # Slightly above the circle

    cv2.putText(cv_image, "X", (px_end[0] + 5, px_end[1] - 5),
                font, font_scale, (0, 0, 255), thickness)
    cv2.putText(cv_image, "Y", (py_end[0] + 5, py_end[1] - 5),
                font, font_scale, (0, 255, 0), thickness)
    cv2.putText(cv_image, "Z", (pz_end[0] + 5, pz_end[1] - 5),
                font, font_scale, (255, 0, 0), thickness)

    cv2.putText(
        cv_image,
        caption,
        (text_x, text_y),
        font,
        0.6,
        (0, 0, 0),
        thickness,
    )


class TFOnImage:
    def __init__(self) -> None:

        cam_topic = rospy.get_param(
            "~cam_topic",
            "/nicla/camera/image_raw/compressed",
        )
        cam_info_topic = rospy.get_param(
            "~cam_info_topic",
            "/nicla/camera/camera_info",
        )
        self.cam_compressed = rospy.get_param(
            "~cam_compressed", True
        )

        img_out_topic = rospy.get_param(
            "~img_out_topic",
            "/nicla/camera_with_tf/image_raw/compressed",
        )

        self.target_frames = rospy.get_param(
            "~target_frames",
            ["target"],
        )

        if self.cam_compressed:
            self.image_sub = rospy.Subscriber(
                cam_topic, CompressedImage, self.image_callback
            )
            self.image = CompressedImage()
        else:
            self.image_sub = rospy.Subscriber(
                cam_topic, Image, self.image_callback
            )
            self.image = Image()

        rospy.loginfo("Waiting for camera info on topic: %s", cam_info_topic)
        self.cam_info = rospy.wait_for_message(
            cam_info_topic, CameraInfo)
        rospy.loginfo("Camera info received.")

        self.img_pub = rospy.Publisher(
            img_out_topic, CompressedImage, queue_size=10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.cv_image = np.array([])
        self.out_image = CompressedImage()

        self.cam_T_targets = []
        for target_frame in self.target_frames:
            self.cam_T_targets.append(TransformStamped())

        self.bridge = CvBridge()

        self.img_arrived = False

        rospy.loginfo("TFOnImage initialized")

    def image_callback(self, data):
        self.img_arrived = True
        self.image = data

    def range_callback(self, data):
        self.range = data

    def get_image(self):
        try:
            if self.cam_compressed:
                self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
                    self.image, "passthrough"
                )
            else:
                self.cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return False   
        return True
    
    def get_transforms(self):
        try:
            for i, target_frame in enumerate(self.target_frames):
                self.cam_T_targets[i] = self.tf_buffer.lookup_transform(
                    self.cam_info.header.frame_id,
                    target_frame,
                    rospy.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logerr("Error getting transforms: %s", e)
            return False
        
        return True
                

    def run(self):

        if not self.img_arrived:
            return False

        if not self.get_image():
            return False
      
        if not self.get_transforms():
            return False
        
        for cam_T_target in self.cam_T_targets:
            draw_tf_on_image(self.cv_image, cam_T_target, self.cam_info)

        try:
            self.out_image.data = self.bridge.cv2_to_compressed_imgmsg(
                self.cv_image, dst_format="jpg"
            ).data
        except CvBridgeError as e:
            rospy.logerr(e)
            return False

        self.img_pub.publish(self.out_image)

        return True
