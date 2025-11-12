#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from sensor_msgs.msg import Range
from cv_bridge import CvBridge, CvBridgeError
import cv2

def draw_dot_on_image(cv_image, range_value:float, color=(0, 0, 255)):

    # Draw a red dot at the center of the image
    height, width, _ = cv_image.shape
    center = (int((width / 2)), int((height / 2)))
    center_off = (center[0] + 6, center[1] - 19)

    cv2.circle(cv_image, center_off, 2, (0, 0, 255), -1)

    # Add a caption with the TOF range value near the center of the image
    caption = f"{range_value:.2f}m"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    color = (0, 0, 255)  # Red
    thickness = 1
    text_size, _ = cv2.getTextSize(caption, font, font_scale, thickness)
    text_x = center_off[0] - text_size[0] // 2
    text_y = center_off[1] - text_size[1] +5  # Slightly above the circle

    cv2.putText(
        cv_image,
        caption,
        (text_x, text_y),
        font,
        font_scale,
        color,
        thickness,
    )


class TofOnImage:
    def __init__(self) -> None:

        nicla_name = rospy.get_param("~nicla_name", "nicla")
        nicla_cam_topic = rospy.get_param(
            "~nicla_cam_topic",
            "/" + nicla_name + "/camera/image_raw/compressed",
        )
        img_out_topic = rospy.get_param(
            "~img_out_topic",
            "/" + nicla_name + "/camera_with_tof/image_raw/compressed",
        )
        self.nicla_cam_compressed = rospy.get_param(
            "~nicla_cam_compressed", True
        )
        nicla_range_topic = rospy.get_param(
            "~nicla_range_topic", "/" + nicla_name + "/tof"
        )

        if self.nicla_cam_compressed:
            self.image_sub = rospy.Subscriber(
                nicla_cam_topic, CompressedImage, self.image_callback
            )
            self.image = CompressedImage()
        else:
            self.image_sub = rospy.Subscriber(
                nicla_cam_topic, Image, self.image_callback
            )
            self.image = Image()

        self.range_sub = rospy.Subscriber(
            nicla_range_topic, Range, self.range_callback
        )

        self.img_pub = rospy.Publisher(
            img_out_topic, CompressedImage, queue_size=10
        )

        self.out_image = CompressedImage()
        self.range = Range()

        self.bridge = CvBridge()

        self.img_arrived = False

        rospy.loginfo("TofOnImage initialized")

    def image_callback(self, data):
        self.img_arrived = True
        self.image = data

    def range_callback(self, data):
        self.range = data

    def run(self):

        if not self.img_arrived:
            return

        try:
            if self.nicla_cam_compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(
                    self.image, "passthrough"
                )
            else:
                cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        draw_dot_on_image(cv_image, self.range.range)

        try:
            self.out_image.data = self.bridge.cv2_to_compressed_imgmsg(
                cv_image, dst_format="jpg"
            ).data
        except CvBridgeError as e:
            rospy.logerr(e)

        self.img_pub.publish(self.out_image)

        return True
