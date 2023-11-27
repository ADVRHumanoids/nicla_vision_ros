# edb 20231122
# receiving distance and picture from Arduino Nicla Vision (client)
# and sending them on two ROS messages
# the picture fits into one UDP packet after compression

import time
import socket
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class Server:

    # set maximum packet size
    packet_size = 65540

    def __init__(self) -> None:

        # server address and port (the address of the machine running this code, any available port)
        ip = rospy.get_param("server_ip", "192.168.61.112")
        port = rospy.get_param("server_port", 8000)

        # server socket init 
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((ip, port))
        rospy.loginfo("Waiting for niclabox to stream on " + ip + " : " + str(port))

        distance_topic = rospy.get_param("distance_topic", "niclabox/distance")
        picture_topic = rospy.get_param("picture_topic", "niclabox/picture/compressed")

        self.distance_pub = rospy.Publisher(distance_topic, Int32, queue_size=5)
        self.picture_pub  = rospy.Publisher(picture_topic, CompressedImage, queue_size=5)

        self.cv_bridge = CvBridge()


    def receive_and_ros(self):
        # receive distance value
        packet, _ = self.server.recvfrom(self.packet_size)

        if len(packet) < 100: # a small packet is the distance
            distance = packet
            distance = int.from_bytes(distance, "big")

            # ros_dist = Int32()
            # ros_dist.data = distance
            self.distance_pub.publish(Int32(distance))
            # print("distance", distance)

        else:  # a big packet is the picture

            picture = packet
           
            # image = cv2.imdecode(np.frombuffer(picture, np.uint8), cv2.IMREAD_COLOR)
            # print("image", image)

            # self.picture_pub.publish(self.cv_bridge.cv2_to_compressed_imgmsg(image, 'bgr8'))
            # self.picture_pub.publish(self.cv_bridge.cv2_to_compressed_imgmsg(image))
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = picture

            self.picture_pub.publish(msg)

            


if __name__ == "__main__":

    rospy.init_node("niclabox")

    server = Server()

    rate = rospy.Rate(50)

    rospy.loginfo("Starting receiving loop")


    while not rospy.is_shutdown():

        try:
            server.receive_and_ros()

        except OSError as e:
            rospy.logerr(e)

            pass # try again
        
        rate.sleep()

