import socket

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import Range
from audio_common_msgs.msg import AudioData
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import NiclaReceiverUDP


class Server:

    def __init__(self) -> None:

        # server address and port (the address of the machine running this code, any available port)
        ip = rospy.get_param("~pc_ip")
        port = rospy.get_param("~pc_port", '8002')
        packet_size = rospy.get_param("~packet_size", '65540')
        audio_buffer = rospy.get_param("~audio_buffer", '100')

        distance_topic = rospy.get_param("~distance_topic")
        self.picture_topic = rospy.get_param("~picture_topic")
        self.image_raw_topic = rospy.get_param("~image_raw_topic")
        camera_info_topic = rospy.get_param("~camera_info_topic")
        audio_topic = rospy.get_param("~audio_topic")

        self.nicla_receiver_udp = NiclaReceiverUDP.NiclaReceiverUDP(ip, port, packet_size, audio_buffer)
        self.nicla_receiver_udp.connect()

        self.distance_pub = rospy.Publisher(distance_topic, Range, queue_size=5)
        if self.picture_topic != "":
            self.picture_pub  = rospy.Publisher(self.picture_topic, CompressedImage, queue_size=5)
        if self.image_raw_topic != "":
            self.image_raw_pub  = rospy.Publisher(self.image_raw_topic, Image, queue_size=5)
        self.camera_info_pub  = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=5)

        self.range_msg = Range()
        self.range_msg.header.frame_id = rospy.get_param("~niclabx_distance_tf")
        self.range_msg.radiation_type = Range.INFRARED
        self.range_msg.min_range = 0
        self.range_msg.max_range = 4
        self.range_msg.field_of_view = 15

        self.image_msg = CompressedImage()
        self.image_msg.format = "jpeg"

        self.image_raw_msg = Image()
        self.camera_info_msg = CameraInfo()

        self.audio_msg = AudioData()
        self.audio_pub = rospy.Publisher(audio_topic, AudioData, queue_size=10)


    def run(self):
        # receive distance value
        self.nicla_receiver_udp.receive() 
        ros_time = rospy.Time.now()

        distance = int.from_bytes(self.nicla_receiver_udp.distance, "big")

        self.range_msg.range = distance
        self.range_msg.header.stamp = ros_time

        self.distance_pub.publish(self.range_msg)

        ### PUBLISH COMPRESSED
        if self.picture_topic != "":
            self.image_msg.header.stamp = ros_time
            self.image_msg.data = self.nicla_receiver_udp.image
            self.picture_pub.publish(self.image_msg)

        
        ### PUBLISH IMG RAW
        if self.image_raw_topic != "":
            # Convert the byte array to a numpy array
            nparr = np.frombuffer(self.nicla_receiver_udp.image, np.uint8)

            # Decode the compressed image
            img_raw = cv2.imdecode(nparr, cv2.IMREAD_COLOR) #NOTE: BGR CONVENTION 

            #To visualize the received img converted in img_raw 
            # cv2.imshow("Visualize img_raw", img_raw)
            # cv2.waitKey(1) 

            # Fill in the header
            self.image_raw_msg.header.stamp = ros_time
            self.image_raw_msg.header.frame_id = "nicla"   

            # Fill in image height and width
            self.image_raw_msg.height = img_raw.shape[0]
            self.image_raw_msg.width = img_raw.shape[1]

            # Fill in encoding
            self.image_raw_msg.encoding = "bgr8"  # Assuming OpenCV returns BGR format

            # Fill in endianness and step
            self.image_raw_msg.is_bigendian = 0  # Not big endian
            self.image_raw_msg.step = img_raw.shape[1] * 3  # Width * number of channels

            # Convert the OpenCV image to ROS Image format using cv_bridge
            bridge = CvBridge()
            try:
                self.image_raw_msg.data = bridge.cv2_to_imgmsg(img_raw, encoding="bgr8").data
            except CvBridgeError as e:
                print(e)                

            # Publish the ROS Image message
            self.image_raw_pub.publish(self.image_raw_msg)

        ### PUBLISH CAMERA INFO
        self.camera_info_msg.header.stamp = ros_time
        self.camera_info_pub.publish(self.camera_info_msg)

        ### AUDIO DATA
        audio_data = [self.nicla_receiver_udp.audio_deque.popleft() for _ in range(len(self.nicla_receiver_udp.audio_deque))]
        self.audio_msg.data = audio_data
        self.audio_pub.publish(self.audio_msg)
 
            


if __name__ == "__main__":

    rospy.init_node("nicla_receiver")

    server = Server()

    rate = rospy.Rate(50)

    rospy.loginfo("Starting receiving loop")

    while not rospy.is_shutdown():

        try:
            server.run()

        except OSError as e:
            rospy.logerr(e)

            pass # try again
        
        rate.sleep()

