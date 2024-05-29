#!/usr/bin/env python

import struct
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from audio_common_msgs.msg import AudioData, AudioDataStamped, AudioInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

import NiclaReceiverUDP


class NiclaRosPublisher:

    def __init__(self) -> None:

        # used for some header, be sure to use the same name here and in the urdf for proper rviz visualization
        nicla_name = rospy.get_param("~nicla_name", "nicla")

        # server address and port (the address of the machine running this code, any available port)
        ip = rospy.get_param("~receiver_ip")
        port = rospy.get_param("~receiver_port", '8002')
        packet_size = rospy.get_param("~packet_size", '65540')
        audio_buffer = rospy.get_param("~audio_buffer", '100')

        self.enable_range = rospy.get_param("~enable_range", True)
        self.enable_camera_raw = rospy.get_param("~enable_camera_raw", False)
        self.enable_camera_compressed = rospy.get_param("~enable_camera_compressed", True)
        self.enable_audio = rospy.get_param("~enable_audio", True)
        self.enable_audio_stamped = rospy.get_param("~enable_audio_stamped", False)
        self.enable_imu = rospy.get_param("~enable_imu", True)

        if self.enable_range:
            range_topic = nicla_name + "/tof" 
            self.range_pub = rospy.Publisher(range_topic, Range, queue_size=5)
            self.range_msg = Range()
            self.range_msg.header.frame_id = nicla_name + "_tof"
            self.range_msg.radiation_type = Range.INFRARED
            self.range_msg.min_range = 0
            self.range_msg.max_range = 4
            self.range_msg.field_of_view = 15

        if self.enable_camera_raw:
            #default topic name of image transport (which is not available in python so we do not use it)
            self.image_raw_topic = nicla_name + "/camera/image_raw"
            self.image_raw_msg = Image()
            self.image_raw_msg.header.frame_id = nicla_name + "_camera"
            self.image_raw_pub  = rospy.Publisher(self.image_raw_topic, Image, queue_size=5)

        if self.enable_camera_compressed:
            self.image_compressed_topic = nicla_name + "/camera/image_raw/compressed"
            self.image_compressed_msg = CompressedImage()
            self.image_compressed_msg.header.frame_id = nicla_name + "_camera"
            self.image_compressed_msg.format = "jpeg"
            self.image_compressed_pub  = rospy.Publisher(self.image_compressed_topic, CompressedImage, queue_size=5)

        if self.enable_camera_raw or self.enable_camera_compressed:
            camera_info_topic = nicla_name + "/camera/camera_info"
            self.camera_info_msg = CameraInfo() #TODO
            self.camera_info_pub  = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=5)

        if self.enable_audio:
            audio_topic = nicla_name + "/audio"
            self.audio_msg = AudioData()
            self.audio_pub = rospy.Publisher(audio_topic, AudioData, queue_size=10)

        if self.enable_audio_stamped:
            audio_stamped_topic = nicla_name + "/audio_stamped"
            self.audio_stamped_msg = AudioDataStamped()
            self.audio_stamped_pub = rospy.Publisher(audio_stamped_topic, AudioDataStamped, queue_size=10)

        if self.enable_audio or self.enable_audio_stamped:
            audio_info_topic = nicla_name + "/audio_info"
            self.audio_info_msg = AudioInfo()
            self.audio_info_msg.channels = 1
            self.audio_info_msg.sample_rate = 16000
            self.audio_info_msg.sample_format = "S16LE"
            self.audio_info_msg.bitrate = 0
            self.audio_info_msg.coding_format = "raw"
            self.audio_info_pub = rospy.Publisher(audio_info_topic, AudioInfo, queue_size=1)
        
        if self.enable_imu:
            imu_topic = nicla_name + "/imu"
            self.imu_msg = Imu()
            self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=5)

        self.nicla_receiver_udp = NiclaReceiverUDP.NiclaReceiverUDP2(ip, port, 
                                                                     enable_range=self.enable_range, 
                                                                     enable_image=self.enable_camera_raw or self.enable_camera_compressed,
                                                                     enable_audio=self.enable_audio or self.enable_audio_stamped,
                                                                     enable_imu=self.enable_imu)
        self.nicla_receiver_udp.serve()


    def run(self):

        if self.enable_range and ((range := self.nicla_receiver_udp.get_range()) is not None):

            self.range_msg.header.stamp = rospy.Time.from_sec(range[0]/1000)
            self.range_msg.range = int.from_bytes(range[1], "big")
            self.range_pub.publish(self.range_msg)

        ### PUBLISH IMAGE
        if self.enable_camera_raw or self.enable_camera_compressed:
            self.camera_info_msg.header.stamp = rospy.Time.now()
            self.camera_info_pub.publish(self.camera_info_msg)

            if (image := self.nicla_receiver_udp.get_image()) is not None:

                ### PUBLISH COMPRESSED
                if self.enable_camera_compressed:
                    self.image_compressed_msg.header.stamp = rospy.Time.from_sec(image[0]/1000)
                    self.image_compressed_msg.data = image[1]
                    self.image_compressed_pub.publish(self.image_compressed_msg)

                ### PUBLISH IMG RAW
                if self.enable_camera_raw:
                    # Convert the byte array to a numpy array
                    nparr = np.frombuffer([1], np.uint8)

                    # Decode the compressed image
                    img_raw = cv2.imdecode(nparr, cv2.IMREAD_COLOR) #NOTE: BGR CONVENTION 

                    self.image_raw_msg.header.stamp = rospy.Time.from_sec(image[0]/1000)
                    self.image_raw_msg.height = img_raw.shape[0]
                    self.image_raw_msg.width = img_raw.shape[1]
                    self.image_raw_msg.encoding = "bgr8"  # Assuming OpenCV returns BGR format
                    self.image_raw_msg.is_bigendian = 0  # Not big endian
                    self.image_raw_msg.step = img_raw.shape[1] * 3  # Width * number of channels

                    # Convert the OpenCV image to ROS Image format using cv_bridge
                    bridge = CvBridge()
                    try:
                        self.image_raw_msg.data = bridge.cv2_to_imgmsg(img_raw, encoding="bgr8").data
                    except CvBridgeError as e:
                        print(e)                

                    self.image_raw_pub.publish(self.image_raw_msg)



        ### AUDIO DATA
        if self.enable_audio or self.enable_audio_stamped :

            self.audio_info_pub.publish(self.audio_info_msg)

            if (audio_data := self.nicla_receiver_udp.get_audio()) is not None:

                if self.enable_audio:
                    self.audio_msg.data = audio_data[1]
                    self.audio_pub.publish(self.audio_msg)

                if self.enable_audio_stamped:
                    self.audio_stamped_msg.header.stamp = rospy.Time.from_sec(audio_data[0]/1000)
                    self.audio_stamped_msg.audio.data = audio_data[1]
                    self.audio_stamped_pub.publish(self.audio_stamped_msg)

        ### IMU DATA
        if self.enable_imu and ((imu := self.nicla_receiver_udp.get_imu()) is not None):
            self.imu_msg.header.stamp = rospy.Time.from_sec(imu[0]/1000)

            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = struct.unpack('>ffffff', imu[1])
            self.imu_msg.orientation = 0,0,0,1
            self.imu_msg.angular_velocity = 0.017453 * gyro_x, 0.017453 * gyro_y, 0.017453 * gyro_z
            self.imu_msg.linear_acceleration = acc_x, acc_y, acc_z
            self.imu_pub.publish(self.imu_msg)

    def stop(self):
        self.nicla_receiver_udp.stop_serve()



if __name__ == "__main__":

    rospy.init_node("nicla_receiver")

    nicla_ros_publisher = NiclaRosPublisher()

    rate = rospy.Rate(500)

    rospy.loginfo("Starting receiving loop")

    while not rospy.is_shutdown():

        try:
            nicla_ros_publisher.run()

        except OSError as e:
            rospy.logerr(e)

            pass # try again
        
        rate.sleep()

    nicla_ros_publisher.stop()
