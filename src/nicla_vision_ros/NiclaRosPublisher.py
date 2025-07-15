#!/usr/bin/env python3

import struct
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData, AudioDataStamped, AudioInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2

from nicla_vision_ros import NiclaReceiverUDP, NiclaReceiverTCP, NiclaReceiverSerial


class NiclaRosPublisher:

    def __init__(self) -> None:

        # used for some header, be sure to use the same name here and 
        # in the urdf for proper rviz visualization
        nicla_name = rospy.get_param("~nicla_name", "nicla")

        # server address and port (the address of the machine running 
        # this code, any available port)
        ip = rospy.get_param("~receiver_ip")
        port = rospy.get_param("~receiver_port", "8002")
        connection_type = rospy.get_param("~connection_type", "udp")
        baudrate = rospy.get_param("~baudrate", 0)

        self.enable_range = rospy.get_param("~enable_range", True)
        self.enable_camera_raw = rospy.get_param("~enable_camera_raw", True)
        self.enable_camera_compressed = rospy.get_param(
            "~enable_camera_compressed", False
        )
        self.camera_receive_compressed = rospy.get_param(
            "~camera_receive_compressed", False
        )
        self.camera_pixel_format = rospy.get_param(
            "~camera_pixel_format", "rgb565"
        )
        self.camera_width = rospy.get_param(
            "~camera_width", 320
        )
        self.camera_height = rospy.get_param(
            "~camera_height", 240
        )
        self.enable_audio = rospy.get_param("~enable_audio", True)
        self.enable_audio_stamped = rospy.get_param(
            "~enable_audio_stamped", False
        )
        self.enable_audio_recognition_vosk = rospy.get_param(
            "~enable_audio_recognition_vosk", False
        )
        self.audio_recognition_model = rospy.get_param(
            "~audio_recognition_model", ""
        )
        self.audio_recognition_grammar = rospy.get_param(
            "~audio_recognition_grammar", ""
        )
        self.audio_recognition_listen_seconds = rospy.get_param(
            "~audio_recognition_listen_seconds", 2
        )
        self.audio_recognition_wave_output_filename = rospy.get_param(
            "~audio_recognition_wave_output_filename", ""
        )
        self.enable_imu = rospy.get_param("~enable_imu", True)

        if self.enable_range:
            range_topic = nicla_name + "/tof"
            self.range_pub = rospy.Publisher(range_topic, Range, queue_size=5)
            self.range_msg = Range()
            self.range_msg.header.frame_id = nicla_name + "_tof"
            self.range_msg.radiation_type = Range.INFRARED
            self.range_msg.min_range = 0
            self.range_msg.max_range = 3.60
            self.range_msg.field_of_view = (
                0.471239  # 27degrees according to VL53L1X spec
            )

        if self.enable_camera_raw:
            # default topic name of image transport (which is not available in python so we do not use it)
            self.image_raw_topic = nicla_name + "/camera/image_raw"
            self.image_raw_msg = Image()
            self.image_raw_msg.header.frame_id = nicla_name + "_camera"
            self.image_raw_pub = rospy.Publisher(
                self.image_raw_topic, Image, queue_size=5
            )

        if self.enable_camera_compressed:
            self.image_compressed_topic = (
                nicla_name + "/camera/image_raw/compressed"
            )
            self.image_compressed_msg = CompressedImage()
            self.image_compressed_msg.header.frame_id = nicla_name + "_camera"
            self.image_compressed_msg.format = "jpeg"
            self.image_compressed_pub = rospy.Publisher(
                self.image_compressed_topic, CompressedImage, queue_size=5
            )

        if self.enable_camera_raw or self.enable_camera_compressed:

            self.cv_bridge = CvBridge()

            camera_info_topic = nicla_name + "/camera/camera_info"
            self.camera_info_msg = CameraInfo()
            self.camera_info_msg.header.frame_id = nicla_name + "_camera"
            self.camera_info_msg.height = self.camera_height
            self.camera_info_msg.width = self.camera_width 
            self.camera_info_msg.distortion_model = "plumb_rob"
            #WARNING: this calibration consider 320x240 resolution with RGB565 pixel format
            self.camera_info_msg.K = [
                416.650528,
                0.000000,
                166.124514,
                0.000000,
                419.404643,
                104.410543,
                0.000000,
                0.000000,
                1.000000,
            ]
            self.camera_info_msg.D = [
                0.176808,
                -0.590488,
                -0.008412,
                0.015473,
                0.000000,
            ]
            self.camera_info_msg.P = [
                421.373566,
                0.000000,
                168.731782,
                0.000000,
                0.000000,
                426.438812,
                102.665989,
                0.000000,
                0.000000,
                0.000000,
                1.000000,
                0.000000,
            ]
            self.camera_info_pub = rospy.Publisher(
                camera_info_topic, CameraInfo, queue_size=5
            )

        if self.enable_audio:
            audio_topic = nicla_name + "/audio"
            self.audio_msg = AudioData()
            self.audio_pub = rospy.Publisher(
                audio_topic, AudioData, queue_size=10
            )

        if self.enable_audio_stamped:
            audio_stamped_topic = nicla_name + "/audio_stamped"
            self.audio_stamped_msg = AudioDataStamped()
            self.audio_stamped_pub = rospy.Publisher(
                audio_stamped_topic, AudioDataStamped, queue_size=10
            )

        if self.enable_audio or self.enable_audio_stamped:
            audio_info_topic = nicla_name + "/audio_info"
            self.audio_info_msg = AudioInfo()
            self.audio_info_msg.channels = 1
            self.audio_info_msg.sample_rate = 16000
            self.audio_info_msg.sample_format = "S16LE"
            self.audio_info_msg.bitrate = 0
            self.audio_info_msg.coding_format = "raw"
            self.audio_info_pub = rospy.Publisher(
                audio_info_topic, AudioInfo, queue_size=1
            )

        if self.enable_imu:
            imu_topic = nicla_name + "/imu"
            self.imu_msg = Imu()
            self.imu_msg.header.frame_id = nicla_name + "_imu"
            self.imu_msg.orientation.x = 0
            self.imu_msg.orientation.y = 0
            self.imu_msg.orientation.z = 0
            self.imu_msg.orientation.w = 1
            self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=5)

        if connection_type == "udp":
            self.nicla_receiver_server = NiclaReceiverUDP(
                ip,
                port,
                enable_range=self.enable_range,
                enable_image=self.enable_camera_raw
                or self.enable_camera_compressed,
                enable_audio=self.enable_audio or self.enable_audio_stamped,
                enable_imu=self.enable_imu,
            )
        elif connection_type == "tcp":
            self.nicla_receiver_server = NiclaReceiverTCP(
                ip,
                port,
                enable_range=self.enable_range,
                enable_image=self.enable_camera_raw
                or self.enable_camera_compressed,
                enable_audio=self.enable_audio or self.enable_audio_stamped,
                enable_imu=self.enable_imu,
            )
        elif connection_type == "serial":
            self.nicla_receiver_server = NiclaReceiverSerial(
                port=port,
                baudrate=baudrate,
                enable_range=self.enable_range,
                enable_image=self.enable_camera_raw
                or self.enable_camera_compressed,
                camera_receive_compressed=self.camera_receive_compressed,
                camera_pixel_format = self.camera_pixel_format,
                camera_width = self.camera_width,
                camera_height = self.camera_height,
                enable_audio=self.enable_audio or self.enable_audio_stamped,
                enable_imu=self.enable_imu,
            )

        else:
            rospy.logerr("Connection type ", connection_type, " not known")
            raise Exception("Connection type not known")

        if self.enable_audio_recognition_vosk:
            if not self.audio_recognition_model:
                rospy.logerr(
                    "Path for VOSK recognizer model is an empty string! Please provide 'audio_recognition_model_path' and 'audio_recognition_model_name' args"
                )
                exit()

            from nicla_vision_ros import SpeechRecognizer

            self.speech_recognizer = SpeechRecognizer.SpeechRecognizer(
                self.audio_recognition_model,
                self.audio_recognition_grammar,
                self.audio_recognition_listen_seconds,
                self.audio_recognition_wave_output_filename,
            )
            self.speech_recognizer_pub = rospy.Publisher(
                nicla_name + "/audio_recognized", String, queue_size=10
            )

        self.nicla_receiver_server.serve()

    def run(self):

        if self.enable_range and (
            (range := self.nicla_receiver_server.get_range()) is not None
        ):

            self.range_msg.header.stamp = rospy.Time.from_sec(range[0])
            self.range_msg.range = int.from_bytes(range[1], "little") / 1000
            self.range_pub.publish(self.range_msg)

        # PUBLISH IMAGE
        if self.enable_camera_raw or self.enable_camera_compressed:

            if (image := self.nicla_receiver_server.get_image()) is not None:

                # Publish info
                self.camera_info_msg.header.stamp = rospy.Time.from_sec(
                    image[0]
                )
                self.camera_info_pub.publish(self.camera_info_msg)

                img_raw = image[1]

                # PUBLISH COMPRESSED
                if self.enable_camera_compressed: 
                    self.image_compressed_msg.header.stamp = (
                        rospy.Time.from_sec(image[0])
                    )

                    try:
                        self.image_compressed_msg.data = (
                            self.cv_bridge.cv2_to_compressed_imgmsg(
                                img_raw, dst_format="jpg"
                            ).data
                        )
                    except CvBridgeError as e:
                        rospy.logerr(e)

                    self.image_compressed_pub.publish(
                        self.image_compressed_msg
                    )

                # PUBLISH IMG RAW
                if self.enable_camera_raw:

                    self.image_raw_msg.header.stamp = rospy.Time.from_sec(
                        image[0]
                    )
                    self.image_raw_msg.height = img_raw.shape[0]
                    self.image_raw_msg.width = img_raw.shape[1]
                    self.image_raw_msg.encoding = (
                        "bgr8"  # Assuming OpenCV returns BGR format
                    )
                    self.image_raw_msg.is_bigendian = 0  # Not big endian
                    self.image_raw_msg.step = (
                        img_raw.shape[1] * 3
                    )  # Width * number of channels

                    try:
                        self.image_raw_msg.data = self.cv_bridge.cv2_to_imgmsg(
                            img_raw, encoding="bgr8"
                        ).data
                    except CvBridgeError as e:
                        print(e)

                    self.image_raw_pub.publish(self.image_raw_msg)

        # AUDIO DATA
        if self.enable_audio or self.enable_audio_stamped:
            self.audio_info_pub.publish(self.audio_info_msg)

        if (
            self.enable_audio
            or self.enable_audio_stamped
            or self.enable_audio_recognition_vosk
        ):

            if (
                audio_data := self.nicla_receiver_server.get_audio()
            ) is not None:

                if self.enable_audio:
                    self.audio_msg.data = audio_data[1]
                    self.audio_pub.publish(self.audio_msg)

                if self.enable_audio_stamped:
                    self.audio_stamped_msg.header.stamp = rospy.Time.from_sec(
                        audio_data[0]
                    )
                    self.audio_stamped_msg.audio.data = audio_data[1]
                    self.audio_stamped_pub.publish(self.audio_stamped_msg)

                if self.enable_audio_recognition_vosk:
                    audio_recognized = self.speech_recognizer.process_audio(
                        audio_data[1]
                    )
                    if audio_recognized:  # if not empty
                        self.speech_recognizer_pub.publish(audio_recognized)

        ### IMU DATA
        if self.enable_imu and (
            (imu := self.nicla_receiver_server.get_imu()) is not None
        ):
            self.imu_msg.header.stamp = rospy.Time.from_sec(imu[0])

            try:
                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = struct.unpack(
                    "<ffffff", imu[1]
                )  # <: little endian
            except Exception as e:
                rospy.logerr("imu pack has ", len(imu[1]), " bytes")
                raise e

            self.imu_msg.angular_velocity.x = 0.017453 * gyro_x
            self.imu_msg.angular_velocity.y = 0.017453 * gyro_y
            self.imu_msg.angular_velocity.z = 0.017453 * gyro_z
            self.imu_msg.linear_acceleration.x = 9.80665 * acc_x
            self.imu_msg.linear_acceleration.y = 9.80665 * acc_y
            self.imu_msg.linear_acceleration.z = 9.80665 * acc_z
            self.imu_pub.publish(self.imu_msg)

    def stop(self):
        self.nicla_receiver_server.stop_serve()
