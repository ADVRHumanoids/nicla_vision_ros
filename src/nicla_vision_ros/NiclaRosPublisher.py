#!/usr/bin/env python3

import struct
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from audio_common_msgs.msg import AudioData, AudioDataStamped, AudioInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from nicla_vision_ros import NiclaReceiverUDP, NiclaReceiverTCP


from std_msgs.msg import String
import sounddevice as sd 
from vosk import Model, KaldiRecognizer
import json 
import wave

import whisper  
import torch 
import speech_recognition as sr
from os import path
from wit import Wit

from threading import Thread
import audioop



class SpeechRecognizer:
    def __init__(self):
        
        self.FORMAT = 2
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 512
        self.RECORD_SECONDS = 3  # Duration to record audio in seconds
        self.WAVE_OUTPUT_FILENAME = "/home/dgasperini/output"
        self.WAVE_OUTPUT_FILENAME_i = 0
         

        self.audio_buffer = np.array([], dtype='int16')
        self.recording_frames = []  # Added for storing audio frames
        self.numbers = []

        self.command_pub = rospy.Publisher('/verbal_command', String, queue_size=10)

        self.model = Model("/home/dgasperini/ros_ws/src/relax_multi_modal/python/models/en")
        self.recognizer = KaldiRecognizer(self.model, 16000) # NOTE 16kHz sampling
        self.recognizer.SetGrammar('["open", "bottle", "cup", "[unk]"]')

        # self.model= whisper.load_model("base")
        # self.r = sr.Recognizer()

        # self.client = Wit("JE3DVMKWDKGRRUALGY2BROTGEO4W7EFW")
        # self.start = False

        # self.recognizing_thread = Thread(target=self.recognize_audio) 
        # self.recognizing_thread.start()
        
        rospy.loginfo("Speech recognizer is ready")


    def recognize_audio(self):
        while not rospy.is_shutdown():
            try:      

                resp = None  

                with open(self.WAVE_OUTPUT_FILENAME+str(self.WAVE_OUTPUT_FILENAME_i)+".wav", 'rb') as f:
                    resp = self.client.speech(f, {'Content-Type': 'audio/wav', 'Transfer-encoding': 'chunked'})
                
                print(str(resp['text']))

                # if result['text']:
                #         #rospy.loginfo(f"Recognized: {result['text']}")
                #         self.command_pub.publish(result['text'])

            except Exception as e:
                pass


    def process_audio(self, data):
        tmp = np.frombuffer(data, dtype="int16") #512 -> 1 sec     

        self.audio_buffer = np.concatenate((self.audio_buffer, tmp))
    
        self.recording_frames.append(data)

        if not self.start:

            if len(self.audio_buffer) >= 512*10:
                self.audio_buffer = self.audio_buffer[512*7:]

                self.recording_frames = self.recording_frames[7:]

            energy = audioop.rms(data, 2)

            # print("ENERGY: ", energy)

            if energy > 5000:
                self.start = True 

        #print(len(self.recording_frames))
        #print(int(self.RATE / self.CHUNK * self.RECORD_SECONDS))
        if self.start and len(self.recording_frames) >= int(self.RATE / self.CHUNK * self.RECORD_SECONDS):
            self.save_audio_to_wav()
            self.start = False
            self.audio_buffer = np.array([], dtype='int16')

    def process_audio_luca(self, data):
        tmp = np.frombuffer(data, dtype="int16")
        # new = tmp.tolist()
        # self.numbers.extend(new)

        # if len(self.numbers)>= 8000*5:
        #     # pcm_data = np.cumsum(self.numbers, dtype=np.int32)
        #     # pcm_data = np.array(pcm_data, dtype=np.int16) >> 16

        #     self.numbers = np.array(self.numbers, dtype="int16")

        #     with wave.open("/home/dgasperini/output.wav", 'w') as wf:
        #         wf.setnchannels(1)  # Mono
        #         wf.setsampwidth(2)  # 2 bytes (16 bits)
        #         wf.setframerate(8000)
        #         wf.writeframes(self.numbers.tobytes())

        #         print("Saved output.wav")

        #         self.numbers = []

        
        self.audio_buffer = np.concatenate((self.audio_buffer, tmp))
        self.recording_frames.append(data)

        if len(self.audio_buffer) > self.RATE*2:  # process in blocks of 1 second
            chunk = self.audio_buffer[:self.RATE*2]
            self.audio_buffer = self.audio_buffer[self.RATE*2:]

            #self.stream.write(self.audio_buffer)
            
            if self.recognizer.AcceptWaveform(chunk.tobytes()):
                result = json.loads(self.recognizer.Result())
                if result['text']:
                    #rospy.loginfo(f"Recognized: {result['text']}")
                    self.command_pub.publish(result['text'])

        #print(len(self.recording_frames))
        #print(int(self.RATE / self.CHUNK * self.RECORD_SECONDS))
        if len(self.recording_frames) >= int(self.RATE / self.CHUNK * self.RECORD_SECONDS):
            self.save_audio_to_wav()

    # def process_audio_damiano(self):

    #     if len(self.recording_frames_damiano)>= 16000*5:

    #         # pcm_data = np.cumsum(self.numbers, dtype=np.int32)
    #         # pcm_data = np.array(pcm_data, dtype=np.int16) >> 16

    #         daminao_np = np.array(self.recording_frames_damiano, dtype="int16")

    #         with wave.open("audio_dam.wav", 'w') as wf:
    #             wf.setnchannels(1)  # Mono
    #             wf.setsampwidth(2)  # 2 bytes (16 bits)
    #             wf.setframerate(16000)
    #             wf.writeframes(daminao_np.tobytes())
    #             rospy.loginfo(f"Saved recording aaaaaaaaaa")

    #         self.recording_frames_damiano = []



    def save_audio_to_wav(self):  # Added method to save audio to a WAV file
        self.WAVE_OUTPUT_FILENAME_i+=1
        if self.WAVE_OUTPUT_FILENAME_i == 21:
            self.WAVE_OUTPUT_FILENAME_i = 1

        with wave.open(self.WAVE_OUTPUT_FILENAME+str(self.WAVE_OUTPUT_FILENAME_i)+".wav", 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(self.recording_frames))

        rospy.loginfo(f"Saved recording as {self.WAVE_OUTPUT_FILENAME+'.wav'}")
        self.recording_frames = []



class NiclaRosPublisher:

    def __init__(self) -> None:

        # used for some header, be sure to use the same name here and in the urdf for proper rviz visualization
        nicla_name = rospy.get_param("~nicla_name", "nicla")

        # server address and port (the address of the machine running this code, any available port)
        ip = rospy.get_param("~receiver_ip")
        port = rospy.get_param("~receiver_port", '8002')
        connection_type = rospy.get_param("~connection_type", 'udp')

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
            self.range_msg.field_of_view = 0.471239 #27degrees according to arduino doc

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
            self.camera_info_msg = CameraInfo() 
            self.camera_info_msg.header.frame_id = nicla_name + "_camera"
            self.camera_info_msg.height = 240
            self.camera_info_msg.width = 320
            self.camera_info_msg.distortion_model = "plumb_rob"
            self.camera_info_msg.K = [416.650528, 0.000000, 166.124514,
                                      0.000000, 419.404643, 104.410543,
                                      0.000000, 0.000000, 1.000000]
            self.camera_info_msg.D = [0.176808, -0.590488, -0.008412, 0.015473, 0.000000]
            self.camera_info_msg.P = [421.373566, 0.000000, 168.731782, 0.000000,
                                      0.000000, 426.438812, 102.665989, 0.000000,
                                      0.000000, 0.000000, 1.000000, 0.000000]
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
            self.speech_recognizer = SpeechRecognizer()

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
            self.nicla_receiver_server = NiclaReceiverUDP(ip, port, 
                                                                        enable_range=self.enable_range, 
                                                                        enable_image=self.enable_camera_raw or self.enable_camera_compressed,
                                                                        enable_audio=self.enable_audio or self.enable_audio_stamped,
                                                                        enable_imu=self.enable_imu)
        elif connection_type == "tcp":
            self.nicla_receiver_server = NiclaReceiverTCP(ip, port, 
                                                            enable_range=self.enable_range, 
                                                            enable_image=self.enable_camera_raw or self.enable_camera_compressed,
                                                            enable_audio=self.enable_audio or self.enable_audio_stamped,
                                                            enable_imu=self.enable_imu)

        else:
            rospy.logerr("Connection type ", connection_type, " not known")
            raise Exception("Connection type not known")
            

        self.nicla_receiver_server.serve()

    # Function to convert a pair of bytes to a 16-bit unsigned integer
    def bytes_to_uint16(self, byte1, byte2):
        return (byte1 << 8) | byte2
    
    def run(self):

        if self.enable_range and ((range := self.nicla_receiver_server.get_range()) is not None):

            self.range_msg.header.stamp = rospy.Time.from_sec(range[0])
            self.range_msg.range = int.from_bytes(range[1], "little")/1000
            self.range_pub.publish(self.range_msg)

        ### PUBLISH IMAGE
        if self.enable_camera_raw or self.enable_camera_compressed:

            if (image := self.nicla_receiver_server.get_image()) is not None:

                ##Publish info
                self.camera_info_msg.header.stamp = rospy.Time.from_sec(image[0])
                self.camera_info_pub.publish(self.camera_info_msg)

                ### PUBLISH COMPRESSED
                if self.enable_camera_compressed:
                    self.image_compressed_msg.header.stamp = rospy.Time.from_sec(image[0])
                    self.image_compressed_msg.data = image[1]
                    self.image_compressed_pub.publish(self.image_compressed_msg)

                ### PUBLISH IMG RAW
                if self.enable_camera_raw:
                    # # Convert the byte array to a numpy array
                    # nparr = np.frombuffer(image[1], np.uint8)

                    # # Decode the compressed image
                    # img_raw = cv2.imdecode(nparr, cv2.IMREAD_COLOR) #NOTE: BGR CONVENTION 

                    img_raw = image[1]

                    self.image_raw_msg.header.stamp = rospy.Time.from_sec(image[0])
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

            if (audio_data := self.nicla_receiver_server.get_audio()) is not None:
                self.speech_recognizer.process_audio_luca(audio_data[1])

                if self.enable_audio:
                    self.audio_msg.data = audio_data[1]
                    self.audio_pub.publish(self.audio_msg)

                if self.enable_audio_stamped:
                    self.audio_stamped_msg.header.stamp = rospy.Time.from_sec(audio_data[0])
                    self.audio_stamped_msg.audio.data = audio_data[1]
                    self.audio_stamped_pub.publish(self.audio_stamped_msg)

        ### IMU DATA
        if self.enable_imu and ((imu := self.nicla_receiver_server.get_imu()) is not None):
            self.imu_msg.header.stamp = rospy.Time.from_sec(imu[0])

            try:
                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = struct.unpack('>ffffff', imu[1])
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
