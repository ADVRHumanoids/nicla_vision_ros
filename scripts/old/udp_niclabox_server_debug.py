#!/usr/bin/env python3

import queue
import socket
import socketserver
from threading import Thread
import time
import struct
import numpy as np


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import wave

import cv2
from PIL import Image
import struct


IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
RANGE_TYPE = 0b10
IMU_TYPE = 0b11


class UDPHandler(socketserver.BaseRequestHandler):
    def handle(self):

        # with udp, self.request is a pair (data, socket)
        packet = self.request[0]
        # socket = self.request[1]

        print("LEN RECV: ", len(packet))

        # print(np.frombuffer(packet, dtype="uint8"))

        # size_packet = int.from_bytes(packet[:4], "big")

        # if size_packet == len(packet[4:]):

        #     timestamp = time.time() #int.from_bytes(packet[4:8], "big")
        #     data_type = packet[8]
        #     data = packet[9:]

        #     if data_type == RANGE_TYPE:
        #         if self.server.enable_range:
        #             self.server.range_buffer.put_nowait((timestamp, data))
        #         else:
        #             pass

        #     elif data_type == IMAGE_TYPE:
        #         if self.server.enable_image:
        #             self.server.image_buffer.put_nowait((timestamp, data))
        #         else:
        #             pass

        #     elif data_type == AUDIO_TYPE:
        #         if self.server.enable_audio:
        #             self.server.audio_buffer.put_nowait((timestamp, data))
        #         else:
        #             pass

        #     elif data_type == IMU_TYPE:
        #         if self.server.enable_imu:
        #             self.server.imu_buffer.put_nowait((timestamp, data))
        #         else:
        #             pass

        # else:
        #     print("Warning: received packet of length {}, but expected length was {}!".format(len(packet[4:]), size_packet))


class NiclaReceiverUDP(socketserver.UDPServer):

    def __init__(
        self,
        server_ip,
        server_port,
        enable_range=False,
        enable_image=False,
        enable_audio=False,
        enable_imu=False,
    ):

        super().__init__((server_ip, server_port), UDPHandler)

        self.enable_range = enable_range
        self.enable_image = enable_image
        self.enable_audio = enable_audio
        self.enable_imu = enable_imu

        if self.enable_range:
            self.range_buffer = queue.Queue(maxsize=10)
        if self.enable_image:
            self.image_buffer = queue.Queue(maxsize=10)
        if self.enable_audio:
            self.audio_buffer = queue.Queue(maxsize=10)
        if self.enable_imu:
            self.imu_buffer = queue.Queue(maxsize=10)

        self.server_thread = None

    def serve(self):
        self.server_thread = Thread(target=self.serve_forever)
        self.server_thread.start()

    def stop_serve(self):
        print("stopping")
        self.shutdown()
        self.server_thread.join()
        self.server_close()

    def get_range(self):

        if not self.range_buffer.empty():
            return self.range_buffer.get_nowait()
        else:
            return None

    def get_image(self):
        if not self.image_buffer.empty():
            return self.image_buffer.get_nowait()
        else:
            return None

    def get_audio(self):
        if not self.audio_buffer.empty():
            return self.audio_buffer.get_nowait()
        else:
            return None

    def get_imu(self):
        if not self.imu_buffer.empty():
            return self.imu_buffer.get_nowait()
        else:
            return None


if __name__ == "__main__":
    # 192.168.1.103   10.240.23.49

    nicla_receiver_server = NiclaReceiverUDP(
        "10.42.0.1",
        8002,
        enable_range=False,
        enable_image=False,
        enable_audio=True,
        enable_imu=False,
    )

    try:
        nicla_receiver_server.serve()
    except Exception as e:
        print(e)

    while True:
        pass
        # print("Server running!")

    nicla_receiver_server.stop()


##########################################################################################################à

# import time
# import socket
# import numpy as np
# import cv2
# from pydub import AudioSegment

# IMAGE_TYPE = 0b00
# AUDIO_TYPE = 0b01
# DISTANCE_TYPE = 0b10
# IMU_TYPE = 0b11

# FLAG = 1
# accumulated_audio_data = []


# # server address and port (the address of the machine running this code, any available port)
# ip = "10.240.23.87"
# port = 8002

# # set maximum packet size
# packet_size = 65540

# # image window init
# cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)

# # server socket init
# server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# server.bind((ip, port))
# print("Waiting for niclabox to stream on", ip, ":", port)

# def receive_and_ros():
#     global FLAG

#     packet, client_address = server.recvfrom(packet_size)

#     data_type = packet[0]
#     packet = packet[1:]

#     # if len(packet) < 100: # a small packet is the distance
#     if data_type == DISTANCE_TYPE:
#         distance = packet
#         distance = int.from_bytes(distance, "big")

#         # Print distance in terminal
#         print("Distance (mm): ", distance)

#     elif data_type == IMAGE_TYPE:
#         picture = packet

#         # Show image with numpy OpenCV
#         image = cv2.imdecode(np.frombuffer(picture, np.uint8), cv2.IMREAD_COLOR)
#         cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)
#         cv2.imshow("niclabox", image)
#         if cv2.waitKey(1) == ord('q'): # Press Q to exit
#             exit(0)

#         # uncomment to output to a file without using numpy and OpenCV
#         # distance_file = open("distance.txt", "w")
#         # distance_file.write(str(distance))
#         # distance_file.close()
#         # picture_file = open("picture.jpg", "wb")
#         # picture_file.write(picture)
#         # picture_file.close()
#     elif data_type == AUDIO_TYPE:

#         if FLAG == 1:
#             # Convert PCM data to numpy array
#             # pcm_data = np.array(packet, dtype=np.int16)
#             pcm_data = np.frombuffer(packet, dtype=np.int16)
#             accumulated_audio_data.append(pcm_data)

#             if len(accumulated_audio_data) > 100:
#                 FLAG = 2

#         elif FLAG == 2:
#             pcm_data = np.concatenate(accumulated_audio_data)

#             # Create an AudioSegment from the PCM data
#             audio_segment = AudioSegment(pcm_data.tobytes(), frame_rate=16000, sample_width=2, channels=1)

#             # Export AudioSegment to an MP3 file
#             audio_segment.export("recording.mp3", format="mp3")

#             FLAG = 0


# while True:
#     try:
#         receive_and_ros()

#     except OSError as e:
#         print("Error: ", e)

#         pass # try again
