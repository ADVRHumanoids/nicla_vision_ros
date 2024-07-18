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
        
class TCPHandler(socketserver.BaseRequestHandler):
    def handle(self): 
        # self.request.settimeout(15.0)  # Set a timeout  
        self.server.nicla_disconnect = False

        while True: 
            try:
                packet = self.request.recv(65000)
                print("LEN RECV: ", len(packet))
                # print("IN RECV: ", np.frombuffer(packet, dtype="int16"))

                self.server.pdm_buffer.put_nowait(packet)
                

            #     if not packet:
            #         break 
            #     timestamp = time.time()
            #     timestamp = struct.pack('>d', timestamp) 
            #     packet = timestamp + packet 
            #     self.server.receiving_buffer.put_nowait(packet) 
            # except socket.timeout:
            #     print("Warning: Nicla disconnected! Resetting server... ")
            #     self.server.receiving_buffer.queue.clear()
            #     self.server.nicla_disconnect = True
            #     break 
            except Exception as e:
                print(f"Exception: {e}")
                self.server.receiving_buffer.queue.clear()
                self.server.nicla_disconnect = True
                break  # Break the loop on any other exception 

            
class NiclaReceiverTCP(socketserver.TCPServer):
    
    def __init__(self, server_ip, server_port, enable_range=False, enable_image=False, enable_audio=False, enable_imu=False):
         
        super().__init__((server_ip, server_port), TCPHandler)

        self.enable_range = enable_range
        self.enable_image = enable_image
        self.enable_audio = enable_audio
        self.enable_imu = enable_imu

        if self.enable_range:
            self.range_buffer = queue.Queue(maxsize=100)
        if self.enable_image:
            self.image_buffer = queue.Queue(maxsize=100)
        if self.enable_audio:
            self.audio_buffer = queue.Queue(maxsize=100)
        if self.enable_imu:
            self.imu_buffer = queue.Queue(maxsize=100)
        
        if self.enable_range or self.enable_image or self.enable_audio or self.enable_imu:
            self.receiving_buffer = queue.Queue(maxsize=200)

        self.server_thread = None   
        self.nicla_disconnect = False  

        # Set up the plot
        self.max_points = 200
        self.numbers = []


        self.pdm_buffer = queue.Queue(maxsize=60000)

        self.pdm_plotter = Thread(target=self.plotting)
        self.pdm_plotter.daemon = True
        self.pdm_plotter.start()

          
    def debug_recv(self):
        first = True 
        size = 0

        bytes_arr = bytes([])

        cond = True 
        while cond:
            try:  
                self.bytes_packets = self.pdm_buffer.get_nowait()

                if first:
                    first = False
                    # print(self.bytes_packets[0])
                    # print(self.bytes_packets[1])
                    size = int.from_bytes(self.bytes_packets[:4], "little")
                    self.bytes_packets = self.bytes_packets[4:]

                    print("SIZE: ", size)
 
                bytes_arr = bytes_arr + self.bytes_packets

                print("BYTES LEN: ", len(bytes_arr))

                if len(bytes_arr)>= size:
                    print("DENTRO")
                    bytes_arr = bytes_arr[:size] #153600


                    with open("my_file.txt", "wb") as binary_file:
   
                        # Write bytes to file
                        binary_file.write(bytes_arr)


                    # Convert Python list to NumPy array
                    # nparr = np.array(self.numbers, dtype="uint8")

                    # Convert byte data to a numpy array
                    # raw_image = np.frombuffer(bytes_arr, dtype=np.uint8)#.reshape((240, 320))
 
 
                    # image_rgb888 = cv2.imdecode(raw_image, cv2.IMREAD_COLOR)

                     

                    # # Display the image using OpenCV
                    # cv2.imshow('Image', rgb_image)
                    # cv2.waitKey(0)
                    # cv2.destroyAllWindows()
 
                    # Decode the compressed image
                    # img_raw = cv2.imdecode(nparr, cv2.IMREAD_COLOR) #NOTE: BGR CONVENTION 

                    #To visualize the received img converted in img_raw 
                    # cv2.imshow("Visualize img_raw", img_raw)
                    # cv2.waitKey(1) 


 
                    # print("ONE ARR: \n")
                    # print(self.numbers[:4000])
                    # print("\n")

                    # print("TWO ARR: \n")
                    # print(self.numbers[4000:8000])
                    # print("\n")

                    # print("THREE ARR: \n")
                    # print(self.numbers[8000:12000])
                    # print("\n")

                    # print("FOUR ARR: \n")
                    # print(self.numbers[12000:16000])
                    # print("\n")

                    cond = False


            except:
                continue

    def plotting(self):          

        while True:
            try:  
                self.bytes_packets = self.pdm_buffer.get_nowait()
                # print("IN THREAD: ", np.frombuffer(self.bytes_packets, dtype="int16"))
                print("LEN CAL: ", len(self.bytes_packets))
                
 
                new = np.frombuffer(self.bytes_packets, dtype="int16")
                new = new.tolist()
                self.numbers.extend(new) 
                # if len(self.numbers) > self.max_points:
                #     # Plotting the data
                #     plt.figure(figsize=(50, 5))
                #     plt.plot(self.numbers, marker='o', linestyle='-', color='b')

                #     # Adding titles and labels
                #     plt.title('Simple Plot of Data from a List')
                #     plt.xlabel('Index')
                #     plt.ylabel('Value')

                #     # Display the plot
                #     plt.grid(True)
                #     plt.show()
                #     self.numbers = []

                print("LEN: ", len(self.numbers))

                if len(self.numbers)>= 10000*5:
                    # pcm_data = np.cumsum(self.numbers, dtype=np.int32)
                    # pcm_data = np.array(pcm_data, dtype=np.int16) >> 16

                    self.numbers = np.array(self.numbers, dtype="int16")

                    with wave.open("audio.wav", 'w') as wf:
                        wf.setnchannels(1)  # Mono
                        wf.setsampwidth(2)  # 2 bytes (16 bits)
                        wf.setframerate(10000)
                        wf.writeframes(self.numbers.tobytes())

                        print("recorded")

                    break

            except:
                continue 

    def serve(self):
        self.thread_regularizer = True
        self.sorting_thread = Thread(target=self.sorting)
        self.server_thread = Thread(target=self.serve_forever)
        self.sorting_thread.start()
        self.server_thread.start()

    def sorting(self):

        bkp_bytes_packets = bytes([])
        timestamp = None

        while self.thread_regularizer:

            try:  
                bytes_packets = self.receiving_buffer.get_nowait()
                timestamp = struct.unpack('>d', bytes_packets[:8])[0]
                bytes_packets = bytes_packets[8:]

                if self.nicla_disconnect:
                    bytes_packets = bytes([])
                    bkp_bytes_packets = bytes([])
            except:
                continue

            bytes_packets = bkp_bytes_packets + bytes_packets
            bkp_bytes_packets = bytes([])
             
            if len(bytes_packets) < 9:
                print("Got a packet from receiver less than header size!")
                continue
            else:
                total_length = len(bytes_packets)                 
                loop_termination_flag = True
                
                while loop_termination_flag:
                    size_packet = int.from_bytes(bytes_packets[:4], "big")

                    if total_length - 4 >= size_packet:
                        packet = bytes_packets[4:size_packet+4]
                        bytes_packets = bytes_packets[size_packet+4:]

                        #timestamp = int.from_bytes(packet[:4], "big")
                        
                        data_type = packet[4]
                        data = packet[5:]
 
                        if data_type == RANGE_TYPE:
                            if self.enable_range:
                                self.range_buffer.put_nowait((timestamp, data))
                            else:
                                pass

                        elif data_type == IMAGE_TYPE:
                            if self.enable_image:
                                self.image_buffer.put_nowait((timestamp, data))
                            else:
                                pass

                        elif data_type == AUDIO_TYPE:
                            if self.enable_audio:
                                self.audio_buffer.put_nowait((timestamp, data))
                            else:
                                pass

                        elif data_type == IMU_TYPE:
                            if self.enable_imu:
                                self.imu_buffer.put_nowait((timestamp, data))
                            else:
                                pass
                            
                    else: 
                        bkp_bytes_packets = bytes_packets 
                        loop_termination_flag = False

                    total_length = len(bytes_packets) 
                    if total_length == 0: 
                        loop_termination_flag = False

                


    def stop_serve(self):
        print("stopping")
        self.thread_regularizer = False
        self.sorting_thread.join()
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
 
    nicla_receiver_server = NiclaReceiverTCP("10.240.23.49", 8002, 
                                            enable_range=False, 
                                            enable_image=False,
                                            enable_audio=True,
                                            enable_imu=False)
   
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
