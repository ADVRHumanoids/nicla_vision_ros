import socket
from collections import deque

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
DISTANCE_TYPE = 0b10

class NiclaReceiverUDP:
    def __init__(self, pc_ip, pc_port, packet_size, audio_buffer=1000):
        self.ip = pc_ip
        self.port = pc_port
        self.packet_size = packet_size
        self.audio_buffer = audio_buffer

        #receiving data
        self.distance = 0
        self.image = 0
        self.audio_deque = deque([], maxlen=audio_buffer)

    def connect(self):
        print("Waiting for niclabox to stream on", self.ip, ":", self.port)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.ip, self.port))

        print("Connected to niclabox!")

    def receive(self):

        packet, _ = self.server.recvfrom(self.packet_size)

        data_type = packet[0]
        data = packet[1:]
    
        if data_type == DISTANCE_TYPE:
            self.distance = int.from_bytes(data, "big")

            print("Distance (mm): ", self.distance)           

        elif data_type == IMAGE_TYPE:
        
            # Show image with numpy OpenCV
            self.image = data

        elif data_type == AUDIO_TYPE:

                self.audio_deque.append(data)





