import socket
import queue
import socketserver

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
DISTANCE_TYPE = 0b10

class NiclaReceiverUDP:
    def __init__(self, pc_ip, pc_port, packet_size, audio_buffer_size=1000):
        self.ip = pc_ip
        self.port = pc_port
        self.packet_size = packet_size

        #receiving data
        self.distance = bytes()
        self.image = bytes()
        self.audio_buffer = queue.Queue(maxsize=audio_buffer_size)
        self.imu = bytes()

    def connect(self):
        print("Waiting for niclabox to stream on", self.ip, ":", self.port)

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.ip, self.port))

        print("Connected to niclabox!")

    def receive(self):

        packet, _ = self.server.recvfrom(self.packet_size)
        
        data_type = packet[0]
        data = packet[1:]
    
        if data_type == DISTANCE_TYPE:
            self.distance = data

        elif data_type == IMAGE_TYPE:
        
            # Show image with numpy OpenCV
            self.image = data

        elif data_type == AUDIO_TYPE:

            self.audio_deque.append(data)


#Socket server version

class MyUDPHandler:
    def handle(self):

        packet = self.request[0].strip()
        #socket = self.request[1]
        
        data_type = packet[0]
        data = packet[1:]
    
        if data_type == DISTANCE_TYPE:
            self.distance = data

        elif data_type == IMAGE_TYPE:
            # Show image with numpy OpenCV
            self.image = data

        elif data_type == AUDIO_TYPE:
            self.audio_deque.append(data)
class NiclaReceiverUDP2:
    def __init__(self, pc_ip, pc_port, packet_size, audio_buffer_size=1000):
        self.ip = pc_ip
        self.port = pc_port
        self.packet_size = packet_size

        #receiving data
        self.distance = bytes()
        self.image = bytes()
        self.audio_buffer = queue.Queue(maxsize=audio_buffer_size)
        self.imu = bytes()

    def connect_and_serve(self):
        print("Waiting for niclabox to stream on", self.ip, ":", self.port)

        self.server = socketserver.UDPServer((self.ip, self.port), MyUDPHandler)
        print("Connected to niclabox!")

        self.server_thread = self.server.serve_forever()
        self.server_thread.start()

    def stop_serve(self):
        self.server.server_close()

        






