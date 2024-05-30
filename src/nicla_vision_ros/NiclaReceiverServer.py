#!/usr/bin/env python

import queue
import socketserver
from threading import Thread

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
RANGE_TYPE = 0b10
IMU_TYPE = 0b11


class UDPHandler(socketserver.BaseRequestHandler):
    def handle(self):

        #with udp, self.request is a pair (data, socket)
        packet = self.request[0]
        #socket = self.request[1]
        
        timestamp = int.from_bytes(packet[:4], "big")
        data_type = packet[4]
        data = packet[5:]
        if data_type == RANGE_TYPE:
            if self.server.enable_range:
                self.server.range_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == IMAGE_TYPE:
            if self.server.enable_image:
                self.server.image_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == AUDIO_TYPE:
            if self.server.enable_audio:
                self.server.audio_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == IMU_TYPE:
            if self.server.enable_imu:
                self.server.imu_buffer.put_nowait((timestamp, data))
            else:
                pass

class NiclaReceiverUDP(socketserver.UDPServer):

    def __init__(self, server_ip, server_port, enable_range=False, enable_image=False, enable_audio=False, enable_imu=False):

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
        
        
class TCPHandler(socketserver.BaseRequestHandler):
    def handle(self):

        #with tcp, self.request is the data
        packet = self.request
        
        timestamp = int.from_bytes(packet[:4], "big")
        data_type = packet[4]
        data = packet[5:]
        if data_type == RANGE_TYPE:
            if self.server.enable_range:
                self.server.range_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == IMAGE_TYPE:
            if self.server.enable_image:
                self.server.image_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == AUDIO_TYPE:
            if self.server.enable_audio:
                self.server.audio_buffer.put_nowait((timestamp, data))
            else:
                pass

        elif data_type == IMU_TYPE:
            if self.server.enable_imu:
                self.server.imu_buffer.put_nowait((timestamp, data))
            else:
                pass

class NiclaReceiverTCP(socketserver.TCPServer):

    def __init__(self, server_ip, server_port, enable_range=False, enable_image=False, enable_audio=False, enable_imu=False):

        super().__init__((server_ip, server_port), TCPHandler)

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




        






