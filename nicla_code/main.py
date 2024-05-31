# Developed by Davide Torielli and Damiano Gasperini

# Streaming distance and picture from Arduino Nicla Vision (client)
# to a device running Python (server) via UDP or TCP.
# The picture fits into one UDP/TCP packet after compression,
# same for audio, distance and IMU (but not compressed).
# Leds are used for warning in case of errors:
#   blue for network
#   green for unforseen
#   red for picture quality
# Use verbose to display on OpenMV terminal.

import time
import socket
import network
import sensor
from machine import LED
from machine import I2C
from vl53l1x import VL53L1X

import image
import audio
from ulab import numpy as np
from ulab import utils

from lsm6dsox import LSM6DSOX
from machine import Pin
from machine import SPI
import struct

import json


class ValueErrorImage(Exception):
    # Raised when the picture packet is too big (>65000 bytes)
    def __init__(self, message="Error: the (compressed) picture packet is too big (>65000 bytes). Lower the quality!"):
        self.message = message
        print(f"\033[91m{self.message}\033[0m")
        super().__init__(self.message)


class ValueErrorAudio(Exception):
    # Raised when the audio packet is too big (>65000 bytes)
    def __init__(self, message="Error: the audio packet is too big (>65000 bytes). Lower the quality!"):
        self.message = message
        print(f"\033[91m{self.message}\033[0m")
        super().__init__(self.message)

class TCPError(Exception):
    # Raised when the TCP connection is lost
    def __init__(self, message="Error: the TCP connection is lost! Trying to establish the connection again..."):
        self.message = message
        print(f"\033[91m{self.message}\033[0m")
        super().__init__(self.message)

class UDPError(Exception):
    # Raised when the UDP connection is lost
    def __init__(self, message="Error: the UDP connection is lost! Trying to establish the connection again..."):
        self.message = message
        print(f"\033[91m{self.message}\033[0m")
        super().__init__(self.message)


class StreamManager():
    def __init__(self, ssid, pwd, ip, connection_type, verbose):

        # warning settings
        self.verbose = verbose #False
        self.error_timeout = 5 # seconds to display error warning led

        # error handling init
        self.error = False
        self.error_time = 0
        self.error_network = LED("LED_BLUE")
        self.error_unforseen = LED("LED_GREEN")
        self.error_quality = LED("LED_RED")


        # CONNECTION TYPE is
        # 1 for TCP, or
        # 0 for UDP
        self.CONNECTION_TYPE = connection_type

        # wifi ssid and password
        self.ssid = ssid #"DamianoHotspot"
        self.password = pwd #"DamianoHotspot"

        # server address and port
        self.ip = ip #"10.240.23.49"
        self.port = 8002

        # wifi init
        self.wlan = network.WLAN(network.STA_IF)

        ## TCP
        if self.CONNECTION_TYPE:
            print("Establishing TCP connection...")
            self.connect_TCP()
        else:
            ## UDP
            print("Initiating UDP connection...")
            self.connect_UDP()

        # Define data types for headers (1 byte)
        self.IMAGE_TYPE = 0b00
        self.AUDIO_TYPE = 0b01
        self.DISTANCE_TYPE = 0b10
        self.IMU_TYPE = 0b11


        # Defining package dimensions (bytes) utils
        self.int2bytes_size = 4 # size for conversion of distance and timestamp from Int to bytes
        self.packet_size = 65000 # safely less than 65540 bytes that is the maximum for UDP

        self.HEADER_LENGTH = self.int2bytes_size + self.int2bytes_size + len(bytes([self.IMAGE_TYPE])) #bytes (pkg size + timestamp size + data type size)

        self.IMU_SIZE = self.HEADER_LENGTH - self.int2bytes_size + 6*self.int2bytes_size # 6 Floats and each Float is 4 bytes (as the Int)
        self.IMU_SIZE = self.IMU_SIZE.to_bytes(self.int2bytes_size, "big")

        self.DISTANCE_SIZE = self.HEADER_LENGTH - self.int2bytes_size + self.int2bytes_size
        self.DISTANCE_SIZE = self.DISTANCE_SIZE.to_bytes(self.int2bytes_size, "big")

        self.HEADER_SIZE_DIM = self.HEADER_LENGTH - self.int2bytes_size

        # camera
        self.picture_quality = 30 # going higher than 30 creates ENOMEM error (led green)
        sensor.reset()
        sensor.set_framesize(sensor.QVGA)
        sensor.set_pixformat(sensor.RGB565)

        # distance sensor
        self.tof = VL53L1X(I2C(2))

        # microphone
        CHANNELS = 1
        self.mic_buf = []
        self.audio_buf = None
        audio.init(channels=CHANNELS, frequency=16000, gain_db=24, highpass=0.9883)
        audio.start_streaming(self.audio_callback) # Start audio streaming

        # IMU
        self.lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))

    def audio_callback(self, buf):
        self.mic_buf.append(buf)

        if len(self.mic_buf) > 20:
            del self.mic_buf[:10]

    def connect(self):
        if self.verbose == True:
            print("Connecting to wifi network", self.ssid)
        self.wlan.active(False)
        time.sleep(2)
        self.wlan.active(True)
        self.wlan.connect(self.ssid, self.password)
        while not self.wlan.isconnected():
            self.error_network.on()
            time.sleep(1)
        self.error_network.off()
        if self.verbose == True:
            print(self.wlan.ifconfig())

    def connect_TCP(self):
        connected = False
        while not connected:
            try:
                # Create a socket (SOCK_STREAM means a TCP socket)
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Connect to TCP server and send data
                self.client.connect((self.ip, self.port))
                connected = True
            except:
                if self.verbose:
                    print("Waiting for TCP connection to be established...")
                continue

    def connect_UDP(self):
        connected = False
        while not connected:
            try:
                # Create a socket (SOCK_DGRAM means a UDP socket)
                client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                connected = True
            except:
                if self.verbose:
                    print("Waiting for UDP connection to be initiated...")
                continue


    def sense_and_send(self):

        # Directions and signs set by evidence
        acc_y, acc_x, acc_z = self.lsm.accel()  # Accelerometer
        acc_z = -acc_z
        gyro_y, gyro_x, gyro_z = self.lsm.gyro()# Gyroscope
        gyro_z = -gyro_z

        # Converting IMU values to bytes
        imu_packet = struct.pack('>ffffff', acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

        # sensing, first distance and then camera that takes time to save in memory
        distance = self.tof.read() # class int
        picture = sensor.snapshot() # class Image, bytes not readable

        # printing sensors output
        if self.verbose == True:
            print("Distance:", distance)
            print("Picture:")
            print(picture)

        # converting with known size the distance
        distance = distance.to_bytes(self.int2bytes_size, "big")

        # compressing/converting and sizing the picture
        picture.compress(self.picture_quality) # class Image, bytes readable as jpeg
        picture_size = len(picture) # dimension of the compressed picture

        # audio
        self.audio_buf = bytearray([])
        if len(self.mic_buf)>1:
            self.audio_buf += self.mic_buf.pop(0)
            self.audio_buf += self.mic_buf.pop(0)
        elif len(self.mic_buf)>0:
            self.audio_buf += self.mic_buf.pop(0)

        # printing transmission info and data to transmit
        if self.verbose == True:
            print("Picture quality:", self.picture_quality)
            print("Picture size (bytes):", picture_size,  "must be less than", self.packet_size, " bytes")
            print("Picture:")
            print(picture)
            print("Distance size:", self.int2bytes_size, "set by the user")
            print("Distance:", distance)

            print("Mic Buffer Length (audio packets 0.032ms of 1024 bytes each): ", len(self.mic_buf))
            print("Mic Buffer Size (bytes): ", len(self.mic_buf)*1024)
            print("Audio Buffer Size (bytes): ", len(self.audio_buf))

        timestamp = time.ticks_ms()
        #print("TIMESTAMP: ", timestamp, "PACKET: ", imu_packet, "LENGTH: ", len(imu_packet))
        timestamp = timestamp.to_bytes(self.int2bytes_size, "big")


        # picture packet too big, skip transmission
        if (picture_size + self.HEADER_LENGTH > self.packet_size):
            raise ValueErrorImage
        else:
            pkg_size = self.HEADER_SIZE_DIM + picture_size
            pkg_size = pkg_size.to_bytes(self.int2bytes_size, "big")
            try:
                self.client.sendto( pkg_size + timestamp + bytes([self.IMAGE_TYPE]) + picture, (self.ip, self.port))
                #client.sendall( pkg_size + timestamp + bytes([IMAGE_TYPE]) + picture)
            except:
                if self.CONNECTION_TYPE:
                    raise TCPError
                else:
                    raise UDPError
                pass

        # audio packet too big, skip transmission
        if (len(self.audio_buf) + self.HEADER_LENGTH > self.packet_size):
            raise ValueErrorAudio
        else:
            pkg_size = self.HEADER_SIZE_DIM + len(self.audio_buf)
            pkg_size = pkg_size.to_bytes(self.int2bytes_size, "big")
            try:
                self.client.sendto( pkg_size + timestamp + bytes([self.AUDIO_TYPE]) + self.audio_buf, (self.ip, self.port))
                #client.sendall( pkg_size + timestamp + bytes([AUDIO_TYPE]) + audio_buf)
            except:
                if self.CONNECTION_TYPE:
                    raise TCPError
                else:
                    raise UDPError
                pass

        try:
            self.client.sendto( self.DISTANCE_SIZE + timestamp + bytes([self.DISTANCE_TYPE]) + distance, (self.ip, self.port))
            #client.sendall( DISTANCE_SIZE + timestamp + bytes([DISTANCE_TYPE]) + distance)
        except:
            if self.CONNECTION_TYPE:
                raise TCPError
            else:
                raise UDPError
            pass

        try:
            self.client.sendto( self.IMU_SIZE + timestamp + bytes([self.IMU_TYPE]) + imu_packet, (self.ip, self.port))
            #client.sendall( IMU_SIZE + timestamp + bytes([IMU_TYPE]) + imu_packet)
        except:
            if self.CONNECTION_TYPE:
                raise TCPError
            else:
                raise UDPError
            pass

        ## DEBUG PACKET
        #debug_packet = bytearray([IMAGE_TYPE, AUDIO_TYPE, DISTANCE_TYPE, IMU_TYPE, IMU_TYPE, IMU_TYPE])
        #client.sendto( IMU_SIZE + timestamp + bytes([IMU_TYPE]) + debug_packet, (ip, port))

        if self.verbose == True:
            print("Transmission completed")

    def run(self):

        while True:
            try:
                if not self.wlan.isconnected():
                    self.connect()

                # error warning reset
                if self.error == True:
                    print("\033[91mError in the last", self.error_timeout, "seconds\033[0m")
                    if time.time() - self.error_time > self.error_timeout:
                        self.error_time = 0
                        self.error_network.off()
                        self.error_quality.off()
                        self.error_unforseen.off()
                        self.error = False

                self.sense_and_send()

            except (ValueErrorImage, ValueErrorAudio) as e:
                self.error = True
                self.error_time = time.time()
                self.error_quality.on()
                pass


            except TCPError as e:
                self.error = True
                self.error_time = time.time()
                self.error_network.on()
                self.connect_TCP()
                pass

            except UDPError as e:
                self.error = True
                self.error_time = time.time()
                self.error_network.on()
                self.connect_UDP()
                pass


            except OSError as e: # unforseen error, debug with OpenMV
                if self.verbose == True:
                    print("\033[91mError: ", e, "\033[0m")
                self.error = True
                self.error_time = time.time()
                self.error_unforseen.on()
                pass


def main():

    try:
#        f = open('./config.json', 'w')
#        print(dir(f))
#        print(f.readline(0))
#        print("AAAAAAAAAAAAAA")
        params = None
        with open('config.json', 'r') as f:
            data = f.read()
            params = json.loads(data)

        if params == None:
            print("No parameters in the config file")
            raise Exception
        ssid = params["ssid"]
        pwd = params["password"]
        ip = params["ip"]
        connection_type = params["connection_type"]
        verbose = params["verbose"]

        manager = StreamManager(ssid, pwd, ip, connection_type, verbose)

        manager.connect()

        manager.run()


    except Exception as e:
        print("Exception caught in main: ", e)
        pass


    # Stop audio streaming
    audio.stop_streaming()

if __name__ == "__main__":
    main()




