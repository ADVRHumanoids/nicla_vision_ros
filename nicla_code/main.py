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

# Define data types
IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
DISTANCE_TYPE = 0b10
IMU_TYPE = 0b11


# wifi ssid and password
ssid = "DamianoHotspot"
password = "DamianoHotspot"


# server address and port
ip = "10.240.23.x"
port = 8002

# sensing settings
picture_quality = 30 # going higher than 30 creates ENOMEM error (led green)

# warning settings
verbose = False
error_timeout = 5 # seconds to display error warning led

# error handeling init
error = False
error_time = 0
error_network = LED("LED_BLUE")
error_unforseen = LED("LED_GREEN")
error_quality = LED("LED_RED")

# camera init
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)

# distance sensor init
tof = VL53L1X(I2C(2))

# microphone
CHANNELS = 1
mic_buf = []
audio_buf = None
audio.init(channels=CHANNELS, frequency=16000, gain_db=24, highpass=0.9883)


def audio_callback(buf):
    # NOTE: do Not call any function that allocates memory.
    mic_buf.append(buf)

    if len(mic_buf) > 20:
        del mic_buf[:10]

# IMU
lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))

# wifi init
wlan = network.WLAN(network.STA_IF)

# transmission init
int2bytes_size = 4 # size for conversion of distance and timestamp from Int to bytes
packet_size = 65000 # safely less than 65540 bytes that is the maximum for UDP

HEADER_LENGTH = int2bytes_size + int2bytes_size +1 #bytes (pkg size + timestamp size + data type size)

IMU_SIZE = HEADER_LENGTH - int2bytes_size + 24
IMU_SIZE = IMU_SIZE.to_bytes(int2bytes_size, "big")

DISTANCE_SIZE = HEADER_LENGTH - int2bytes_size + int2bytes_size
DISTANCE_SIZE = DISTANCE_SIZE.to_bytes(int2bytes_size, "big")

HEADER_SIZE_DIM = HEADER_LENGTH - int2bytes_size

## UDP
#client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # SOCK_STREAM SOCK_DGRAM

## TCP
# Create a socket (SOCK_STREAM means a TCP socket)
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect to server and send data
client.connect((ip, port))

def connect():
    if verbose == True:
        print("Connecting to wifi network", ssid)
    wlan.active(False)
    time.sleep(2)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        error_network.on()
        time.sleep(1)
    error_network.off()
    if verbose == True:
        print(wlan.ifconfig())

    # TODO: put client.connect of TCP here or in other function

    # Start audio streaming after connection established
    audio.start_streaming(audio_callback)

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


def sense_and_send():
    global audio_buf
    global mic_buf
    global client
    global IMU_SIZE, DISTANCE_SIZE, HEADER_SIZE_DIM, HEADER_LENGTH


    # Directions and signs set by evidence
    acc_y, acc_x, acc_z = lsm.accel()  # Accelerometer
    acc_z = -acc_z
    gyro_y, gyro_x, gyro_z = lsm.gyro()# Gyroscope
    gyro_z = -gyro_z

    # Converting IMU values to bytes
    imu_packet = struct.pack('>ffffff', acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    # sensing, first distance and then camera that takes time to save in memory
    distance = tof.read() # class int
    picture = sensor.snapshot() # class Image, bytes not readable

    # printing sensors output
    if verbose == True:
        print("Distance:", distance)
        print("Picture:")
        print(picture)

    # converting with known size the distance
    distance = distance.to_bytes(int2bytes_size, "big")

    # compressing/converting and sizing the picture
    picture.compress(picture_quality) # class Image, bytes readable as jpeg
    picture_size = len(picture) # dimension of the compressed picture

    # audio
    audio_buf = bytearray([])
    if len(mic_buf)>1:
        audio_buf += mic_buf.pop(0)
        audio_buf += mic_buf.pop(0)
    elif len(mic_buf)>0:
        audio_buf += mic_buf.pop(0)

    # printing transmission info and data to transmit
    if verbose == True:
        print("Picture quality:", picture_quality)
        print("Picture size (bytes):", picture_size,  "must be less than", packet_size, " bytes")
        print("Picture:")
        print(picture)
        print("Distance size:", int2bytes_size, "set by the user")
        print("Distance:", distance)

        print("Mic Buffer Length (audio packets 0.032ms of 1024 bytes each): ", len(mic_buf))
        print("Mic Buffer Size (bytes): ", len(mic_buf)*1024)
        print("Audio Buffer Size (bytes): ", len(audio_buf))

    timestamp = time.ticks_ms()
    #print("TIMESTAMP: ", timestamp, "PACKET: ", imu_packet, "LENGTH: ", len(imu_packet))
    timestamp = timestamp.to_bytes(int2bytes_size, "big")


    # picture packet too big, skip transmission
    if (picture_size + HEADER_LENGTH > packet_size):
        raise ValueErrorImage
    else:
        pkg_size = HEADER_SIZE_DIM + picture_size
        pkg_size = pkg_size.to_bytes(int2bytes_size, "big")
        #client.sendto( pkg_size + timestamp + bytes([IMAGE_TYPE]) + picture, (ip, port))
        client.sendall( pkg_size + timestamp + bytes([IMAGE_TYPE]) + picture)

    # audio packet too big, skip transmission
    if (len(audio_buf) + HEADER_LENGTH > packet_size):
        raise ValueErrorAudio
    else:
        pkg_size = HEADER_SIZE_DIM + len(audio_buf)
        pkg_size = pkg_size.to_bytes(int2bytes_size, "big")
        #client.sendto( pkg_size + timestamp + bytes([AUDIO_TYPE]) + audio_buf, (ip, port))
        client.sendall( pkg_size + timestamp + bytes([AUDIO_TYPE]) + audio_buf)

    #client.sendto( DISTANCE_SIZE + timestamp + bytes([DISTANCE_TYPE]) + distance, (ip, port))
    client.sendall( DISTANCE_SIZE + timestamp + bytes([DISTANCE_TYPE]) + distance)

    #client.sendto( IMU_SIZE + timestamp + bytes([IMU_TYPE]) + imu_packet, (ip, port))
    client.sendall( IMU_SIZE + timestamp + bytes([IMU_TYPE]) + imu_packet)

    ## DEBUG PACKET
    #debug_packet = bytearray([IMAGE_TYPE, AUDIO_TYPE, DISTANCE_TYPE, IMU_TYPE, IMU_TYPE, IMU_TYPE])
    #client.sendto( IMU_SIZE + timestamp + bytes([IMU_TYPE]) + debug_packet, (ip, port))

    if verbose == True:
        print("Transmission completed")

connect()


while True:
    try:
        if not wlan.isconnected():
            connect()

        # error warning reset
        if error == True:
            #if verbose == True:
            print("\033[91mError in the last", error_timeout, "seconds\033[0m")
            if time.time() - error_time > error_timeout:
                error_time = 0
                error_quality.off()
                error_unforseen.off()
                error = False

        sense_and_send()

    except (ValueErrorImage, ValueErrorAudio) as e:
        error = True
        error_time = time.time()
        error_quality.on()
        pass


    except OSError as e: # unforseen error, debug with OpenMV
        if verbose == True:
            print("\033[91mError: ", e, "\033[0m")
        error = True
        error_time = time.time()
        error_unforseen.on()
        pass


# Stop audio streaming
audio.stop_streaming()
