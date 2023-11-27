# edb 20231122
# streaming distance and picture from Arduino Nicla Vision (client)
# to a device running Python (server) via UDP
# the picture fits into one UDP packet after compression
# leds are used for warning in case of errors:
#   blue for network
#   green for unforseen
#   red for picture quality

import time
import socket
import network
import sensor
from machine import LED
from machine import I2C
from vl53l1x import VL53L1X


# wifi ssid and password
ssid = "edb"
password = "pitagora"
# ssid = "TIM-36282322"
# password = "7kDqVHcfyeypvcQT"

# server address and port
ip = "192.168.61.112"
port = 8000

# sensing settings
picture_quality = 50

### warning settings
verbose = True
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

# wifi init
wlan = network.WLAN(network.STA_IF)

# transmission init
distance_size = 4 # size for conversion of distance from Int to bytes
packet_size = 65000 # safely less than 65540 that is the maximum for UDP
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


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

def sense_and_send():

    # sensing, first distance and then camera that takes time to save in memory
    distance = tof.read() # class int
    picture = sensor.snapshot() # class Image, bytes not readable

    # printing sensors output
    if verbose == True:
        print("Distance:", distance)
        print("Picture:")
        print(picture)

    # converting with known size the distance
    distance = distance.to_bytes(distance_size, "big")

    # compressing/converting and sizing the picture
    picture.compress(picture_quality) # class Image, bytes readable as jpeg
    picture_size = len(picture) # dimension of the compressed picture

    # printing transmission info and data to transmit
    if verbose == True:
        print("Distance size:", distance_size, "set by the user")
        print("Picture quality:", picture_quality)
        print("Picture size:", picture_size,  "must be less than", packet_size)
        print("Distance:", distance)
        print("Picture:")
        print(picture)

    if picture_size > packet_size: # picture too big, skip transmission
        raise ValueError
    else:
        client.sendto(distance, (ip, port))
        client.sendto(picture, (ip, port))
        if verbose == True:
            print("Transmission completed")

connect()

while True:
    try:
        if not wlan.isconnected():
            connect()

        # error warning reset
        if error == True:
            if verbose == True:
                print("\033[91mError in the last", error_timeout, "seconds\033[0m")
            if time.time() - error_time > error_timeout:
                error_time = 0
                error_quality.off()
                error_unforseen.off()
                error = False

        sense_and_send()

    except ValueError as e:
        if verbose == True:
            print("\033[91mError: the compressed picture is too big. Lower the quality!\033[0m")
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
