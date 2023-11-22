# edb 20231116
# streaming distance and picture from Arduino Nicla Vision (client)
# to a device running Python (server)

import time
import socket
import network
import sensor
from machine import LED
from machine import I2C
from vl53l1x import VL53L1X

# wifi ssid and password, server address and port
SSID = "edb"
PASSWORD = "pitagora"
# SSID = "TIM-36282322"
# PASSWORD = "7kDqVHcfyeypvcQT"

IP_SERVER = "192.168.2.112"
PORT_SERVER = 8000 # must be the same on server

# set picture quality
QUALITY = 50

# set maximum packet size
packet_size = 65000

# led init
blue = LED("LED_BLUE")
green = LED("LED_GREEN")
red = LED("LED_RED")

# camera init
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)

# distance sensor init
tof = VL53L1X(I2C(2))

# connecting to wifi,
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
while not wlan.isconnected():
    blue.on() # led blue while trying to connect
    time.sleep_ms(500)
blue.off()
print(wlan.ifconfig())

# client socket init
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def sense_and_send():
    # sensing, first distance and then camera that takes time to save in memory
    distance_int = tof.read() # class int
    picture = sensor.snapshot() # class Image, bytes not readable

    # printing sensors output
    # print("Distance: ", distance_int)
    # print("Picture: ")
    # print(picture)

    # converting numeric info into bites
    distance = distance_int.to_bytes(4, "big")

    # compressing and sizing the picture
    picture.compress(quality=QUALITY) # class Image, bytes readable as jpeg
    picture_size = len(picture)
    
    # if the picture is too big, we don't transmit
    if picture_size > packet_size:
        red.on() # led red if picture is too big
    else:
        red.off()

        # transmitting
        client.sendto(distance, (IP_SERVER, PORT_SERVER))
        # print("distance transmitted")
        client.sendto(picture, (IP_SERVER, PORT_SERVER))
        # print("picture transmitted")

while True:
    try:
        sense_and_send()
    except OSError as e:
        print("Error: ", e)
        green.on() # led green if there has been an error
        pass




