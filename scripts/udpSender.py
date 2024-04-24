import time
import socket
import network
import sensor
from machine import LED
from machine import I2C
from vl53l1x import VL53L1X

UDP_IP = "10.24.4.77"
UDP_PORT = 8002
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
while True:
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
