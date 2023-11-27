# edb 20231122
# receiving distance and picture from Arduino Nicla Vision (client)
# and sending them on two ROS messages
# the picture fits into one UDP packet after compression

import time
import socket
import numpy as np
import cv2


# server address and port (the address of the machine running this code, any available port)
ip = "192.168.61.112"
port = 8000

# set maximum packet size
packet_size = 65540

# server socket init 
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind((ip, port))
print("Waiting for niclabox to stream on", ip, ":", port)

def receive_and_ros():
    packet, client_address = server.recvfrom(packet_size)

    if len(packet) < 100: # a small packet is the distance
        distance = packet
        distance = int.from_bytes(distance, "big")
        
        print("Distance (mm): ", distance)           

    else:
        picture = packet
       
        image = cv2.imdecode(np.frombuffer(picture, np.uint8), cv2.IMREAD_COLOR)
        cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)
        cv2.imshow("niclabox", image)
        if cv2.waitKey(1) == ord('q'):
            exit(0)

        # file output
        # distance_file = open("distance.txt", "w")
        # distance_file.write(str(distance))
        # distance_file.close()
        # picture_file = open("picture.jpg", "wb")
        # picture_file.write(picture)
        # picture_file.close()

    
while True:
    try:
        receive_and_ros()

    except OSError as e:
        print("Error: ", e)

        pass # try again

