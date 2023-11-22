# edb 20231116
# receiving distance and picture from Arduino Nicla Vision (client)
# and sending them on two ROS messages


import time
import socket
import numpy as np
import cv2


# server address and port (the address of the machine running this code)
IP_SERVER = "192.168.2.112"
PORT_SERVER = 8000 # must be the same on client

# set maximum packet size
packet_size = 65540

# server socket init 
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind((IP_SERVER, PORT_SERVER))
print("Socket, created, waiting for transmission...")

def receive_and_ros():
    print("trying to receive")
    # receiving packet
    packet, client_address = server.recvfrom(packet_size)
    print("received packet")

    #if packet is small, it is the distance
    if len(packet) < 100:
        distance_bytes = packet
        distance = int.from_bytes(distance_bytes, "big")

        # receiving next packet that should be the picture
        picture, address = server.recvfrom(packet_size)

        # checking if it is as big es expected
        if len(picture) < 100:
            print("Picture not received!")

        else:
            # printing
            print("Disance (mm): ", distance)
            print("Picture: ")
            #print(picture)
            # image = np.array(picture)
            # print(image)
            image = cv2.imdecode(np.frombuffer(picture, np.uint8), cv2.IMREAD_COLOR)

            cv2.namedWindow("dfsdf", cv2.WINDOW_NORMAL)
            cv2.imshow("dfsdf", image)
            if cv2.waitKey(1) == ord('q'):
                exit(0)

            # saving pic
            # binary_file = open("picture.jpg", "wb")
            # binary_file.write(picture)
            # binary_file.close()

    
while True:
    try:
        receive_and_ros()
    except OSError as e:
        print("error!")
        pass

