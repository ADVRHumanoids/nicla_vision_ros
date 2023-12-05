# BSD 3-Clause License

# Copyright (c) 2023, Edoardo Del Bianco, Federico Rollo

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



# receiving distance and picture from Arduino Nicla Vision (client)
# and showing them using the terminal, numpy and OpenCV
# the picture fits into one UDP packet after compression

import time
import socket
import numpy as np
import cv2


# server address and port (the address of the machine running this code, any available port)
ip = "YourServerIP"
port = 8002

# set maximum packet size
packet_size = 65540

# image window init
cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)

# server socket init 
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind((ip, port))
print("Waiting for niclabox to stream on", ip, ":", port)

def receive_and_ros():
    packet, client_address = server.recvfrom(packet_size)

    if len(packet) < 100: # a small packet is the distance
        distance = packet
        distance = int.from_bytes(distance, "big")

        # Print distance in terminal
        print("Distance (mm): ", distance)           

    else:
        picture = packet
       
        # Show image with numpy OpenCV
        image = cv2.imdecode(np.frombuffer(picture, np.uint8), cv2.IMREAD_COLOR)
        cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)
        cv2.imshow("niclabox", image)
        if cv2.waitKey(1) == ord('q'): # Press Q to exit
            exit(0)

        # uncomment to output to a file without using numpy and OpenCV
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

