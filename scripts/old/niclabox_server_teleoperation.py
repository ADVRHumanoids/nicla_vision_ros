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
from pydub import AudioSegment

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
DISTANCE_TYPE = 0b10

FLAG = 1
accumulated_audio_data = []


# server address and port (the address of the machine running this code, any available port)
ip = "10.240.23.87"
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
    global FLAG

    packet, client_address = server.recvfrom(packet_size)

    data_type = packet[0]
    packet = packet[1:]

    # if len(packet) < 100: # a small packet is the distance
    if data_type == DISTANCE_TYPE:
        distance = packet
        distance = int.from_bytes(distance, "big")

        # Print distance in terminal
        print("Distance (mm): ", distance)

    elif data_type == IMAGE_TYPE:
        picture = packet

        # Show image with numpy OpenCV
        image = cv2.imdecode(np.frombuffer(picture, np.uint8), cv2.IMREAD_COLOR)
        cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)
        cv2.imshow("niclabox", image)
        if cv2.waitKey(1) == ord("q"):  # Press Q to exit
            exit(0)

        # uncomment to output to a file without using numpy and OpenCV
        # distance_file = open("distance.txt", "w")
        # distance_file.write(str(distance))
        # distance_file.close()
        # picture_file = open("picture.jpg", "wb")
        # picture_file.write(picture)
        # picture_file.close()
    elif data_type == AUDIO_TYPE:

        if FLAG == 1:
            # Convert PCM data to numpy array
            # pcm_data = np.array(packet, dtype=np.int16)
            pcm_data = np.frombuffer(packet, dtype=np.int16)
            accumulated_audio_data.append(pcm_data)

            if len(accumulated_audio_data) > 100:
                FLAG = 2

        elif FLAG == 2:
            pcm_data = np.concatenate(accumulated_audio_data)

            # Create an AudioSegment from the PCM data
            audio_segment = AudioSegment(
                pcm_data.tobytes(), frame_rate=16000, sample_width=2, channels=1
            )

            # Export AudioSegment to an MP3 file
            audio_segment.export("recording.mp3", format="mp3")

            FLAG = 0


while True:
    try:
        receive_and_ros()

    except OSError as e:
        print("Error: ", e)

        pass  # try again
