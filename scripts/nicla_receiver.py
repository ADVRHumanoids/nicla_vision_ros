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

import socket
import numpy as np
import cv2
from pydub import AudioSegment
import argparse

import NiclaReceiverUDP

parser = argparse.ArgumentParser(description='Read nicla data and show accordingly to the data type (no ros version)')
parser.add_argument('--ip', type=str, required=True)
parser.add_argument('--port', type=int, default=8002)
parser.add_argument('--packet_size', type=int, default=65540)
parser.add_argument('--audio_buffer', type=int, default=1000)
args = parser.parse_args()

FLAG = 1
accumulated_audio_data = []

# image window init
cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)

nicla_receiver_udp = NiclaReceiverUDP.NiclaReceiverUDP(args.ip, args.port, args.packet_size, args.audio_buffer)


def receive():
    global FLAG 
 
    # Print distance
    print("Distance (mm): ", nicla_receiver_udp.distance)           

    # Show image with numpy OpenCV
    image = cv2.imdecode(np.frombuffer(nicla_receiver_udp.image, np.uint8), cv2.IMREAD_COLOR)
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

    # Audio

    if nicla_receiver_udp.audio_deque.size() > args.audio_buffer:

        nicla_receiver_udp.audio_deque 

        accumulated_audio_data = [nicla_receiver_udp.audio_deque.popleft() for _ in xrange(nicla_receiver_udp.size)]

        pcm_data = np.concatenate(accumulated_audio_data)

        # Create an AudioSegment from the PCM data
        audio_segment = AudioSegment(pcm_data.tobytes(), frame_rate=16000, sample_width=2, channels=1)

        # Export AudioSegment to an MP3 file
        audio_segment.export("recording.mp3", format="mp3")

        FLAG = 0





if __name__ == '__main__':

  
    while True:
        try:
            receive()

        except OSError as e:
            print("Error: ", e)
            pass # try again

        except KeyboardInterrupt:
            print('Closing!')
