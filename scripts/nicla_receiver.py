# BSD 3-Clause License
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
parser.add_argument('--audio_buffer', type=int, default=100)
args = parser.parse_args()

FLAG = 1
accumulated_audio_data = []

# image window init
cv2.namedWindow("niclabox", cv2.WINDOW_NORMAL)

nicla_receiver_udp = NiclaReceiverUDP.NiclaReceiverUDP(args.ip, args.port, args.packet_size, args.audio_buffer)


def run():
    global FLAG 
    
    nicla_receiver_udp.receive()
 
    # Print distance
    print("Distance (mm): ", nicla_receiver_udp.distance)           

    if nicla_receiver_udp.image :
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
    if len(nicla_receiver_udp.audio_deque) >= args.audio_buffer:
        
        print("saving recordings!")


        audio_data = [nicla_receiver_udp.audio_deque.popleft() for _ in range(len(nicla_receiver_udp.audio_deque))]
        
        
        accumulated_audio_data = []
        for i in audio_data:
            accumulated_audio_data.append(np.frombuffer(i, dtype=np.int16))
        
        pcm_data = np.concatenate(accumulated_audio_data)

        # Create an AudioSegment from the PCM data
        audio_segment = AudioSegment(pcm_data.tobytes(), frame_rate=16000, sample_width=2, channels=1)

        # Export AudioSegment to an MP3 file
        audio_segment.export("recording.mp3", format="mp3")

        FLAG = 0





if __name__ == '__main__':

  
    nicla_receiver_udp.connect()
  
    while True:
        try:
            run()

        except OSError as e:
            print("Error: ", e)
            pass # try again

        except KeyboardInterrupt:
            print('Closing!')
