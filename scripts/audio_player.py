#!/usr/bin/env python3

import rospy
import queue
from audio_common_msgs.msg import AudioData

from pydub import AudioSegment
import io
import pygame
import numpy as np

from threading import Thread


class AudioPlay():
    def __init__(self):
        
        rospy.init_node('audio_subscriber', anonymous=True)

        self.rate = rospy.Rate(30) 
        self.audio_data = queue.Queue(maxsize=100000)

        # Sottoscrizione al topic 'audio' con la callback audio_callback
        audio_sub = rospy.Subscriber('/nicla/audio', AudioData, self.audio_callback)

        rospy.loginfo("Nodo audio_subscriber pronto per ricevere audio...")


        # Initialize pygame mixer
        pygame.mixer.init()
        
        self.thread_regularizer = True

        self.player_thread = Thread(target=self.audio_play)
        self.player_thread.start()  


    def audio_callback(self, audio):
        # print(audio.data) 
        data = np.frombuffer(audio.data, dtype=np.uint8)

        non_zero_mask = data != 0

        filtered_data = data[non_zero_mask]

        if filtered_data.size > 0:
            self.audio_data.put_nowait(filtered_data.tobytes())


    def audio_play(self):

        while self.thread_regularizer:

            try: 
                pcm_data = self.audio_data.get_nowait()

                # Create an AudioSegment instance from the raw PCM data
                # audio_segment = AudioSegment(
                #     data=pcm_data,
                #     sample_width=1,  # 8-bit samples
                #     frame_rate=16000,  # 16 kHz sample rate
                #     channels=1  # mono
                # )

                # Export the audio segment to an in-memory MP3 file
                # mp3_io = io.BytesIO()
                # audio_segment.export(mp3_io, format="mp3")
                # mp3_io.seek(0)

                # # Initialize pygame mixer
                # pygame.mixer.init()

                # Load the MP3 data into a pygame Sound object
                # mp3_data = mp3_io.read()
                sound = pygame.mixer.Sound(buffer = pcm_data) #file=io.BytesIO(mp3_data) ) #buffer=mp3_data)

                # Play the sound
                sound.play()

                # Keep the program running until the sound is finished
                # while pygame.mixer.get_busy():
                #     pygame.time.Clock().tick(10)


            except Exception as e:
                rospy.logerr(f"Error playing audio: {e}")
                pass


    def run(self):
         
        while not rospy.is_shutdown():  
            #self.audio_play()
            self.rate.sleep()

        self.player_thread.join()
        self.thread_regularizer = False

 

if __name__ == '__main__':
    try:
        player = AudioPlay()
        player.run()
    except rospy.ROSInterruptException:
        pass
