#!/usr/bin/env python3

import rospy
import pyaudio

from audio_common_msgs.msg import AudioData

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            output=True)

def audio_callback(audio):
    stream.write(audio.data)

def audio_subscriber():
    rospy.init_node('audio_subscriber', anonymous=True)

    # Sottoscrizione al topic 'audio' con la callback audio_callback
    rospy.Subscriber('/nicla/audio', AudioData, audio_callback)

    rospy.loginfo("Nodo audio_subscriber pronto per ricevere audio...")

    p = pyaudio.PyAudio()



    # Mantenimento del nodo attivo finché non viene interrotto esplicitamente
    rospy.spin()

if __name__ == '__main__':
    try:
        audio_subscriber()
    except rospy.ROSInterruptException:
        pass
