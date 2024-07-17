#!/usr/bin/env python3

from vosk import Model, KaldiRecognizer
import json 
import wave
import numpy as np

class SpeechRecognizer:
    #grammar format example = '["open", "bottle", "cup", "[unk]"]'
    def __init__(self, vosk_model_path: str, grammar: str, wave_output_filename = ""):
        
        self.FORMAT = 2
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 512
        self.RECORD_SECONDS = 3  # Duration to record audio in seconds
        self.WAVE_OUTPUT_FILENAME = wave_output_filename
        self.WAVE_OUTPUT_FILENAME_i = 0
         
        self.audio_buffer = np.array([], dtype='int16')
        if self.WAVE_OUTPUT_FILENAME: 
            self.recording_frames = []  # Added for storing audio frames
        self.numbers = []

        self.model = Model(vosk_model_path)
        self.recognizer = KaldiRecognizer(self.model, self.RATE)
        print(grammar)
        self.recognizer.SetGrammar(grammar)

    def process_audio(self, data):

        recognized_audio = ""

        tmp = np.frombuffer(data, dtype="int16")
        
        self.audio_buffer = np.concatenate((self.audio_buffer, tmp))
        if self.WAVE_OUTPUT_FILENAME: 
            self.recording_frames.append(data)

        if len(self.audio_buffer) > self.RATE*2:  # process in blocks of 1 second
            chunk = self.audio_buffer[:self.RATE*2]
            self.audio_buffer = self.audio_buffer[self.RATE*2:]
            
            if self.recognizer.AcceptWaveform(chunk.tobytes()):
                result = json.loads(self.recognizer.Result())
                if result['text']:
                    #print(f"Recognized: {result['text']}")
                    recognized_audio = result['text']

        #print(len(self.recording_frames))
        #print(int(self.RATE / self.CHUNK * self.RECORD_SECONDS))
        if self.WAVE_OUTPUT_FILENAME: 
            if len(self.recording_frames) >= int(self.RATE / self.CHUNK * self.RECORD_SECONDS):
                self.save_audio_to_wav()

        return recognized_audio

    def save_audio_to_wav(self): 
        self.WAVE_OUTPUT_FILENAME_i+=1
        if self.WAVE_OUTPUT_FILENAME_i == 21:
            self.WAVE_OUTPUT_FILENAME_i = 1

        with wave.open(self.WAVE_OUTPUT_FILENAME+str(self.WAVE_OUTPUT_FILENAME_i)+".wav", 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(2)
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(self.recording_frames))

        print(f"Saved recording as {self.WAVE_OUTPUT_FILENAME+str(self.WAVE_OUTPUT_FILENAME_i)+'.wav'}")
        self.recording_frames = []

