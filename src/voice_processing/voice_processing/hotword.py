######## IMPORTS ##########
# -*- coding: utf-8 -*-
#!/usr/bin/env python
import os
import sounddevice as sd
from scipy.io.wavfile import write
import librosa
import numpy as np
from tensorflow.keras.models import load_model
from gtts import gTTS
import playsound
from pydub import AudioSegment
import rclpy
from sana_msgs.msg import Sana
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile


class VoiceProcessing(Node):
    def __init__(self):
        super().__init__('hotword')
        qos = QoSProfile(depth=10)

        self.fs = 44100
        self.seconds = 2
        self.filename = "install/voice_processing/share/voice_processing/voice/prediction.wav"
        self.class_names = ["Wake Word NOT Detected", "Wake Word Detected"]
        self.model_path = "install/voice_processing/share/voice_processing/model/WWD.h5"
        self.model = load_model(self.model_path)
        
        self.hotword_pub = self.create_publisher(Sana, '/hotword', qos)
        self.sttcon_pub = self.create_publisher(Sana, '/sttonoff', qos)
        
        
        self.sub_hotword = self.create_subscription(Sana, '/hotonoff', self.hotonoff, qos)
        
        self.mp3 = "install/voice_processing/share/voice_processing/voice/prediction.mp3"
        self.hotcon = True
        
        self.recognition()
    def hotonoff(self, hot_msg):
        self.hotcon = hot_msg.hotword
    
    def speak(self, text):
        tts = gTTS(text=text, lang='ko')
        tts.save(self.mp3)
        playsound.playsound(self.mp3)
    
    def recognition(self):
        print("Prediction Started: ")
        i = 0
        while self.hotcon:
            print("Say Now: ")
            myrecording = sd.rec(int(self.seconds * self.fs), samplerate=self.fs, channels=2)
            sd.wait()
            write(self.filename, self.fs, myrecording)

            audio, sample_rate = librosa.load(self.filename)
            mfcc = librosa.feature.mfcc(y=audio, sr=sample_rate, n_mfcc=40)
            mfcc_processed = np.mean(mfcc.T, axis=0)
        
            prediction = self.model.predict(np.expand_dims(mfcc_processed, axis=0))
            if prediction[:, 1] > 0.99:
                print(f"Wake Word Detected for ({i})") #사니
                print("Confidence:", prediction[:, 1])
                i += 1
                sana=Sana()
                sana.hotword = True
                sana.sttcon = True
                self.hotword_pub.publish(sana)
                self.sttcon_pub.publish(sana)
                self.speak("네, 듣고 있어요.")
                self.hotcon = False
                
            
            else:
                print(f"Wake Word NOT Detected")
                print("Confidence:", prediction[:, 1])
                sana=Sana()
                sana.hotword = False
                sana.sttcon = True
                self.hotword_pub.publish(sana)
                self.sttcon_pub.publish(sana)
                self.hotcon = True
        
        
def main(args=None):
    rclpy.init(args=args)
    voice_processing = VoiceProcessing()
    rclpy.spin(voice_processing)

    dqn_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
