# -*- coding: utf-8 -*-
#!/usr/bin/env python

import os
import sys
import playsound
from pydub import AudioSegment
import speech_recognition as sr
import rclpy
from sana_msgs.msg import Sana
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

class VoiceProcessing(Node):
    def __init__(self):
        super().__init__('stt')
        qos = QoSProfile(depth=10)
        
        self.stt_pub = self.create_publisher(Sana, '/stt', qos)
        self.sub_hotword = self.create_subscription(Sana, '/hotword', self.hot_sub, qos)
        self.sub_stt = self.create_subscription(Sana, '/sttonoff', self.stt, qos)
        self.hot_arr = False
        self.sttcon = True
        
    def hot_sub(self, hotword_msg):
        self.hot_arr = hotword_msg.hotword
        
    def stt(self, stt_msg):
        self.sttcon = stt_msg.sttcon
        print("stt")
        if self.hot_arr:
            while self.sttcon:
                r = sr.Recognizer()
                print(sr.Microphone())
                with sr.Microphone() as source:
                    r.adjust_for_ambient_noise(source)  # here
                    r.energy_threshold = 70000
                    r.dynamic_energy_threshold = True
                    r.pause_threshold=1.2
                    print("Say Something")
                    speech = r.listen(source)
                    said = " "
                    try:
                        said = r.recognize_google(speech, language="ko-KR")
                        sana=Sana()
                        sana.stt = said
                        self.stt_pub.publish(sana)
                        print("Your speech thinks like\n " + said)
                        self.sttcon = False
                    except sr.UnknownValueError:
                        print("Your speech can not understand")
                    except sr.RequestError as e:
                       print("Request Error!; {0}".format(e))
            

def main(args=None):
    rclpy.init(args=args)
    voice_processing = VoiceProcessing()
    rclpy.spin(voice_processing)

    dqn_environment.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
