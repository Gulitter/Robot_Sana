# -*- coding: utf-8 -*-
#!/usr/bin/env python
import os
import sys
import uuid
import queue
import random
import time
# 위 세 줄 없으면 컴파일 안 됨. 유니코드 에러 생김. 한글.

from gtts import gTTS
import playsound
from pydub import AudioSegment
import rclpy
from google.cloud import storage
from google.cloud import dialogflow_v2 as dialogflow
from google.protobuf import struct_pb2
from google.type import latlng_pb2
from google.api_core import exceptions
from sana_msgs.msg import Sana
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, Pose

#sound = AudioSegment.from_adts("voice.wav")
#sound.export("voice.wav", format="wav")
class Slamtopic(Node):
    def __init__(self):
        super().__init__('tts')
        qos = QoSProfile(depth=10)

        self.stt_pub = self.create_publisher(Sana, '/sttonoff', qos)
        
        self.tts_sub = self.create_subscription(Sana, '/tts', self.tts, qos)
        
        self.mp3 = "install/voice_processing/share/voice_processing/voice/voice.mp3"
        
    def tts(self, tts_msg):
        self.speak(tts_msg.tts)
        print("tts")
        if(not ((tts_msg.tts == "도착했습니다. 물품을 저의 상단에 놓아주세요.") or (tts_msg.tts == "안내를 종료합니다. 이용해주셔서 감사합니다."))):
            self.speak("계산하시겠습니까? 아니면 다른 코너로 안내해드릴까요?")
            sana = Sana()
            sana.sttcon = True
            self.stt_pub.publish(sana)
        #if(tts_msg.tts != "물품을 카메라에 인식시켜주세요"):
            #check = Pose()
            #time.sleep(2)
            #self.check_pub.publish(check)
        
    def speak(self, text):
        tts = gTTS(text=text, lang='ko')
        tts.save(self.mp3)
        playsound.playsound(self.mp3)
        




def main(args=None):
    rclpy.init(args=args)
    slam_topic = Slamtopic()
    rclpy.spin(slam_topic)

    slam_topic.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
저장하지 못할 수도 있습니다. gTTS는 오디오 클립을 mp3로 저장하는 옵션을 제공합니다. 이름을 .wav로 지정해도 인식하지 못하고 기본 저장 옵션을 사용합니다. wav 파일만 필요한 경우 pydub 모듈을 사용하여 파일 형식을 변경하십시오.
'''

