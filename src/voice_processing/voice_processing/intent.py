# -*- coding: utf-8 -*-
#!/usr/bin/env python3


import os
import sys
import rclpy
import uuid
import queue
import random
import time
from gtts import gTTS
import playsound #playsound가 gtts 밑으로 가야됨 ;;
from pydub import AudioSegment
from google.cloud import storage
from google.cloud import dialogflow_v2 as dialogflow
from google.protobuf import struct_pb2
from google.type import latlng_pb2
from google.api_core import exceptions
from sana_msgs.msg import Sana
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
key_path = "install/voice_processing/share/voice_processing/config/pro-masze-idwc-766df8ed87b0.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"]=key_path


class VoiceProcessing(Node):
    def __init__(self):
        super().__init__('intent')
        qos = QoSProfile(depth=10)
        
        self.project_id ="pro-masze-idwc"
        self.session_id = str(uuid.uuid4())
        self.language = 'ko'
        disable_audio =  False

        time_before_start = 0.8
        save_audio_requests = True

        self.session_client = dialogflow.SessionsClient()
        self.session = self.session_client.session_path(self.project_id, self.session_id)
        self.list_intents(self.project_id)
        
        self.intent_pub = self.create_publisher(Sana, '/intent', qos)
        self.sttcon_pub = self.create_publisher(Sana, '/sttonoff', qos)
        self.camera_pub = self.create_publisher(Sana, '/camera', qos)
        
        self.sub_stt = self.create_subscription(Sana, '/stt', self.implicit, qos)

        
        self.mp3 = "install/voice_processing/share/voice_processing/voice/prediction.mp3"
    def implicit(self, stt_msg):
        texts = []
        texts.append(stt_msg.stt)#{"의약품 코너 어디있어요?"}
        print(texts)
        for text in texts:
            text_input = dialogflow.TextInput(text=text, language_code="ko")

            query_input = dialogflow.QueryInput(text=text_input)

            response = self.session_client.detect_intent(request={"session": self.session, "query_input": query_input})

            print("=" * 20)
            print("Query text: {}".format(response.query_result.query_text))
            print("Detected intent: {} (confidence: {})\n".format(response.query_result.intent.display_name, response.query_result.intent_detection_confidence,))
            print("Fulfillment text: {}\n".format(response.query_result.fulfillment_text))
            self.speak(response.query_result.fulfillment_text)
            if(response.query_result.fulfillment_text == "의약품 코너로 안내하겠습니다. 사니와 같이 가요!"):
                sana=Sana()
                sana.intent = "move_drug"
                self.intent_pub.publish(sana)
            elif(response.query_result.fulfillment_text == "음료수코너로 안내하겠습니다. 사니와 같이 가요!"):
                sana=Sana()
                sana.intent = "move_drink"
                self.intent_pub.publish(sana)
            elif(response.query_result.fulfillment_text == "네, 계산대로 안내하겠습니다. 저를 잡고 따라와주세요"):
                sana=Sana()
                sana.intent = "move_cal"
                self.intent_pub.publish(sana)
            elif(response.query_result.fulfillment_text == "물품을 저의 상단에 놓아주세요."):
                sana = Sana()
                sana.cam = True
                self.camera_pub.publish(sana)
            elif(response.query_result.intent.display_name == "Default Fallback Intent"):
                sana=Sana()
                sana.hotword = True
                sana.sttcon = True
                self.sttcon_pub.publish(sana)
                
    

    def speak(self, text):
        tts = gTTS(text=text, lang='ko')
        tts.save(self.mp3)
        playsound.playsound(self.mp3)    

        
    def list_intents(self, project_id):
        intents_client = dialogflow.IntentsClient()

        parent = dialogflow.AgentsClient.agent_path(project_id)

        intents = intents_client.list_intents(request={"parent": parent})
        
def main(args=None):
    rclpy.init(args=args)
    voice_processing = VoiceProcessing()
    rclpy.spin(voice_processing)

    dqn_environment.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
