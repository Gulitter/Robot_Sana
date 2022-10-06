# -*- coding: utf-8 -*-
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sana_msgs.msg import Sana

class Yolov3(Node):
    def __init__(self):
        super().__init__('yolov3')
        qos = QoSProfile(depth=10)
        
        self.tts_pub = self.create_publisher(Sana, '/tts', qos)
        
        self.camera_sub = self.create_subscription(Sana, '/camera', self.camera, qos)
        
        self.weights = "install/yolov3/share/yolov3/weights/_yolov3_data5_2_train_final.weights"
        self.config = "install/yolov3/share/yolov3/config/_yolov3_data5_2_test.cfg"
        self.labels = "install/yolov3/share/yolov3/parameter/labels.names"

        self.net = cv2.dnn.readNet(self.weights, self.config)
        self.classes = []
        with open(self.labels, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        self.cap = cv2.VideoCapture(0) #웹캠 킴
        self.camonoff = False
        self.pay = ["2300원", "2400원", "2500원", "1700원", "3100원"]
        self.naming = ["코카콜라", "스프라이트", "포카리스웨이트", "배아제", "타이레놀"]

    def camera(self, cam_msg):
        self.camonoff = cam_msg.cam
        cam_rec = [0, 0, 0, 0, 0]
        while(self.camonoff):
            ret, frame = self.cap.read() #사진 찍음
            height, width, channels = frame.shape
            blob = cv2.dnn.blobFromImage(frame, 0.00392, (416,416), (0,0,0), True, crop=False) #net용 이미지 변환
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)
            class_ids = []
            confidences = []
            boxes = []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.8:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        # 좌표
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
               
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

            font = cv2.FONT_HERSHEY_PLAIN
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(self.classes[class_ids[i]])
                    color = self.colors[i]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(frame, label, (x, y + 30), font, 3, color, 3)
            cv2.imshow("Image", frame)
            try:
                if(label == 'cocacola'):
                    num = 0
                elif(label == 'sprite'):
                    num = 1
                elif(label == 'pocari'):
                    num = 2
                elif(label == 'bearse'):
                    num = 3
                elif(label == 'tylenol'):
                    num = 4
                
                cam_rec[num] = cam_rec[num] + 1
                print(cam_rec)
                if(cam_rec[num]>10):
                    self.camonoff = False
                    sana = Sana()
                    if(num == 3):
                        sana.tts = "이 제품은 배아제입니다. 가격은 1700원입니다. 이 약은 소화를 돕는 소화효소제입니다. 구매를 원하시면 한 칸 아래 짐칸에 놓아주세요."
                    elif(num == 4):
                        sana.tts = "이 제품은 타이레놀입니다. 가격은 3100원입니다. 이 약은 감기로 인한 발열 및 통증 등에 복용합니다. 구매를 원하시면 한 칸 아래 짐칸에 놓아주세요."
                    else:
                        sana.tts = "이 제품은 " + self.naming[num] +"입니다. 가격은 "+ self.pay[num]+"입니다. 구매를 원하시면 한 칸 아래 짐칸에 놓아주세요."
                    #time.sleep(2)
                    self.tts_pub.publish(sana)
            except:
                pass
            if cv2.waitKey(1) == ord('q'):
                break
                
def main(args=None):
    rclpy.init(args=args)
    yolov3 = Yolov3()
    rclpy.spin(yolov3)
    slam_topic.destroy()
    rclpy.shutdown()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
