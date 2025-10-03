ㅇ 음성 안내

<img width="396" height="117" alt="image" src="https://github.com/user-attachments/assets/eb7afb61-1de9-4c09-9da2-f1beea36813f" />

  ㅇ 목적지 안내
  
<img width="534" height="182" alt="image" src="https://github.com/user-attachments/assets/da71ef71-700f-4c32-9b32-2dd4bb3b3e29" />


  ㅇ 물품 안내
  
<img width="383" height="207" alt="image" src="https://github.com/user-attachments/assets/caa503ec-7027-47c8-b0ce-46a43566b6a0" />

ㅇ 서비스 흐름도


<img width="386" height="383" alt="image" src="https://github.com/user-attachments/assets/e3cd428c-1cf9-4d03-b229-456c4c9e9960" />


<img width="329" height="313" alt="image" src="https://github.com/user-attachments/assets/fc1ed2b3-4740-4475-9396-1de18d5f6c38" />


ㅇ 하드웨어 설계도

<img width="543" height="307" alt="image" src="https://github.com/user-attachments/assets/e636e956-4b2d-4f87-883d-35161cf8b060" />



**자율주행**

SLAM을 통해 라이다센서로 거리정보를 받아서 편의점이나 마트의 지도를 그리고, 지도를 기반으로 자율주행합니다.



**음성대화**

사용자가 로봇의 이름을 부르면 대화 알고리즘을 실행합니다. 음성대화를 통해 사용자의 요구를 수행하고, 물품의 정보를 안내해주며 양방향 대화가 가능합니다. 



**객체인식**

점자로 구별이 어려운 물품들의 구별을 위해, 카메라를 통해 물품이 인식되면 딥러닝 이미지 학습을 통해 상품명을 알아내어 물품에 대한 정보를 알 수 있도록 합니다.




**개발환경 OS**

Windows10, Ubuntu20.04
자율주행, 객체인식, 음성인식 등 작품 전체적으로  적용되었음.



**개발환경(IDE)**

Colab, Visual Studio Code
객체인식 시 적용 , 자율주행 음성인식 적용 되었음.



**개발도구**

Yolov3, Dialogflow,  Google Cloud, ROS2 foxy
객체인식 적용, 음성인식 적용 , 자율주행  적용되었음.



**개발언어**

Python, C
자율주행, 객체인식, 음성인식 등 작품 전체적으로  적용되었음.



**구성장비**

Notebook, ESP32, PG42-4266-1270E(DC Motor), L298N(Motor Driver)



**센서**

YDLidarG2, Camera, OSTSen-B055(IMU)
객체를 감지해 거리를 맵핑함, 객체를 인식 함, 편의점 내의 방향, 적정한 가속도, 위치를 구함.



**통신**

micro-ROS를 이용한 USB-serial 통신
ESP32와 ROS2에서 통신하기 위함.



<img width="264" height="310" alt="image" src="https://github.com/user-attachments/assets/786198fa-bd90-4b5d-92df-3bd8284c8e48" />
<img width="160" height="311" alt="image" src="https://github.com/user-attachments/assets/5f2cac92-4b0c-482e-8d78-11737cb1b7b1" />


**문제점 및 해결방안**

  - 주변소음 및 억양과 목소리의 차이로 인한 음성인식 오류
⦁음성인식 전 주변소음을 측정하여 노이즈를 제거하고, STT API를 이용하여 일반적인 음성에 대해서 Text로 변환시켜 사용

  - 카메라에 찍히는 물품의 방향에 따라서 인식에 오차가 생길 수 있음
⦁물품을 여러방향에서 찍은 사진을 학습시켜서 해결

  - 바퀴와 바닥의 마찰력과 IMU센서의 오차로 인해 로봇의 위치를 파악하는데 어려움이 있음
⦁칼만필터를 통해 IMU센서의 오차를 최소화하고, 엔코더 정보를 같이 결합하여서 해결

  - 로봇이 급정지를 해서 주행 안정성에 문제가 있음 
⦁가속도를 통한 PID제어를 통해 해결

  - 로봇을 제어할 노트북과 원활히 통신하는데 문제가 있음
⦁원격제어 프로그램을 통해 해결

**기대효과**

  ㅇ 물건의 종류를 정확히 알 수 없어 어려움을 겪던 시각장애인이 딥러닝과 음성인식을 통한 물품안내를 통해 원하는 물품을 구매할 수 있게 됩니다.
  ㅇ SLAM 자율주행을 통해 시각장애인이 로봇을 잡고 움직일 수 있도록 제어하여 실내에서 부딪히지 않고 위치를 파악하여 사용자의 속도에 맞추어 안전하게 목적지까지 도착할 수 있도록 합니다.



**활용분야**

  ㅇ 무인점포에서 시각장애인을 위한 로봇을 활용 가능하다.
  ㅇ 시각장애인뿐만이 아닌 안내가 필요한 사람들도 로봇을 통해 도움을 받을 수 있다.

  
