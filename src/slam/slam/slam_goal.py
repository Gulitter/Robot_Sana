import math
import numpy
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, Pose
from sana_msgs.msg import Sana


class Slamtopic(Node):
    def __init__(self):
        super().__init__('goal')

        qos = QoSProfile(depth=10)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self.camera_pub = self.create_publisher(Sana, '/camera', qos)
        self.tts_pub = self.create_publisher(Sana, '/tts', qos)
        
        self.intent_sub = self.create_subscription(Sana, '/intent', self.goal, qos)
        self.check_sub = self.create_subscription(Sana, '/check', self.check, qos)#수정해야됨.
        self.arrive = False
    def goal(self, intent_msg):
        goal_position = intent_msg.intent
        if(goal_position == "move_drink"):
            goal = PoseStamped()
            goal.pose.position.x = 3.5
            goal.pose.position.y = 3.5
            self.goal_pub.publish(goal)
        elif(goal_position == "move_drug"):
            goal = PoseStamped()
            goal.pose.position.x = 3.5
            goal.pose.position.y = 3.5
            self.goal_pub.publish(goal)
        elif(goal_position == "move_cal"):
            goal = PoseStamped()
            goal.pose.position.x = 3.5
            goal.pose.position.y = 3.5
            self.goal_pub.publish(goal)
            
    def check(self, check_msg):#ros2 topic pub --once /check geometry_msgs/msg/Pose
        sana = Sana()
        print("test")
        sana.cam = True
        self.arrive = check_msg.arrive
        if(self.arrive):
            sana.tts = "안내를 종료합니다. 이용해주셔서 감사합니다."
        else:
            sana.tts = "도착했습니다. 물품을 저의 상단에 놓아주세요."
        self.tts_pub.publish(sana)
        time.sleep(5)
        if(not self.arrive):
            self.camera_pub.publish(sana)
        
def main(args=None):
    rclpy.init(args=args)
    slam_topic = Slamtopic()
    rclpy.spin(slam_topic)

    slam_topic.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
