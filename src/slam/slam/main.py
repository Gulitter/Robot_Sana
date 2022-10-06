import math
import numpy

from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState


class Slamtopic(Node):
    def __init__(self):
        super().__init__('slam_topic')

        qos = QoSProfile(depth=10)

        self.pub_JointStates = self.create_publisher(JointState, '/joint_states', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.sub_odom = self.create_subscription(Odometry, '/slam/odom', self.odom_subscribe, qos)
        
    def odom_subscribe(self, odom_msg):
        q = self.quaternion_from_euler(0, 0, odom_msg.pose.pose.position.z)
        timestamp_now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = timestamp_now
        odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = odom_msg.twist.twist.linear.x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = odom_msg.twist.twist.angular.z
        self.odom_pub.publish(odom)
        
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now

        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_OdomTF.sendTransform(odom_tf)

        joint_states = JointState()
        joint_states.header.frame_id = "base_link"
        joint_states.header.stamp = timestamp_now
        joint_states.name = ['left_wheel', 'right_wheel']
        joint_states.position = [odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z]
        joint_states.velocity = [odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y]
        joint_states.effort = []
        self.pub_JointStates.publish(joint_states)
    
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q
        
def main(args=None):
    rclpy.init(args=args)
    slam_topic = Slamtopic()
    rclpy.spin(slam_topic)

    slam_topic.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
