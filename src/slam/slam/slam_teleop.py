#!/usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)
qos = QoSProfile(depth=10)
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

Speed up:
        w
   a    s    d
   
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1)
	       }

speedBindings={
		'w':(0.01,0),
		's':(-0.01,0),
		'd':(0, 0.01),
		'a':(0,-0.01)
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	
	if args is None:
		args = sys.argv

	rclpy.init(args=args)
	node = rclpy.create_node('teleop_twist_keyboard')
		
	pub = node.create_publisher(Twist, 'cmd_vel', 	qos)

	speed = 0.01
	turn = 0.01
	x = 0
	th = 0
	status = 0

	try:
		print(msg)
		print(vels(speed,turn))
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				th = moveBindings[key][1]
			elif key in speedBindings.keys():
				speed = speed + speedBindings[key][0]
				turn = turn + speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15
			else:
				x = 0
				th = 0
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed;
			twist.angular.z = th*turn
			pub.publish(twist)

	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


