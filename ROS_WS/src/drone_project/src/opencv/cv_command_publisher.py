#! /usr/bin/env python2
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class image_info:
	def __init__(self):
		rospy.init_node('command_publisher')
		self.image_sub = rospy.Subscriber("/cv_info", Twist, self.callback)
		self.command_pub = rospy.Publisher("/cv_command", Twist, queue_size=2)
		self.center_horizontal_deg = [-5,5]
		self.center_vertical_deg = [-5,5]
		self.ball_distance_cm = [150,200]
		self.twist = Twist()
		rospy.spin()

	def callback(self, msgs):
		x = msgs.linear.x
		y = msgs.linear.y
		z = msgs.linear.z
		if x == 1024 and y == 1024 and z == 1024: #get error code, not thing input
			x = 0
			y = 0
			z = 0
			
		if x < self.center_horizontal_deg[0]:
			pass 
		elif x > self.center_horizontal_deg[1]:
			pass
		else:
			x = 0

		if y < self.center_vertical_deg[0]:
			pass
		elif y > self.center_vertical_deg[1]:
			pass
		else:
			y = 0

		if z < self.ball_distance_cm[0]:
			pass
		elif z > self.ball_distance_cm[1]:
			pass
		else:
			z = 0

		print(x,y,z)
		self.twist.linear.x = x
		self.twist.linear.y = y
		self.twist.linear.z = z
		self.command_pub.publish(self.twist)

if __name__ == "__main__":
	image_cv = image_info()
