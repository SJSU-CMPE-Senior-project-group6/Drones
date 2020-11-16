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
		self.ball_distance_cm = [80,120]
		self.ball_center_ptr = self.ball_distance_cm[0] + (self.ball_distance_cm[1] - self.ball_distance_cm[0])/2
		self.twist = Twist()
		rospy.spin()

	def callback(self, msgs):
		x = msgs.linear.x
		y = msgs.linear.y
		z = msgs.linear.z
		found = msgs.angular.x
		x_pos = ""
		y_pos = ""
		z_pos = ""
		print(x,y,z)
		if found == False: #get error code, not thing input
			if x == 1024 and y == 1024 and z == 1024: 
				x = 0
				y = 0
				z = self.ball_center_ptr
				x_pos = "N/A"
				y_pos = "N/A"
				z_pos = "N/A"
		
		else: #found target
			if x < self.center_horizontal_deg[0]:
				x_pos = "left"
				pass 
			elif x > self.center_horizontal_deg[1]:
				x_pos = "right"
				pass
			else:
				x_pos = "center"
				x = 0

			if y < self.center_vertical_deg[0]:
				y_pos = "up"
				pass
			elif y > self.center_vertical_deg[1]:
				y_pos = "down"
				pass
			else:
				y_pos = "center"
				y = 0

			if z < self.ball_distance_cm[0]:
				z_pos = "close"
				z = z - self.ball_center_ptr
			elif z > self.ball_distance_cm[1]:
				z_pos = "far"
				z = z - self.ball_center_ptr
			else:
				z_pos = "center"
				z = 0
    

		#print(x_pos,y_pos,z_pos)
		self.twist.linear.x = x
		self.twist.linear.y = y
		self.twist.linear.z = z
		self.command_pub.publish(self.twist)

if __name__ == "__main__":
	image_cv = image_info()
