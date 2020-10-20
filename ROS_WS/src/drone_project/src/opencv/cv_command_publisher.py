#! /usr/bin/env python2
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class image_info:
	def __init__(self):
		rospy.init_node('command_publisher')
		self.image_sub = rospy.Subscriber("/cv_info", String, self.callback)
		self.image_pub = rospy.Publisher("/cv_command", Twist, queue_size=2)
		self.value = [0,0,0,0] #x y w h

		rospy.spin()

	def callback(self, msgs):
		command = msgs.data
		comma_counter = 0
		i = 0
		temp = ""

		while True:
			if command[i] == ",":
				self.value[comma_counter] = int(temp)
				temp = ""	
				comma_counter += 1
			else:	
				temp += command[i]
			i += 1
			if comma_counter == 4:
				break
		
		print(self.value)

if __name__ == "__main__":
	image_cv = image_info()
