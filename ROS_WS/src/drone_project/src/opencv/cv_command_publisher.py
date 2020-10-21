#! /usr/bin/env python2
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class image_info:
	def __init__(self):
		rospy.init_node('command_publisher')
		self.image_sub = rospy.Subscriber("/cv_info", String, self.callback)
		self.command_pub = rospy.Publisher("/cv_command", Twist, queue_size=2)
		self.value = [0,0,0,0] #x y w h
		self.twist = Twist()
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
		#self.value = [x,y,w,h]
		#[left - | right +, up - | down +, far - | close +, far - | close +]
		#	    0-400		    0-600		  10 - 400
		#center = [250-300, 170-230, 60-90, N/C]
		y,z,x = None, None, None

		if self.value[0] < 250:
			x = -1 #need to go left
		elif self.value[0] > 300:
			x = 1 #need to go right
		else:
			x = 0

		if self.value[1] < 170:
			y = -1 #need to go up
		elif self.value[1] > 230:
			y = 1 #need to go down
		else:
			y = 0 #stay in the middle

		if self.value[2] < 60:
			z = -1 #need to go closer
		elif self.value[2] > 90:
			z = 1 #need to go away
		else:
			z = 0
		print(x,y,z)
		self.twist.linear.x = x
		self.twist.linear.y = y
		self.twist.linear.z = z
		self.command_pub.publish(self.twist)
if __name__ == "__main__":
	image_cv = image_info()
