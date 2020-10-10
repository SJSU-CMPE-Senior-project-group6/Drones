#! /usr/bin/env python2
# -*- coding: UTF-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class image_analysis:
	def __init__(self):
		rospy.init_node('opencv_publisher', anonymous=True)
		self.image_sub = rospy.Subscriber("topic_cv_image", Image, self.callback)
		self.image_pub = rospy.Publisher("cv_info", String, queue_size=1)
		face_cascade = cv2.CascadeClassifier('/home/ros/Desktop/Drones/ROS_WS/src/drone_project/src/opencv/haarcascade_frontalface_default.xml')
		print("hi")		
		rospy.spin()

	def callback(self, image):
		print("call")		
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, scaleFactor = 1.05, minNeighbors = 5)
		for x,y,w,h in faces:
			info += ( (x+w/2)/gray.shape[1], (y+h/2)/gray.shape[0], (w*h)/gray.size )
		self.image_pub.publish(info)
		print(type(info))

if __name__=="__main__":
	image_processor = image_analysis()
