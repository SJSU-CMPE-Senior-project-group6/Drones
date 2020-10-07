#! /usr/bin/env python2
# -*- coding: UTF-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class image_analysis:
	def __init__(self):
		self.image_sub = rospy.Subscriber("topic_cv_image", Image, self.callback)
		self.image_pub = rospy.Publisher("topic_cv_image", String, queue_size=1)
		face_cascade = cv2.CascadeClassifier(
	cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

	def callback(self, image):
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, scaleFactor = 1.05, minNeighbors = 5)
		for x,y,w,h in faces:
			info += ( (x+w/2)/gray.shape[1], (y+h/2)/gray.shape[0], (w*h)/gray.size )
		self.image_pub.publish(info)