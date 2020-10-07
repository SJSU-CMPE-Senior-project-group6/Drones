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

	def callback(self, image):
		