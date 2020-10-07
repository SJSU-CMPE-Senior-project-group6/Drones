#! /usr/bin/env python2
# -*- coding: UTF-8 -*-

import roslib
roslib.load_manifest('my_package')
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
	def __init__(self):
		self.image_sub = rospy.Subscriber("topic_ros_image", Image, self.callback)
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("topic_cv_image", Image, queue_size=1)
		rospy.init_node('image_converter', anonymous=True)

	def callback(self, image):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
			self.image_pub.publish(cv_image)
		except CvBridgeError as e:
			print(e)