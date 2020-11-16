#! /usr/bin/env python2
# -*- coding: UTF-8 -*-

from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(
	image_message, desired_encoding='passthrough')