#! /usr/bin/env python2
# -*- coding: UTF-8 -*-

import rospy, sys, 

def main(args):
	ic = image_converter()
	ia = image_analysis()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
