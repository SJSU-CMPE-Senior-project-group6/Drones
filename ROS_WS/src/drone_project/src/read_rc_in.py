#!/usr/bin/env python

import rospy
from mavros_msgs.msg import RCIn 
from geometry_msgs.msg import Twist

def listener():
    rospy.Subscriber("/mavros/rc/in",RCIn,read_input)
    print("set subscriber")
    
    rospy.spin()
    print("end sub")

def read_input(msg):
    print("read input")
    print(msg.rssi," ",msg.channels)
  

if __name__ == "__main__":
    rospy.init_node("rc_reading")
    listener()
