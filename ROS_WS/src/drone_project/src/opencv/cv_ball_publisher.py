from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy
import time
import cv2
import numpy as np
import math

class ball_recognition:
	def __init__(self):
		#ROS
		self.cap = cv2.VideoCapture(0)
		rospy.init_node('opencv_publisher')
		self.pub = rospy.Publisher('/cv_info',Twist,queue_size=2)
		self.frame_w = 640
		self.frame_h = 480
		self.angle_per_pixel = 78.0/self.frame_w
		self.ball_diameter = 14.7
		self.twist = Twist()
		print("ball_command publisher starts")

	def camera_publisher(self):
		# initialize the camera and grab a reference to the raw camera capture
		cols = 640
		rows = 480   
		while(True):
			ret, image = self.cap.read()
			ksize = (6,6)
			image = cv2.blur(image,ksize)	
			x_medium = int(cols/2)
			center = int(cols/2)
	
			#blur = cv2.GaussianBlur(image, (5,5), 0)
			hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

			#red color 
			lower_red = np.array([161, 155, 84])
			upper_red = np.array([179, 255, 255])
			red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
			_, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
	
			str_send = ""
			if len(contours) <= 0:
				self.twist.linear.x = 1024
				self.twist.linear.y = 1024
				self.twist.linear.z = 1024
				self.pub.publish(self.twist)

			center_x, center_y = 0.0, 0.0
			for cnt in contours:
				(center_x,center_y),radius = cv2.minEnclosingCircle(cnt)			
				center = (int(center_x),int(center_y))
				#print(center,radius)
				cv2.circle(image,center,int(radius),(0,255,0),2)
				(x, y, w, h) = cv2.boundingRect(cnt)
				target_angle_horizontal = (center_x - 320) * self.angle_per_pixel
				target_angle_vertical = (center_y - 240) * self.angle_per_pixel 
				if radius == 0:
					dist_to_target = -1
				else:
					dist_to_target = (self.ball_diameter / 2) / math.tan(math.radians(radius* self.angle_per_pixel))
				str_send += str(target_angle_horizontal)+","+str(target_angle_vertical)+","+str(dist_to_target)		
				#print(str_send)
				self.twist.linear.x = target_angle_horizontal
				self.twist.linear.y = target_angle_vertical
				self.twist.linear.z = dist_to_target
				self.pub.publish(self.twist)
				break
			cv2.line(image, (int(center_x), 0), (int(center_x), 480), (255, 0, 0), 2)

			# show the frame
			cv2.imshow("Frame", image)
			#cv2.imshow("mask", red_mask)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

if __name__ == "__main__":
	image_pub = ball_recognition()
	image_pub.camera_publisher()
