from picamera.array import PiRGBArray
from picamera import PiCamera
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import time
import cv2
import numpy as np
#ROS
rospy.init_node('opencv_publisher')
pub = rospy.Publisher('/cv_info',String,queue_size=2)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array	
	
	cols = len(image[0])
	rows = len(image)	
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

	for cnt in contours:
		(x, y, w, h) = cv2.boundingRect(cnt)

		str_send += str(x)+","+str(y)+","+str(w)+","+str(h)+"," 		
		print(str_send)
		pub.publish(str_send)

		x_medium = int((x + x + w) / 2 )
		break
	cv2.line(image, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)

	# show the frame
	cv2.imshow("Frame", image)
	cv2.imshow("mask", red_mask)
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)


	#if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
