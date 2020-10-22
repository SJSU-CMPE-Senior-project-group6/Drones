from picamera.array import PiRGBArray
from picamera import PiCamera
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import time
import cv2

#ROS
rospy.init_node('opencv_publisher')
pub = rospy.Publisher('/cv_info',String,queue_size=2)
# Create a CascadeClassifier Object
face_cascade = cv2.CascadeClassifier('/home/ros/Desktop/Drones/ROS_WS/src/drone_project/src/opencv/model/lbpcascade_frontalface_improved.xml')

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
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# Search the co-ordinates of the image
	faces = face_cascade.detectMultiScale(gray,
		scaleFactor = 1.1, minNeighbors = 4)
	str_send = ""
	for x,y,w,h in faces:
		str_send += str(x)+","+str(y)+","+str(w)+","+str(h)+"," 		
		print(str_send)
		pub.publish(str_send)
		
		image = cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 3)
		cv2.putText(image, 'face', (x, y),
			cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)        
		#print(x," ",y," ",w," ",h)	

	# show the frame
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)


	#if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
