from std_msgs.msg import String
import rospy
import cv2

class face_recognition:
	def __init__(self):
		#ROS
		self.cap = cv2.VideoCapture(0)
		rospy.init_node('opencv_publisher')
		self.pub = rospy.Publisher('/cv_info',String,queue_size=2)
        print("image_command publisher starts")

	def camera_publisher(self):
		face_cascade=cv2.CascadeClassifier('/home/ros/Desktop/Drones/ROS_WS/src/drone_project/src/opencv/model/lbpcascade_frontalface_improved.xml')
        
		while(True):
			ret, image = self.cap.read()
			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

			# Search the co-ordinates of the image
			faces = face_cascade.detectMultiScale(gray, scaleFactor = 1.1, minNeighbors = 4)
			str_send = ""
			for x,y,w,h in faces:
				str_send += str(x)+","+str(y)+","+str(w)+","+str(h)+"," 		
				print(str_send)
				self.pub.publish(str_send)
		
				image = cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 3)
				cv2.putText(image, 'face', (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

			# show the frame
			cv2.imshow("Frame", image)
			#cv2.imshow("mask", red_mask)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

if __name__ == "__main__":
	image_pub = face_recognition()
	image_pub.camera_publisher()
