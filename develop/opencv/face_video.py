#! /usr/bin/env python2
# -*- coding: UTF-8 -*-

import cv2

# Create a CascadeClassifier Object
face_cascade = cv2.CascadeClassifier(
	cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# 打开摄像头
cap = cv2.VideoCapture(0)

while(True):
	# capture frame-by-frame
	ret , frame = cap.read()

	# our operation on the frame come here
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# Search the co-ordintes of the image
	faces = face_cascade.detectMultiScale(gray,
		scaleFactor = 1.05, minNeighbors = 5)

	for x,y,w,h in faces:
		frame = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 3)
		cv2.putText(frame, 'face', (x, y),
			cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

	# display the resulting frame
	cv2.imshow('frame', frame)
	if cv2.waitKey(1) &0xFF ==ord('q'):  #按q键退出
		break

# when everything done, release the capture
cap.release()
cv2.destroyAllWindows()
