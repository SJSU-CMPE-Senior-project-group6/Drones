#! /usr/bin/env python2

import cv2

# Create a CascadeClassifier Object
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# Read image
img = cv2.imread("2.png")

# Read image as gray scale image
gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# gray_img = cv2.imread("2.png", 0)

# Search the co-ordintes of the image
faces = face_cascade.detectMultiScale(gray_img, scaleFactor = 1.05, minNeighbors = 5)

for x,y,w,h in faces:
	img = cv2.rectangle(img, (x,y), (x+w,y+h),(0,255,0),3)

#resized = cv2.resize(img, (int(img.shape[1]/7),int(img.shape[0]/7)))
cv2.imshow("Gray", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
