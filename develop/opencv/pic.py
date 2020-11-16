#! /usr/bin/env python2

import cv2

# colored Image
img = cv2.imread ("1.png",1)
# Black and White (gray scale)
img_1 = cv2.imread ("1.png",0)

# (y, x, color)
print(img.shape)
print(img_1.shape)

cv2.imshow("1", img)
# waitKey(time) time = 0 for long term
cv2.waitKey(0)
cv2.destroyAllWindows()

resized_image = cv2.resize(img, ( int(img.shape[1]/2), int(img.shape[0]/2) ))
cv2.imshow("1", resized_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
