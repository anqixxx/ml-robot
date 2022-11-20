#! /usr/bin/env python3

import numpy as np
import cv2 as cv2
import os
import matplotlib.pyplot as plt

path = '/home/fizzer/ros_ws/src/my_controller/node/p_image.jpg/'
# https://docs.opencv.org/3.4/da/df5/tutorial_py_sift_intro.html
if not os.path.exists(path):
    print("False")

img = cv2.imread(path, 1)
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT_create()
kp = sift.detect(gray,None)

img=cv2.drawKeypoints(gray,kp,img)
cv2.imwrite('sift_keypoints.jpg',img)
plt.imshow(img)
plt.show()

img=cv2.drawKeypoints(gray,kp,img,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imwrite('sift_keypoints.jpg',img)
plt.imshow(img)
plt.show()

sift = cv2.SIFT_create()
kp, des = sift.detectAndCompute(gray,None)
