#! /usr/bin/env python3

import numpy as np
import cv2 as cv2
import os
import matplotlib.pyplot as plt

# https://docs.opencv.org/3.4/da/df5/tutorial_py_sift_intro.html

# https://www.reddit.com/r/learnpython/comments/szmoq9/visual_code_opencv_cant_openread_file_problem/
file_name = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
assert os.path.exists(file_name)

img = cv2.imread(file_name, 1)
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
