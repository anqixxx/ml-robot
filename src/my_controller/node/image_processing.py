#! /usr/bin/env python3

from __future__ import print_function
import cv2
import sys
import numpy as np
import os
import matplotlib.pyplot as plt
from numpy.linalg import inv
import csv
from random import randint
import string
from PIL import Image, ImageFont, ImageDraw


file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
assert os.path.exists(file_name1)
file_name2 = os.path.join(os.path.dirname(__file__), 'plate_img/plate_546.bmp')
assert os.path.exists(file_name2)

img1 = cv2.imread(file_name1, 0)          # queryImage
img2 = cv2.imread(file_name2, 0)      # trainImage, ONLY FOR TESTING

# plt.imshow(img1, 'gray'),plt.show()
plt.imshow(img2, 'gray'),plt.show()

# Initiate SIFT detector
sift = cv2.SIFT_create()
# Gets keypoints and descriptors of key frame image
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1,des2,k=2)
# store all the good matches as per Lowe's ratio test.
good = []
MIN_MATCH_COUNT = 7
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)

# print(len(good))
if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    
    if M is not None:
        invM = inv(M)
        # Take matrix (M) and find the matrix inverse and apply to the image
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,M)
        invM = np.linalg.inv(M)
        img3 = cv2.warpPerspective(img2, invM, (w, h))

        # plt.imshow(img3, 'gray'),plt.show()
        
        # Cannot implement unless adaptive thresholding is used
        # ret,img3 = cv2.threshold(img3,110,255,cv2.THRESH_BINARY)
        # plt.imshow(img3, 'gray'),plt.show()


if img3 is not None:
    # Find location
    #x1,y1,x2,y2
    # in openCV: cropped = img[start_row:end_row, start_col:end_col]
    # location = img3[564:368, 848:535]
    location = img3[367:535, 560:848]
    plt.imshow(location, 'gray'),plt.show()

    # Find plate
    # plate_1 = img3.crop((247, 661, 362, 747))
    plate_1 = img3[661:747, 247:362]
    plt.imshow(plate_1, 'gray'),plt.show()

    # plate_2 = img3.crop((326, 661, 478, 747))
    plate_2 = img3[661:747, 362:478]
    plt.imshow(plate_2, 'gray'),plt.show()

    # plate_3 = img3.crop((590, 661, 701, 747))
    plate_3 = img3[661:747, 583:698]
    plt.imshow(plate_3, 'gray'),plt.show()

    # plate_4 = img3.crop((701, 661, 817, 747))
    plate_4 = img3[661:747, 698:813]
    plt.imshow(plate_4, 'gray'),plt.show()