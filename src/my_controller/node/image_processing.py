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
# CNN
import string
import random
import re

from random import randint
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw
from matplotlib import pyplot as plt
import matplotlib.image as mpimg

import tensorflow as tf
from tensorflow import keras

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend


# From https://stackoverflow.com/questions/44650888/resize-an-image-without-distortion-opencv
def resize_image(img, size=(100,160)):

    h, w = img.shape[:2]
    c = img.shape[2] if len(img.shape)>2 else 1

    if h == w: 
        return cv2.resize(img, size, cv2.INTER_AREA)

    dif = h if h > w else w

    interpolation = cv2.INTER_AREA if dif > (size[0]+size[1])//2 else cv2.INTER_CUBIC

    x_pos = (dif - w)//2
    y_pos = (dif - h)//2

    if len(img.shape) == 2:
        mask = np.ones((dif, dif), dtype=img.dtype)
        mask[y_pos:y_pos+h, x_pos:x_pos+w] = img[:h, :w]
    else:
        mask = np.ones((dif, dif, c), dtype=img.dtype)
        mask[y_pos:y_pos+h, x_pos:x_pos+w, :] = img[:h, :w, :]

    return cv2.resize(mask, size, interpolation)
def one_hot_rev(index):
  # List to allow maping from character to row numbers
  one_hot_map = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
  return one_hot_map[index]

# Display images in the training data set. 
def displayImage(img):
  img_aug = np.expand_dims(img, axis=0)
  y_predict = conv_model.predict(img_aug)[0]
  
  plt.imshow(tf.squeeze( img))
  print(y_predict)
  print(one_hot_rev(int(y_predict.argmax())))


file_name = '/home/fizzer/ros_ws/src/my_controller/node/'
assert os.path.exists(file_name)
conv_model = keras.models.load_model(file_name)

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
    img3 = img3/255
    # Find location
    #x1,y1,x2,y2
    # in openCV: cropped = img[start_row:end_row, start_col:end_col]
    # location = img3[564:368, 848:535]
    location = img3[350:550, 560:848]
    # location = resize_image(location, (100,160))
    location = cv2.resize(location, (100,160))
    plt.imshow(location, 'gray'),plt.show()
    
    displayImage(tf.expand_dims(location, axis=-1))
    # Test code for cnn, ignore
    # img_aug = np.expand_dims(location, axis=0)
    # y_predict = conv_model.predict(img_aug)[0] # defined in CNN
    # print(one_hot_rev(int(y_predict. argmax())))


    # Find plate
    # plate_1 = img3.crop((247, 661, 362, 747))
    plate_1 = img3[650:750, 247:362]
    # plate_1 = resize_image(plate_1, (100,160))
    plate_1 = cv2.resize(plate_1, (100,160))

    plt.imshow(plate_1, 'gray'),plt.show()
    displayImage(tf.expand_dims(plate_1, axis=-1))


    # plate_2 = img3.crop((326, 661, 478, 747))
    plate_2 = img3[650:750, 362:478]
    plate_2 = cv2.resize(plate_2, (100,160))

    # plate_2 = resize_image(plate_2, (100, 160))
    plt.imshow(plate_2, 'gray'),plt.show()
    displayImage(tf.expand_dims(plate_2, axis=-1))

    # plate_3 = img3.crop((590, 661, 701, 747))
    plate_3 = img3[650:750, 583:698]
    plate_3 = cv2.resize(plate_3, (100,160))

    # plate_3 = resize_image(plate_3, (100, 160))
    plt.imshow(plate_3, 'gray'),plt.show()
    displayImage(tf.expand_dims(plate_3, axis=-1))

    # plate_4 = img3.crop((701, 661, 817, 747))
    plate_4 = img3[650:750, 698:813]
    # plate_4 = resize_image(plate_4, (100, 160))
    plate_4 = cv2.resize(plate_4, (100,160))

    plt.imshow(plate_4, 'gray'),plt.show()
    displayImage(tf.expand_dims(plate_4, axis=-1))

