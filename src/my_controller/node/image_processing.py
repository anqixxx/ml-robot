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


def plate_hot_rev(index):
  # List to allow maping from character to row numbers
  one_hot_map = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
  return one_hot_map[index]

# Finds plate numbers in data set 
def find_plate(img, type):
  img_aug = np.expand_dims(img, axis=0)
  y_predict = plate_model.predict(img_aug)[0]
  if (type == 'a'):
    y_predict[26:]= [0 for y_val in y_predict[26:]] # Set 1234567890 to none
  if (type == 'n'):
    y_predict[0:25]= [0 for y_val in y_predict[0:25]] # Set ABCDEFGHIJKLMNOPQRSTUVWXYZ to zero
  return (plate_hot_rev(int(y_predict.argmax())))

def location_hot_rev(index):
  # List to allow maping from character to row numbers
  one_hot_map = "12345678"
  return one_hot_map[index]

# Finds location in data set
def find_location(img):
  img_aug = np.expand_dims(img, axis=0)
  y_predict = location_model.predict(img_aug)[0]
  return (location_hot_rev(int(y_predict.argmax())))

file_name = '/home/fizzer/ros_ws/src/my_controller/node/plate_cnn'
assert os.path.exists(file_name)
plate_model = keras.models.load_model(file_name)

file_name = '/home/fizzer/ros_ws/src/my_controller/node/location_cnn'
assert os.path.exists(file_name)
location_model = keras.models.load_model(file_name)

file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
assert os.path.exists(file_name1)
file_name2 = os.path.join(os.path.dirname(__file__), 'plate_img/plate_546.bmp')
assert os.path.exists(file_name2)

img1 = cv2.imread(file_name1, 0)          # queryImage
img2 = cv2.imread(file_name2, 0)      # trainImage, ONLY FOR TESTING
img3 = None
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
MIN_MATCH_COUNT = 12
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)

print(len(good))
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

if img3 is not None:
    plt.imshow(img3, 'gray'),plt.show()
    ret,img3 = cv2.threshold(img3,65,255,cv2.THRESH_BINARY)
    plt.imshow(img3, 'gray'),plt.show()
    img3 = img3/255

    plate_name = 'P'

    # Find location
    # location = img3[564:368, 848:535]
    location = img3[369:522, 569:840]
    location = cv2.resize(location, (220,302))
    plt.imshow(location, 'gray'),plt.show()
    plate_name += str(find_location(tf.expand_dims(location, axis=-1)))
    plate_name += '_'
    
    # Find plate
    # plate_1 = img3.crop((247, 661, 362, 747))
    # y from math
    plate_1 = img3[660:744, 250:362]
    plate_1 = cv2.resize(plate_1, (100,160))
    plt.imshow(plate_1, 'gray'),plt.show()
    plate_name += str(find_plate(tf.expand_dims(plate_1, axis=-1), type='a'))


    # plate_2 = img3.crop((326, 661, 478, 747))
    plate_2 = img3[660:744, 362:475]
    plate_2 = cv2.resize(plate_2, (100,160))
    plt.imshow(plate_2, 'gray'),plt.show()
    plate_name += str(find_plate(tf.expand_dims(plate_2, axis=-1), type='a'))

    # plate_3 = img3.crop((590, 661, 701, 747))
    plate_3 = img3[660:744, 586:698]
    plate_3 = cv2.resize(plate_3, (100,160))
    plt.imshow(plate_3, 'gray'),plt.show()
    plate_name += str(find_plate(tf.expand_dims(plate_3, axis=-1), type='n'))

    # plate_4 = img3.crop((701, 661, 817, 747))
    plate_4 = img3[660:744, 698:811]
    plate_4 = cv2.resize(plate_4, (100,160))
    plate_name += str(find_plate(tf.expand_dims(plate_4, axis=-1), type='n'))
    print(plate_name)





