#! /usr/bin/env python3

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

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend

# Function that converts row number 
# (i.e. the changed label from the Y dataset)
# into a one_hot vector array
def convert_to_one_hot(Y, C):
    Y = np.eye(C)[Y.reshape(-1)].T # C is the amount of labels we have (36)
    return Y # Y must be in number format corresponding to matrix rows

def one_hot_label(character):
  # List to allow maping from character to row numbers
  one_hot_map = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
  return one_hot_map.find(character)

def one_hot_rev(index):
  # List to allow maping from character to row numbers
  one_hot_map = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
  return one_hot_map[index]

def count():
    # folder path
    dir_path = '/home/fizzer/ros_ws/src/my_controller/node/unlabelled/'
    count = 0
    # Iterate directory
    for path in os.scandir(dir_path):
        if path.is_file():
            count += 1
    print("Count is {}".format(count))
    return(count+1)

NUMBER_OF_LABELS = 36
dir_path = '/home/fizzer/ros_ws/src/my_controller/node/unlabelled/'
all_dataset = [] 

for path in os.scandir(dir_path):
    if path.is_file():
        im = Image.open(path)
        # so we knowh the name of the plate
        # name is 11 letters long
        # last 4 are .png
        # first is P
        # take last 10 to last 4
        # string[ start_index_pos: end_index_pos: step_size]
        plate_name = path[-10:-4]

        # section off first image 
        label_loc = one_hot_label(plate_name[0])
        location = im.crop((48, 75, 148, 223))
        # imgplot = plt.imshow(image_1) # image
        temp_tuple = [np.array(image_1), label_1]
        all_dataset.append(tuple(temp_tuple))


        label_1 = one_hot_label(plate_name[2])
        image_1 = im.crop((48, 75, 148, 223))
        # imgplot = plt.imshow(image_1) # image
        temp_tuple = [np.array(image_1), label_1]
        all_dataset.append(tuple(temp_tuple))

        label_2 = one_hot_label(plate_name[3])
        image_2 = im.crop((149, 75, 249, 223))
        # imgplot = plt.imshow(image_2) # image
        temp_tuple = [np.array(image_2), label_2]
        all_dataset.append(tuple(temp_tuple))

        label_3 = one_hot_label(plate_name[4])
        image_3 = im.crop((351, 75, 451, 223))
        # imgplot = plt.imshow(image_3) # image
        temp_tuple = [np.array(image_3), label_3]
        all_dataset.append(tuple(temp_tuple))

        label_4 = one_hot_label(plate_name[5])
        image_4 = im.crop((452, 75, 552, 223))
        # imgplot = plt.imshow(image_4) # image
        temp_tuple = [np.array(image_4), label_4]
        all_dataset.append(tuple(temp_tuple))

# Genereate X and Y datasets
X_dataset_orig = np.array([data[0] for data in all_dataset[:]])
Y_dataset_orig = np.array([[data[1]] for data in all_dataset])
# X is examples, Y is labels for the examples

# Normalize X (images) dataset
X_dataset = X_dataset_orig/255.

# Convert Y dataset to one-hot encoding
Y_dataset = convert_to_one_hot(Y_dataset_orig, NUMBER_OF_LABELS).T
print(Y_dataset)
print(X_dataset)

# Model definition
conv_model = models.Sequential()
conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                             input_shape=(148, 100, 3)))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Flatten())
conv_model.add(layers.Dropout(0.5))
conv_model.add(layers.Dense(512, activation='relu'))
conv_model.add(layers.Dense(36, activation='softmax')) # Amount of labels, ie. things we are trying to classify

conv_model.summary()
LEARNING_RATE = 1e-4 # How fast we are changing the gradient
conv_model.compile(loss='categorical_crossentropy',
                   optimizer=optimizers.RMSprop(lr=LEARNING_RATE),
                   metrics=['acc'])
# Metrics is like eval critera, in this case acc is accuracy, we want to track this
VALIDATION_SPLIT = .2 # used to validate dataset

history_conv = conv_model.fit(X_dataset, Y_dataset, 
                              validation_split=VALIDATION_SPLIT, 
                              epochs=80, # Epoch is one full iteration of the dataset
                              batch_size=16) 
                              # Every time you do a training set, how many examples from the data you take