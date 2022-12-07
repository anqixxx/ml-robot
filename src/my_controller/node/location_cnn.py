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

import tensorflow as tf
from tensorflow import keras

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers

from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend


# Function that converts row number 
# (i.e. the changed label from the Y dataset)
# into a one_hot vector array
def convert_to_one_hot(Y, C):
    Y = np.eye(C)[Y.reshape(-1)].T # C is the amount of labels we have (8)
    return Y # Y must be in number format corresponding to matrix rows

def one_hot_label(character):
  # List to allow maping from character to row numbers
  one_hot_map = "12345678"
  return one_hot_map.find(character)

def one_hot_rev(index):
  # List to allow maping from character to row numbers
  one_hot_map = "12345678"
  return one_hot_map[index]

# Display images in the training data set. 
def displayImage(index):
  print('here')
  # img = tf.shape( tf.squeeze( X_dataset[index]))
  img = X_dataset[index]

  img_aug = np.expand_dims(img, axis=0)
  # img_aug = X_dataset[index]
  # print(conv_model.predict(img_aug)[0])
  y_predict = conv_model.predict(img_aug)[0]
  
  plt.imshow(tf.squeeze( img))
  # print( np.max(y_predict))

  print(one_hot_rev(int(y_predict. argmax())))

NUMBER_OF_LABELS = 8
dir_path = '/home/fizzer/ros_ws/src/my_controller/node/unlabelled/'
all_dataset = [] 

for path in os.scandir(dir_path):
    if path.is_file():
        print(path)
        file_name = os.path.join(os.path.dirname(dir_path), str(path.name))
        assert os.path.exists(file_name)

        im = cv2.imread(file_name, 0)
        im = cv2.GaussianBlur(im,(7,7),cv2.BORDER_DEFAULT)
        ret,im = cv2.threshold(im,127,255,cv2.THRESH_BINARY)
        # plt.imshow(im, 'gray'),plt.show()

        plate_name = str(path.name)

        label = one_hot_label(plate_name[1])
        location = im[145:447, 340:560]
        # plt.imshow(location, 'gray'),plt.show()
        temp_tuple = [np.array(location), label]
        all_dataset.append(tuple(temp_tuple))

# Genereate X and Y datasets
X_dataset_orig = np.array([data[0] for data in all_dataset[:]])
Y_dataset_orig = np.array([[data[1]] for data in all_dataset])
plt.imshow(X_dataset_orig[6], 'gray'),plt.show()
# X is examples, Y is labels for the examples
# Normalize X (images) dataset
X_dataset = (X_dataset_orig/255)
X_dataset = tf.expand_dims(X_dataset, axis=-1)

Y_dataset = convert_to_one_hot(Y_dataset_orig, NUMBER_OF_LABELS).T

# Model definition
conv_model = models.Sequential()
conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                            input_shape=(302, 220, 1)))
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
conv_model.add(layers.Dense(8, activation='softmax')) # Amount of labels, ie. things we are trying to classify

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

conv_model.save('/home/fizzer/ros_ws/src/my_controller/node/location_cnn')

# Model Loss
plt.plot(history_conv.history['loss'])
plt.plot(history_conv.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train loss', 'val loss'], loc='upper left')
plt.show()

# Model Accuracy
plt.plot(history_conv.history['acc'])
plt.plot(history_conv.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy (%)')
plt.xlabel('epoch')
plt.legend(['train accuracy', 'val accuracy'], loc='upper left')
plt.show()

displayImage(6)
