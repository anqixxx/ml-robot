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

# From https://stackoverflow.com/questions/44650888/resize-an-image-without-distortion-opencv
def resize_image(img, size=(160,100)):

    h, w = img.shape[:2]
    c = img.shape[2] if len(img.shape)>2 else 1

    if h == w: 
        return cv2.resize(img, size, cv2.INTER_AREA)

    dif = h if h > w else w

    interpolation = cv2.INTER_AREA if dif > (size[0]+size[1])//2 else cv2.INTER_CUBIC

    x_pos = (dif - w)//2
    y_pos = (dif - h)//2

    if len(img.shape) == 2:
        mask = np.zeros((dif, dif), dtype=img.dtype)
        mask[y_pos:y_pos+h, x_pos:x_pos+w] = img[:h, :w]
    else:
        mask = np.zeros((dif, dif, c), dtype=img.dtype)
        mask[y_pos:y_pos+h, x_pos:x_pos+w, :] = img[:h, :w, :]

    return cv2.resize(mask, size, interpolation)

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

def define_model():
  # Model definition
  conv_model = models.Sequential()
  conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                              input_shape=(160, 100, 1)))
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

NUMBER_OF_LABELS = 36
dir_path = '/home/fizzer/ros_ws/src/my_controller/node/unlabelled/'
all_dataset = [] 

for path in os.scandir(dir_path):
    if path.is_file():
        print(path)
        file_name = os.path.join(os.path.dirname(dir_path), str(path.name))
        assert os.path.exists(file_name)
        # print(file_name)
        im = cv2.imread(file_name, 0)
        im = cv2.GaussianBlur(im,(7,7),cv2.BORDER_DEFAULT)
        ret,im = cv2.threshold(im,127,255,cv2.THRESH_BINARY)
        plt.imshow(im, 'gray'),plt.show()

        plate_name = str(path.name)
        label_1 = one_hot_label(plate_name[3])

        image_1 = im[725:885, 48:148]
        plt.imshow(image_1, 'gray'),plt.show()
        temp_tuple = [np.array(image_1), label_1]
        all_dataset.append(tuple(temp_tuple))

        label_2 = one_hot_label(plate_name[4])
        image_2 = im[725:885, 149:249]
        plt.imshow(image_2, 'gray'),plt.show()
        temp_tuple = [np.array(image_2), label_2]
        all_dataset.append(tuple(temp_tuple))

        label_3 = one_hot_label(plate_name[5])
        image_3 = im[725:885, 351:451]
        plt.imshow(image_3, 'gray'),plt.show()
        temp_tuple = [np.array(image_3), label_3]
        all_dataset.append(tuple(temp_tuple))

        label_4 = one_hot_label(plate_name[6])
        image_4 = im[725:885, 452:552]
        plt.imshow(image_4, 'gray'),plt.show()
        temp_tuple = [np.array(image_4), label_4]
        all_dataset.append(tuple(temp_tuple))
    

# Genereate X and Y datasets
X_dataset_orig = np.array([data[0] for data in all_dataset[:]])
Y_dataset_orig = np.array([[data[1]] for data in all_dataset])

# X is examples, Y is labels for the examples
# Normalize X (images) dataset
X_dataset = (X_dataset_orig/255)
X_dataset = tf.expand_dims(X_dataset, axis=-1)

Y_dataset = convert_to_one_hot(Y_dataset_orig, NUMBER_OF_LABELS).T

# Model definition
file_name = '/home/fizzer/ros_ws/src/my_controller/node/plate_cnn'
assert os.path.exists(file_name)
conv_model = keras.models.load_model(file_name)

VALIDATION_SPLIT = .2 # used to validate dataset
history_conv = conv_model.fit(X_dataset, Y_dataset, 
                              validation_split=VALIDATION_SPLIT, 
                              epochs=80, # Epoch is one full iteration of the dataset
                              batch_size=16) 
                              # Every time you do a training set, how many examples from the data you take

conv_model.save('/home/fizzer/ros_ws/src/my_controller/node/plate_cnn')

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

