#! /usr/bin/env python3

from __future__ import print_function
import cv2
import sys
import rospy
import numpy as np
import roslib
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from numpy.linalg import inv
import tensorflow as tf
from tensorflow import keras
import time

## The constructor creates a subsciber that subscribes to the camera feed
#  The subscriber takes in a callback function as a parameter which 
#  invokes the callback function when a new image from the camera feed
#  is passed to the subscriber.
#
#  The camera feed to cv image is initialized
#  A publisher is initialized that publishes images to show what is being done 
class image_converter:
    def __init__(self):
        self.image = None
        self.bridge = CvBridge()
        
        self.pub = rospy.Publisher("license_plate", String, queue_size=1)
        time.sleep(1)
        
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image ,self.callback)
        self.rate = rospy.Rate(2)
        
        file_name = '/home/fizzer/ros_ws/src/my_controller/node/plate_cnn'
        assert os.path.exists(file_name)
        self.plate_model = keras.models.load_model(file_name)

        file_name = '/home/fizzer/ros_ws/src/my_controller/node/location_cnn'
        assert os.path.exists(file_name)
        self.location_model = keras.models.load_model(file_name)

    def publish_plate(self, plate_name):
        self.pub.publish("team11, team11,"+plate_name[1]+','+plate_name[3:])
        print(plate_name)
        time.sleep(1)

    def plate_hot_rev(self, index):
        # List to allow maping from character to row numbers
        one_hot_map = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890"
        return one_hot_map[index]

    # Finds plate numbers in data set 
    def find_plate(self, img, type):
        img_aug = np.expand_dims(img, axis=0)
        y_predict = self.plate_model.predict(img_aug)[0]
        if (type == 'a'):
            y_predict[26:]= [0 for y_val in y_predict[26:]] # Set 1234567890 to zero
        if (type == 'n'):
            y_predict[0:25]= [0 for y_val in y_predict[0:25]] # Set ABCDEFGHIJKLMNOPQRSTUVWXYZ to zero
        # print(y_predict)
        if (np.max(y_predict) < 0.4): 
            return -1
        return (self.plate_hot_rev(int(y_predict.argmax())))

    def location_hot_rev(self, index):
        one_hot_map = "12345678" # List to allow mapping from character to row numbers
        return one_hot_map[index]

    def find_location(self, img):     # Finds location in data set
        img_aug = np.expand_dims(img, axis=0)
        y_predict = self.location_model.predict(img_aug)[0]
        # print(y_predict)
        if (np.max(y_predict) < 0.4): 
            return -1
        return (self.location_hot_rev(int(y_predict.argmax())))
    
    def initalize_sift(self):
        print("here")
        file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
        assert os.path.exists(file_name1)
        img1 = cv2.imread(file_name1, 0)          # queryImage

    def count(self):
        # folder path
        dir_path = '/home/fizzer/ros_ws/src/my_controller/node/plate_img/'
        count = 0
        # Iterate directory
        for path in os.scandir(dir_path):
            if path.is_file():
                count += 1
        # print("Count is {}".format(count))
        return(count+1)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        img2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)    # trainImage
        status = cv2.imwrite('/home/fizzer/ros_ws/src/my_controller/node/plate_img/plate_{}.bmp'.format(self.count()), img2)
        
## The main function for running the image converter
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    self = image_converter()
    # start_time = self.start_system()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # self.stop_system()
        print("Shutting down")
        cv2.destroyAllWindows()

# Run main function when invoked
if __name__ == '__main__':
    main(sys.argv)
