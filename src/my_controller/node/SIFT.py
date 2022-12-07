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
        # Can I create a new thing to publish to?
        self.image = None
        # self.image_pub = rospy.Publisher("/R1/pi_camera/image_raw/theora", Image, queue_size = 10)
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
        # MIN_MATCH_COUNT = 9
        # #12
        # # 22 seems to always hold, but however we may not be able to always see 22 points
        # file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
        # assert os.path.exists(file_name1)

        # img1 = cv2.imread(file_name1, 0)          # queryImage
        # img2 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)    # trainImage

        # # Initiate SIFT detector
        # sift = cv2.SIFT_create()
        # # Gets keypoints and descriptors of key frame image
        # kp1, des1 = sift.detectAndCompute(img1,None)
        # kp2, des2 = sift.detectAndCompute(img2,None)
        # FLANN_INDEX_KDTREE = 1
        # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        # search_params = dict(checks = 50)
        # flann = cv2.FlannBasedMatcher(index_params, search_params)
        # matches = flann.knnMatch(des1,des2,k=2)    # store all the good matches as per Lowe's ratio test.
        # good = []
        # # plt.imshow(img2, 'gray'),plt.show()

        # for m,n in matches:
        #     if m.distance < 0.7*n.distance:
        #         good.append(m)

        # if len(good)>MIN_MATCH_COUNT:
        #     status = cv2.imwrite('/home/fizzer/ros_ws/src/my_controller/node/plate_img/plate_{}.bmp'.format(self.count()), img2)
        #     print("Image written to file-system : ",status)
        #     print("Match found, count is {} images".format(self.count()))
        # else:
        #     print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        #     matchesMask = None

    # def image_processing(self):
    #     file_name = '/home/fizzer/ros_ws/src/my_controller/node/plate_cnn'
    #     assert os.path.exists(file_name)
    #     plate_model = keras.models.load_model(file_name)

    #     file_name = '/home/fizzer/ros_ws/src/my_controller/node/location_cnn'
    #     assert os.path.exists(file_name)
    #     location_model = keras.models.load_model(file_name)

    #     file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
    #     assert os.path.exists(file_name1)
    #     # Change file name 2 to read from folder
    #     file_name2 = os.path.join(os.path.dirname(__file__), 'plate_img/plate_546.bmp')
    #     # file_name2 = os.path.join(os.path.dirname(__file__), 'pte_img_3.jpg')
    #     assert os.path.exists(file_name2)

    #     img1 = cv2.imread(file_name1, 0)          # queryImage
    #     img2 = cv2.imread(file_name2, 0)      # trainImage, ONLY FOR TESTING
    #     img3 = None
    #     plt.imshow(img1, 'gray'),plt.show()
    #     plt.imshow(img2, 'gray'),plt.show()

    #     # Initiate SIFT detector
    #     sift = cv2.SIFT_create()
    #     # Gets keypoints and descriptors of key frame image
    #     kp1, des1 = sift.detectAndCompute(img1,None)
    #     kp2, des2 = sift.detectAndCompute(img2,None)
    #     FLANN_INDEX_KDTREE = 1
    #     index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    #     search_params = dict(checks = 50)
    #     flann = cv2.FlannBasedMatcher(index_params, search_params)
    #     matches = flann.knnMatch(des1,des2,k=2)
    #     # store all the good matches as per Lowe's ratio test.
    #     good = []
    #     MIN_MATCH_COUNT = 12
    #     for m,n in matches:
    #         if m.distance < 0.7*n.distance:
    #             good.append(m)

    #     print(len(good))
    #     if len(good)>MIN_MATCH_COUNT:
    #         src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    #         dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    #         M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            
    #         if M is not None:
    #             invM = inv(M)
    #             # Take matrix (M) and find the matrix inverse and apply to the image
    #             h,w = img1.shape
    #             pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    #             dst = cv2.perspectiveTransform(pts,M)
    #             invM = np.linalg.inv(M)
    #             img2 = cv2.warpPerspective(img2, invM, (w, h))
    #             plt.imshow(img2, 'gray'),plt.show()

    #     if img3 is not None:
    #         plt.imshow(img3, 'gray'),plt.show()
    #         ret,img3 = cv2.threshold(img3,65,255,cv2.THRESH_BINARY)
    #         plt.imshow(img3, 'gray'),plt.show()
    #         img3 = img3/255

    #         plate_name = 'P'

    #         # Find location
    #         # location = img3[564:368, 848:535]
    #         location = img3[369:522, 569:840]
    #         location = cv2.resize(location, (220,302))
    #         plt.imshow(location, 'gray'),plt.show()
    #         plate_name += str(self.find_location(tf.expand_dims(location, axis=-1)))
    #         plate_name += '_'
            
    #         # Find plate
    #         # plate_1 = img3.crop((247, 661, 362, 747))
    #         # y from math
    #         plate_1 = img3[660:744, 250:362]
    #         plate_1 = cv2.resize(plate_1, (100,160))
    #         plt.imshow(plate_1, 'gray'),plt.show()
    #         plate_name += str(self.find_plate(tf.expand_dims(plate_1, axis=-1), type='a'))


    #         # plate_2 = img3.crop((326, 661, 478, 747))
    #         plate_2 = img3[660:744, 362:475]
    #         plate_2 = cv2.resize(plate_2, (100,160))
    #         plt.imshow(plate_2, 'gray'),plt.show()
    #         plate_name += str(self.find_plate(tf.expand_dims(plate_2, axis=-1), type='a'))

    #         # plate_3 = img3.crop((590, 661, 701, 747))
    #         plate_3 = img3[660:744, 586:698]
    #         plate_3 = cv2.resize(plate_3, (100,160))
    #         plt.imshow(plate_3, 'gray'),plt.show()
    #         plate_name += str(self.find_plate(tf.expand_dims(plate_3, axis=-1), type='n'))

    #         # plate_4 = img3.crop((701, 661, 817, 747))
    #         plate_4 = img3[660:744, 698:811]
    #         plate_4 = cv2.resize(plate_4, (100,160))
    #         plate_name += str(self.find_plate(tf.expand_dims(plate_4, axis=-1), type='n'))
    #         # self.publish_plate(plate_name)
    #         if '-1' not in plate_name:
    #             self.publish_plate(plate_name)
    #         else:
    #             print(plate_name)


        
        
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
