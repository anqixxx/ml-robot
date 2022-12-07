#! /usr/bin/env python3

from __future__ import print_function
import cv2
import sys
import rospy
import numpy as np
from silx.image import sift as s
import roslib
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from numpy.linalg import inv

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
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image ,self.callback)
        self.rate = rospy.Rate(2)

    def count(self):
        # folder path
        dir_path = '/home/fizzer/ros_ws/src/my_controller/node/plate_img/'
        count = 0
        # Iterate directory
        for path in os.scandir(dir_path):
            if path.is_file():
                count += 1
        print("Count is {}".format(count))
        return(count+1)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        MIN_MATCH_COUNT = 7

        file_name1 = os.path.join(os.path.dirname(__file__), 'p_image.jpg')
        assert os.path.exists(file_name1)

        img1 = cv2.imread(file_name1, 0)          # queryImage
        img2 = cv_image    # trainImage

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
        
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

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

        if img3 is not None:
            # Find location
            # in openCV: cropped = img[start_row:end_row, start_col:end_col]
            # location = img3[367:535, 560:848]
            # plt.imshow(location, 'gray'),plt.show()

            # # Find plate, proper cuts
            # plate_1 = img3[661:747, 247:362]
            # plt.imshow(plate_1, 'gray'),plt.show()

            # plate_2 = img3[661:747, 362:478]
            # plt.imshow(plate_2, 'gray'),plt.show()

            # plate_3 = img3[661:747, 583:698]
            # plt.imshow(plate_3, 'gray'),plt.show()

            # plate_4 = img3[661:747, 698:813]
            # plt.imshow(plate_4, 'gray'),plt.show()
            location = img3[367:535, 560:848]
            plt.imshow(location, 'gray'),plt.show()

            # Find plate
            plate_1 = img3[661:747, 247:362]
            plt.imshow(plate_1, 'gray'),plt.show()

            plate_2 = img3[661:747, 362:478]
            plt.imshow(plate_2, 'gray'),plt.show()

            plate_3 = img3[661:747, 583:698]
            plt.imshow(plate_3, 'gray'),plt.show()

            plate_4 = img3[661:747, 698:813]
            plt.imshow(plate_4, 'gray'),plt.show()

        else:
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
            matchesMask = None
        
        
## The main function for running the image converter
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# Run main function when invoked
if __name__ == '__main__':
    main(sys.argv)
