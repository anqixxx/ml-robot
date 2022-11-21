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
        self.i = 414

    def count(self):
      # folder path
      dir_path =  r'E:/home/fizzer/ros_ws/src/my_controller/node/plate_img'
      count = 0
      # Iterate directory
      for path in os.scandir(dir_path):
          if path.is_file():
              count += 1
      print(count)
      return str(count)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        MIN_MATCH_COUNT = 3

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
            path = 'D:/home/fizzer/ros_ws/src/my_controller/node/plate_img/'
            status = cv2.imwrite('/home/fizzer/ros_ws/src/my_controller/node/plate_img/plate_{}.bmp'.format(self.i), img2)
            print("Image written to file-system : ",status)
            # cv2.imwrite(os.path.join(path , 'waka.bmp'), img2)
            # cv2.waitKey(0)  
            print("Match found, count is {} images".format(self.i))
            self.i = self.i+1

            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()
            h,w = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

            # sa = s.LinearAlign(img1)
            # plt.imshow(sa.align(img2))

        else:
            print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
            matchesMask = None
        
        # if len(good)> 1:
        #     draw_params = dict(matchColor = (0,255,0), # draw matches in green color
        #             singlePointColor = None,
        #             matchesMask = matchesMask, # draw only inliers
        #             flags = 2)
        #     img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
        #     plt.imshow(img3, 'gray'),plt.show()

        
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
