#! /usr/bin/env python3

from __future__ import print_function
import cv2
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  ## The constructor creates a subsciber that subscribes to the camera feed
  #  The subscriber takes in a callback function as a parameter which 
  #  invokes the callback function when a new image from the camera feed
  #  is passed to the subscriber.
  #  The camera feed to cv image and the robot movement are initialized
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image ,self.sift)
    rate = rospy.Rate(2)

  ## The callback function first converts the image to a CV image format
  #  This image is then grayscaled, gaussian blurred, and then thresholded into a binary map
  #  The center of mass of the binary map is calculated to provide the location of the path
  #  @param self The object pointer
  #  @param data The data from the camera feed
  def sift(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Image cleaning pipeline: Grayscale, gaussian blur image, convert to a binary map
    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    plt.imshow(gray)
    plt.show()
    cv2.imwrite('opencv_rgb.png', gray)

    grayInverseImage = ~gray
    gblur = cv2.GaussianBlur(gray, (5,5), 0)
    ret,binary = cv2.threshold(gblur,127,255, cv2.THRESH_BINARY_INV)
    bw = cv2.threshold(grayInverseImage, 147, 255, cv2.THRESH_BINARY)[1]


  # Source: stackoverflow.com/questions/34232632/
  def SLOT_query_camera(self, cv_img):
    # Feature match
    index_params = dict(algorithm=0, trees=5)
    search_params = dict()

    # Loads the flann algorythm to find the matching features
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Now detect the features and descriptions of the frame from the image  
    # then compare it to the query image
    grayframe = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)  # trainimage

    # Gets keypoints and descriptors of key frame image
    kp_grayframe, desc_grayframe = self.sift.detectAndCompute(grayframe, None)
    matches = flann.knnMatch(self.desc_image, desc_grayframe, k=2)
    good_points = []
    
    for m, n in matches:
        if m.distance < 0.6 * n.distance:
            good_points.append(m)
    
    if len(good_points) > 4:
        # Obtains matrix
        query_pts = np.float32([self.kp_image[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

        # Perspective transform using points and matrixs
        # Makes it so we can obtain the actual image without viewpoint constraints
        h, w = cv_img.shape
        pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, matrix)

        homography = cv2.polylines(cv_img, [np.int32(dst)], True, (255, 0, 0), 3)

        # cv2.imshow("Homography", homography)
        pixmap = QtGui.QPixmap( self.convert_cv_to_pixmap(homography))
    else:
        pixmap = QtGui.QPixmap(self. convert_cv_to_pixmap(cv_img))

    self.live_image_label.setPixmap(pixmap)


## The main function for running the image converter to robot movement
#
#  @param command line arguments
#  function initializes node and sets up image converter.
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# Run main function when invoked
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    sys.exit(app.exec_())
