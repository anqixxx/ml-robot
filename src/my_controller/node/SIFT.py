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

  # https://docs.opencv.org/3.4/da/df5/tutorial_py_sift_intro.html


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
  main(sys.argv)
