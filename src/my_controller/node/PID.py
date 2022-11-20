#!/usr/bin/env python3

## @package enph353_ros_lab
#  Line following script documentation.
#
#  The following script implements line following for the robot in the in the ROS environment. 
#  The image_converter class can be used to publish robot steering commands from analysis of
#  the camera feed it is subscribed too.

from __future__ import print_function
import cv2
import sys
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

## Image converter to robot movement for line following
#
#  Class is used to intake camera images from the robot, convert to open cv format
#  condition image, determine the center of mass, and publish information to control robot movement
#  using the center of mass.  
class image_converter:

  ## The constructor creates a subsciber that subscribes to the camera feed
  #  The subscriber takes in a callback function as a parameter which 
  #  invokes the callback function when a new image from the camera feed
  #  is passed to the subscriber.
  #  The camera feed to cv image and the robot movement are initialized
  #  A publisher is initialized that publishes information on the robot movement
  #  in order to move the robot.
  def __init__(self):
    self.image_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size = 1)
    self.move = Twist()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

  ## The callback function first converts the image to a CV image format
  #  This image is then grayscaled, gaussian blurred, and then thresholded into a binary map
  #  The center of mass of the binary map is calculated to provide the location of the path
  #  Based on the x and y coordinates of the center of mass, the robot will adjust itself accordingly
  #  @param self The object pointer
  #  @param data The data from the camera feed
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Image cleaning pipeline: Grayscale, gaussian blur image, convert to a binary map
    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    gblur = cv2.GaussianBlur(gray, (5,5), 0)
    ret,binary = cv2.threshold(gblur,75,255, cv2.THRESH_BINARY)
    
        
    # Calculate the center of mass of the image using Open cv moments
    M = cv2.moments(binary)
    cX = int(M["m10"]/M["m00"])
    cY = int(M["m01"]/M["m00"])
    Cy1 = cY
    print(cY)
    newim = gblur[(cY):,:]
    ret,bin3 = cv2.threshold(newim,85,255, cv2.THRESH_BINARY_INV)
    ret,bin4 = cv2.threshold(newim,80,255, cv2.THRESH_BINARY_INV)
    bin5 = cv2.subtract(bin3,bin4)
    M = cv2.moments(bin5)
    print("Okay")
    print(M["m00"])
    cX = int(M["m10"]/M["m00"])
    cY = int(M["m01"]/M["m00"]) + Cy1
    #start at 100
    # Draw a circle at the center of mass coordinates for checking functionality
    final = cv2.circle(cv_image,(cX,cY),15,(255,0,0),cv2.FILLED)
    
    # Line following robot movement control.
    # If the x centre of mass is lower than the lthresh,
    # the robot will turn to the left until the centre of mass is in the center.
    # The same happens when the x center of mass is greater than the rthersh, but
    # this time the robot turns to the right
    # Otherwise when the x center of mass is in the middle range, the robot moves forward.
    # The ythresh is used to ensure that distant noise (paths) do not alter the robots direction
    lthresh = 600
    rthresh = 680
    ythresh = 700
    print(final.shape)
    print(cX, cY, "CM!")
    if(cX < lthresh or cY > ythresh):
        self.move.linear.x = 0
        self.move.angular.z = 0.5
        self.image_pub.publish(self.move)
        print("111111")
    elif(cX > rthresh):
        self.move.linear.x = 0
        self.move.angular.z = -0.5 #neg
        self.image_pub.publish(self.move)
        print("2222222222")
    else:
        self.move.linear.x = 0.075
        self.move.angular.z = 0
        self.image_pub.publish(self.move)
        print("3333333333")

    #Display camera image with center of mass dot to check functionality
    cv2.imshow("Image", bin5)
    #print(cY)
    #cv2.imshow("Image window", final)
    cv2.waitKey(3)

## The main function for running the image converter to robot movement
#
#  @param command line arguments
#  function initializes node and sets up image converter.
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    print("yay!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# Run main function when invoked
if __name__ == '__main__':
    main(sys.argv)