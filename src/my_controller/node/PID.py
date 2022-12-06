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
import numpy as np

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
    self.min = 1000000000
    self.count = 1
    self.preimerg = 0
    self.recordCx = True
    self.Cx = 0
    self.pedestDetected = False
    self.captureCount = 0
    self.reposition = False
    self.hillSection = False
    self.hillcounter = 0
    self.hillDone = False
    self.redLineDetected = False
    self.redlines = 0
    self.startdelay = False
    self.delay2 = 0
    self.twocross = False

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
    redLineDetected = False
    pedestDetected = False
    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    gblur = cv2.GaussianBlur(gray, (5,5), 0)
    ret,binary = cv2.threshold(gblur,75,255, cv2.THRESH_BINARY)

#Start Comment
    ret,binary235 = cv2.threshold(gblur,200,255, cv2.THRESH_BINARY)



    # kernel = np.ones((10, 10), np.uint8)
    # binary235 = cv2.dilate(binary23, kernel, iterations=2)
    # gt = binary235[650:,640:]
    # Mg = cv2.moments(gt)
    #cz = int(Mg["m10"]/Mg["m00"]) + 640
    #cr = int(Mg["m01"]/Mg["m00"]) + 650
    #binary2357 = cv2.circle(binary235,(cz,cr),15,(255,0,0),cv2.FILLED)

    #print(self.count)
    #self.count = self.count + 1
    #if (self.count > 250):
    #   self.count = 1
    # Calculate the center of mass of the image using Open cv moments


#End note

    #print(self.min)
    M = cv2.moments(binary)
    #print(M["m00"])
    cX = int(M["m10"]/M["m00"])
    cY = int(M["m01"]/M["m00"])
    Cy1 = cY
    # print(cY)
    newim = gblur[(cY):,:]
    newerim = newim[int(newim.shape[0]/2):, :]


    ret,bin3 = cv2.threshold(newim,85,255, cv2.THRESH_BINARY_INV)
    ret,bin4 = cv2.threshold(newim,80,255, cv2.THRESH_BINARY_INV)
    bin5 = cv2.subtract(bin3,bin4)
    M = cv2.moments(bin5)
    # print("Okay")
    print(M["m00"], "This is it, hopefully, faithfully -----------------------")
    cX = int(M["m10"]/M["m00"])
    cY = int(M["m01"]/M["m00"]) + Cy1

    ret,bin9 = cv2.threshold(newerim,31,255, cv2.THRESH_BINARY_INV)
    ret,bin10 = cv2.threshold(newerim,28,255, cv2.THRESH_BINARY_INV)
    bin11 = cv2.subtract(bin9,bin10)
    M2 = cv2.moments(bin11)

    cXX = 0
    cYY = 0

    ret,bin37 = cv2.threshold(gblur,180,255, cv2.THRESH_BINARY_INV)#120
    ret,bin47 = cv2.threshold(gblur,120,255, cv2.THRESH_BINARY_INV)
    bin57 = cv2.subtract(bin37,bin47)
    M7 = cv2.moments(bin57)

    #ret,bin97 = cv2.threshold(gblur,180,255, cv2.THRESH_BINARY_INV)
    #ret,bin98 = cv2.threshold(gblur,110,255, cv2.THRESH_BINARY_INV)
    #bin99 = cv2.subtract(bin97,bin98)
    #M9 = cv2.moments(bin99)
    
    cX7 = int(M7["m10"]/M7["m00"])
    cY7 = int(M7["m01"]/M7["m00"])
    #cX9 = int(M9["m10"]/M9["m00"])
    #cY9 = int(M9["m01"]/M9["m00"])

    if (self.startdelay):
      if (self.delay2 < 500):
        self.delay2 = self.delay2 + 1

    if (self.delay2 > 200):
      if (self.twocross):
        self.redlines = self.redlines + 1
        self.twocross = False
        self.startdelay = False
      print("We start now now now now now now now now")


    if(M2["m00"] != 0):
      cXX = int(M2["m10"]/M2["m00"])
      cYY = int(M2["m01"]/M2["m00"]) + Cy1 + int(newim.shape[0]/2)
      if(M2["m00"] > 2000000 and cXX > 550 and cXX < 750):
        redLineDetected = True
        self.redLineDetected = True
        if(self.recordCx):
          print("abcdefghijklmnopqrstuvwxyz")
          self.recordCx = False
          self.Cx = cX
    
    if(self.redLineDetected and not(self.pedestDetected)): ##changed
      imerg = gblur[320:550,self.Cx-125:self.Cx+125]
      print(type(self.preimerg))
      if(type(self.preimerg) == int):
        self.preimerg = imerg
        self.captureCount = self.captureCount + 1
      else:
        ret,bin91 = cv2.threshold(imerg,100,255, cv2.THRESH_BINARY)
        ret,bin101 = cv2.threshold(self.preimerg,100,255, cv2.THRESH_BINARY)
        new_imerg = cv2.subtract(bin91,bin101)
        print(imerg.shape, "lololololololololol")
        print(self.preimerg.shape, "rtwwewewerwerwrwerew")
        self.preimerg = imerg
        M3 = cv2.moments(new_imerg)
        self.captureCount = self.captureCount + 1
        print(M3["m00"], "yeeeeeeeeeeet!")
        if(self.reposition and self.captureCount > 4 and M3["m00"]>100000 and not(self.twocross)):
          print("Pedestrian foundddddddddddddd!")
          self.pedestDetected = True
    
    if(self.twocross):
      self.redLineDetected = False


    if(self.pedestDetected):
      print("Let's gooooooooooooooo1312123123")

    #if(M["m00"] < self.min):
    #  self.min = M["m00"]
    #self.min = self.min + 1
    #print("current min", self.min)
    # start at 100
    # Draw a circle at the center of mass coordinates for checking functionality
    final1 = cv2.circle(cv_image,(cXX,cYY),15,(255,0,0),cv2.FILLED)
    final = cv2.circle(final1,(self.Cx,cY),15,(255,0,0),cv2.FILLED)
    #final = cv2.circle(final,(cX9,cY9),15,(255,0,0),cv2.FILLED)

    #final = cv2.rectangle(final2,(cX-125,320),(cX+125,550),(255,0,0),cv2.FILLED)

    # Line following robot movement control.
    # If the x centre of mass is lower than the lthresh,
    # the robot will turn to the left until the centre of mass is in the center.
    # The same happens when the x center of mass is greater than the rthersh, but
    # this time the robot turns to the right
    # Otherwise when the x center of mass is in the middle range, the robot moves forward.
    # The ythresh is used to ensure that distant noise (paths) do not alter the robots direction
    lthresh = 600
    rthresh = 680
    ythresh = 650#700
    print(final.shape)
    print(cX, cY, "CM!")
    if(self.redLineDetected and not(self.reposition) and self.redlines % 2 == 0): ##changed
      if (cX7 > 445):
          self.move.linear.x = 0
          self.move.angular.z = -0.05
          self.image_pub.publish(self.move)
          print("Repositioning")
      elif (cX7 < 444):
          self.move.linear.x = 0
          self.move.angular.z = 0.05
          self.image_pub.publish(self.move)
          print("Repositioning")
      else:  
          self.move.linear.x = 0
          self.move.angular.z = 0
          self.image_pub.publish(self.move)
          self.reposition = True
          print("Repositioning")
    elif(self.redLineDetected and not(self.reposition) and self.redlines % 2 != 0): ##changed
      if (cX > 650):
          self.move.linear.x = 0
          self.move.angular.z = -0.05
          self.image_pub.publish(self.move)
          print("Repositioning Part 2 --------:", self.redlines)
      elif (cX < 630):
          self.move.linear.x = 0
          self.move.angular.z = 0.05
          self.image_pub.publish(self.move)
          print("Repositioning Part 2 --------:", self.redlines)
      else:  
          self.move.linear.x = 0
          self.move.angular.z = 0
          self.image_pub.publish(self.move)
          self.reposition = True
          print("Repositioning Part 2 --------:", self.redlines)
    elif(self.redLineDetected and not(self.pedestDetected)): ##changed
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.image_pub.publish(self.move)
        print("Red Line Detected")
    elif(self.pedestDetected and self.redlines%2 == 0):
      if (cY7 > 400):
        self.move.linear.x = 0
        self.move.angular.z = 0
        self.image_pub.publish(self.move)
        final = binary235
        self.hillSection = True
        self.pedestDetected = False
        self.redLineDetected = False
        self.redlines = self.redlines + 1
        print("We crossed!!!!!!", cY7)
      else:
        self.move.linear.x = 0.3
        self.move.angular.z = 0
        self.image_pub.publish(self.move)
        print("Pedest Detected - move move move!!!!!", cY7)
    elif(self.pedestDetected and self.redlines%2 != 0):
      self.pedestDetected = False
      self.redLineDetected = False
      self.twocross = True
      self.startdelay = True
      self.delay2 = self.delay2 + 1
      print("checked -------------------------------------------- checked", self.redlines)
    elif(self.hillSection and not(self.hillDone)):
      final = cv2.Canny(binary235, 200,250)
      cty = final[700, 640:]
      g = []
      self.hillcounter = self.hillcounter + 1
      for i in range(len(cty)):
        if (cty[i] > 0):
          g.append(i + 640)
      if (len(g)> 0):
        vp = sum(g)/len(g)
        final = cv2.circle(final1,(int(vp),718),10,(255,0,0),cv2.FILLED)
      else:
        vp = -1

      if (self.hillcounter > 500):
        if (sum(bin5[-1,:]) > 150000):
          self.hillDone = True
          self.reposition = False
          self.pedestDetected = False
      if(vp != -1 and vp < 1000):
        self.move.linear.x = 0
        self.move.angular.z = 0.5
        self.image_pub.publish(self.move)
        print("We are now pushing through the hill section!", vp)
      elif(vp != -1 and vp > 1050):
        self.move.linear.x = 0
        self.move.angular.z = -0.25
        self.image_pub.publish(self.move)
        print("We are now pushing through the hill section!", vp)
      else:
        self.move.linear.x = 0.2
        self.move.angular.z = 0.10
        self.image_pub.publish(self.move)
        print("We are now pushing through the hill section!", vp)
    else:
      print("ohhhhhh yeahahahahaahahahha", cX7, "y:", cY7)
      if(cX < lthresh or cY > ythresh):
          self.move.linear.x = 0
          self.move.angular.z = 1#0.5
          self.image_pub.publish(self.move)
          print("move left")
      elif(cX > rthresh):
          self.move.linear.x = 0
          self.move.angular.z = -1#-0.5 #neg
          self.image_pub.publish(self.move)
          print("move right")
      else:
          self.move.linear.x = 0.2#0.075
          self.move.angular.z = 0
          self.image_pub.publish(self.move)
          print("forward")

    #Display camera image with center of mass dot to check functionality
    #cv2.imshow("Image", new_imerg)
    #print(cYY, "The info you wnat", cXX, "yee",M2["m00"])

    cv2.imshow("Image window", bin5)
    cv2.waitKey(3)

## The main function for running the image converter to robot movement
#
#  @param command line arguments
#  function initializes node and sets up image converter.
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    # print("yay!")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

# Run main function when invoked
if __name__ == '__main__':
    main(sys.argv)