#! /usr/bin/env python3

from __future__ import print_function
import cv2
import sys
import rospy
import roslib
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
        self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image ,self.sift)

    def sift(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        plt.imshow(gray)
        plt.show()

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
