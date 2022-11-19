#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import sys

# Messages to the publisher must be in the form: 
#     str('TeamRed,multi21,4,XR58')
# To start the timer, we must use license plate location 0
# Similarly the last message your robot must publish must use license plate location -1.

rospy.init_node('topic_publisher')
pub = rospy.Publisher("license_plate", String, queue_size=1)
rate = rospy.Rate(2)

location = "0"
tracked = "team11, team11,"+ location + ",0000"
started = False

rate.sleep()

while not rospy.is_shutdown():
   if not started:
      pub.publish(tracked)
      rate.sleep()
      started = True
      location = "-1"

   try:
      rospy.spin()
   except KeyboardInterrupt:
      tracked = "team11, team11,"+ location + ",0000"
      pub.publish(tracked)
      rate.sleep()

# class license_plate:   
#    def __init__(self):
#       # All are objects
#       self.license_pub = rospy.Publisher("license_plate", String, queue_size=1)

# def main(args):
#    rospy.init_node('license_plate', anonymous= True)
#    lp = license_plate()
#    lp.license_pub.publish('team11,team11,0,0000')
#    rate = rospy.Rate(2)
#    rate.sleep()

#    try:
#       rospy.spin()
#    except KeyboardInterrupt:
#       print("Shutting down")
#       lp.license_pub.publish('team11,team11,-1,0000')
#       rate.sleep()
#    cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)

