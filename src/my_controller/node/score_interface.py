#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Messages to the publisher must be in the form: 
#     str('TeamRed,multi21,4,XR58')
# To start the timer, we must use license plate location 0
# Similarly the last message your robot must publish must use license plate location -1.
rospy.init_node('topic_publisher')
pub = rospy.Publisher("license_plate", String, queue_size=1)
rate = rospy.Rate(2)

location = "0"
id = "0000"
tracked = "team11, team11" + ","+ location + "," + id
pub.publish(tracked)
rate.sleep()

while not rospy.is_shutdown():
   rate.sleep()

location = "-1"
tracked = "team11, team11" + ","+ location + "," + id
pub.publish(tracked)
rate.sleep()

