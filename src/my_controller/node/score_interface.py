#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
import time

rospy.init_node('topic_publisher')
pub = rospy.Publisher("license_plate", String, queue_size=1)
time.sleep(1)

pub.publish("team11, team11,0,0000")
print("Timer started at {} for Team 11".format(rospy.get_rostime()))
rospy.sleep(239)

pub.publish("team11, team11,-1,0000")
print("Timer ended at {} for Team 11".format(rospy.get_rostime()))
time.sleep(1)

