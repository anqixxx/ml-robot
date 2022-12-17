#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/R1/cmd_vel', Twist, 
  queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.1
move.angular.z = 0.0

while not rospy.is_shutdown():
   pub.publish(move)
   rate.sleep()
