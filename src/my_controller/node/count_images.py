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

# folder path
dir_path = '/home/fizzer/ros_ws/src/my_controller/node/plate_img/'
count = 0
# Iterate directory
for path in os.scandir(dir_path):
    if path.is_file():
        count += 1
print(count)
