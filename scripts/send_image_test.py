#!/usr/bin/env python

# Send Image Test Script
# Author: Michael Ohradzansky
# University of Dayton
# June 3, 2015

import rospy
import time
import os
import sys
import argparse
import cv2
import cv_bridge

from sensor_msgs.msg import Image
import baxter_interface

from baxter_interface import CHECK_VERSION

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)

# Node Initialization
rospy.init_node("Send_image_test")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
if ~rs._state.enabled:
	rs.enable()

send_image('/home/mike/ros_ws/src/baxter_examples/share/images/baxterworking.png')




