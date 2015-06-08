#!/usr/bin/env python

# Baxter Battery Removal Script
# Author: Michael Ohradzansky
# University of Dayton
# May 25, 2015

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

via_point1 = {'left_w0': -0.7930680664306641, 'left_w1': 1.0377380018188478, 'left_w2': -1.196888508380127, 'left_e0': 0.12348545328369141, 'left_e1': 0.9940195494140626, 'left_s0': -0.7919175808410646, 'left_s1': -0.4229952017761231}

via_point2 = {'left_w0': -1.1301603441833497, 'left_w1': 1.2152962778137208, 'left_w2': -1.1539370463684082, 'left_e0': 0.08858739039916992, 'left_e1': 0.5414952175048828, 'left_s0': -0.7263399022338868, 'left_s1': 0.01112136069946289}

pickup_point = {'left_w0': -1.1205729642700195, 'left_w1': 1.1435826760620118, 'left_w2': -1.1263253922180176, 'left_e0': 0.08666991441650392, 'left_e1': 0.5487816262390137, 'left_s0': -0.7723593258178711, 'left_s1': 0.016106798254394532}

pull_out_point = {'left_w0': -1.081839949420166, 'left_w1': 1.2225826865478517, 'left_w2': -1.0990972332641602, 'left_e0': 0.0851359336303711, 'left_e1': 0.5541505589904786, 'left_s0': -0.6925923249389649, 'left_s1': 0.0122718462890625}

via_point3 = {'left_w0': -1.245975893536377, 'left_w1': 1.3318788175598146, 'left_w2': -1.1550875319580078, 'left_e0': 0.10162622708129883, 'left_e1': 0.3459126672729492, 'left_s0': -0.6822379546325684, 'left_s1': 0.14342720350341798}

pickup_point2 = {'left_w0': -1.2709030813110351, 'left_w1': 1.2570972542358398, 'left_w2': -1.0473253817321777, 'left_e0': 0.09932525590209962, 'left_e1': 0.4291311249206543, 'left_s0': -0.6772525170776368, 'left_s1': 0.15378157380981447}

via_point4 = {'left_w0': -0.9238399284484864, 'left_w1': 1.4407914533752442, 'left_w2': -1.1201894690734864, 'left_e0': 0.11543205415649414, 'left_e1': 0.16758740088500979, 'left_s0': -0.5886651266784668, 'left_s1': 0.13690778516235352}

via_point5 = {'left_w0': -0.8237476821533204, 'left_w1': 1.821602183532715, 'left_w2': -1.5922720560058594, 'left_e0': 0.11466506376342774, 'left_e1': 0.12770390044555666, 'left_s0': -0.5622039581176759, 'left_s1': -0.22357769957885743}

via_point6 = {'left_w0': -1.311937067340088, 'left_w1': 0.9311263371826173, 'left_w2': -0.6243301799560548, 'left_e0': 0.027228158953857422, 'left_e1': 1.9109565643249513, 'left_s0': -0.6722670795227051, 'left_s1': -0.6772525170776368}

dropoff_point = {'left_w0': -2.271058553869629, 'left_w1': 0.7562525275634766, 'left_w2': 0.04640291878051758, 'left_e0': -0.031063110919189455, 'left_e1': 1.968864339001465, 'left_s0': -0.979830227142334, 'left_s1': -0.43104860090332037}

end_point = {'left_w0': -2.2626216595458986, 'left_w1': 0.9038981782287598, 'left_w2': -0.027611654150390626, 'left_e0': -0.3512816000244141, 'left_e1': 1.9224614202209473, 'left_s0': -1.0055244053100587, 'left_s1': -0.7213544646789551}

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
rospy.init_node("Teach_and_Play_Battery_Removal")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
send_image('/home/mike/ros_ws/src/baxter_examples/share/images/SafeImage.png')
time.sleep(5)

if ~rs._state.enabled:
	rs.enable()

send_image('/home/mike/ros_ws/src/baxter_examples/share/images/WorkingImage.png')

# Arm and Gripper Setup
ll = baxter_interface.Limb('left')
lg = baxter_interface.Gripper('left')

ll.move_to_neutral()
lg.calibrate()
lg.open()
lg.set_moving_force(25)
lg.set_holding_force(27)

# Start the battery removal
ll.move_to_joint_positions(via_point1)
ll.move_to_joint_positions(via_point2)
ll.move_to_joint_positions(pickup_point)
#time.sleep(.25)
lg.close()
time.sleep(.25)
ll.move_to_joint_positions(pull_out_point)
lg.open()
#time.sleep(.5)
ll.move_to_joint_positions(via_point3)
ll.move_to_joint_positions(pickup_point2)
#time.sleep(.5)
lg.close()
time.sleep(.25)
ll.move_to_joint_positions(via_point4)
ll.move_to_joint_positions(via_point5)
ll.move_to_joint_positions(via_point6)
ll.move_to_joint_positions(dropoff_point)
lg.open()
time.sleep(.5)
ll.move_to_joint_positions(end_point)
ll.move_to_neutral()
rs.reset()
send_image('/home/mike/ros_ws/src/baxter_examples/share/images/FinishedImage.png')
time.sleep(5)
















