#!/usr/bin/env python

# Test script to connect vision, Baxter, and MatLab
# Author: Michael Ohradzansky
# University of Dayton
# June 1, 2015

import rospy
import baxter_interface
import time
import csv
from copy import copy

import argparse
import struct
import sys

import actionlib

#From ik service client
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from baxter_interface import CHECK_VERSION

rl_camera_pos = {'right_s0': 0.5602864821350098, 'right_s1': -1.1171215075012209, 'right_w0': 0.4498398655334473, 'right_w1': 0.9418642026855469, 'right_w2': -0.49585928911743166, 'right_e0': 0.019174759826660157, 'right_e1': 1.504451655999756}

rl_camera_measure_point = {'right_s0': 1.1658253974609376, 'right_s1': -0.7585534987426759, 'right_w0': 0.417242773828125, 'right_w1': 0.8835729328125, 'right_w2': -0.17985924717407228, 'right_e0': -0.3670049030822754, 'right_e1': 1.2770390044555664}


# Node Initialization
rospy.init_node("VisonTest")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
if ~rs._state.enabled:
	rs.enable()

# Arm and Gripper Setup
ll = baxter_interface.Limb('left')
rl = baxter_interface.Limb('right')
lg = baxter_interface.Gripper('left')

# Move right hands camera to point measuring location
rl.move_to_neutral()
rl.move_to_joint_positions(rl_camera_measure_point)
raw_input("press enter when finished measuring...\r\n")

# Move the right camera into position
rl.move_to_joint_positions(rl_camera_pos)

# Move camera





























