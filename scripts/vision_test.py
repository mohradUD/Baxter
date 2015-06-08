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

def get_ik_solution_angles(limb,Px,Py,Pz,Qx,Qy,Qz,Qw):
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poses = {
	'left': PoseStamped(
		header=hdr,
		pose=Pose(
			position=Point(
				x = Px,
				y = Py,
				z = Pz,
			),
			orientation=Quaternion(
				x = Qx,
				y = Qy,
				z = Qz,
				w = Qw,
			),
		),
	),
	'right': PoseStamped(
		header=hdr,
		pose=Pose(
			position=Point(
				x=Px,
				y=Py,
				z=Pz,
			),
			orientation=Quaternion(
				x=Qx,
				y=Qy, 
				z=Qz,
				w=Qw,
			),
		),
	),
	}
	ikreq.pose_stamp.append(poses[limb])
	try:
		rospy.wait_for_service(ns, 5.0)
		resp = iksvc(ikreq)
	except (rospy.ServiceException, rospy.ROSException), e:
		rospy.logerr("Service call failed: %s" % (e,))
		return 1
	if (resp.isValid[0]):
		# print("SUCCESS - Valid Joint Solution Found:")
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	else:
		print("INVALID POSE - No Valid Joint Solution Found.")
	return 0


rl_camera_pos = {'right_s0': 0.5602864821350098, 'right_s1': -1.1171215075012209, 'right_w0': 0.4498398655334473, 'right_w1': 0.9418642026855469, 'right_w2': -0.49585928911743166, 'right_e0': 0.019174759826660157, 'right_e1': 1.504451655999756}

#Set up the K
limb_choice = 'left'
xa = 0.8
ya = 0.0
za = 0.1
    
qxa = 0.0
qya = 1.0
qza = 0.0
qwa = 0.05

# Node Initialization
rospy.init_node("VisonTest")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
if ~rs._state.enabled:
	rs.enable()

# Arm and Gripper Setup
ll = baxter_interface.Limb('left')
rl = baxter_interface.Limb('right')
lg = baxter_interface.Gripper('left')

# Move the right camera into position
rl.move_to_neutral()
rl.move_to_joint_positions(rl_camera_pos)

# Determine initial left arm pose
new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
ll.move_to_joint_positions(new_angles)

with open('/home/mike/Documents/MATLAB/RR/xy_coords.csv','rb') as csvfile:
    baxter_io = csv.reader(csvfile, delimiter=',',quotechar='"')
    xy = baxter_io.next()

while(1):
    with open('/home/mike/Documents/MATLAB/RR/xy_coords.csv','rb') as csvfile:
        baxter_io = csv.reader(csvfile, delimiter=',',quotechar='"')
        xy = baxter_io.next()
    xa = float(xy[0])
    ya = float(xy[1])

    new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
    if new_angles != 0:
        ll.move_to_joint_positions(new_angles)




    









