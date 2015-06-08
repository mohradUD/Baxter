#!/usr/bin/env python

# Test script to connect vision, Baxter, and MatLab
# Ball tracking
# Author: Michael Ohradzansky
# University of Dayton
# June 5, 2015

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

start_pos = {'left_w0': -0.12770390044555666, 'left_w1': 1.1198059738769532, 'left_w2': -0.48128647164916993, 'left_e0': 0.18752915110473634, 'left_e1': 1.030451593084717, 'left_s0': -1.259398225415039, 'left_s1': -0.6086068768981934}


#Set up the K
limb_choice = 'left'
xa = 0.8
ya = 0.0
za = 0.1
    
qxa = 0.0
qya = 1.0
qza = 0.0
qwa = 0.05

step = .05
# Node Initialization
rospy.init_node("BallTrackingTest")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
if ~rs._state.enabled:
	rs.enable()

# Arm and Gripper Setup
ll = baxter_interface.Limb('left')
lg = baxter_interface.Gripper('left')
ll.move_to_joint_positions(start_pos)

ll.set_joint_position_speed(1)

# Determine initial left arm pose
new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
ll.move_to_joint_positions(new_angles)

new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
if new_angles != 0:
    print('Moving to a new position:')
    ll.move_to_joint_positions(new_angles)
    print('Move Complete')
rospy.sleep(5)
print('Moving')

while(1):
    with open('/home/mike/Documents/MATLAB/RR/direction_output.csv','rb') as csvfile:
        baxter_io = csv.reader(csvfile, delimiter=',',quotechar='"')
        xy_commands = baxter_io.next()
        x_down = int(xy_commands[0])
        x_up = int(xy_commands[1])
        y_down = int(xy_commands[2])
        y_up = int(xy_commands[3])
    if (1):
        # Decrease x
        if x_down == 1:
            xa = xa - step
            print('X step down')
        if x_up == 1:
            xa = xa + step
            print('X step up')
        if y_down == 1:
            ya = ya - step
            print('Y step down')
        if y_up == 1:
            ya = ya + step
            print('X step up')

    new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
    if new_angles != 0:
        print('Moving to a new position:')
        ll.move_to_joint_positions(new_angles, threshold=.05)
        print('Move Complete')












