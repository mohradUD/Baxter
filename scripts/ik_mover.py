#!/usr/bin/python

import rospy
import time
import baxter_interface

import argparse
import struct
import sys

from copy import copy

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
		print("SUCCESS - Valid Joint Solution Found:")
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return limb_joints
	else:
		print("INVALID POSE - No Valid Joint Solution Found.")
	return 0


def main():
    #Set up the K
    limb_choice = 'left'
    xa = 0.8
    ya = 0.0
    za = 0.1
    
    qxa = 0.0
    qya = 1.0
    qza = 0.0
    qwa = 0.05253324362213531

    rospy.init_node('iktest')
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit\n")
    ll = baxter_interface.Limb('left')

    #print("Left Arm Current Joint Angles:\n")
    #print ll.joint_angles()
    #print "\n"
    
    new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
   
    #print("Desired Joint Angles from IK Solution:\n")
    #print new_angles
    #print "\n"
    ll.move_to_neutral()
    ll.set_joint_positions(new_angles)
    time.sleep(5)
    ranger = 30

    for x in range(ranger):
        ya = ya + .01
        new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
        ll.move_to_joint_positions(new_angles)
    time.sleep(2)

    for x in range(ranger):
        ya = ya - .01
        new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
        ll.move_to_joint_positions(new_angles)
    time.sleep(2)

    for x in range(ranger):
        xa = xa - .01
        new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
        ll.set_joint_positions(new_angles)
    time.sleep(2)

    for x in range(ranger):
        xa = xa + .01
        new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
        ll.set_joint_positions(new_angles)
    time.sleep(2)

    for x in range(ranger):
        za = za + .01
        new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
        if new_angles != 0:
            ll.set_joint_positions(new_angles)
    time.sleep(2)

    for x in range(ranger):
        za = za - .01
        new_angles = get_ik_solution_angles(limb_choice,xa,ya,za,qxa,qya,qza,qwa)
        if new_angles != 0:
            ll.set_joint_positions(new_angles)


    
    time.sleep(5)
    ll.move_to_neutral()
    rs.reset()

if __name__ == "__main__":
    main()


















































