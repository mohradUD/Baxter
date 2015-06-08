#!/usr/bin/env python


# Import/Export Test Script
# Author: Michael Ohradzansky
# University of Dayton
# May 26, 2015

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import time
import csv

left_joints = {}
right_joints = {}
length = 8

rospy.init_node('test')

rs = baxter_interface.RobotEnable(CHECK_VERSION)

ll = baxter_interface.Limb('left')
lg = baxter_interface.Gripper('left')

with open('IO/baxter_io.csv', 'rb') as csvfile:
    baxter_io = csv.reader(csvfile, delimiter=',', quotechar='"')
    joint_names = baxter_io.next()
    left_angles = baxter_io.next()
    right_angles = baxter_io.next()

    for x in range(length-1):
        left_joints.update({left_angles[0]+'_'+joint_names[x+1]:float(left_angles[x+1])})
        right_joints.update({right_angles[0]+'_'+joint_names[x+1]:float(right_angles[x+1])})

print left_joints
print right_joints
print ll.joint_angles()
ll.move_to_joint_positions(left_joints)

ll.move_to_neutral()
rs.reset()

#for x in range(length-1):
#    left_joints_object.update({left_joints[x]:left_angles_values[x]})










    
    
