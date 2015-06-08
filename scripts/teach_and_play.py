#!/usr/bin/env python

# Baxter Battery Pickup Script
# Author: Michael Ohradzansky
# University of Dayton
# May 25, 2015

import rospy
import baxter_interface
import time

lstep1 = {'left_w0': -0.09127185677490235, 'left_w1': 0.8149272926330567, 'left_w2': 1.2417574463745118, 'left_e0': 0.03451456768798828, 'left_e1': 1.381733193109131, 'left_s0': -1.100631214050293, 'left_s1': -0.6741845555053712}

lstep2 = {'left_w0': -0.12962137642822266, 'left_w1': 0.6649806707885743, 'left_w2': 1.1765632629638674, 'left_e0': 0.169888372064209, 'left_e1': 1.281640946813965, 'left_s0': -1.251728321484375, 'left_s1': -0.40727189871826175}

lstep3 = {'left_w0': -0.2542573153015137, 'left_w1': 1.578082733734131, 'left_w2': 0.983665179107666, 'left_e0': 0.28532042622070314, 'left_e1': 0.2542573153015137, 'left_s0': -1.4768400018493653, 'left_s1': -0.23968449783325196}

lstep4 = {'left_w0': -0.18829614149780274, 'left_w1': 1.385568145074463, 'left_w2': 0.6902913537597657, 'left_e0': 0.22472818516845705, 'left_e1': -0.03374757729492188, 'left_s0': -1.514039035913086, 'left_s1': 0.19}


lstep5 = {'left_w0': -1.0097428524719239, 'left_w1': 1.6302380804626466, 'left_w2': 0.5767767755859375, 'left_e0': 1.074170045489502, 'left_e1': -0.042184471618652346, 'left_s0': -1.5447186516357423, 'left_s1': -0.05752427947998047}

lstep6 = {'left_w0': -0.12962137642822266, 'left_w1': 0.6649806707885743, 'left_w2': 1.1765632629638674, 'left_e0': 0.169888372064209, 'left_e1': 1.281640946813965, 'left_s0': -1.251728321484375, 'left_s1': -0.35027189871826175}


rospy.init_node("Teach_and_Play_batteries")

ll = baxter_interface.Limb('left')
lg = baxter_interface.Gripper('left')

rl = baxter_interface.Limb('right')
rg = baxter_interface.Gripper('right')

ll.move_to_neutral()
lg.calibrate()
lg.command_position(100)
lg.set_moving_force(25)
lg.set_holding_force(25)

rg.calibrate()
rg.command_position(100)

ll.move_to_joint_positions(lstep1)
ll.move_to_joint_positions(lstep2)
time.sleep(.5)
lg.close()
time.sleep(.5)
print (lg.force())

ll.move_to_joint_positions(lstep3)

ll.move_to_joint_positions(lstep4)
lg.open()
time.sleep(.5)
ll.move_to_joint_positions(lstep3)
#time.sleep(1)
ll.move_to_neutral()

ll.move_to_joint_positions(lstep3)
ll.move_to_joint_positions(lstep4)
lg.close()
time.sleep(.5)
ll.move_to_joint_positions(lstep3)
ll.move_to_joint_positions(lstep2)
lg.open()
time.sleep(.5)
ll.move_to_joint_positions(lstep1)
ll.move_to_neutral()



























































