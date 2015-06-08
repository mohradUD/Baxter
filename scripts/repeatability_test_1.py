#!/usr/bin/env python

# Repeatability Test: Setup #2
# - Different Baxter starting location from repeatability_test_0.py
# - Added new gripper commands for limiting moving and holding force
#
# Author: Michael Ohradzansky
# University of Dayton
# May 26, 2015

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import time

prep = {'left_w0': -0.33172334500122075, 'left_w1': 0.018407769433593752, 'left_w2': -0.07363107773437501, 'left_e0': 0.37007286465454103, 'left_e1': -0.051004861138916016, 'left_s0': -1.5017671896240234, 'left_s1': 0.027228158953857422}

start = {'left_w0': -0.11121360699462891, 'left_w1': -0.06672816419677735, 'left_w2': 0.09894176070556641, 'left_e0': 0.06212622183837891, 'left_e1': 1.5853691424682619,'left_s0': -0.7915340856445313, 'left_s1': 0.016106798254394532}

mid = {'left_w0': -0.17602429520874024, 'left_w1': 1.6731895424743652, 'left_w2': -0.09050486638183594, 'left_e0': 0.1940485694458008, 'left_e1': 0.20440293975219728, 'left_s0': -1.0906603389404297, 'left_s1': -0.30219421486816406}

repeatability_point = {'left_w0': -0.3984515091979981, 'left_w1': 1.1861506428771973, 'left_w2': -0.15454856420288088, 'left_e0': 0.5483981310424805, 'left_e1': -0.04985437554931641, 'left_s0': -1.1282428682006838, 'left_s1': 0.14074273712768556}

end = {'left_w0': -0.19059711267700197, 'left_w1': 0.5913495930541992, 'left_w2': -0.25157284892578125, 'left_e0': 0.38848063408813477, 'left_e1': 0.11619904454956055, 'left_s0': -1.098330242871094, 'left_s1': -0.2818689694519043}



rospy.init_node("RepeatabilityTest")

rs = baxter_interface.RobotEnable(CHECK_VERSION)

#Initialize the Arm and Gripper
left_limb_controller = baxter_interface.Limb('left')
left_limb_controller.set_joint_position_speed(.3)
left_gripper_controller = baxter_interface.Gripper('left')

#Move to a nuetral position and calibrate the gripper
left_limb_controller.move_to_neutral()
left_gripper_controller.calibrate()
left_gripper_controller.set_moving_force(20)
left_gripper_controller.set_holding_force(20)

#Move to the prep position and grab marker
left_limb_controller.move_to_joint_positions(prep)
left_gripper_controller.command_position(100)
time.sleep(1)
left_gripper_controller.close()
time.sleep(1)

#Move back to to neutral
left_limb_controller.move_to_neutral()

#Run the repeatability test
for _i in range(10):
   left_limb_controller.move_to_joint_positions(mid)
   left_limb_controller.move_to_joint_positions(repeatability_point)
   left_limb_controller.move_to_joint_positions(end)
   left_limb_controller.move_to_neutral()

time.sleep(1)
left_limb_controller.move_to_joint_positions(prep)
time.sleep(1)
left_gripper_controller.open()
time.sleep(.5)
left_limb_controller.move_to_neutral()



















