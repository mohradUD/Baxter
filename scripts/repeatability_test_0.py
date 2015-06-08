#!/usr/bin/env python

# Author: Michael Ohradzansky
# University of Dayton
# May 17, 2015

import rospy
import baxter_interface
import time

prep = {'left_w0': -0.002300971179199219, 'left_w1': 0.9917185782348633, 'left_w2': -0.20248546376953128, 'left_e0': 0.17410681922607424, 'left_e1': -0.04947088035278321, 'left_s0': -1.2302525904785158, 'left_s1': -0.16336895372314456}

start = {'left_w0': -0.11121360699462891, 
 'left_w1': -0.06672816419677735, 
 'left_w2': 0.09894176070556641, 
 'left_e0': 0.06212622183837891, 
 'left_e1': 1.5853691424682619, 
 'left_s0': -0.7915340856445313, 
 'left_s1': 0.016106798254394532}

mid = {'left_w0': -0.045635928387451175, 
 'left_w1': 1.4822089346008303, 
 'left_w2': 0.05829126987304688, 
 'left_e0': 0.032213596508789064, 
 'left_e1': 0.7715923354248048, 
 'left_s0': -0.8137768070434571, 
 'left_s1': -0.8030389415405274}

repeatability_point = {'left_w0': -0.40343694675292974, 'left_w1': 1.302349687426758, 'left_w2': 0.06250971703491211, 'left_e0': 0.35665053277587894, 'left_e1': 0.06519418341064454, 'left_s0': -0.975995275177002, 'left_s1': 0.060592241052246094}

rospy.init_node("RepeatabilityTest")

#Initialize the Arm and Gripper
left_limb_controller = baxter_interface.Limb('left')
left_gripper_controller = baxter_interface.Gripper('left')

#Move to a nuetral position and calibrate the gripper
left_limb_controller.move_to_neutral()
left_gripper_controller.calibrate()

left_limb_controller.move_to_joint_positions(prep)
left_gripper_controller.command_position(100)

time.sleep(2)
x = 1
while (left_gripper_controller.force() < 10):
   left_gripper_controller.command_position(100-x)
   x = x + 1

print(left_gripper_controller.force())

time.sleep(2)
left_limb_controller.move_to_neutral()
time.sleep(1)

for _i in range(10):
   time.sleep(.5)
   left_limb_controller.move_to_joint_positions(mid)
   left_limb_controller.move_to_joint_positions(repeatability_point)
   time.sleep(.5)
   left_limb_controller.move_to_joint_positions(mid)
   left_limb_controller.move_to_neutral()

time.sleep(1)
left_limb_controller.move_to_neutral()













