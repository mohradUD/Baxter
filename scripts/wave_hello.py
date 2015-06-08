#!/usr/bin/env python

import rospy
import baxter_interface

from baxter_interface import CHECK_VERSION

pos1 = {'left_w0': -0.3068738935058595, 'left_w1': -1.4802914586181641, 'left_w2': 0.08858739039916992, 'left_e0': 0.21322332927246096, 'left_e1': 0.10661166463623048, 'left_s0': -0.5997864873779297, 'left_s1': -0.07209709694824219}


pos2 = {'left_w0': 0.3286553834289551, 'left_w1': -1.570796325, 'left_w2': -0.07708253450317383, 'left_e0': 0.42414568736572267, 'left_e1': 0.09894176070556641, 'left_s0': -0.6918253345458985, 'left_s1': -0.05138835633544922}

rospy.init_node('Wave')

rs = baxter_interface.RobotEnable(CHECK_VERSION)

ll = baxter_interface.Limb('left')

ll.set_joint_position_speed(1.0)

ll.move_to_neutral()

ll.move_to_joint_positions(pos1)
ll.move_to_joint_positions(pos2)
ll.move_to_joint_positions(pos1)
ll.move_to_neutral()







