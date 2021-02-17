#!/usr/bin/env python3
import rospy
import sys
from farscope_group_project.farscope_robot_utils import BaseDriver

rospy.init_node('move_base', anonymous=True)
if len(sys.argv) >= 2:
    x_trans = float(sys.argv[1])
else:
    x_trans = 0.0
if len(sys.argv) >= 3:
    y_trans = float(sys.argv[2])
else:
    y_trans = 0.0
if len(sys.argv) >= 4:
    z_rot = float(sys.argv[3])
else:
    z_rot = 0.0
d = BaseDriver()
d.move(x_trans, y_trans, z_rot)
