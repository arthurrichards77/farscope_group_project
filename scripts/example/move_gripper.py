#!/usr/bin/env python3
import rospy
import sys
from farscope_group_project.farscope_robot_utils import GripperController

rospy.init_node('move_gripper', anonymous=True)
c = GripperController()
if len(sys.argv) == 2:
    cmd = float(sys.argv[1])
else:
    cmd = 0.0
c.move(cmd)
