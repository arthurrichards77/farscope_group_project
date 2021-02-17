#!/usr/bin/env python3
import sys
import rospy
from math import pi
from farscope_group_project.farscope_robot_utils import ArmMover

rospy.init_node("move_arm")
m = ArmMover()
if len(sys.argv) >= 2:
    shoulder_lift_cmd = float(sys.argv[1])
else:
    shoulder_lift_cmd = -pi/4.0
if len(sys.argv) >= 3:
    elbow_cmd = float(sys.argv[2])
else:
    elbow_cmd = pi/2.0
rospy.loginfo('Target shoulder={} elbow={}'.format(shoulder_lift_cmd,
                                                   elbow_cmd))
m.move(shoulder_lift_cmd, elbow_cmd)
