#!/usr/bin/env python
import roslib
roslib.load_manifest('farscope_group_proj')
import rospy
from std_msgs.msg import Float64
import sys

pub1 = rospy.Publisher('finger1_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('finger2_controller/command', Float64, queue_size=10)

rospy.init_node('move_gripper', anonymous=True)
r=rospy.Rate(10)
r.sleep()

if len(sys.argv)==2:
  cmd = float(sys.argv[1])
else:
  cmd = 0.0

rospy.loginfo('Sending {} to both finger controllers.'.format(cmd))

pub1.publish(cmd)
pub2.publish(cmd)



