#!/usr/bin/env python
import roslib
roslib.load_manifest('farscope_group_proj')
import rospy
from geometry_msgs.msg import Twist
import sys

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rospy.init_node('move_base', anonymous=True)

if len(sys.argv)>=2:
    x_trans = float(sys.argv[1])
else:
    x_trans = 0.0

if len(sys.argv)>=3:
    y_trans = float(sys.argv[2])
else:
    y_trans = 0.0

if len(sys.argv)>=3:
    z_rot = float(sys.argv[3])
else:
    z_rot = 0.0

cmd = Twist()
cmd.linear.x = x_trans
cmd.linear.y = y_trans
cmd.angular.z = z_rot

print('Sending dX={}, dY={}, dT={}'.format(x_trans,y_trans,z_rot))
r=rospy.Rate(10)
for ii in range(10):
  r.sleep()
  pub.publish(cmd)

print('Stopping')
cmd = Twist()
for ii in range(10):
  r.sleep()
  pub.publish(cmd)



