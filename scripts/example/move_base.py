#!/usr/bin/env python
import roslib
roslib.load_manifest('farscope_group_project')
import rospy
from geometry_msgs.msg import Twist
import sys

class BaseDriver:
    """Utility for moving the mobile base"""
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move(self, x_trans_in, y_trans_in=0.0, z_rot_in=0.0, duration_in=1.0):
        cmd = Twist()
        cmd.linear.x = x_trans_in
        cmd.linear.y = y_trans_in
        cmd.angular.z = z_rot_in
        rospy.loginfo('Sending dX={}, dY={}, dT={} for {}s'.format(x_trans_in,y_trans_in,z_rot_in,duration_in))
        r=rospy.Rate(10)
        for ii in range(int(10*duration_in)):
            r.sleep()
            self.pub.publish(cmd)
        rospy.loginfo('Stopping')
        cmd = Twist()
        for ii in range(10):
            r.sleep()
            self.pub.publish(cmd)

if __name__=='__main__':
    rospy.init_node('move_base', anonymous=True)
    if len(sys.argv)>=2:
        x_trans = float(sys.argv[1])
    else:
        x_trans = 0.0
    if len(sys.argv)>=3:
        y_trans = float(sys.argv[2])
    else:
        y_trans = 0.0
    if len(sys.argv)>=4:
        z_rot = float(sys.argv[3])
    else:
        z_rot = 0.0
    d = BaseDriver()
    d.move(x_trans, y_trans, z_rot)


