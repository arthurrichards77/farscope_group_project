#!/usr/bin/env python
import sys
import roslib
import rospy
from math import pi
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient
roslib.load_manifest('farscope_group_proj')


class ArmMover:
    """Client to move the robot arm"""
    def __init__(self):
        self.client = SimpleActionClient("/arm_controller/follow_joint_trajectory",
                                         FollowJointTrajectoryAction)
        rospy.loginfo("waiting to connect...")
        self.client.wait_for_server()
        rospy.loginfo("connected! ")
        self.joint_names = ['shoulder_pan_joint',
                            'shoulder_lift_joint',
                            'elbow_joint',
                            'wrist_1_joint',
                            'wrist_2_joint',
                            'wrist_3_joint']

    def move(self, shoulder_lift_cmd_in=-pi/4.0, elbow_cmd_in=pi/2.0, duration_in=5.0):
        wrist_1_cmd = (-1.0)*(shoulder_lift_cmd_in + elbow_cmd_in)  # keep it level
        wrist_2_cmd = 0.5*pi  # straight along arm
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_names
        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, shoulder_lift_cmd_in, elbow_cmd_in, wrist_1_cmd, wrist_2_cmd, 0.0]
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p1.time_from_start = rospy.Duration(duration_in)
        g.trajectory.points.append(p1)
        self.client.send_goal(g)
        rospy.loginfo("Sent goal and waiting for arm")
        self.client.wait_for_result()
        rospy.loginfo("Arm move complete")


if __name__ == '__main__':
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
    m.move(shoulder_lift_cmd, elbow_cmd)
