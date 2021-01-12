#!/usr/bin/env python
import sys
import roslib
roslib.load_manifest('farscope_group_proj')
import rospy
from math import pi
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient

if len(sys.argv)>=2:
  shoulder_lift_cmd = float(sys.argv[1])
else:
  shoulder_lift_cmd = 0.0

if len(sys.argv)>=3:
  elbow_cmd = float(sys.argv[2])
else:
  elbow_cmd = 0.0

wrist_1_cmd = (-1)*(shoulder_lift_cmd + elbow_cmd)

wrist_2_cmd = 0.5*pi

rospy.init_node("move_arm")
client = SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

print "waiting to connect..."
client.wait_for_server()
print "connected! "

g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
g.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

p1 = JointTrajectoryPoint()
p1.positions = [0.0, shoulder_lift_cmd, elbow_cmd, wrist_1_cmd, wrist_2_cmd, 0.0]
p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p1.time_from_start = rospy.Duration(5.0)
g.trajectory.points.append(p1)

print(g.trajectory.joint_names)
print(p1.positions)

client.send_goal(g)
print "sent the goal"
print "waiting to get there"
client.wait_for_result()
print "got there"

