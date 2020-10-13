#!/usr/bin/env python
import roslib
roslib.load_manifest('farscope_group_proj')
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient

rospy.init_node("simple_traj")
client = SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

print "waiting to connect..."
client.wait_for_server()
print "connected! "

g = FollowJointTrajectoryGoal()
g.trajectory = JointTrajectory()
g.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

p2 = JointTrajectoryPoint()
p2.positions = [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]
p2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p2.time_from_start = rospy.Duration(5.0)
g.trajectory.points.append(p2)

p3 = JointTrajectoryPoint()
p3.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p3.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p3.time_from_start = rospy.Duration(10.0)
g.trajectory.points.append(p3)

client.send_goal(g)
print "sent the goal"
print "waiting to get there"
client.wait_for_result()
print "got there"

