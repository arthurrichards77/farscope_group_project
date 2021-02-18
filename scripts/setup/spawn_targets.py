#!/usr/bin/env python
"""
Script to spawn the target 'trophies' in
the Gazebo environment.  Requires the trophy
URDF to be loaded in a parameter.

Optional random omission of spawning and
additive placement error along shelf
"""
from random import uniform, random
from math import sin, cos
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

rospy.init_node("spawn_targets")

rospy.loginfo('Waiting for Gazebo spawn service')
rospy.wait_for_service('/gazebo/spawn_urdf_model')
rospy.loginfo('Service ready')
spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

shelf_heights = [0.06, 0.43, 0.81, 1.21]

scenario = rospy.get_param('scenario')

random_noise = rospy.get_param('random_noise', False)
if random_noise:
    rospy.loginfo('Random spawning noise ON')
else:
    rospy.loginfo('Random spawning noise OFF')

random_omit = rospy.get_param('random_omit', False)
if random_omit:
    rospy.loginfo('Random spawning omissions ON')
else:
    rospy.loginfo('Random spawning omissions OFF')

target_urdf = rospy.get_param('target_description')

for shelf in scenario:
    for trophy in shelf['trophies']:
        x_pos = shelf['location']['x']
        y_pos = shelf['location']['y']
        shelf_angle = shelf['location']['ang']
        z_pos = shelf_heights[trophy]
        rospy.loginfo('Target at {},{},{}'.format(x_pos, y_pos, z_pos))
        model_name = 'target_{}_{}'.format(shelf['id'], trophy)
        if random_noise:
            along_shelf_error = uniform(-0.35, 0.35)
            x_pos = x_pos + along_shelf_error*cos(shelf_angle)
            y_pos = y_pos + along_shelf_error*sin(shelf_angle)
        omit_flag = False
        if random_omit:
            prob_omit = 0.2
            if random() < prob_omit:
                omit_flag = True
        if not omit_flag:
            initial_pose = Pose()
            initial_pose.position.x = x_pos
            initial_pose.position.y = y_pos
            initial_pose.position.z = z_pos
            resp = spawn_urdf(model_name, target_urdf, "", initial_pose, "")
            #rospy.loginfo(resp.status_message)

rospy.loginfo('Target spawning complete')
rospy.set_param('target_spawning_complete', True)
