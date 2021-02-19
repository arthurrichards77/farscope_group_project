#!/usr/bin/env python
"""
Script to spawn the target 'trophies' in
the Gazebo environment.  Requires the trophy
URDF to be loaded in a parameter.

Optional random omission of spawning and
additive placement error along shelf
"""
from random import uniform, random
from math import sin, cos, fabs
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

rospy.init_node("spawn_targets")

rospy.loginfo('Waiting for Gazebo spawn service')
rospy.wait_for_service('/gazebo/spawn_urdf_model')
rospy.loginfo('Service ready')
spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

shelf_heights = rospy.get_param('shelf_heights')

scenario = rospy.get_param('scenario')

placement_error = fabs(rospy.get_param('placement_error', 0.0))
rospy.loginfo('Random placement error up to {}'.format(placement_error))

prob_random_omit = rospy.get_param('prob_random_omit', 0.0)
rospy.loginfo('Random spawning omission probability {}'.format(prob_random_omit))

prob_random_duplicate = rospy.get_param('prob_random_duplicate', 0.0)
rospy.loginfo('Random spawning duplication probability {}'.format(prob_random_duplicate))

target_urdf = rospy.get_param('target_description')

for shelf in scenario:
    for trophy in shelf['trophies']:
        x_pos = shelf['location']['x']
        y_pos = shelf['location']['y']
        shelf_angle = shelf['location']['ang']
        z_pos = shelf_heights[trophy]
        rospy.loginfo('Target at {},{},{}'.format(x_pos, y_pos, z_pos))
        model_name = 'target_{}_{}'.format(shelf['id'], trophy)
        # add random placement error
        along_shelf_error = uniform(-placement_error, placement_error)
        x_pos = x_pos + along_shelf_error*cos(shelf_angle)
        y_pos = y_pos + along_shelf_error*sin(shelf_angle)
        # check for random omission
        if random() >= prob_random_omit:
            initial_pose = Pose()
            initial_pose.position.x = x_pos
            initial_pose.position.y = y_pos
            initial_pose.position.z = z_pos
            resp = spawn_urdf(model_name, target_urdf, "", initial_pose, "")
            # random duplication
            if random() < prob_random_duplicate:
                dup_offset = 0.3
                if along_shelf_error > 0.0:
                    dup_along_shelf = (-dup_offset)
                else:
                    dup_along_shelf = dup_offset
                dup_x_pos = x_pos + dup_along_shelf*cos(shelf_angle)
                dup_y_pos = y_pos + dup_along_shelf*sin(shelf_angle)
                initial_pose.position.x = dup_x_pos
                initial_pose.position.y = dup_y_pos
                dup_model_name = model_name + '_dup'
                resp = spawn_urdf(dup_model_name, target_urdf, "", initial_pose, "")

rospy.loginfo('Target spawning complete')
rospy.set_param('target_spawning_complete', True)
