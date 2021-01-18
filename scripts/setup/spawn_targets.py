#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

rospy.init_node("spawn_targets")

print('waiting for service')
rospy.wait_for_service('/gazebo/spawn_urdf_model')
print('service ready')
spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

shelf_heights = [0.06, 0.43, 0.81, 1.21]

scenario = rospy.get_param('scenario')

target_urdf = rospy.get_param('target_description')

service_spec = """
string model_name
string model_xml
string robot_namespace
geometry_msgs/Pose initial_pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
string reference_frame
---
bool success
string status_message
"""

for shelf in scenario:
    for trophy in shelf['trophies']:
        rospy.loginfo('Target at {},{},{}'.format(shelf['location']['x'],shelf['location']['y'],shelf_heights[trophy]))
        model_name = 'target_{}_{}'.format(shelf['id'],trophy)
        initial_pose = Pose()
	initial_pose.position.x = shelf['location']['x']
	initial_pose.position.y = shelf['location']['y']
	initial_pose.position.z = shelf_heights[trophy]
        resp = spawn_urdf(model_name, target_urdf, "", initial_pose, "")
        rospy.loginfo(resp.status_message)
	
