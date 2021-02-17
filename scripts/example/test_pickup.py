#!/usr/bin/env python
import rospy
from farscope_group_project.farscope_robot_utils import ArmMover, BaseDriver, GripperController

rospy.init_node("test_pickup")

base_driver = BaseDriver()
arm_mover = ArmMover()
gripper_controller = GripperController()

# open gripper ready
gripper_controller.open()
# lift arm to what seems to be right height
arm_mover.move(shoulder_lift_cmd_in=-0.6, elbow_cmd_in=1.0)
# drive on to the target
base_driver.move(0.3, -0.1)
base_driver.move(0.3, -0.05)
# grab it
gripper_controller.close()
# lift it off shelf
arm_mover.move(shoulder_lift_cmd_in=-0.7, elbow_cmd_in=1.0)
# back away
base_driver.move(-0.3, 0, 0, 2)
# turn base to get target over the receptacle
base_driver.move(0, 0, 0.2, 6)
# forward a bit
base_driver.move(0.1)
# release gripper
gripper_controller.open()
# little nudge to drop it
base_driver.move(-0.5, 0, 0, 0.1)
