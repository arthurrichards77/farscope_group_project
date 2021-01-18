#!/usr/bin/env python
import roslib
roslib.load_manifest('farscope_group_proj')
import rospy
from std_msgs.msg import Float64
import sys

class GripperController:
  """Control the FARSCOPE simple gripper model"""
  def __init__(self):
    self.pub1 = rospy.Publisher('finger1_controller/command', Float64, queue_size=10)
    self.pub2 = rospy.Publisher('finger2_controller/command', Float64, queue_size=10)
    

  def move(self, cmd_in=0.0):
    rospy.loginfo('Sending {} to both finger controllers.'.format(cmd_in))
    r=rospy.Rate(10)
    for ii in range(5):
      r.sleep()
      self.pub1.publish(cmd_in)
      self.pub2.publish(cmd_in)

  def open(self):
    self.move(-0.04)

  def close(self):
    self.move(0.04)

if __name__=='__main__':
  rospy.init_node('move_gripper', anonymous=True)
  c = GripperController()
  if len(sys.argv)==2:
    cmd = float(sys.argv[1])
  else:
    cmd = 0.0
  c.move(cmd)



