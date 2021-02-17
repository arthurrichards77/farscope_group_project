import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import pi
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient


class GripperController:
    """Control the FARSCOPE simple gripper model"""
    def __init__(self):
        self.pub1 = rospy.Publisher('finger1_controller/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('finger2_controller/command', Float64, queue_size=10)

    def move(self, cmd_in=0.0):
        rospy.loginfo('Sending {} to both finger controllers.'.format(cmd_in))
        r = rospy.Rate(10)
        for ii in range(5):
            r.sleep()
            self.pub1.publish(cmd_in)
            self.pub2.publish(cmd_in)

    def open(self):
        self.move(-0.04)

    def close(self):
        self.move(0.04)


class BaseDriver:
    """Utility for moving the mobile base"""
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def move(self, x_trans_in, y_trans_in=0.0, z_rot_in=0.0, duration_in=1.0):
        cmd = Twist()
        cmd.linear.x = x_trans_in
        cmd.linear.y = y_trans_in
        cmd.angular.z = z_rot_in
        rospy.loginfo('Sending dX={}, dY={}, dT={} for {}s'.format(x_trans_in,
                                                                   y_trans_in,
                                                                   z_rot_in,
                                                                   duration_in))
        r = rospy.Rate(10)
        for ii in range(int(10*duration_in)):
            r.sleep()
            self.pub.publish(cmd)
        rospy.loginfo('Stopping')
        cmd = Twist()
        for ii in range(10):
            r.sleep()
            self.pub.publish(cmd)


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

    def move(self,
             shoulder_lift_cmd_in=-pi/4.0,
             elbow_cmd_in=pi/2.0,
             duration_in=5.0):
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
