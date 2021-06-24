# farscope_group_project

This [ROS](https://www.ros.org/) package provides a simulation of the picking challenge for the 1st year group project in the [FARSCOPE](https://www.farscope.bris.ac.uk/) Centre for Doctoral Training at 
[Bristol Robotics Laboratory](http://www.brl.ac.uk).  It includes:
* a simulated environment with targets to be picked from shelves
* a model of FARSCOPE's mobile manipulator robot, the [MMO-700 from Neobotix](https://www.neobotix-robots.com/products/mobile-manipulators/mobile-manipulator-mmo-700),
  comprising a [UR10 arm](https://www.universal-robots.com/products/ur10-robot/) on an [omnidirectional mobile base](https://www.neobotix-robots.com/products/mobile-robots/mobiler-roboter-mpo-700).
* a simple gripper model
* a simple camera model
* example control scripts and launch files
The simulation is implemented in [Gazebo](http://gazebosim.org/).

## Getting Started

### Dependencies

The package was developed in ROS Melodic using Gazebo 9.0.0 on Ubuntu 18.04.  It has also been tested on ROS Noetic.  It requires the following packages:
* [neo_simulation](https://github.com/neobotix/neo_simulation) : simulation of the MMO-700 from Neobotix
* [universal_robot](https://github.com/ros-industrial/universal_robot) : the UR10 support from [ROS Industrial](https://rosindustrial.org/)
* [joint_state_publisher_gui](https://wiki.ros.org/joint_state_publisher) : the GUI for generating fake joint states, needed for visualizing the robot
* [ros_controllers](http://wiki.ros.org/ros_controllers) : standard controllers for the simulated robot

> Note: there is also an indirect dependency on MoveIt, a ROS motion planner which is required by the `universal_robot` package.  You might end up using it, or you might not, so options for a workaround or install MoveIt are given below.

> Note: there are different ROS packages for the UR10 arm depending on what firmware is installed.  This simulation does not guarantee compatibility with the real
> FARSCOPE arms as that has yet to be tested.

> Note: this package is relatively simple but has not been tested with other versions of ROS and Gazebo.

### Installation and Testing

* If you don't have it already, install ROS Melodic and set up a workspace using [these instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* Install the `joint_state_publisher_gui` using `sudo apt install ros-melodic-joint-state-publisher-gui`.  (_Replace `melodic` with your distribution if required._)
* Install `ros_controllers` using `sudo apt install ros-melodic-ros-controllers`.
* Clone this package, [neo_simulation](https://github.com/neobotix/neo_simulation) and [universal_robot](https://github.com/ros-industrial/universal_robot) into the workspace `src` directory.
* Navigate up to the root directory of your ROS workspace (`cd ..` from `src`) and run `catkin_make`.
> If you get an error saying "moveit_core" not found, _either_ re-run as `catkin_make -DCATKIN_BLACKLIST_PACKAGES="ur_kinematics"` _or_ install the missing component using `sudo apt install ros-melodic-moveit` (or another distro instead of melodic).
* Run `roslaunch farscope_group_project farscope_example_robot_visualize.launch`.  You should see an RViz visualization of the robot.
![RViz screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2014-11-59.png)
* Run `roslaunch farscope_group_project farscope_example_robot_simulate.launch`.  You should see a Gazebo simulation of the robot picking a target of a shelf and dropping it.
![Gazebo screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2014-10-59.png)
* Run `roslaunch farscope_group_project farscope_example_robot_simulate.launch use_gui:=false use_rviz:=true`.  Now Gazebo will run headless (_i.e._ no graphics front end) but you can see some of what's happening in RViz, including the LIDAR, the camera views, and the robot pose.
![RViz screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2010-05-52.png)

## Basic Usage

This section describes the ROS interface started by the [`launch/example_robot/farscope_example_robot_simulate.launch`](launch/example_robot/farscope_example_robot_simulate.launch) file.  Unless otherwise stated, these are all published or subscribed to by the Gazebo node, as a result of [plugins enabled in the `robot_description` URDF](http://gazebosim.org/tutorials?tut=ros_gzplugins).

> Also included in that file is the node [`scripts/example/test_pickup.py`](scripts/example/test_pickup.py) which shows examples of how to use the control interface.

![rqt_graph screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/rosgraph.png)

> Note: there are other topics available in the `\gazebo` namespace that are not shown in the diagram above.  *These must not be used as part of your final submission.  They contain direct information feeds from the simulator and are essentially cheating the problem.  You can, however, use them for development, if it serves a purpose.*

### Publishes

(incomplete list)

* _lidar_scan_ : [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html) : output of the front-mounted LIDAR on the mobile base
* _camera1/image_raw_ : [sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) : output of the forearm camera
* _camera2/image_raw_ : [sensor_msgs/Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) : output of the upper arm camera
* _joint_states_ : [sensor_msgs/JointState](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) : states of all robot joints, including arm and gripper
* _odom_ : [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) : odometry from the mobile robot base

### Subscribes

(incomplete list)

* _cmd_vel_ : [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) : command to move robot base
* _finger1_controller/command_ : [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) : position command for gripper finger 1
* _finger2_controller/command_ : [std_msgs/Float64](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float64.html) : position command for gripper finger 2

### Actions 

See [here](http://wiki.ros.org/actionlib/Tutorials) for information on using ROS actions

* _arm_controller/follow_joint_trajectory_ : [control_msgs/FollowJointTrajectoryAction](http://docs.ros.org/en/electric/api/control_msgs/html/msg/FollowJointTrajectoryAction.html) : controls the movement of the UR10 arm

### Parameters

* _robot_description_ : the URDF model of the robot
* _target_description_ : URDF model of an individual pick-up target
* _scenario_ : data on the locations of the targets in the world (see [`scenario_all.yaml`](scenarios/scenario_all.yaml)  for an example of the internal format)

## Customizing your Robot

The robot is represented in [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf), generated using [Xacro](http://wiki.ros.org/xacro) XML macros for flexibility.  The URDF includes [`<gazebo>` tags](http://gazebosim.org/tutorials/?tut=ros_urdf) to encode robot actuation and sensing.  You are provided with a fully implemented model of a suitable robot, including a simple two finger gripper, two cameras, and a scanning LIDAR sensor on a mobile base.  Your options for customizing the robot include:
* Using the integrated example robot provided in [`models/example_robot/farscope_example_robot.urdf.xacro`](models/example_robot/farscope_example_robot.urdf.xacro) _as is_
* Making your own modified version of [`models/example_robot/farscope_example_robot.urdf.xacro`](models/example_robot/farscope_example_robot.urdf.xacro) to move, add or remove cameras
* Replacing the simple gripper with something of your own design including custom CAD (see [URDF tutorials](http://wiki.ros.org/urdf/Tutorials)) and actuation (_warning_: custom actuation is difficult and requires learning [ros_control](http://wiki.ros.org/ros_control))
* Making a complete new robot from scratch (_not recommended_: lots of ROS detail to learn)

## Package Contents

```
package.xml     :  defines package data such as dependencies and authorship, for ROS
CMakeLists.txt  :  used by the catkin_make 
setup.py        :  defines the Python modules that are shared from this package
aws_notes.md    :  incomplete discussions on getting this working on AWS Robomaker

./worlds:
farscope_test_2.world  :  tells Gazebo where to put the shelves and cones

./src/farscope_group_project:
farscope_robot_utils.py  :  Python library providing interfaces for the arm, base and gripper (hiding the ROS away, optionally)

./models/example_robot:
farscope_example_robot.urdf.xacro  :  robot model combining the mobile base, UR10 arm, and gripper, to be used as example for customization

./models/mobile_arm:
mobile_arm.urdf.xacro  :  provides a xacro 'macro' to include the mobile manipulator in other URDF/xacro models

./models/camera:
simple_camera.urdf.xacro  :  xacro macro for including camera (physical model and gazebo functionality) in other URDF/xacro models

./models/target:
target.urdf  :  model of the target or "trophy"
target.stl   :  CAD mesh of the trophy

./models/gripper:
simple_gripper.urdf.xacro  :  xacro macro for the simple gripper, to be included in integrated models

./scripts/example:
move_gripper.py    :  utility script for manual operation of the gripper
move_base.py       :  utility script for manual operation of the base
move_arm.py        :  utility script for manual operation of the base
test_pickup.py     :  example of an integrated robot controller that does just one pickup in very simplistic, open-loop way

./scripts/setup:
spawn_targets.py   :  script that spawns targets, including randomization, to set up the challenge scenario

./controller:
gripper.yaml       :  definition of the controllers used in Gazebo to simulate feedback control of the simple gripper

./scenarios:        :  these files tell the spawn_targets.py script where to put trophies and with what level of randomization
typical.yaml        :  default scenario with random misplacements, omissions and duplicates
all_no_random.yaml  :  scenario with one target placed exactly in the centre of every shelf

./launch/common:
ur10_controllers.launch    :  loads the parameters for the UR10 controller, as simulated by the Gazebo plug-ins
gripper_controller.launch  :  loads the parameters for the gripper controller

./launch/example_robot:
farscope_example_robot_simulate.launch         :  simulates the example robot and the test_pickup.py example controller
simulate.rviz                                  :  config for RViz that shows sensor outputs
farscope_example_robot_visualize.launch        :  displays the example robot model in RViz and lets you manipulate its joints
visualize.rviz                                 :  config for RViz to show robot model
farscope_example_robot_control.launch          :  (for AWS use) just the robot controller parts of the farscope_example_robot_simulate
farscope_example_robot_simulator_only.launch   :  (for AWS use) just the simulator parts of the farscope_example_robot_simulate

./launch/challenge:
farscope_group_challenge_gazebo.launch         :  standard parts of all challenge simulations, including target spawning and Gazebo with the shelves and cones 
```

## Limitations and Disclaimer

The goal of the group project is to experience integration of a robotic system, not to perform research-grade innovation in any individual subsystem.  Therefore the challenge has been subtly modified to simplify some elements.  In particular, the target object has been made easy to grasp, with a handy lip at the top to avoid the need for friction gripping.  Also, it has been made easy to detect, with a contrasting colour and symmetrical appearance.

Physics simulation is hard.  Even with Gazebo, it would take considerable tuning to represent realistic grasping, contact and friction.  Be ready for some odd behaviour:
* The robot gently rotates even while it is commanded to stay still
* If you drive the robot into a shelf, the robot will probably flip itself over, but the shelf will be undamaged
* A target may spontaneously jump from your grasp
* A target may mysteriously cling to you when released
These are things you will just have to work around in the pursuit of your project.
