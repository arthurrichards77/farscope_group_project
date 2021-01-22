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

> Note: there are different ROS packages for the UR10 arm depending on what firmware is installed.  This simulation does not guarantee compatibility with the real
> FARSCOPE arms as that has yet to be tested.

> Note: this package is relatively simple but has not been tested with other versions of ROS and Gazebo.

### Installation and Testing

* If you don't have it already, install ROS Melodic and set up a workspace using [these instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* Install the `joint_state_publisher_gui` using `sudo apt install ros-melodic-joint-state-publisher-gui`.  (_Replace `melodic` with your distribution if required._)
* Clone this package, [neo_simulation](https://github.com/neobotix/neo_simulation) and [universal_robot](https://github.com/ros-industrial/universal_robot) into the workspace `src` directory.
* Navigate up to the root directory of your ROS workspace (`cd ..` from `src`) and run `catkin_make`.
* Run `roslaunch farscope_group_project farscope_example_robot_visualize.launch`.  You should see an RViz visualization of the robot.
![RViz screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2014-11-59.png)
* Run `roslaunch farscope_group_project farscope_example_robot_simulate.launch`.  You should see a Gazebo simulation of the robot picking a target of a shelf and dropping it.
![Gazebo screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2014-10-59.png)
* Run `roslaunch farscope_group_project farscope_example_robot_simulate.launch use_gui:=false use_rviz:=true`.  Now Gazebo will run headless (_i.e._ no graphics front end) but you can see some of what's happening in RViz, including the LIDAR, the camera views, and the robot pose.
![RViz screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2010-05-52.png)

## Basic Usage

This section describes the ROS interface started by the [`launch/example_robot/farscope_example_robot_simulate.launch`](launch/example_robot/farscope_example_robot_simulate.launch) file.  Unless otherwise stated, these are all published or subscribed to by the Gazebo node, as a result of [plugins enabled in the `robot_description` URDF](http://gazebosim.org/tutorials?tut=ros_gzplugins).

> Also included in that file is the node [`scripts/example/test_pickup.py`](scripts/example/test_pickup.py) which shows examples of how to use the control interface.

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

_TO DO_

## Customizing your Robot

The robot is represented in [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf), generated using [Xacro](http://wiki.ros.org/xacro) XML macros for flexibility.  The URDF includes [`<gazebo>` tags](http://gazebosim.org/tutorials/?tut=ros_urdf) to encode robot actuation and sensing.  You are provided with a fully implemented model of a suitable robot, including a simple two finger gripper, two cameras, and a scanning LIDAR sensor on a mobile base.  Your options for customizing the robot include:
* Using the integrated example robot provided in [`models/example_robot/farscope_example_robot.urdf.xacro`](models/example_robot/farscope_example_robot.urdf.xacro) _as is_
* Making your own modified version of [`models/example_robot/farscope_example_robot.urdf.xacro`](models/example_robot/farscope_example_robot.urdf.xacro) to move, add or remove cameras
* Replacing the simple gripper with something of your own design including custom CAD (see [URDF tutorials](http://wiki.ros.org/urdf/Tutorials)) and actuation (_warning_: custom actuation is difficult and requires learning [ros_control](http://wiki.ros.org/ros_control))
* Making a complete new robot from scratch (_not recommended_: lots of ROS detail to learn)

## Package Contents

_TO DO_

### Models

### Scripts

### Launch

### Worlds

### Scenarios

## Limitations and Disclaimer

The goal of the group project is to experience integration of a robotic system, not to perform research-grade innovation in any individual subsystem.  Therefore the challenge has been subtly modified to simplify some elements.  In particular, the target object has been made easy to grasp, with a handy lip at the top to avoid the need for friction gripping.  Also, it has been made easy to detect, with a contrasting colour and symmetrical appearance.

Physics simulation is hard.  Even with Gazebo, it would take considerable tuning to represent realistic grasping, contact and friction.  Be ready for some odd behaviour:
* The robot gently rotates even while it is commanded to stay still
* If you drive the robot into a shelf, the robot will probably flip itself over, but the shelf will be undamaged
* A target may spontaneously jump from your grasp
* A target may mysteriously cling to you when released
These are things you will just have to work around in the pursuit of your project.
