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

## Dependencies

The package was developed in ROS Melodic using Gazebo 9.0.0 on Ubuntu 18.04.  It also requires the following packages:
* [neo_simulation](https://github.com/neobotix/neo_simulation) : simulation of the MMO-700 from Neobotix
* [universal_robot](https://github.com/ros-industrial/universal_robot) : the UR10 support from [ROS Industrial](https://rosindustrial.org/)

> Note: there are different ROS packages for the UR10 arm depending on what firmware is installed.  This simulation does not guarantee compatibility with the real
> FARSCOPE arms as that has yet to be tested.

> Note: this package is relatively simple but has not been tested with other versions of ROS and Gazebo.

## Installation and Testing

* If you don't have it already, install ROS Melodic and set up a workspace using [these instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
* Clone this package and the two packages above into the workspace `src` directory
* Run `roslaunch farscope_group_project farscope_example_robot_visualize.launch`.  You should see an RViz visualization of the robot.
![RViz screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2014-11-59.png)
* Run `roslaunch farscope_group_project farscope_example_robot_simulate.launch`.  You should see a Gazebo simulation of the robot picking a target of a shelf and dropping it.
![Gazebo screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2014-10-59.png)
* Run `roslaunch farscope_group_project farscope_example_robot_simulate.launch use_gui:=false use_rviz:=true`.  Now Gazebo will run headless (_i.e._ no graphics front end) but you can see some of what's happening in RViz, including the LIDAR, the camera views, and the robot pose.
![RViz screenshot](https://raw.githubusercontent.com/arthurrichards77/farscope_group_project/main/Screenshot%20from%202021-01-18%2010-05-52.png)
