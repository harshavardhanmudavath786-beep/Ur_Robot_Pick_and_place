# UR5e Pick-and-Place Robot with MoveIt2

This workspace contains a ROS 2 pick-and-place project built around a UR5e robotic arm fitted with a Robotiq 2F-85 gripper. The project combines a robot description package, a MoveIt configuration package, and a simple C++ motion program that executes a scripted pick-and-place workflow.

The repository is organized into the following packages:

- [moveit_robot_setup/](/home/harsh/robot_ws/src/moveit_robot_setup)
  - MoveIt configuration and launch files for the robot
- [robot_gripper/](/home/harsh/robot_ws/src/robot_gripper)
  - Robot and gripper URDF/Xacro definitions
- [robot_move/](/home/harsh/robot_ws/src/robot_move)
  - Motion execution node for the pick-and-place routine
- [ur_description/](/home/harsh/robot_ws/src/ur_description)
  - Universal Robots UR model and description resources

## Overview

This project demonstrates how to:

- model a UR5e robot with a Robotiq gripper in URDF/Xacro
- generate a MoveIt configuration package
- plan and execute robot motion with MoveIt 2
- perform a basic scripted pick-and-place action with gripper open/close logic

The main execution logic is implemented in [robot_move/src/robot_move_source.cpp](/home/harsh/robot_ws/src/robot_move/src/robot_move_source.cpp). It moves the arm through a sequence of named/pose targets and opens/closes the gripper at the appropriate stages.

## System Requirements

This project is intended for a ROS 2 Humble environment with MoveIt 2 installed.

Required tools and dependencies:

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- colcon
- rosdep
- xacro
- RViz2 (for visualization)
- Optional: Gazebo or a robot simulator for testing

## Workspace Setup

From a terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash