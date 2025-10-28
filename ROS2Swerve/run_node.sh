#!/bin/bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
source /opt/ros/humble/setup.bash
source install/setup.bash
./install/my_robot_tracking/bin/$1
