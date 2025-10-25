#!/bin/bash

# Script to transfer ROS2 tracking package to Ubuntu
# Run this from Windows (with WSL or Git Bash) or from Ubuntu

# Configuration - UPDATE THESE VALUES
UBUNTU_USER="your_username"
UBUNTU_HOST="your_ubuntu_ip"
UBUNTU_PATH="/home/$UBUNTU_USER/ros2_ws/src/"

echo "Transferring ROS2 tracking package to Ubuntu..."

# Create destination directory if it doesn't exist
ssh $UBUNTU_USER@$UBUNTU_HOST "mkdir -p $UBUNTU_PATH"

# Transfer the entire package
scp -r my_robot_tracking/ $UBUNTU_USER@$UBUNTU_HOST:$UBUNTU_PATH

echo "Transfer complete!"
echo "On Ubuntu, run:"
echo "  cd ~/ros2_ws"
echo "  colcon build --packages-select my_robot_tracking"
echo "  source install/setup.bash"
echo "  ros2 launch my_robot_tracking tracking_system.launch.py"
