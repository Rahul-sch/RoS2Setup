#!/bin/bash

# Build script for my_robot_tracking package

echo "Building my_robot_tracking package..."

# Build the package
colcon build --packages-select my_robot_tracking

# Source the workspace
source install/setup.bash

echo "Build complete!"
echo "To run the system:"
echo "  ros2 launch my_robot_tracking tracking_system.launch.py"
echo ""
echo "To test individual nodes:"
echo "  ros2 run my_robot_tracking camera_node"
echo "  ros2 run my_robot_tracking tracking_node"
echo "  ros2 run my_robot_tracking hardware_node"
echo "  ros2 run my_robot_tracking teleop_node"
