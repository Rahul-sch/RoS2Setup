#!/bin/bash
# Fix ROS2 executable location issue
# ROS2 expects executables in lib/my_robot_tracking/ but setuptools puts them in bin/

set -e

cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking

echo "Creating lib/my_robot_tracking directory for executables..."
mkdir -p install/my_robot_tracking/lib/my_robot_tracking

echo "Creating symlinks from lib/my_robot_tracking/ to bin/..."
for exe in install/my_robot_tracking/bin/*; do
    if [ -f "$exe" ] && [ -x "$exe" ]; then
        exe_name=$(basename "$exe")
        ln -sf "../../bin/$exe_name" "install/my_robot_tracking/lib/my_robot_tracking/$exe_name"
        echo "  Linked: $exe_name"
    fi
done

echo "Done! Now try: ros2 run my_robot_tracking camera_node --help"
