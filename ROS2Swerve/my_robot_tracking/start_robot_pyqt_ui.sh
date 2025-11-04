#!/bin/bash

# MedRa Robot Startup Script (PyQt UI Edition)
# Starts all ROS2 nodes, then launches PyQt UI (which connects directly to ROS2)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  echo "ERROR: ROS2 workspace not built. Run 'colcon build --symlink-install' in $WORKSPACE_ROOT first."
  exit 1
fi

source "$WORKSPACE_ROOT/install/setup.bash"

echo "=========================================="
echo "MedRa Robot Control System (PyQt UI) - Starting..."
echo "=========================================="

cleanup() {
  echo ""
  echo "Shutting down..."
  kill $(jobs -p) 2>/dev/null || true
  exit
}

trap cleanup SIGINT SIGTERM

# Start core nodes in sequence so topics come up cleanly

echo "[1/5] Starting camera_node..."
ros2 run my_robot_tracking camera_node &
CAMERA_PID=$!
sleep 2

echo "[2/5] Starting hardware_node..."
ros2 run my_robot_tracking hardware_node &
HARDWARE_PID=$!
sleep 1

echo "[3/5] Starting tracking_node..."
ros2 run my_robot_tracking tracking_node &
TRACKING_PID=$!
sleep 1

echo "[4/5] Starting teleop_node..."
ros2 run my_robot_tracking teleop_node &
TELEOP_PID=$!
sleep 1

echo "[5/5] Starting lidar_guard..."
ros2 run my_robot_tracking lidar_guard &
LIDAR_PID=$!
sleep 2

echo "=========================================="
echo "✅ All ROS2 nodes started!"
echo "=========================================="
echo "Launching PyQt UI..."
echo "Press Ctrl+C to stop all services"
echo "=========================================="

# Launch PyQt UI (connects directly to ROS2)
ros2 run my_robot_tracking pyqt_ui

cleanup
