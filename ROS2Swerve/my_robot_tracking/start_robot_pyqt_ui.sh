#!/bin/bash

# MedRa Robot Startup Script (PyQt UI Edition)
# Starts ROS2 nodes, the Flask API backend, then launches the PyQt UI.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

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

echo "[1/3] Starting ROS2 nodes..."

echo "  → Starting camera_node..."
ros2 run my_robot_tracking camera_node &
CAMERA_PID=$!

sleep 2

echo "  → Starting hardware_node..."
ros2 run my_robot_tracking hardware_node &
HARDWARE_PID=$!

sleep 1

echo "[2/3] Starting Flask backend (port 8080)..."
cd "$SCRIPT_DIR/my_robot_tracking/src/my_robot_tracking/my_robot_tracking"
python3 web_ui_server.py &
FLASK_PID=$!
cd "$SCRIPT_DIR"

sleep 2

echo "[3/3] Launching PyQt UI..."
echo "=========================================="
echo "✅ All systems started!"
echo "=========================================="
echo "API: http://localhost:8080"
echo "PyQt UI will open momentarily."
echo ""
echo "Press Ctrl+C to stop all services"
echo "=========================================="

python3 "$SCRIPT_DIR/src/my_robot_tracking/my_robot_tracking/pyqt_ui.py"

cleanup
