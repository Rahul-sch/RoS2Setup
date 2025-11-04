#!/bin/bash

# MedRa Robot Startup Script
# This script starts the ROS2 nodes and Flask UI, then opens the camera page

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "MedRa Robot Control System - Starting..."
echo "=========================================="

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $(jobs -p) 2>/dev/null || true
    exit
}

trap cleanup SIGINT SIGTERM

# Start ROS2 nodes in background
echo "[1/3] Starting ROS2 nodes..."

# Camera node
echo "  → Starting camera_node..."
ros2 run my_robot_tracking camera_node &
CAMERA_PID=$!

sleep 2

# Hardware node
echo "  → Starting hardware_node..."
ros2 run my_robot_tracking hardware_node &
HARDWARE_PID=$!

sleep 1

# Start Flask backend
echo "[2/3] Starting Flask backend (port 8080)..."
cd "$SCRIPT_DIR/my_robot_tracking/src/my_robot_tracking/my_robot_tracking"
python3 web_ui_server.py &
FLASK_PID=$!
cd "$SCRIPT_DIR"

sleep 3

# Give backend a moment to finish binding the port
sleep 2

# Open browser to camera page
echo "[3/3] Opening browser..."
echo ""
echo "=========================================="
echo "✅ All systems started!"
echo "=========================================="
echo "Camera: http://localhost:8080/camera"
echo "Dashboard: http://localhost:8080/dashboard"
echo ""
echo "Press Ctrl+C to stop all services"
echo "=========================================="

# Try to open browser (works on most systems)
if command -v xdg-open &> /dev/null; then
    xdg-open "http://localhost:8080/camera" 2>/dev/null &
elif command -v open &> /dev/null; then
    open "http://localhost:8080/camera" 2>/dev/null &
elif command -v start &> /dev/null; then
    start "http://localhost:8080/camera" 2>/dev/null &
fi

# Wait for all background processes
wait
