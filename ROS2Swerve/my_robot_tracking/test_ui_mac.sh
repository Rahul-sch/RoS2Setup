#!/bin/bash

# Test script to run MedRa Web UI on Mac (without ROS2)

echo "MedRa Web UI - Mac Test Mode"
echo "============================"

# Check if we're in the right directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Navigate to the web UI server location
cd src/my_robot_tracking/my_robot_tracking

# Check if Flask is installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo ""
    echo "Flask not found. Installing dependencies..."
    echo ""
    echo "Option 1: Use a virtual environment (recommended):"
    echo "  python3 -m venv venv"
    echo "  source venv/bin/activate"
    echo "  pip install flask flask-cors opencv-python"
    echo ""
    echo "Option 2: Install globally (if you have permissions):"
    echo "  pip3 install flask flask-cors opencv-python"
    echo ""
    echo "After installing, run this script again."
    exit 1
fi

# Run the test server
echo "Starting test web server..."
echo "Open your browser to: http://localhost:8080"
echo ""
python3 web_ui_server_test.py

