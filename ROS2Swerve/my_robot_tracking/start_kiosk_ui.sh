#!/bin/bash

# Start MedRa UI in kiosk mode on Raspberry Pi HDMI display
# This script launches a browser in fullscreen pointing to the UI

# Wait for display to be ready (important on boot)
sleep 5

# Set display (if not already set)
export DISPLAY=:0

# Start Flask backend (if not already running)
cd /home/$USER/Ros2Setup/ROS2Swerve/my_robot_tracking/src/my_robot_tracking/my_robot_tracking

# Check if Flask is already running
if ! pgrep -f "web_ui_server.py" > /dev/null; then
    # Source ROS2
    source /opt/ros/humble/setup.bash
    source ~/Ros2Setup/ROS2Swerve/install/setup.bash
    
    # Start Flask backend
    python3 web_ui_server.py &
    sleep 3
fi

# Launch browser in kiosk mode (fullscreen, no UI)
# Using Chromium (most common on Raspberry Pi)
if command -v chromium-browser &> /dev/null; then
    chromium-browser \
        --kiosk \
        --noerrdialogs \
        --disable-infobars \
        --disable-session-crashed-bubble \
        --disable-restore-session-state \
        --autoplay-policy=no-user-gesture-required \
        http://localhost:8080/camera &
elif command -v chromium &> /dev/null; then
    chromium \
        --kiosk \
        --noerrdialogs \
        --disable-infobars \
        --disable-session-crashed-bubble \
        --disable-restore-session-state \
        --autoplay-policy=no-user-gesture-required \
        http://localhost:8080/camera &
elif command -v firefox &> /dev/null; then
    firefox -kiosk http://localhost:8080/camera &
else
    echo "No suitable browser found. Please install chromium-browser"
    exit 1
fi

# Keep script running
wait

