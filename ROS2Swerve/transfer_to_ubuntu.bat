@echo off
REM Batch file to transfer ROS2 package to Ubuntu
REM Edit the variables below with your Ubuntu details

set UBUNTU_USER=your_username
set UBUNTU_IP=192.168.1.100
set UBUNTU_PATH=/home/%UBUNTU_USER%/ros2_ws/src/

echo Transferring my_robot_tracking package to Ubuntu...
echo Target: %UBUNTU_USER%@%UBUNTU_IP%:%UBUNTU_PATH%

REM Transfer the package
scp -r my_robot_tracking/ %UBUNTU_USER%@%UBUNTU_IP%:%UBUNTU_PATH%

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ✅ Transfer successful!
    echo.
    echo Next steps on Ubuntu:
    echo 1. cd ~/ros2_ws
    echo 2. colcon build --packages-select my_robot_tracking
    echo 3. source install/setup.bash
    echo 4. ros2 launch my_robot_tracking tracking_system.launch.py
) else (
    echo.
    echo ❌ Transfer failed!
    echo Check your Ubuntu IP, username, and SSH connection.
    echo.
    echo Make sure SSH is enabled on Ubuntu:
    echo sudo systemctl enable ssh
    echo sudo systemctl start ssh
)

pause
