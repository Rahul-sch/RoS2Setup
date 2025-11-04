# Raspberry Pi Build Instructions

## Your Workspace Structure

```
~/Ros2Setup/ROS2Swerve/
└── my_robot_tracking/              ← This is your ROS2 workspace root
    ├── src/
    │   └── my_robot_tracking/      ← This is the ROS2 package
    │       ├── my_robot_tracking/  ← Python module with all nodes
    │       ├── setup.py
    │       ├── package.xml
    │       └── launch/
    ├── build/                      ← Created by colcon
    ├── install/                    ← Created by colcon
    ├── log/                        ← Created by colcon
    └── start_robot_pyqt_ui.sh     ← Start script
```

## Step-by-Step Build Process on Raspberry Pi

### 1. Navigate to workspace root
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
```

### 2. Source ROS2 (if not already in ~/.bashrc)
```bash
source /opt/ros/humble/setup.bash
```

### 3. Install Python dependencies
```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install flask flask-cors pyqt5 opencv-python-headless
```

### 4. Build the workspace
```bash
colcon build --symlink-install
```

**Important:** Run this from `~/Ros2Setup/ROS2Swerve/my_robot_tracking` NOT from inside `src/`

### 5. Source the workspace
```bash
source install/setup.bash
```

### 6. Verify installation
```bash
ros2 pkg list | grep my_robot_tracking
```

You should see: `my_robot_tracking`

### 7. Check executables are available
```bash
ros2 run my_robot_tracking camera_node --help
ros2 run my_robot_tracking tracking_node --help
ros2 run my_robot_tracking hardware_node --help
ros2 run my_robot_tracking teleop_node --help
ros2 run my_robot_tracking lidar_guard --help
ros2 run my_robot_tracking pyqt_ui --help
```

Each should respond (even if just with an error about missing arguments).

### 8. Make start script executable
```bash
chmod +x start_robot_pyqt_ui.sh
```

### 9. Run the system
```bash
./start_robot_pyqt_ui.sh
```

## Common Issues and Fixes

### Issue: "No executable found"
**Cause:** Workspace not built or not sourced

**Fix:**
```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
colcon build --symlink-install
source install/setup.bash
./start_robot_pyqt_ui.sh
```

### Issue: "Package 'my_robot_tracking' not found"
**Cause:** Building from wrong directory

**Fix:** Make sure you run `colcon build` from `my_robot_tracking/` (where start_robot_pyqt_ui.sh is), NOT from `my_robot_tracking/src/`

### Issue: PyQt5 import errors
**Cause:** Missing system dependencies on Raspberry Pi

**Fix:**
```bash
sudo apt install -y python3-pyqt5 python3-pyqt5.qtwidgets
pip3 install pyqt5
```

### Issue: Camera not showing
**Cause:** Camera permissions or camera not detected

**Fix:**
```bash
# Check camera
ls /dev/video*

# Add user to video group
sudo usermod -a -G video $USER

# Reboot
sudo reboot
```

### Issue: Arduino not detected
**Cause:** Serial port permissions

**Fix:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot
sudo reboot
```

## Auto-start on Boot (Optional)

To make the system start automatically when the Pi boots:

### 1. Add to bashrc
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Ros2Setup/ROS2Swerve/my_robot_tracking/install/setup.bash" >> ~/.bashrc
```

### 2. Create systemd service
```bash
sudo nano /etc/systemd/system/medra-robot.service
```

Add:
```ini
[Unit]
Description=MedRa Robot Control System
After=network.target

[Service]
Type=simple
User=thiru
WorkingDirectory=/home/thiru/Ros2Setup/ROS2Swerve/my_robot_tracking
ExecStart=/home/thiru/Ros2Setup/ROS2Swerve/my_robot_tracking/start_robot_pyqt_ui.sh
Restart=on-failure
Environment="DISPLAY=:0"

[Install]
WantedBy=multi-user.target
```

### 3. Enable service
```bash
sudo systemctl daemon-reload
sudo systemctl enable medra-robot.service
sudo systemctl start medra-robot.service
```

## Testing Individual Components

### Test camera node only
```bash
ros2 run my_robot_tracking camera_node
```

### Test PyQt UI only (after camera is running)
```bash
ros2 run my_robot_tracking pyqt_ui
```

### View available topics
```bash
ros2 topic list
```

Should show:
- `/camera/image`
- `/manual/cmd_vel`
- `/auto/cmd_vel`
- `/tracking/active`
- etc.

### View camera feed (test)
```bash
ros2 topic echo /camera/image --once
```

## What the Start Script Does

1. Sources ROS2 Humble
2. Sources your workspace install
3. Starts nodes in sequence:
   - camera_node (publishes camera frames)
   - hardware_node (talks to Arduino)
   - tracking_node (optical flow tracking)
   - teleop_node (joystick control)
   - lidar_guard (safety system)
4. Launches PyQt UI (connects to all topics)

## Expected Behavior

When you run `./start_robot_pyqt_ui.sh`:

1. Terminal shows nodes starting
2. PyQt window opens on screen
3. Login screen appears
4. Click "Login" → Dashboard
5. Click "Camera" → See live camera feed
6. Click "Manual Mode" → Control robot with arrow buttons

## Troubleshooting Checklist

- [ ] Running from `~/Ros2Setup/ROS2Swerve/my_robot_tracking`
- [ ] ROS2 Humble installed: `which ros2`
- [ ] Workspace built: `ls install/setup.bash`
- [ ] Workspace sourced: `echo $AMENT_PREFIX_PATH`
- [ ] Camera connected: `ls /dev/video*`
- [ ] Arduino connected: `ls /dev/ttyACM* /dev/ttyUSB*`
- [ ] User in video group: `groups | grep video`
- [ ] User in dialout group: `groups | grep dialout`
- [ ] Display available: `echo $DISPLAY` (should show `:0` or `:1`)

## Need Help?

If you still get "No executable found", run these diagnostic commands and share the output:

```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
pwd
ls -la
ls -la src/
ls -la install/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep my_robot
which ros2
echo $AMENT_PREFIX_PATH
```
