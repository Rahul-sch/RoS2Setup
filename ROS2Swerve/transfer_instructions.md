# Transfer Code to Ubuntu

## Method 1: SCP (Secure Copy) - Recommended

### Step 1: Get Ubuntu IP and Username
On Ubuntu, run:
```bash
ip addr show | grep inet
# Note the IP address (e.g., 192.168.1.100)
whoami
# Note your username
```

### Step 2: Transfer from Windows
Open PowerShell or Command Prompt on Windows:

```bash
# Replace with your Ubuntu details
scp -r my_robot_tracking/ username@192.168.1.100:/home/username/ros2_ws/src/
```

### Step 3: Build on Ubuntu
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tracking
source install/setup.bash
```

## Method 2: Git (If you have Git)

### On Windows:
```bash
git init
git add my_robot_tracking/
git commit -m "Add ROS2 tracking package"
git remote add origin https://github.com/yourusername/robot-tracking.git
git push -u origin main
```

### On Ubuntu:
```bash
git clone https://github.com/yourusername/robot-tracking.git
cd robot-tracking
```

## Method 3: USB Drive

1. Copy `my_robot_tracking/` folder to USB
2. Plug USB into Ubuntu
3. Copy to `~/ros2_ws/src/`

## Method 4: Network Share

### On Windows:
1. Right-click `my_robot_tracking` folder
2. Properties → Sharing → Share
3. Note the network path

### On Ubuntu:
```bash
# Mount Windows share (if on same network)
sudo mkdir /mnt/windows
sudo mount -t cifs //windows-ip/share /mnt/windows -o username=windows-user
cp -r /mnt/windows/my_robot_tracking ~/ros2_ws/src/
```

## Quick Test Commands

After transfer, test on Ubuntu:

```bash
# Check if package is there
ls ~/ros2_ws/src/my_robot_tracking/

# Build the package
cd ~/ros2_ws
colcon build --packages-select my_robot_tracking

# Source the workspace
source install/setup.bash

# Test individual nodes
ros2 run my_robot_tracking camera_node
ros2 run my_robot_tracking tracking_node

# Run complete system
ros2 launch my_robot_tracking tracking_system.launch.py
```

## Troubleshooting

### SCP Permission Denied:
```bash
# On Ubuntu, ensure SSH is enabled
sudo systemctl enable ssh
sudo systemctl start ssh
```

### Build Errors:
```bash
# Install missing dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-joy
pip3 install opencv-python numpy pyserial
```

### Camera Issues:
```bash
# Add user to video group
sudo usermod -a -G video $USER
# Logout and login again
```
