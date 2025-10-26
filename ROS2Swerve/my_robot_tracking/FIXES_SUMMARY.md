# ROS2 Robot Tracking System - Fixes Summary

## Overview
Your ROS2 code has been updated to work smoothly with your Arduino and track.py implementation. All critical issues have been fixed!

---

## ✅ Issues Fixed

### 1. **Steering Communication Chain** ✓
- **Problem**: Tracking node calculated steering but never sent commands
- **Fix**: 
  - Added `/steering_angles` topic (Float64MultiArray)
  - Tracking node now publishes steering angles
  - Hardware node subscribes and sends to Arduino

### 2. **Missing `compose_wheel_angles()` Function** ✓
- **Problem**: Per-wheel offsets and directions weren't applied
- **Fix**: Added to hardware_node.py (lines 118-125)
- Now correctly applies `STEER_DIR` and `STEER_OFFSETS` for each wheel

### 3. **Mode Arbitration (Auto vs Manual)** ✓
- **Problem**: Both teleop and tracking published to same `/cmd_vel` topic
- **Fix**: 
  - Auto tracking → `/auto/cmd_vel`
  - Manual teleop → `/manual/cmd_vel`
  - Hardware node subscribes to both

### 4. **Teleop Node Initialization** ✓
- **Problem**: Variables used before initialization
- **Fix**: Added initialization for all axis variables (lines 30-37)
- Added trigger edge detection for 45° steering increments

### 5. **Frame Width/Height Missing** ✓
- **Problem**: Tracking node needed frame dimensions for normalization
- **Fix**: Added as parameters (default 640x480)

### 6. **Object Selection GUI** ✓
- **Problem**: No way to select tracking target
- **Fix**: Created object_selector node with:
  - Click-to-select interface
  - Real-time tracking visualization
  - Publishes to `/tracking/select_point`

### 7. **Serial Port Auto-Detection** ✓
- **Problem**: Hardcoded serial port
- **Fix**: Empty string = auto-detect `/dev/ttyACM*` or `/dev/ttyUSB*`

---

## 🎯 How the System Works Now

### Architecture Flow:

```
┌─────────────────┐
│  Camera Node    │──────┐
│  (Picam2/USB)   │      │
└─────────────────┘      │
                         │ /camera/image
                         ▼
┌─────────────────────────────────────────┐
│         Object Selector Node            │
│  (Click to select tracking target)      │
└────────────┬────────────────────────────┘
             │ /tracking/select_point
             ▼
┌─────────────────────────────────────────┐
│          Tracking Node                  │
│  - Optical flow tracking                │
│  - Angle calculation                    │
│  - Speed control                        │
└────┬────────────────┬───────────────────┘
     │                │
     │ /steering_angles│ /auto/cmd_vel
     ▼                ▼
┌────────────────────────────────────────┐
│                                         │
│         Hardware Interface Node         │
│  - Subscribes to auto/manual topics    │
│  - Applies wheel offsets via           │
│    compose_wheel_angles()               │
│  - Sends D/S/C commands to Arduino     │
│                                         │
└─────────────────┬───────────────────────┘
                  │
                  │ Serial (D/S/C commands)
                  ▼
        ┌─────────────────┐
        │   Arduino Mega   │
        │  - 4x Steppers   │
        │  - 4x ESCs       │
        └─────────────────┘

┌──────────────────┐
│   Xbox Controller│────────┐
└──────────────────┘        │
                            │ /joy
                            ▼
                  ┌───────────────────┐
                  │   Teleop Node     │
                  │  - Y button: mode │
                  │  - Triggers: steer│
                  │  - Joystick: drive│
                  └─────┬─────────────┘
                        │
                        │ /manual/cmd_vel
                        │ /manual/steering_angles
                        ▼
                  (to Hardware Node)
```

---

## 🚀 How to Use

### 1. Build the Package
```bash
cd ~/RoS2Setup/ROS2Swerve/my_robot_tracking
./build.sh
```

### 2. Launch Everything
```bash
source install/setup.bash
ros2 launch my_robot_tracking tracking_system.launch.py
```

### 3. Using the System

#### **Manual Mode (Default)**
1. Press **Y** button on Xbox controller to toggle modes
2. **Left joystick Y-axis**: Forward/backward
3. **Left/Right triggers**: Rotate wheels ±45°
4. All commands go to `/manual/cmd_vel` and `/manual/steering_angles`

#### **Auto Tracking Mode**
1. Press **Y** button to switch to AUTO mode
2. In the "Object Selector" window, **click** on an object
3. Robot will:
   - Extract optical flow features
   - Calculate angle to object
   - Steer wheels to face object
   - Drive forward to approach
4. Press **T** to stop tracking

### 4. Launch Options
```bash
# Disable specific nodes
ros2 launch my_robot_tracking tracking_system.launch.py \
    camera_enabled:=true \
    tracking_enabled:=true \
    hardware_enabled:=true \
    teleop_enabled:=true \
    object_selector_enabled:=true

# Run without hardware (testing)
ros2 launch my_robot_tracking tracking_system.launch.py \
    hardware_enabled:=false
```

---

## 📋 Topic Reference

### Published Topics:
- `/camera/image` - Camera feed (Image)
- `/steering_angles` - Target wheel angles (Float64MultiArray)
- `/auto/cmd_vel` - Auto mode velocity (Twist)
- `/manual/cmd_vel` - Manual mode velocity (Twist)
- `/manual/steering_angles` - Manual steering (Float64MultiArray)
- `/tracking/active` - Tracking status (Bool)
- `/tracking/center` - Object center position (Point)
- `/tracking/select_point` - Selected point (Point)
- `/joy` - Joystick input (Joy)

### Services:
- `/tracking/toggle` - Start/stop tracking (SetBool)
- `/teleop/toggle_mode` - Switch manual/auto (SetBool)

---

## 🔧 Configuration

### Calibrate Your Robot:
Edit `config/tracking_config.yaml`:

```yaml
hardware_node:
  ros__parameters:
    wheel_drive_dir: [1, 1, 1, 1]    # 1 = forward, -1 = reverse
    steer_dir: [1, 1, 1, 1]          # Steering direction per wheel
    steer_offsets: [0, 0, 0, 0]      # Angle offsets (degrees)
```

### Tune Tracking:
```yaml
tracking_node:
  ros__parameters:
    max_speed: 100                    # Max PWM offset
    center_threshold: 30.0            # Stop distance (pixels)
    angle_send_threshold: 5.0         # Steering update threshold (degrees)
    steering_delay: 0.3               # Wait after steering (seconds)
```

---

## 🐛 Troubleshooting

### Arduino Not Connecting
- Check USB cable
- Verify permissions: `sudo chmod 666 /dev/ttyACM0`
- Set serial port explicitly in launch file or config

### No Camera Image
- Check camera connection
- Try: `ls /dev/video*`
- For Picam2: Ensure camera is enabled in `raspi-config`

### Tracking Not Working
1. Check that object_selector window appears
2. Click on a high-contrast object
3. Ensure object has trackable features
4. Check `/tracking/active` topic: `ros2 topic echo /tracking/active`

### Steering Commands Not Working
- Verify Arduino receives commands: Check serial monitor
- Test manually: `ros2 topic pub /steering_angles std_msgs/msg/Float64MultiArray "data: [45, 45, 45, 45]"`
- Check `compose_wheel_angles()` logic matches your hardware

### Mode Switching Issues
- Ensure joy_node is running: `ros2 node list`
- Check joystick connection: `ls /dev/input/js*`
- Test: `ros2 topic echo /joy`

---

## 📝 Code Changes Summary

### Files Modified:
1. **tracking_node.py** - Added steering publishing, frame params
2. **hardware_node.py** - Added compose_wheel_angles, mode arbitration
3. **teleop_node.py** - Fixed initialization, added steering commands
4. **object_selector.py** - Complete rewrite with visualization
5. **tracking_system.launch.py** - Added object_selector, updated params
6. **tracking_config.yaml** - Updated all parameters

### New Topics:
- `/steering_angles` - Main steering command channel
- `/auto/cmd_vel` - Auto mode driving
- `/manual/cmd_vel` - Manual mode driving
- `/manual/steering_angles` - Manual steering
- `/tracking/center` - Current object position
- `/tracking/select_point` - Point selection

---

## ✨ Key Improvements

1. **Proper separation of concerns**: Each node has a clear role
2. **No conflicts**: Auto and manual modes use separate topics
3. **Hardware abstraction**: compose_wheel_angles() handles per-wheel config
4. **Visual feedback**: Object selector shows tracking status
5. **Robust serial**: Auto-detection and reconnection
6. **Parameter-driven**: Easy tuning without code changes

---

## 🎉 You're Ready!

Your ROS2 setup should now work smoothly with your Arduino swerve drive robot. The code properly:
- ✅ Sends steering commands to Arduino
- ✅ Tracks objects with optical flow
- ✅ Handles manual control via Xbox controller
- ✅ Applies per-wheel offsets and directions
- ✅ Prevents mode conflicts
- ✅ Provides visual feedback

Build, launch, and enjoy! 🚀

