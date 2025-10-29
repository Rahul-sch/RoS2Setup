# Robot Tracking System - ROS2 Package

This package converts your `track.py` functionality into a modular ROS2 system.

## Package Structure

```
my_robot_tracking/
├── src/my_robot_tracking/
│   ├── __init__.py
│   ├── camera_node.py          # Camera capture (from track.py camera_loop)
│   ├── tracking_node.py        # Object tracking (from track.py auto_tracking_loop)
│   ├── hardware_node.py        # Arduino communication (from track.py serial code)
│   ├── teleop_node.py          # Xbox controller (from track.py controller_loop)
│   ├── lidar_guard_node.py     # LIDAR safety monitor (stop/caution zones)
│   ├── launch/
│   │   └── tracking_system.launch.py
│   ├── config/
│   │   └── tracking_config.yaml
│   └── scripts/
│       └── object_selector.py
├── package.xml
├── setup.py
└── README.md
```

## Nodes Overview

### 1. Camera Node (`camera_node.py`)

- **Purpose**: Captures camera frames and publishes them
- **Input**: Camera (USB/Pi camera)
- **Output**: `/camera/image` (sensor_msgs/Image)
- **From**: `track.py` camera_loop thread

### 2. Tracking Node (`tracking_node.py`)

- **Purpose**: Tracks objects using optical flow
- **Input**: `/camera/image`
- **Output**: `/cmd_vel` (geometry_msgs/Twist)
- **From**: `track.py` auto_tracking_loop thread

### 3. Hardware Node (`hardware_node.py`)

- **Purpose**: Communicates with Arduino
- **Input**: `/cmd_vel`
- **Output**: Serial commands to Arduino
- **Safety**: Listens to `/safety/stop` and `/safety/caution` to halt/limit motion
- **From**: `track.py` serial communication code

### 4. Teleop Node (`teleop_node.py`)

- **Purpose**: Xbox controller input
- **Input**: `/joy` (sensor_msgs/Joy)
- **Output**: `/cmd_vel` (geometry_msgs/Twist)
- **From**: `track.py` controller_loop thread

### 5. LIDAR Guard Node (`lidar_guard_node.py`)

- **Purpose**: Monitors Slamtec C1 scans for close obstacles
- **Input**: `/scan` (sensor_msgs/LaserScan)
- **Output**: `/safety/stop`, `/safety/caution`, `/safety/min_distance`
- **Behaviour**: Forces hardware node to stop or slow when people/equipment enter safety zones

## Installation

1. **Build the package:**

```bash
cd my_robot_tracking
colcon build --packages-select my_robot_tracking
source install/setup.bash
```

2. **Install dependencies:**

```bash
sudo apt install ros-humble-joy ros-humble-cv-bridge
pip3 install opencv-python numpy
```

## Usage

### Start the complete system:

```bash
ros2 launch my_robot_tracking tracking_system.launch.py
```

### Start individual components:

```bash
# Camera only
ros2 run my_robot_tracking camera_node

# Tracking only (needs camera running)
ros2 run my_robot_tracking tracking_node

# Hardware interface only
ros2 run my_robot_tracking hardware_node

# Teleop only
ros2 run my_robot_tracking teleop_node
```

### Object Selection:

```bash
# Run object selector GUI
ros2 run my_robot_tracking object_selector.py
```

### LIDAR Safety Guard:

> The driver for the Slamtec C1 is not included here. Install and launch `sllidar_ros2` (or `rplidar_ros`) so that a `sensor_msgs/LaserScan` topic is available (e.g. `/scan`).

```bash
# Once the driver publishes /scan:
ros2 run my_robot_tracking lidar_guard --ros-args \
  -p scan_topic:=/scan \
  -p stop_distance:=0.45 \
  -p caution_distance:=1.2
```

When enabled, the guard node publishes `/safety/stop`. The hardware node subscribes to this topic and immediately sends neutral PWMs to the Arduino whenever the stop flag is true.

## Configuration

Edit `config/tracking_config.yaml` to adjust:

- Camera resolution and FPS
- Tracking parameters (speed, thresholds)
- Serial port settings
- Controller deadzone
- LIDAR stop/caution radii and topics

## Topics

| Topic              | Type                | Description       |
| ------------------ | ------------------- | ----------------- |
| `/camera/image`    | sensor_msgs/Image   | Camera frames     |
| `/cmd_vel`         | geometry_msgs/Twist | Movement commands |
| `/joy`             | sensor_msgs/Joy     | Controller input  |
| `/tracking/active` | std_msgs/Bool       | Tracking status   |
| `/safety/stop`     | std_msgs/Bool       | True while LIDAR stop zone is violated |
| `/safety/caution`  | std_msgs/Bool       | True while LIDAR caution zone is violated |
| `/safety/min_distance` | std_msgs/Float32 | Closest obstacle distance (optional) |

## Services

| Service               | Type             | Description         |
| --------------------- | ---------------- | ------------------- |
| `/tracking/toggle`    | std_srvs/SetBool | Start/stop tracking |
| `/teleop/toggle_mode` | std_srvs/SetBool | Manual/auto mode    |

## Key Differences from track.py

### Advantages:

- ✅ Modular - can restart individual components
- ✅ Can run on different computers
- ✅ Easy to add new sensors/features
- ✅ Built-in visualization tools (RViz)
- ✅ Can record/replay sessions (rosbag)

### Disadvantages:

- ❌ More complex setup
- ❌ Slightly higher latency (message passing)
- ❌ Steeper learning curve

## Migration Notes

Your original `track.py` logic is preserved:

- **Camera capture**: Same OpenCV/PiCamera2 code
- **Optical flow tracking**: Same algorithm
- **Arduino communication**: Same serial commands
- **Controller input**: Same Xbox controller handling

The main change is **message passing** instead of **shared variables**.

## Troubleshooting

1. **Camera not found**: Check camera permissions and device paths
2. **Arduino not connected**: Verify serial port and permissions
3. **Controller not working**: Install joy package and check device permissions
4. **Tracking not starting**: Use object selector to click on target

## Next Steps

1. Test individual nodes first
2. Use `ros2 topic echo /cmd_vel` to see movement commands
3. Use `ros2 node list` to see running nodes
4. Use `ros2 topic list` to see available topics
