# ROS2 Swerve Robot System Architecture
## Complete Node Documentation with LIDAR Integration

---

## 📋 System Overview

**Purpose**: Autonomous swerve-drive robot with visual object tracking, manual control, and LIDAR-based obstacle avoidance.

**Hardware**:
- Raspberry Pi (ROS2 Ubuntu)
- Arduino Mega (motor/stepper control)
- Camera (Picamera2 or USB)
- Xbox Controller
- LIDAR sensor (RPLidar/YDLidar/etc.)
- 4x Swerve modules (stepper + ESC)

---

## 🎯 Node Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      PERCEPTION LAYER                        │
├─────────────────┬───────────────────┬───────────────────────┤
│  camera_node    │  lidar_node       │ object_selector_node  │
│  (Vision)       │  (Range)          │ (GUI)                 │
└────────┬────────┴────────┬──────────┴──────────┬────────────┘
         │                 │                     │
         │ /camera/image   │ /scan               │ /tracking/select_point
         │                 │                     │
         ▼                 ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    PROCESSING LAYER                          │
├──────────────────┬──────────────────┬───────────────────────┤
│  tracking_node   │ obstacle_node    │  navigation_node      │
│  (Vision track)  │ (Collision det)  │  (Path planning)      │
└────────┬─────────┴────────┬─────────┴────────┬──────────────┘
         │                  │                  │
         │ /auto/cmd_vel    │ /obstacles       │ /nav/cmd_vel
         │ /steering_angles │ /emergency_stop  │ /nav/steering
         │                  │                  │
         ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────┐
│                    ARBITRATION LAYER                         │
├─────────────────────────────────────────────────────────────┤
│              cmd_vel_mux_node (Priority Mux)                 │
│  Prioritizes: Emergency > Manual > Navigation > Tracking     │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           │ /cmd_vel (final)
                           │ /steering_angles (final)
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                      CONTROL LAYER                           │
├──────────────────┬──────────────────────────────────────────┤
│  teleop_node     │          hardware_node                    │
│  (Manual)        │      (Arduino Interface)                  │
└────────┬─────────┴────────────────┬───────────────────────────┘
         │                          │
         │ /manual/cmd_vel          │ Serial: D/S/C commands
         │ /manual/steering         │
         │                          ▼
         │                    ┌──────────┐
         └───────────────────►│ Arduino  │
                              │   Mega   │
                              └──────────┘
```

---

## 🔷 NODE 1: camera_node

### **Purpose**
Capture video frames from camera and publish to ROS2 network.

### **Responsibilities**
- Initialize camera (Picamera2 or USB webcam)
- Capture frames at configured FPS
- Convert to ROS Image message
- Publish with timestamp and frame_id

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/camera/image` | `sensor_msgs/Image` | 30 Hz | BGR8 camera frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | Camera calibration (optional) |

### **Parameters**
- `frame_width`: 640 (pixels)
- `frame_height`: 480 (pixels)
- `fps`: 30 (frames per second)
- `camera_index`: 0 (for USB cameras)

### **Dependencies**
- `cv2` (OpenCV)
- `picamera2` (for Pi Camera)
- `cv_bridge`

---

## 🔷 NODE 2: lidar_node

### **Purpose**
Interface with LIDAR sensor and publish laser scan data.

### **Responsibilities**
- Connect to LIDAR via serial/USB
- Read distance measurements
- Convert to LaserScan message
- Handle LIDAR health monitoring
- Automatic reconnection on failure

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | 360° laser scan data |
| `/lidar/health` | `std_msgs/String` | 1 Hz | LIDAR health status |

### **Parameters**
- `serial_port`: `/dev/ttyUSB0`
- `serial_baudrate`: 115200
- `frame_id`: `laser_frame`
- `scan_frequency`: 10 Hz
- `angle_min`: -π radians
- `angle_max`: π radians
- `range_min`: 0.15 meters
- `range_max`: 12.0 meters

### **Dependencies**
- `rplidar` or `ydlidar` driver package
- `pyserial`

### **Example Implementation Notes**
```python
# Will publish LaserScan with:
# - ranges: list of distances (meters)
# - intensities: signal strength
# - angle_increment: angular resolution
# - time_increment: time between measurements
```

---

## 🔷 NODE 3: object_selector_node

### **Purpose**
GUI interface for user to select visual tracking targets.

### **Responsibilities**
- Subscribe to camera feed
- Display video with OpenCV window
- Handle mouse clicks for object selection
- Show tracking status overlay
- Visualize tracking center
- Provide keyboard shortcuts

### **Subscribed Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image` | `sensor_msgs/Image` | Camera feed to display |
| `/tracking/active` | `std_msgs/Bool` | Tracking state |
| `/tracking/center` | `geometry_msgs/Point` | Current track position |

### **Published Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/tracking/select_point` | `geometry_msgs/Point` | User-selected point |

### **Services Used**
| Service | Type | Description |
|---------|------|-------------|
| `/tracking/toggle` | `std_srvs/SetBool` | Start/stop tracking |

### **Keyboard Commands**
- `ESC`: Exit
- `T`: Stop tracking

---

## 🔷 NODE 4: tracking_node

### **Purpose**
Visual object tracking using optical flow.

### **Responsibilities**
- Subscribe to camera frames
- Track selected point using Lucas-Kanade optical flow
- Calculate object center position
- Compute angle to object (relative to robot)
- Calculate distance from original position
- Smooth angle changes to reduce jitter
- Generate steering and velocity commands
- Implement steering-before-driving logic

### **Subscribed Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image` | `sensor_msgs/Image` | Camera frames |
| `/tracking/select_point` | `geometry_msgs/Point` | Target selection |
| `/obstacles/emergency` | `std_msgs/Bool` | Emergency stop from obstacle detection |

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/auto/cmd_vel` | `geometry_msgs/Twist` | Variable | Tracking velocity commands |
| `/steering_angles` | `std_msgs/Float64MultiArray` | Variable | Target wheel angles [base, base, base, base] |
| `/tracking/active` | `std_msgs/Bool` | 10 Hz | Tracking status |
| `/tracking/center` | `geometry_msgs/Point` | Variable | Current object center |

### **Services Provided**
| Service | Type | Description |
|---------|------|-------------|
| `/tracking/toggle` | `std_srvs/SetBool` | Enable/disable tracking |

### **Parameters**
- `max_speed`: 100 (PWM units)
- `center_threshold`: 30.0 (pixels - stop distance)
- `angle_send_threshold`: 5.0 (degrees - steering update threshold)
- `steering_delay`: 0.3 (seconds - wait after steering)
- `max_distance`: 200.0 (pixels - max speed distance)
- `frame_width`: 640
- `frame_height`: 480
- `pwm_change_threshold`: 20 (minimum change to send)
- `angle_alpha`: 0.3 (smoothing factor)

### **Algorithm Flow**
```
1. Receive point selection → initialize optical flow features
2. Track features frame-by-frame
3. Calculate center of tracked features
4. Compute angle to object: atan2(-dx, dy)
5. Smooth angle with exponential filter
6. If angle changed > threshold:
   - Send steering command
   - Stop wheels
   - Wait for steering_delay
7. Calculate distance to object
8. Compute speed = f(distance)
9. Send velocity command
```

---

## 🔷 NODE 5: obstacle_node (NEW)

### **Purpose**
Process LIDAR data for obstacle detection and collision avoidance.

### **Responsibilities**
- Subscribe to LIDAR scan data
- Detect obstacles in safety zones
- Calculate obstacle distances and angles
- Identify collision threats
- Publish emergency stop signals
- Generate obstacle map for navigation
- Filter and cluster LIDAR points

### **Subscribed Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR data |
| `/cmd_vel` | `geometry_msgs/Twist` | Current velocity intent |

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/obstacles/emergency` | `std_msgs/Bool` | 20 Hz | Immediate stop required |
| `/obstacles/map` | `sensor_msgs/PointCloud2` | 10 Hz | Obstacle point cloud |
| `/obstacles/closest` | `geometry_msgs/Point` | 10 Hz | Nearest obstacle (distance, angle) |
| `/obstacles/zones` | Custom msg | 10 Hz | Occupancy in safety zones |

### **Parameters**
- `emergency_stop_distance`: 0.3 (meters - immediate stop)
- `warning_distance`: 0.6 (meters - slow down)
- `safety_zones`:
  - `front`: 90° arc, 0.5m depth
  - `sides`: 60° arcs, 0.3m depth
  - `rear`: 90° arc, 0.25m depth
- `min_valid_range`: 0.15 (meters)
- `max_valid_range`: 5.0 (meters)

### **Safety Logic**
```
FOR each LIDAR point:
  IF distance < emergency_stop_distance:
    - Publish emergency=True
    - Stop all motion
  ELIF distance < warning_distance:
    - Reduce speed by 50%
    - Publish warning
  
  Categorize into zones (front/left/right/rear)
  
  IF moving forward AND front_zone occupied:
    - Emergency stop
  IF turning AND side_zone occupied:
    - Prevent turn
```

---

## 🔷 NODE 6: navigation_node (NEW)

### **Purpose**
High-level path planning and autonomous navigation using LIDAR.

### **Responsibilities**
- Accept navigation goals
- Plan collision-free paths
- Generate waypoints
- Compute velocity and steering commands
- React to dynamic obstacles
- Re-plan when path blocked
- Integrate with visual tracking

### **Subscribed Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | For obstacle awareness |
| `/obstacles/map` | `sensor_msgs/PointCloud2` | Processed obstacles |
| `/tracking/center` | `geometry_msgs/Point` | Visual target (optional) |

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/nav/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | Navigation velocity |
| `/nav/steering` | `std_msgs/Float64MultiArray` | 10 Hz | Navigation steering |
| `/nav/path` | `nav_msgs/Path` | 1 Hz | Planned path |
| `/nav/status` | Custom msg | 1 Hz | Navigation state |

### **Services Provided**
| Service | Type | Description |
|---------|------|-------------|
| `/nav/set_goal` | Custom srv | Set navigation target |
| `/nav/cancel` | `std_srvs/Trigger` | Cancel navigation |

### **Parameters**
- `planning_frequency`: 10 Hz
- `goal_tolerance_distance`: 0.2 (meters)
- `goal_tolerance_angle`: 10 (degrees)
- `max_velocity`: 0.5 (m/s)
- `obstacle_clearance`: 0.4 (meters)
- `planner_type`: "DWA" or "TEB"

### **Navigation Modes**
1. **Point-to-Point**: Navigate to XY coordinate
2. **Visual Tracking**: Follow tracked object while avoiding obstacles
3. **Explore**: Random exploration with obstacle avoidance
4. **Return Home**: Navigate back to start position

---

## 🔷 NODE 7: cmd_vel_mux_node (NEW)

### **Purpose**
Arbitrate between multiple command sources based on priority.

### **Responsibilities**
- Subscribe to multiple cmd_vel sources
- Implement priority system
- Handle emergency stops
- Smooth transitions between sources
- Prevent conflicting commands
- Publish final unified commands

### **Subscribed Topics**
| Topic | Type | Priority | Description |
|-------|------|----------|-------------|
| `/obstacles/emergency` | `std_msgs/Bool` | 0 (highest) | Emergency stop |
| `/manual/cmd_vel` | `geometry_msgs/Twist` | 1 | Manual control |
| `/nav/cmd_vel` | `geometry_msgs/Twist` | 2 | Navigation |
| `/auto/cmd_vel` | `geometry_msgs/Twist` | 3 (lowest) | Visual tracking |
| `/manual/steering_angles` | `Float64MultiArray` | 1 | Manual steering |
| `/nav/steering` | `Float64MultiArray` | 2 | Nav steering |
| `/steering_angles` | `Float64MultiArray` | 3 | Tracking steering |

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | Final velocity command |
| `/cmd_steering` | `std_msgs/Float64MultiArray` | 20 Hz | Final steering command |
| `/mux/active_source` | `std_msgs/String` | 5 Hz | Current command source |

### **Priority Logic**
```
1. IF emergency_stop == True:
     → cmd_vel = ZERO, ignore all others

2. ELIF manual command received (age < 0.5s):
     → Use manual commands
     → Block all autonomous sources

3. ELIF navigation active AND has recent command:
     → Use navigation commands

4. ELIF tracking active AND has recent command:
     → Use tracking commands

5. ELSE:
     → cmd_vel = ZERO (safety)
```

### **Parameters**
- `timeout_manual`: 0.5 (seconds)
- `timeout_auto`: 1.0 (seconds)
- `smooth_transition_time`: 0.2 (seconds)

---

## 🔷 NODE 8: teleop_node

### **Purpose**
Manual control via Xbox controller.

### **Responsibilities**
- Subscribe to joy messages
- Interpret controller inputs
- Generate manual cmd_vel commands
- Handle steering triggers
- Mode switching (manual/auto)
- Apply deadzone filtering

### **Subscribed Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Xbox controller input |

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/manual/cmd_vel` | `geometry_msgs/Twist` | 20 Hz | Manual velocity |
| `/manual/steering_angles` | `Float64MultiArray` | Event | Manual steering |

### **Services Provided**
| Service | Type | Description |
|---------|------|-------------|
| `/teleop/toggle_mode` | `std_srvs/SetBool` | Switch manual/auto |

### **Controller Mapping**
| Input | Function |
|-------|----------|
| Left Stick Y | Forward/backward speed |
| Left Stick X | Lateral strafe (future) |
| Right Stick X | Rotation (future) |
| Left Trigger | Steer -45° |
| Right Trigger | Steer +45° |
| Y Button | Toggle manual/auto mode |

### **Parameters**
- `max_speed`: 100 (PWM units)
- `deadzone`: 0.18
- `max_angular`: 1.0

---

## 🔷 NODE 9: hardware_node

### **Purpose**
Interface between ROS2 and Arduino hardware.

### **Responsibilities**
- Manage serial connection to Arduino
- Subscribe to velocity/steering commands
- Apply per-wheel calibration offsets
- Convert ROS commands to Arduino protocol
- Handle serial reconnection
- Initialize/calibrate hardware on startup
- Send D/S/C commands

### **Subscribed Topics**
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Final velocity (from mux) |
| `/cmd_steering` | `Float64MultiArray` | Final steering (from mux) |
| `/auto/cmd_vel` | `geometry_msgs/Twist` | Direct auto commands (no mux) |
| `/manual/cmd_vel` | `geometry_msgs/Twist` | Direct manual commands (no mux) |
| `/steering_angles` | `Float64MultiArray` | Direct steering (no mux) |
| `/manual/steering_angles` | `Float64MultiArray` | Manual steering |

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/hardware/status` | `std_msgs/String` | 1 Hz | Connection status |
| `/hardware/feedback` | Custom msg | 10 Hz | Arduino feedback (future) |

### **Parameters**
- `serial_port`: "" (auto-detect)
- `serial_baud`: 115200
- `max_speed`: 100
- `wheel_drive_dir`: [1, 1, 1, 1] (per-wheel direction)
- `steer_dir`: [1, 1, 1, 1] (steering direction)
- `steer_offsets`: [0, 0, 0, 0] (angle offsets in degrees)

### **Arduino Protocol**
```
D speed1 speed2 speed3 speed4   → Drive command (PWM 1000-2000)
S angle1 angle2 angle3 angle4   → Steering command (degrees 0-359)
C module_id angle               → Calibrate stepper position
```

### **compose_wheel_angles() Function**
```python
# Applies per-wheel offsets and directions
def compose_wheel_angles(base_angle):
    angles = []
    for i in range(4):
        angle = (steer_dir[i] * base_angle + steer_offsets[i]) % 360
        angles.append(int(angle))
    return angles
```

---

## 🔷 NODE 10: joy_node (External Package)

### **Purpose**
Publish joystick/controller input as ROS messages.

### **Published Topics**
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | 20 Hz | Controller state |

### **Parameters**
- `dev`: `/dev/input/js0`
- `deadzone`: 0.3
- `autorepeat_rate`: 20.0

---

## 📊 Complete Communication Graph

```
                    ┌─────────────┐
                    │ joy_node    │
                    │ (external)  │
                    └──────┬──────┘
                           │ /joy
                           ▼
┌────────────┐      ┌─────────────┐
│camera_node │─────►│ object_     │
│            │/image│ selector    │
└────────────┘      └──────┬──────┘
                           │ /tracking/select_point
                           ▼
┌────────────┐      ┌─────────────────┐      ┌──────────────┐
│ lidar_node │─────►│  tracking_node  │─────►│ cmd_vel_mux  │
│            │/scan └─────────────────┘/auto/└──────┬───────┘
└────────────┘            │              cmd_vel    │
      │                   │ /steering              │/cmd_vel
      │/scan              │  _angles               │/cmd_steering
      ▼                   ▼                        │
┌──────────────┐    ┌─────────────┐               │
│obstacle_node │    │navigation_  │───────────────┤
│              │───►│    node     │/nav/cmd_vel   │
└──────┬───────┘    └─────────────┘               │
       │ /emergency                                │
       └──────────────────────────────────────────►│
                                                   │
┌─────────────┐                                    │
│ teleop_node │───────────────────────────────────►│
│             │/manual/cmd_vel                     │
└─────────────┘                                    │
                                                   ▼
                                          ┌─────────────────┐
                                          │ hardware_node   │
                                          │                 │
                                          └────────┬────────┘
                                                   │ Serial
                                                   ▼
                                              ┌─────────┐
                                              │ Arduino │
                                              └─────────┘
```

---

## 🔄 Data Flow Examples

### **Example 1: Visual Tracking with Obstacle Avoidance**
```
1. User clicks object in object_selector
   → /tracking/select_point

2. tracking_node starts tracking
   → Publishes /auto/cmd_vel (forward)
   → Publishes /steering_angles (toward object)

3. lidar_node detects obstacle ahead
   → /scan

4. obstacle_node processes scan
   → Detects obstacle < 0.3m
   → Publishes /obstacles/emergency = True

5. cmd_vel_mux_node receives emergency
   → Overrides tracking commands
   → Publishes /cmd_vel = ZERO
   → Publishes /cmd_steering = HOLD

6. hardware_node receives zero velocity
   → Sends "D 1500 1500 1500 1500" to Arduino
   → Robot stops
```

### **Example 2: Manual Control**
```
1. User presses Y button on controller
   → joy_node publishes /joy

2. teleop_node detects Y button
   → Switches to manual mode

3. User pushes left stick forward
   → teleop_node publishes /manual/cmd_vel (linear.x = 50)

4. User presses right trigger
   → teleop_node publishes /manual/steering_angles = [45, 45, 45, 45]

5. cmd_vel_mux_node sees manual commands (priority 1)
   → Blocks auto/nav commands
   → Publishes /cmd_vel and /cmd_steering

6. hardware_node receives commands
   → Applies compose_wheel_angles()
   → Sends "S 45 45 45 45" and "D 1550 1550 1550 1550"

7. Arduino executes
   → Steppers turn to 45°
   → ESCs run at 1550 PWM
```

### **Example 3: Autonomous Navigation**
```
1. User sends navigation goal via service
   → /nav/set_goal

2. navigation_node receives goal
   → Plans path using LIDAR data
   → Starts publishing /nav/cmd_vel and /nav/steering

3. tracking_node also running
   → Publishing /auto/cmd_vel (lower priority)

4. cmd_vel_mux_node arbitrates
   → Navigation (priority 2) > Tracking (priority 3)
   → Publishes navigation commands

5. Robot follows path
   → Avoids obstacles from obstacle_node
   → Reaches goal

6. navigation_node stops publishing
   → Tracking commands now active (no higher priority)
   → Robot starts tracking visual target
```

---

## 🛠️ Implementation Phases

### **Phase 1: Current System** ✅
- camera_node
- tracking_node
- object_selector_node
- teleop_node
- hardware_node
- joy_node

### **Phase 2: LIDAR Integration** 🚧
1. Add lidar_node
2. Add obstacle_node
3. Test emergency stop
4. Integrate with tracking

### **Phase 3: Navigation** 🔜
1. Add navigation_node
2. Implement path planning
3. Test autonomous navigation

### **Phase 4: Arbitration** 🔜
1. Add cmd_vel_mux_node
2. Implement priority system
3. Test mode switching

---

## 📁 Suggested File Structure

```
my_robot_tracking/
├── camera_node.py          ✅ Exists
├── tracking_node.py        ✅ Exists
├── hardware_node.py        ✅ Exists
├── teleop_node.py          ✅ Exists
├── lidar_node.py           ❌ NEW
├── obstacle_node.py        ❌ NEW
├── navigation_node.py      ❌ NEW
├── cmd_vel_mux_node.py     ❌ NEW
├── scripts/
│   └── object_selector.py  ✅ Exists
├── launch/
│   ├── tracking_system.launch.py       ✅ Exists
│   ├── navigation_system.launch.py     ❌ NEW (full system)
│   └── sensors.launch.py               ❌ NEW (camera+lidar)
├── config/
│   ├── tracking_config.yaml            ✅ Exists
│   ├── lidar_config.yaml               ❌ NEW
│   ├── navigation_config.yaml          ❌ NEW
│   └── safety_zones.yaml               ❌ NEW
└── msg/
    ├── ObstacleZones.msg               ❌ NEW
    └── NavigationStatus.msg            ❌ NEW
```

---

## 🎓 Key Design Principles

1. **Separation of Concerns**: Each node has ONE clear responsibility
2. **Priority-Based Arbitration**: Safety always wins
3. **Loose Coupling**: Nodes communicate via topics, not direct calls
4. **Fail-Safe**: Emergency stop at hardware level
5. **Modularity**: Can disable any node without breaking system
6. **Scalability**: Easy to add new sensors or behaviors

---

## 🚀 Next Steps

### To add LIDAR support:
1. Install LIDAR driver: `sudo apt install ros-humble-rplidar-ros`
2. Create `lidar_node.py` wrapper
3. Create `obstacle_node.py` for safety
4. Update launch file to include LIDAR nodes
5. Test emergency stop functionality

### To add navigation:
1. Create `navigation_node.py`
2. Implement simple path planning (e.g., VFH or DWA)
3. Create `cmd_vel_mux_node.py` for priority arbitration
4. Test integration with tracking and obstacle avoidance

### Configuration tips:
- Start with high `emergency_stop_distance` (0.5m+) for safety
- Tune tracking parameters before adding navigation
- Test each layer independently before combining
- Use `ros2 topic echo` to debug topic flow

---

**This document provides the complete blueprint for your autonomous swerve robot system!** 🎯🤖

