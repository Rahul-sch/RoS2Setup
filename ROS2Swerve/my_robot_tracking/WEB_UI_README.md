# MedRa Web UI

Web-based user interface for controlling the MedRa robot via ROS2.

## Features

- **Login/Register Page**: MedRa branding with login/register buttons (auth not implemented yet)
- **Dashboard**: Main menu with Manual Mode and Camera buttons, doctor name display
- **Manual Control**: Arrow buttons for robot movement, top half controls with +/- buttons
- **Camera View**: Live camera feed with tracking status (click to select tracking target)

## Setup

1. **Install dependencies:**
   ```bash
   pip install flask flask-cors
   ```

2. **Build the package:**
   ```bash
   cd ~/Ros2Setup/ROS2Swerve
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running

1. **Start ROS2 nodes** (in separate terminals):
   ```bash
   # Terminal 1: Camera
   ros2 run my_robot_tracking camera_node
   
   # Terminal 2: Hardware interface
   ros2 run my_robot_tracking hardware_node
   
   # Terminal 3: Web UI Server
   ros2 run my_robot_tracking web_ui_server
   ```

2. **Open browser:**
   - Navigate to: `http://localhost:5000`
   - Or from another device on the network: `http://<robot-ip>:5000`

## Usage

1. **Login Page**: Click "Login" or "Register" to enter (currently just navigates to dashboard)

2. **Dashboard**:
   - Click "Manual Mode" for robot control
   - Click "Camera" for camera/tracking view

3. **Manual Control**:
   - **Arrow buttons**: Control robot movement
     - ↑ = Forward
     - ↓ = Backward
     - ← = Rotate left
     - → = Rotate right
   - **Top Half Controls**:
     - Two ↑ buttons: Move top half up
     - Two ↓ buttons: Move top half down
     - +/- buttons: Adjust top half position

4. **Camera View**:
   - Shows live camera feed
   - Click on image to select tracking target
   - Status indicator shows tracking state

## Keyboard Controls (Manual Mode)

- Arrow keys: Same as button controls
- Release to stop

## API Endpoints

- `GET /` - Login page
- `GET /dashboard` - Main dashboard
- `GET /manual` - Manual control page
- `GET /camera` - Camera view page
- `POST /api/move` - Move robot (direction: 'up', 'down', 'left', 'right')
- `POST /api/stop` - Stop robot
- `POST /api/steer` - Steer robot (angle in degrees)
- `POST /api/top_half` - Control top half (action: 'up', 'down', etc.)
- `GET /api/status` - Get robot status
- `GET /api/camera_frame` - Get latest camera frame (base64)

## Customization

- **Doctor Name**: Edit `dashboard.html` template or pass via Flask route
- **Styling**: Edit `static/style.css`
- **Functionality**: Edit `web_ui_server.py` and JavaScript files in `static/`

## Future Enhancements

- Supabase authentication integration
- Real-time tracking visualization on camera page
- Settings/preferences page
- Battery status and other telemetry

