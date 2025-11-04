# Raspberry Pi Auto-Start UI Setup

This guide shows how to make the MedRa UI automatically start on boot and display fullscreen on the HDMI screen.

## Quick Setup

### 1. Make scripts executable

```bash
cd ~/Ros2Setup/ROS2Swerve/my_robot_tracking
chmod +x start_kiosk_ui.sh
```

### 2. Edit the systemd service file

Edit `medra-ui.service` and replace:
- `YOUR_USERNAME` with your actual Raspberry Pi username (usually `pi` or `ubuntu`)

### 3. Install systemd service

```bash
# Copy service file to systemd directory
sudo cp medra-ui.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service to start on boot
sudo systemctl enable medra-ui.service

# Start service now (optional, to test)
sudo systemctl start medra-ui.service
```

### 4. Install Chromium (if not already installed)

```bash
sudo apt update
sudo apt install chromium-browser -y
```

## How It Works

1. **On boot**: The systemd service starts automatically
2. **Flask backend**: Starts on port 8080 (connects to ROS2)
3. **Browser**: Launches in kiosk mode (fullscreen, no UI)
4. **Display**: Shows camera page automatically

## Manual Start (for testing)

```bash
./start_kiosk_ui.sh
```

## Troubleshooting

### Check if service is running

```bash
sudo systemctl status medra-ui.service
```

### View logs

```bash
sudo journalctl -u medra-ui.service -f
```

### Stop the service

```bash
sudo systemctl stop medra-ui.service
```

### Disable auto-start

```bash
sudo systemctl disable medra-ui.service
```

### Check if Flask is running

```bash
ps aux | grep web_ui_server
```

### Check if browser is running

```bash
ps aux | grep chromium
```

## Manual Control

If you need to exit kiosk mode:
- Press `Alt+F4` or `Ctrl+Alt+T` to open terminal
- Or SSH into the Pi and stop the service

## Notes

- The UI will automatically show the **camera page** on startup
- Make sure ROS2 nodes are running (camera_node, hardware_node, etc.)
- The screen will be fullscreen with no browser UI
- To access other pages, you'll need to manually navigate (or add navigation buttons)

