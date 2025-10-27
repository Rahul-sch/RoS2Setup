#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
import serial
import time
import threading
import glob

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '')  # Empty = auto-detect
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('wheel_drive_dir', [1, 1, 1, 1])
        self.declare_parameter('steer_dir', [1, 1, 1, 1])
        self.declare_parameter('steer_offsets', [0, 0, 0, 0])
        self.declare_parameter('auto_mode_topic', '/auto/cmd_vel')
        self.declare_parameter('manual_mode_topic', '/manual/cmd_vel')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_drive_dir = self.get_parameter('wheel_drive_dir').value
        self.steer_dir = self.get_parameter('steer_dir').value
        self.steer_offsets = self.get_parameter('steer_offsets').value
        self.auto_mode_topic = self.get_parameter('auto_mode_topic').value
        self.manual_mode_topic = self.get_parameter('manual_mode_topic').value
        
        # Serial communication
        self.serial_lock = threading.Lock()
        self.ser = None
        self.last_drive_pwms = [1500, 1500, 1500, 1500]
        self.current_steering_angles = [0, 0, 0, 0]
        self.pending_steering = None
        
        # Initialize serial connection
        self.connect_to_arduino()
        
        # Create subscribers - both auto and manual cmd_vel
        self.auto_cmd_vel_subscription = self.create_subscription(
            Twist, self.auto_mode_topic, self.auto_cmd_vel_callback, 10)
        self.manual_cmd_vel_subscription = self.create_subscription(
            Twist, self.manual_mode_topic, self.manual_cmd_vel_callback, 10)
        
        # Subscribe to steering angles
        self.steering_subscription = self.create_subscription(
            Float64MultiArray, '/steering_angles', self.steering_callback, 10)
        
        # Subscription for manual steering from teleop
        self.manual_steering_subscription = self.create_subscription(
            Float64MultiArray, '/manual/steering_angles', self.steering_callback, 10)
        
        # Create timer for serial communication
        self.serial_timer = self.create_timer(0.05, self.process_serial)
        
        self.get_logger().info('Hardware node started')
    
    def connect_to_arduino(self):
        """Connect to Arduino (from your track.py)"""
        try:
            # Auto-detect serial port if not specified
            port = self.serial_port
            if not port:
                candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
                if not candidates:
                    raise FileNotFoundError("No serial devices matching /dev/ttyACM* or /dev/ttyUSB* found")
                port = candidates[0]
                self.get_logger().info(f"Auto-detected serial port: {port}")
            
            self.ser = serial.Serial(
                port,
                self.serial_baud,
                timeout=1,
                write_timeout=2,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            time.sleep(0.05)
            self.ser.setDTR(True)
            self.ser.setRTS(True)
            time.sleep(0.1)
            self.get_logger().info(f"Connected to Arduino on {port}")
            
            # Initialize steppers to 0 degrees
            self.send_steer_command([0, 0, 0, 0])
            time.sleep(0.3)
            
            # Calibrate positions
            self.calibrate_steppers()
            
        except Exception as e:
            self.get_logger().error(f"Could not connect to Arduino: {e}")
            self.get_logger().info("Will retry connection in background...")
            self.ser = None
    
    def calibrate_steppers(self):
        """Calibrate stepper positions (from your track.py)"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"C 0 0\nC 1 0\nC 2 0\nC 3 0\n")
                time.sleep(0.1)
                self.get_logger().info("Stepper calibration complete")
            except Exception as e:
                self.get_logger().error(f"Calibration failed: {e}")
    
    def compose_wheel_angles(self, base_angle):
        """Apply per-wheel steering offsets and directions (from track.py)"""
        angles = []
        base = int(round(base_angle)) % 360
        for i in range(4):
            ang = (self.steer_dir[i] * base + self.steer_offsets[i]) % 360
            angles.append(int(round(ang)))
        return angles
    
    def steering_callback(self, msg):
        """Handle steering angle commands"""
        if len(msg.data) >= 4:
            # If all 4 angles are the same, it's a base angle to be composed
            if msg.data[0] == msg.data[1] == msg.data[2] == msg.data[3]:
                base_angle = msg.data[0]
                angles = self.compose_wheel_angles(base_angle)
            else:
                # Already composed angles
                angles = [int(a) for a in msg.data]
            
            self.pending_steering = angles
            self.get_logger().debug(f"Received steering command: {angles}")
    
    def auto_cmd_vel_callback(self, msg):
        """Handle movement commands from tracking node (auto mode)"""
        self._handle_cmd_vel(msg, "auto")
    
    def manual_cmd_vel_callback(self, msg):
        """Handle movement commands from teleop (manual mode)"""
        self._handle_cmd_vel(msg, "manual")
    
    def _handle_cmd_vel(self, msg, source):
        """Handle movement commands - unified for auto and manual"""
        # Convert Twist to wheel speeds
        # For now, just forward/backward movement
        if abs(msg.linear.x) > 0.1:
            speed = int(msg.linear.x)
            # Clamp speed
            speed = max(-self.max_speed, min(self.max_speed, speed))
            
            # Convert to PWM values
            pwms = [1500 + self.wheel_drive_dir[i] * speed for i in range(4)]
            self.send_drive_command(pwms)
        else:
            # Stop
            self.send_drive_command([1500, 1500, 1500, 1500])
    
    def send_drive_command(self, speeds):
        """Send drive command to Arduino (from your track.py)"""
        speeds = [int(min(2000, max(1000, round(v)))) for v in speeds]
        cmd = f"D {speeds[0]} {speeds[1]} {speeds[2]} {speeds[3]}\n"
        
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(cmd.encode())
                    self.get_logger().debug(f"Drive command: {cmd.strip()}")
                except serial.SerialException as e:
                    self.get_logger().error(f"Drive command failed: {e}")
                    self.attempt_reconnect()
    
    def send_steer_command(self, angles):
        """Send steering command to Arduino (from your track.py)"""
        angles = [int(round(v)) for v in angles]
        cmd = f"S {angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
        
        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(cmd.encode())
                    self.get_logger().debug(f"Steer command: {cmd.strip()}")
                    return True
                except serial.SerialException as e:
                    self.get_logger().error(f"Steer command failed: {e}")
                    self.attempt_reconnect()
                    return False
        return False
    
    def attempt_reconnect(self):
        """Attempt to reconnect to Arduino"""
        with self.serial_lock:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
        
        self.get_logger().warn("Attempting to reconnect to Arduino...")
        time.sleep(2.0)
        self.connect_to_arduino()
    
    def process_serial(self):
        """Process serial communication and send pending commands"""
        # Reconnect if disconnected
        if self.ser is None or not self.ser.is_open:
            self.connect_to_arduino()
            return
        
        # Send pending steering commands
        if self.pending_steering is not None:
            if self.send_steer_command(self.pending_steering):
                self.current_steering_angles = self.pending_steering
                self.pending_steering = None
    
    def destroy_node(self):
        """Clean up resources"""
        if self.ser and self.ser.is_open:
            try:
                # Stop all motors
                self.send_drive_command([1500, 1500, 1500, 1500])
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
