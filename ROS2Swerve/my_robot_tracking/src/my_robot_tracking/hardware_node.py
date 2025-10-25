#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial
import time
import threading

class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('wheel_drive_dir', [1, 1, 1, 1])
        self.declare_parameter('steer_dir', [1, 1, 1, 1])
        self.declare_parameter('steer_offsets', [0, 0, 0, 0])
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_drive_dir = self.get_parameter('wheel_drive_dir').value
        self.steer_dir = self.get_parameter('steer_dir').value
        self.steer_offsets = self.get_parameter('steer_offsets').value
        
        # Serial communication
        self.serial_lock = threading.Lock()
        self.ser = None
        self.last_drive_pwms = [1500, 1500, 1500, 1500]
        self.last_sent_angle = None
        
        # Initialize serial connection
        self.connect_to_arduino()
        
        # Create subscribers
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Create timer for serial communication
        self.serial_timer = self.create_timer(0.05, self.process_serial)
        
        self.get_logger().info('Hardware node started')
    
    def connect_to_arduino(self):
        """Connect to Arduino (from your track.py)"""
        try:
            self.ser = serial.Serial(
                self.serial_port,
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
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
            
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
    
    def cmd_vel_callback(self, msg):
        """Handle movement commands from tracking node"""
        # Convert Twist to wheel speeds and angles
        # This is a simplified version - in practice you'd need proper swerve kinematics
        
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
                except serial.SerialException as e:
                    self.get_logger().error(f"Steer command failed: {e}")
                    self.attempt_reconnect()
    
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
        """Process serial communication (placeholder for now)"""
        # In a full implementation, this would handle incoming serial data
        # For now, we just ensure the connection is maintained
        if self.ser is None:
            self.connect_to_arduino()
    
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
