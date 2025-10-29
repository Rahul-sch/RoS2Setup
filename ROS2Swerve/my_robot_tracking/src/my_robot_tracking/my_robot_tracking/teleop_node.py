#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool
import math

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Declare parameters
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('deadzone', 0.18)
        self.declare_parameter('max_angular', 1.0)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.max_angular = self.get_parameter('max_angular').value
        
        # State
        self.manual_mode = True
        self.last_y_press_time = 0.0
        self.manual_target_angle = 0.0
        self.current_manual_angle = None
        self.tracking_active = False
        
        # Initialize joystick axes
        self.forward_axis = 0.0
        self.lateral_axis = 0.0
        self.angular_axis = 0.0
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.last_left_trigger = False
        self.last_right_trigger = False
        
        # Create subscribers and publishers
        self.joy_subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.tracking_status_subscription = self.create_subscription(
            Bool, '/tracking/active', self.tracking_status_callback, 10)
        
        # Publish to /manual/cmd_vel to avoid conflicts with auto tracking
        self.cmd_vel_publisher = self.create_publisher(Twist, '/manual/cmd_vel', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/manual/steering_angles', 10)
        
        # Create service for mode switching
        self.mode_service = self.create_service(
            SetBool, '/teleop/toggle_mode', self.toggle_mode_callback)
        
        # Create timer for publishing commands
        self.cmd_timer = self.create_timer(0.05, self.publish_commands)
        
        self.get_logger().info('Teleop node started')
    
    def joy_callback(self, msg):
        """Handle joystick input (from your track.py controller_loop)"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0] / 1e9
        
        # Y button to toggle mode (button 3 in Joy message)
        if len(msg.buttons) > 3 and msg.buttons[3] == 1:
            if current_time - self.last_y_press_time > 0.5:
                if self.tracking_active:
                    self.get_logger().warn("Tracking active - ignoring manual/auto toggle")
                    self.last_y_press_time = current_time
                    return
                self.manual_mode = not self.manual_mode
                mode = "MANUAL" if self.manual_mode else "AUTO"
                self.get_logger().info(f"Mode switched to: {mode}")
                self.last_y_press_time = current_time
                
                if self.manual_mode:
                    # Reset manual state
                    self.manual_target_angle = 0.0
                    self.current_manual_angle = None
        
        # Manual control
        if self.manual_mode and len(msg.axes) >= 4:
            # Left stick Y axis (forward/backward)
            self.forward_axis = -msg.axes[1] if len(msg.axes) > 1 else 0.0
            
            # Left stick X axis (left/right strafe)
            self.lateral_axis = msg.axes[0] if len(msg.axes) > 0 else 0.0
            
            # Right stick X axis (rotation)
            self.angular_axis = msg.axes[2] if len(msg.axes) > 2 else 0.0
            
            # Triggers for steering (axes 2 and 5 on Xbox controller)
            # On Linux, triggers are typically axes 2 and 5, ranging from -1 to 1
            self.left_trigger = (msg.axes[2] + 1.0) / 2.0 if len(msg.axes) > 2 else 0.0
            self.right_trigger = (msg.axes[5] + 1.0) / 2.0 if len(msg.axes) > 5 else 0.0
            
            # Handle steering with triggers (edge detection)
            left_trigger_pressed = self.left_trigger > 0.5
            right_trigger_pressed = self.right_trigger > 0.5
            
            if left_trigger_pressed and not self.last_left_trigger:
                self.manual_target_angle = (self.manual_target_angle - 45) % 360
                self.send_steering_command()
                self.get_logger().info(f"Steering left to {self.manual_target_angle}°")
            
            if right_trigger_pressed and not self.last_right_trigger:
                self.manual_target_angle = (self.manual_target_angle + 45) % 360
                self.send_steering_command()
                self.get_logger().info(f"Steering right to {self.manual_target_angle}°")
            
            self.last_left_trigger = left_trigger_pressed
            self.last_right_trigger = right_trigger_pressed
        else:
            # Auto mode - don't send manual commands
            self.forward_axis = 0.0
            self.lateral_axis = 0.0
            self.angular_axis = 0.0
    
    def publish_commands(self):
        """Publish movement commands based on current state"""
        if self.manual_mode:
            # Manual control
            twist = Twist()
            
            # Apply deadzone
            if abs(self.forward_axis) < self.deadzone:
                self.forward_axis = 0.0
            if abs(self.lateral_axis) < self.deadzone:
                self.lateral_axis = 0.0
            if abs(self.angular_axis) < self.deadzone:
                self.angular_axis = 0.0
            
            # Convert to movement commands
            twist.linear.x = self.forward_axis * self.max_speed
            twist.linear.y = self.lateral_axis * self.max_speed
            twist.angular.z = self.angular_axis * self.max_angular
            
            self.cmd_vel_publisher.publish(twist)
        else:
            # Auto mode - don't publish manual commands
            # The tracking node will handle movement
            pass
    
    def toggle_mode_callback(self, request, response):
        """Service callback to toggle manual/auto mode"""
        self.manual_mode = request.data
        mode = "MANUAL" if self.manual_mode else "AUTO"
        self.get_logger().info(f"Mode switched to: {mode}")
        response.success = True
        response.message = f"Mode switched to {mode}"
        return response
    
    def send_steering_command(self):
        """Send steering command to hardware node"""
        steering_msg = Float64MultiArray()
        # Send same angle for all 4 wheels - hardware node will apply offsets
        steering_msg.data = [float(self.manual_target_angle)] * 4
        self.steering_publisher.publish(steering_msg)
    
    def tracking_status_callback(self, msg: Bool):
        """Automatically enter auto mode when tracking is active."""
        self.tracking_active = msg.data
        if self.tracking_active:
            if self.manual_mode:
                self.manual_mode = False
                self.get_logger().info("Tracking active - switching teleop to AUTO mode")
        else:
            if not self.manual_mode:
                self.manual_mode = True
                self.get_logger().info("Tracking inactive - returning teleop to MANUAL mode")
                self.manual_target_angle = 0.0
                self.current_manual_angle = None
    
    def normalize_axis(self, value, min_val=-1.0, max_val=1.0):
        """Normalize axis value (from your track.py)"""
        if max_val == min_val:
            return 0.0
        mid = (max_val + min_val) / 2.0
        span = (max_val - min_val) / 2.0
        norm = (value - mid) / span
        if abs(norm) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, norm))

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
