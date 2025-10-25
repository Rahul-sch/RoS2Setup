#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import threading

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        # Declare parameters
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('center_threshold', 30.0)
        self.declare_parameter('angle_send_threshold', 5.0)
        self.declare_parameter('steering_delay', 0.3)
        self.declare_parameter('max_distance', 200.0)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.center_threshold = self.get_parameter('center_threshold').value
        self.angle_send_threshold = self.get_parameter('angle_send_threshold').value
        self.steering_delay = self.get_parameter('steering_delay').value
        self.max_distance = self.get_parameter('max_distance').value
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Tracking state
        self.tracking_active = False
        self.original_center = None
        self.last_center = None
        self.smoothed_angle = None
        self.steering_ready = True
        self.last_sent_angle = None
        self.last_steer_command_time = 0.0
        
        # Optical flow parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03)
        )
        self.prev_gray = None
        self.prev_points = None
        self.angle_alpha = 0.3  # Smoothing factor
        
        # Create subscribers and publishers
        self.image_subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tracking_status_publisher = self.create_publisher(Bool, '/tracking/active', 10)
        
        # Create service for starting/stopping tracking
        from std_srvs.srv import SetBool
        self.tracking_service = self.create_service(
            SetBool, '/tracking/toggle', self.toggle_tracking_callback)
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('Tracking node started')
    
    def image_callback(self, msg):
        """Process incoming camera frames"""
        if not self.tracking_active:
            return
        
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            if self.prev_points is not None and self.prev_gray is not None:
                # Track points using optical flow
                next_points, status, err = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray, self.prev_points, None, **self.lk_params
                )
                
                if next_points is not None and status is not None:
                    next_points = next_points.reshape(-1, 2)
                    status = status.reshape(-1)
                    good_new = next_points[status == 1]
                    
                    if len(good_new) > 0:
                        # Update tracking points
                        self.prev_points = good_new.reshape(-1, 1, 2)
                        self.prev_gray = gray.copy()
                        
                        # Calculate center of tracked points
                        center = np.mean(good_new, axis=0)
                        cx, cy = int(center[0]), int(center[1])
                        
                        # Process movement
                        self.process_movement((cx, cy))
                    else:
                        self.get_logger().warn("Lost tracking points")
                        self.stop_robot()
            else:
                self.prev_gray = gray.copy()
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def process_movement(self, center):
        """Process robot movement based on object position (from your moveRobot function)"""
        cx, cy = center
        
        # Initialize on first call
        if self.last_center is None:
            self.last_center = (cx, cy)
            if self.original_center is None:
                self.original_center = (cx, cy)
                self.get_logger().info("Tracking started - will follow object")
            return
        
        # Calculate displacement from original center
        dx = cx - self.original_center[0]
        dy = cy - self.original_center[1]
        distance = math.hypot(dx, dy)
        
        # Stop if too close to center
        if distance < self.center_threshold:
            self.stop_robot()
            self.last_center = (cx, cy)
            return
        
        # Calculate angle to object
        raw_angle = (math.degrees(math.atan2(-dx, dy)) + 360.0) % 360.0
        
        # Smooth angle to reduce jitter
        if self.smoothed_angle is None:
            self.smoothed_angle = raw_angle
        else:
            angle_diff = self.shortest_angle_diff(raw_angle, self.smoothed_angle)
            self.smoothed_angle = (self.smoothed_angle + self.angle_alpha * angle_diff) % 360.0
        
        # Check if we need to update steering
        if self.last_sent_angle is not None:
            angle_diff = abs(self.shortest_angle_diff(self.smoothed_angle, self.last_sent_angle))
            if angle_diff > self.angle_send_threshold:
                # Send steering command
                self.send_steering_command(self.smoothed_angle)
                self.last_sent_angle = self.smoothed_angle
                self.last_steer_command_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.steering_ready = False
                self.stop_robot()  # Stop while turning
                self.last_center = (cx, cy)
                return
        
        # Wait for steering to complete
        if not self.steering_ready:
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            if current_time - self.last_steer_command_time >= self.steering_delay:
                self.steering_ready = True
            else:
                self.last_center = (cx, cy)
                return
        
        # Calculate speed based on distance
        speed_factor = min(distance / self.max_distance, 1.0)
        drive_speed = int(round(speed_factor * self.max_speed))
        
        # Send movement command
        self.send_movement_command(drive_speed)
        self.last_center = (cx, cy)
    
    def send_steering_command(self, angle):
        """Send steering command (will be handled by hardware node)"""
        # For now, we'll publish the angle as a custom message
        # In a full implementation, this would be handled by the hardware node
        self.get_logger().info(f"Steering to angle: {angle}")
    
    def send_movement_command(self, speed):
        """Send movement command as Twist message"""
        twist = Twist()
        twist.linear.x = float(speed)  # Forward/backward speed
        twist.linear.y = 0.0          # Lateral speed (not used in this implementation)
        twist.angular.z = 0.0          # Rotation speed
        self.cmd_vel_publisher.publish(twist)
    
    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
    
    def shortest_angle_diff(self, a, b):
        """Calculate shortest angle difference"""
        return (a - b + 180.0) % 360.0 - 180.0
    
    def toggle_tracking_callback(self, request, response):
        """Service callback to start/stop tracking"""
        if request.data:
            # Start tracking - this would be triggered by clicking on image
            self.tracking_active = True
            self.original_center = None
            self.last_center = None
            self.smoothed_angle = None
            self.steering_ready = True
            self.last_sent_angle = None
            response.success = True
            response.message = "Tracking started"
        else:
            # Stop tracking
            self.tracking_active = False
            self.stop_robot()
            response.success = True
            response.message = "Tracking stopped"
        return response
    
    def publish_status(self):
        """Publish tracking status"""
        status_msg = Bool()
        status_msg.data = self.tracking_active
        self.tracking_status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
