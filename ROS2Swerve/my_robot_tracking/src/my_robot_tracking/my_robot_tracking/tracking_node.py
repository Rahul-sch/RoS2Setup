#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time as time_module

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        
        # Declare parameters
        self.declare_parameter('max_speed', 100)
        self.declare_parameter('center_threshold', 30.0)
        self.declare_parameter('angle_send_threshold', 5.0)
        self.declare_parameter('steering_delay', 0.3)
        self.declare_parameter('max_distance', 200.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('pwm_change_threshold', 20)
        self.declare_parameter('min_drive_speed', 10)
        self.declare_parameter('stop_while_turning', False)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.center_threshold = self.get_parameter('center_threshold').value
        self.angle_send_threshold = self.get_parameter('angle_send_threshold').value
        self.steering_delay = self.get_parameter('steering_delay').value
        self.max_distance = self.get_parameter('max_distance').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.pwm_change_threshold = self.get_parameter('pwm_change_threshold').value
        self.min_drive_speed = self.get_parameter('min_drive_speed').value
        self.stop_while_turning = self.get_parameter('stop_while_turning').value
        
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
        self.last_drive_speed = 0
        self.angle_alpha = 0.3  # Smoothing factor for angle (from track.py)
        
        # Optical flow parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03)
        )
        self.prev_gray = None
        self.prev_points = None
        self.lost_frames = 0
        self.REACQUIRE_INTERVAL = 10
        self.REACQUIRE_RADIUS = 20
        
        # Create subscribers and publishers
        self.image_subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        # Publish to /auto/cmd_vel to avoid conflicts with manual control
        self.cmd_vel_publisher = self.create_publisher(Twist, '/auto/cmd_vel', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/steering_angles', 10)
        self.tracking_status_publisher = self.create_publisher(Bool, '/tracking/active', 10)
        self.tracking_center_publisher = self.create_publisher(Point, '/tracking/center', 10)
        
        # Create services
        from std_srvs.srv import SetBool
        self.tracking_service = self.create_service(
            SetBool, '/tracking/toggle', self.toggle_tracking_callback)
        
        # Subscription for tracking point selection (from object_selector)
        self.select_point_subscription = self.create_subscription(
            Point, '/tracking/select_point', self.select_point_callback, 10)
        
        # Create timer for status publishing
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info('Tracking node started')

    def _now(self):
        """Return current time in seconds with sub-second precision."""
        return time_module.time()
    
    def select_point_callback(self, msg):
        """Handle point selection from object selector GUI"""
        x, y = int(msg.x), int(msg.y)
        self.get_logger().info(f"Point selected at ({x}, {y})")
        
        # This will be set on next frame
        self.tracking_active = True
        self.original_center = (x, y)
        self.last_center = None
        self.smoothed_angle = None
        self.steering_ready = True
        self.last_sent_angle = None
        self.prev_points = None  # Reset optical flow
        self.prev_gray = None
        self.lost_frames = 0  # Reset lost frames counter
    
    def image_callback(self, msg):
        """Process incoming camera frames"""
        if not self.tracking_active:
            return
        
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Initialize tracking on first frame after point selection
            if self.prev_points is None and self.original_center is not None:
                x, y = self.original_center
                half = 5
                x1, y1 = max(0, x - half), max(0, y - half)
                x2, y2 = min(gray.shape[1] - 1, x + half), min(gray.shape[0] - 1, y + half)
                mask = np.zeros_like(gray)
                cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
                self.prev_points = cv2.goodFeaturesToTrack(
                    gray, mask=mask, maxCorners=30, qualityLevel=0.15, minDistance=5, blockSize=10
                )
                if self.prev_points is not None:
                    self.prev_gray = gray.copy()
                    self.get_logger().info(f"Initialized tracking with {len(self.prev_points)} points")
                else:
                    self.get_logger().warn("No good features found at selected point")
                    self.tracking_active = False
                return
            
            if self.prev_points is not None and self.prev_gray is not None:
                # EXACT track.py optical flow implementation
                next_points, status, err = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray, self.prev_points, None, **self.lk_params
                )
                
                if next_points is None or status is None or err is None:
                    self.tracking_active = False
                    return
                    
                next_points = next_points.reshape(-1, 2)
                status = status.reshape(-1)
                err = err.reshape(-1)
                good_new = next_points[status == 1]
                good_old = self.prev_points[status == 1]
                good_err = err[status == 1]
                
                if len(good_new) == 0:
                    self.lost_frames += 1
                    self.stop_robot()
                    return
                    
                # Track.py error and motion filtering
                avg_err = np.mean(good_err)
                motion_mag = np.mean(np.linalg.norm(good_new - good_old, axis=1))
                if avg_err > 25.0 or motion_mag > 40.0:
                    self.get_logger().warn(f"[OCCLUDED] AvgErr={avg_err:.1f}, Motion={motion_mag:.1f} → freeze")
                    self.stop_robot()
                    self.lost_frames += 1
                    return
                    
                # Use only reliable points (like track.py)
                reliable_mask = good_err < 25.0
                reliable_points = good_new[reliable_mask]
                
                if len(reliable_points) >= 1:
                    # Tracking successful
                    self.lost_frames = 0
                    self.prev_points = reliable_points.reshape(-1, 1, 2)
                    self.prev_gray = gray.copy()
                    
                    # Calculate center
                    avg_new = np.mean(reliable_points, axis=0)
                    cx, cy = int(avg_new[0]), int(avg_new[1])
                    self.last_center = (cx, cy)
                    
                    # Publish tracking center for visualization
                    center_msg = Point()
                    center_msg.x = float(cx)
                    center_msg.y = float(cy)
                    center_msg.z = 0.0
                    self.tracking_center_publisher.publish(center_msg)
                    
                    # Process movement
                    self.process_movement((cx, cy))
                else:
                    # No reliable points - try reacquisition
                    self.lost_frames += 1
                    self.stop_robot()
                    
                    # Track.py reacquisition logic
                    if self.lost_frames % self.REACQUIRE_INTERVAL == 0 and self.last_center is not None:
                        expansion = min(3 * self.REACQUIRE_RADIUS, self.REACQUIRE_RADIUS + int(self.lost_frames * 1.5))
                        x, y = map(int, self.last_center)
                        mask = np.zeros_like(gray)
                        x1, y1 = max(0, x - expansion), max(0, y - expansion)
                        x2, y2 = min(gray.shape[1] - 1, x + expansion), min(gray.shape[0] - 1, y + expansion)
                        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
                        new_features = cv2.goodFeaturesToTrack(
                            gray, mask=mask, maxCorners=40, qualityLevel=0.15, minDistance=5, blockSize=10
                        )
                        if new_features is not None:
                            self.get_logger().info(f"[AUTO] Reacquired {len(new_features)} features")
                            self.prev_points = new_features
                            self.prev_gray = gray.copy()
                            self.lost_frames = 0
            else:
                self.prev_gray = gray.copy()
                
        except Exception as e:
            import traceback
            self.get_logger().error(f"Image processing error: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
    
    def process_movement(self, center):
        """Process robot movement based on object position"""
        cx, cy = center
        
        # Initialize on first call
        if self.last_center is None:
            self.last_center = (cx, cy)
            if self.original_center is None:
                self.original_center = (cx, cy)
                self.get_logger().info("Tracking started - will follow object")
            return
        
        # Calculate displacement from original center (EXACTLY like track.py)
        dx = cx - self.original_center[0]
        dy = cy - self.original_center[1]
        vector_mag = math.hypot(dx, dy)
        
        # Don't calculate angle if object is within CENTER_THRESHOLD pixels
        CENTER_THRESHOLD = self.center_threshold
        if vector_mag < CENTER_THRESHOLD:
            # Keep current steering, don't send new commands
            self.last_center = (cx, cy)
            return
        
        # Calculate raw angle only when object has moved away from center
        raw_angle = None
        if vector_mag > CENTER_THRESHOLD:
            raw_angle = (math.degrees(math.atan2(-dx, dy)) + 360.0) % 360.0
        
        # Smooth angle to reduce jitter (EXACTLY like track.py)
        if raw_angle is not None:
            if self.smoothed_angle is None:
                self.smoothed_angle = raw_angle
            else:
                # Handle angle wrapping (e.g., 359° -> 1°)
                angle_diff = self.shortest_angle_diff(raw_angle, self.smoothed_angle)
                self.smoothed_angle = (self.smoothed_angle + self.angle_alpha * angle_diff) % 360.0
            target_angle = self.smoothed_angle
        elif self.last_sent_angle is not None:
            target_angle = self.last_sent_angle
        else:
            target_angle = None
        
        # Send steering command if angle changed significantly
        if target_angle is not None:
            angle_diff = abs(self.shortest_angle_diff(
                target_angle,
                self.last_sent_angle
            )) if self.last_sent_angle is not None else float('inf')
            
            # Send steering only if angle changed by threshold
            ANGLE_SEND_THRESHOLD = self.angle_send_threshold
            if angle_diff > ANGLE_SEND_THRESHOLD:
                self.send_steering_command(target_angle)
                self.last_sent_angle = target_angle
                self.last_steer_command_time = self._now()
                self.steering_ready = False
                if self.stop_while_turning:
                    # Stop wheels while turning (matches legacy behaviour)
                    self.stop_robot()
                    self.last_center = (cx, cy)
                    return
        
        # Wait for steppers to reach position
        if not self.steering_ready:
            current_time = self._now()
            if current_time - self.last_steer_command_time >= self.steering_delay:
                self.steering_ready = True
            elif self.stop_while_turning:
                # Still waiting for steppers to turn
                self.last_center = (cx, cy)
                return
        
        # Stop if object is at center (within deadzone) - EXACTLY like track.py
        SIDE_LATERAL_DEADZONE = self.center_threshold
        if vector_mag < SIDE_LATERAL_DEADZONE:
            self.stop_robot()
            self.last_center = (cx, cy)
            return
        
        # Drive forward proportional to distance from center (EXACTLY like track.py)
        speed_factor = min(vector_mag / self.max_distance, 1.0)  # 0.0 to 1.0
        drive_speed = int(round(speed_factor * self.max_speed))
        
        if drive_speed > 0 and self.min_drive_speed > 0:
            drive_speed = max(self.min_drive_speed, drive_speed)
        
        # Send movement command
        self.send_movement_command(drive_speed)
        self.last_center = (cx, cy)
    
    def send_steering_command(self, angle):
        """Send steering command to hardware node"""
        # Publish raw angle - hardware node will apply per-wheel offsets
        steering_msg = Float64MultiArray()
        steering_msg.data = [float(angle), float(angle), float(angle), float(angle)]
        self.steering_publisher.publish(steering_msg)
        self.get_logger().info(f"[STEERING] Publishing angle: {angle:.1f}°")  # Changed to INFO
    
    def send_movement_command(self, speed):
        """Send movement command as Twist message"""
        # Only send if speed changed significantly
        if abs(speed - self.last_drive_speed) >= self.pwm_change_threshold:
            twist = Twist()
            twist.linear.x = float(speed)  # Forward/backward speed
            twist.linear.y = 0.0          # Lateral speed (not used in this implementation)
            twist.angular.z = 0.0          # Rotation speed
            self.cmd_vel_publisher.publish(twist)
            self.last_drive_speed = speed
            self.get_logger().debug(f"Drive speed: {speed}")
    
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
        node.get_logger().info("Shutting down - stopping robot...")
        node.stop_robot()  # Stop all movement
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
