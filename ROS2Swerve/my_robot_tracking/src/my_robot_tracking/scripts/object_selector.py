#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectSelector(Node):
    def __init__(self):
        super().__init__('object_selector')
        
        self.bridge = CvBridge()
        self.current_frame = None
        self.tracking_active = False
        self.tracking_center = None
        
        # Create subscriber for camera feed
        self.image_subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        # Subscribe to tracking status
        self.tracking_status_subscription = self.create_subscription(
            Bool, '/tracking/active', self.tracking_status_callback, 10)
        
        # Subscribe to tracking center for visualization
        self.tracking_center_subscription = self.create_subscription(
            Point, '/tracking/center', self.tracking_center_callback, 10)
        
        # Publisher for selected point
        self.point_publisher = self.create_publisher(Point, '/tracking/select_point', 10)
        
        # Create service client for tracking toggle
        self.tracking_client = self.create_client(SetBool, '/tracking/toggle')
        
        self.get_logger().info('Object selector ready. Click on an object to start tracking.')
        self.get_logger().info('Press ESC to exit, T to stop tracking.')
    
    def tracking_status_callback(self, msg):
        """Update tracking status"""
        self.tracking_active = msg.data
    
    def tracking_center_callback(self, msg):
        """Update tracking center for visualization"""
        self.tracking_center = (int(msg.x), int(msg.y))
    
    def image_callback(self, msg):
        """Process camera image and handle mouse clicks"""
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = frame.copy()
            
            # Draw tracking center if active
            if self.tracking_active and self.tracking_center is not None:
                cv2.circle(frame, self.tracking_center, 8, (0, 255, 0), 2)
                cv2.circle(frame, self.tracking_center, 2, (0, 0, 255), -1)
                cv2.putText(frame, "TRACKING", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "Click to select object", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Create window and set mouse callback
            cv2.namedWindow("Object Selector")
            cv2.setMouseCallback("Object Selector", self.mouse_callback)
            
            # Display image
            cv2.imshow("Object Selector", frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info("Exiting...")
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif key == ord('t') or key == ord('T'):  # 't' key to stop tracking
                self.stop_tracking()
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to select object"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Selected point at ({x}, {y})")
            # Publish the selected point to tracking node
            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0
            self.point_publisher.publish(point_msg)
            self.get_logger().info("Point sent to tracking node")
    
    def stop_tracking(self):
        """Stop object tracking"""
        # Wait for service to be available
        if not self.tracking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Tracking service not available')
            return
        
        request = SetBool.Request()
        request.data = False
        future = self.tracking_client.call_async(request)
        future.add_done_callback(self.tracking_stop_callback)
    
    def tracking_stop_callback(self, future):
        """Handle tracking stop response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Tracking stopped")
            else:
                self.get_logger().error(f"Failed to stop tracking: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
