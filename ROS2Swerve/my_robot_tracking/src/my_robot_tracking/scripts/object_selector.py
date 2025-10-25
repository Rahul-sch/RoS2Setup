#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectSelector(Node):
    def __init__(self):
        super().__init__('object_selector')
        
        self.bridge = CvBridge()
        self.tracking_service = None
        
        # Create subscriber for camera feed
        self.image_subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        
        # Create service client for tracking
        self.tracking_client = self.create_client(SetBool, '/tracking/toggle')
        
        # Wait for service to be available
        while not self.tracking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for tracking service...')
        
        self.get_logger().info('Object selector ready. Click on an object to start tracking.')
        self.get_logger().info('Press ESC to exit.')
    
    def image_callback(self, msg):
        """Process camera image and handle mouse clicks"""
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
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
            elif key == ord('s'):  # 's' key to start tracking
                self.start_tracking()
            elif key == ord('t'):  # 't' key to stop tracking
                self.stop_tracking()
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to select object"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Selected point at ({x}, {y})")
            # In a full implementation, this would send the coordinates to the tracking node
            # For now, just start tracking
            self.start_tracking()
    
    def start_tracking(self):
        """Start object tracking"""
        request = SetBool.Request()
        request.data = True
        future = self.tracking_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Tracking started")
        else:
            self.get_logger().error(f"Failed to start tracking: {future.result().message}")
    
    def stop_tracking(self):
        """Stop object tracking"""
        request = SetBool.Request()
        request.data = False
        future = self.tracking_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Tracking stopped")
        else:
            self.get_logger().error(f"Failed to stop tracking: {future.result().message}")

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
