#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        
        # Get parameters
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        
        # Initialize camera
        self.camera = self._init_camera()
        self.bridge = CvBridge()
        
        # Create publisher
        self.publisher = self.create_publisher(Image, '/camera/image', 10)
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        
        self.get_logger().info('Camera node started')
    
    def _init_camera(self):
        """Initialize camera (from your track.py CameraSource class)"""
        if Picamera2 is not None:
            try:
                picam2 = Picamera2()
                config = picam2.create_video_configuration(
                    main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
                )
                picam2.configure(config)
                picam2.start()
                self.get_logger().info("Picamera2 initialized successfully")
                return picam2
            except Exception as exc:
                self.get_logger().warn(f"Picamera2 init failed: {exc}")
                picam2 = None
        
        # Fallback to USB cameras
        for index in range(0, 4):
            try:
                cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            except:
                cap = cv2.VideoCapture(index)  # Fallback for Windows
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            if cap.isOpened():
                self.get_logger().info(f"USB camera initialized at index {index}")
                return cap
            cap.release()
        
        raise RuntimeError("Unable to access any camera")
    
    def publish_frame(self):
        """Capture and publish camera frame"""
        try:
            if hasattr(self.camera, 'capture_array'):  # Picamera2
                frame = self.camera.capture_array("main")
                if frame is not None:
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    self._publish_frame(frame_bgr)
            else:  # OpenCV VideoCapture
                ret, frame = self.camera.read()
                if ret:
                    self._publish_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Camera error: {e}")
    
    def _publish_frame(self, frame):
        """Convert and publish frame"""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Publishing error: {e}")
    
    def destroy_node(self):
        """Clean up camera resources"""
        if hasattr(self.camera, 'stop'):  # Picamera2
            try:
                self.camera.stop()
                self.camera.close()
            except Exception:
                pass
        else:  # OpenCV VideoCapture
            try:
                self.camera.release()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
