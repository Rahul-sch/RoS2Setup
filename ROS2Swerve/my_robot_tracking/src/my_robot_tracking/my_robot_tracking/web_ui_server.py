#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Image
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
from threading import Thread
import cv2
from cv_bridge import CvBridge
import base64
import time
import os

# Get the directory where this script is located
package_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.join(package_dir, 'templates')
static_dir = os.path.join(package_dir, 'static')

app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
CORS(app)

# Global ROS2 node reference
ros_node = None
bridge = CvBridge()
latest_camera_frame = None
camera_lock = False

class WebUINode(Node):
    def __init__(self):
        super().__init__('web_ui_server')
        
        # Publishers
        self.manual_cmd_vel_pub = self.create_publisher(Twist, '/manual/cmd_vel', 10)
        self.manual_steering_pub = self.create_publisher(Float64MultiArray, '/manual/steering_angles', 10)
        
        # Subscribers for status
        self.tracking_active_sub = self.create_subscription(Bool, '/tracking/active', self.tracking_callback, 10)
        self.safety_stop_sub = self.create_subscription(Bool, '/safety/stop', self.safety_stop_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        
        self.tracking_active = False
        self.safety_stop = False
        
        self.get_logger().info('Web UI ROS2 node started')
    
    def tracking_callback(self, msg):
        self.tracking_active = msg.data
    
    def safety_stop_callback(self, msg):
        self.safety_stop = msg.data
    
    def camera_callback(self, msg):
        global latest_camera_frame, camera_lock
        try:
            if not camera_lock:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                latest_camera_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

def init_ros():
    global ros_node
    rclpy.init()
    ros_node = WebUINode()
    
    def spin_ros():
        rclpy.spin(ros_node)
    
    ros_thread = Thread(target=spin_ros, daemon=True)
    ros_thread.start()

# Flask Routes
@app.route('/')
def index():
    return render_template('login.html')

@app.route('/dashboard')
def dashboard():
    return render_template('dashboard.html', doctor_name="Dr. Smith")  # Placeholder

@app.route('/manual')
def manual():
    return render_template('manual.html')

@app.route('/camera')
def camera():
    return render_template('camera.html')

# API Routes
@app.route('/api/status', methods=['GET'])
def get_status():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    return jsonify({
        'tracking_active': ros_node.tracking_active,
        'safety_stop': ros_node.safety_stop
    })

@app.route('/api/move', methods=['POST'])
def move_robot():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.json
    direction = data.get('direction')  # 'up', 'down', 'left', 'right'
    speed = data.get('speed', 50)  # Default speed
    
    twist = Twist()
    
    if direction == 'up':
        twist.linear.x = float(speed)
    elif direction == 'down':
        twist.linear.x = float(-speed)
    elif direction == 'left':
        twist.angular.z = float(speed)
    elif direction == 'right':
        twist.angular.z = float(-speed)
    
    ros_node.manual_cmd_vel_pub.publish(twist)
    return jsonify({'status': 'success'})

@app.route('/api/stop', methods=['POST'])
def stop_robot():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    twist = Twist()  # All zeros = stop
    ros_node.manual_cmd_vel_pub.publish(twist)
    return jsonify({'status': 'stopped'})

@app.route('/api/steer', methods=['POST'])
def steer_robot():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.json
    angle = data.get('angle', 0)  # Angle in degrees
    
    msg = Float64MultiArray()
    msg.data = [float(angle), float(angle), float(angle), float(angle)]
    ros_node.manual_steering_pub.publish(msg)
    return jsonify({'status': 'success'})

@app.route('/api/top_half', methods=['POST'])
def control_top_half():
    global ros_node
    if ros_node is None:
        return jsonify({'error': 'ROS2 not initialized'}), 500
    
    data = request.json
    action = data.get('action')  # 'up', 'down', 'up_plus', 'up_minus', 'down_plus', 'down_minus'
    
    # TODO: Implement top half control logic
    # For now, just acknowledge
    return jsonify({'status': 'success', 'action': action})

@app.route('/api/camera_frame', methods=['GET'])
def get_camera_frame():
    global latest_camera_frame, camera_lock
    if latest_camera_frame is None:
        return jsonify({'error': 'No camera frame'}), 404
    
    try:
        camera_lock = True
        _, buffer = cv2.imencode('.jpg', latest_camera_frame)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        camera_lock = False
        return jsonify({'frame': f'data:image/jpeg;base64,{img_base64}'})
    except Exception as e:
        camera_lock = False
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    # Initialize ROS2 in background
    init_ros()
    
    # Give ROS2 time to initialize
    time.sleep(1)
    
    # Start Flask server (using 8080 to avoid macOS AirPlay conflict on 5000)
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

