#!/usr/bin/env python3

"""
Test version of web UI server that runs without ROS2
Use this to test the UI design on your Mac without Ubuntu
"""

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import os
import base64
import numpy as np
import cv2

# Get the directory where this script is located
package_dir = os.path.dirname(os.path.abspath(__file__))
template_dir = os.path.join(package_dir, 'templates')
static_dir = os.path.join(package_dir, 'static')

app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
CORS(app)

# Mock status
mock_tracking_active = False
mock_safety_stop = False

# Mock camera frame (test pattern)
def create_test_frame():
    """Create a test pattern image"""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, "MedRa Test Camera", (150, 200), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(img, "No ROS2 connection", (150, 250), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    # Draw a circle
    cv2.circle(img, (320, 240), 100, (0, 255, 0), 2)
    return img

# Flask Routes
@app.route('/')
def index():
    return render_template('login.html')

@app.route('/dashboard')
def dashboard():
    return render_template('dashboard.html', doctor_name="Dr. Test User")

@app.route('/manual')
def manual():
    return render_template('manual.html')

@app.route('/camera')
def camera():
    return render_template('camera.html')

# API Routes (mock versions)
@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify({
        'tracking_active': mock_tracking_active,
        'safety_stop': mock_safety_stop
    })

@app.route('/api/move', methods=['POST'])
def move_robot():
    data = request.json
    direction = data.get('direction')
    print(f"[TEST] Move command: {direction}")  # Just print, don't actually move
    return jsonify({'status': 'success', 'note': 'Test mode - no actual movement'})

@app.route('/api/stop', methods=['POST'])
def stop_robot():
    print("[TEST] Stop command")
    return jsonify({'status': 'stopped', 'note': 'Test mode'})

@app.route('/api/steer', methods=['POST'])
def steer_robot():
    data = request.json
    angle = data.get('angle', 0)
    print(f"[TEST] Steer command: {angle} degrees")
    return jsonify({'status': 'success', 'note': 'Test mode'})

@app.route('/api/top_half', methods=['POST'])
def control_top_half():
    data = request.json
    action = data.get('action')
    print(f"[TEST] Top half control: {action}")
    return jsonify({'status': 'success', 'action': action, 'note': 'Test mode'})

@app.route('/api/camera_frame', methods=['GET'])
def get_camera_frame():
    try:
        test_img = create_test_frame()
        _, buffer = cv2.imencode('.jpg', test_img)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        return jsonify({'frame': f'data:image/jpeg;base64,{img_base64}'})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    PORT = 8080  # Use 8080 instead of 5000 (macOS AirPlay uses 5000)
    print("=" * 60)
    print("MedRa Web UI - TEST MODE (No ROS2)")
    print("=" * 60)
    print(f"Open your browser to: http://localhost:{PORT}")
    print("=" * 60)
    app.run(host='0.0.0.0', port=PORT, debug=True)

