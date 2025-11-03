#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

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
        self.declare_parameter('lidar_stop_topic', '/safety/stop')
        self.declare_parameter('lidar_caution_topic', '/safety/caution')
        self.declare_parameter('caution_speed_scale', 0.4)
        # Retry steering to improve robustness if a module misses a command
        self.declare_parameter('steering_retries', 2)
        
        # Track last sent commands to avoid redundant writes
        self.last_sent_pwms = None
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_drive_dir = self.get_parameter('wheel_drive_dir').value
        self.steer_dir = self.get_parameter('steer_dir').value
        self.steer_offsets = self.get_parameter('steer_offsets').value
        self.auto_mode_topic = self.get_parameter('auto_mode_topic').value
        self.manual_mode_topic = self.get_parameter('manual_mode_topic').value
        self.lidar_stop_topic = self.get_parameter('lidar_stop_topic').value
        self.lidar_caution_topic = self.get_parameter('lidar_caution_topic').value
        self.caution_speed_scale = float(self.get_parameter('caution_speed_scale').value)
        self.caution_speed_scale = min(1.0, max(0.05, self.caution_speed_scale))
        self.steering_retries = int(self.get_parameter('steering_retries').value)
        
        # Serial communication
        self.serial_lock = threading.Lock()
        self.ser = None
        self.current_steering_angles = [0, 0, 0, 0]
        self.pending_steering = None
        self.pending_steering_retries = 0
        self.last_steering_sent = None
        self.serial_retry_delay = 2.0
        self._reconnect_active = False
        self._shutdown_requested = False
        self.safety_stop_active = False
        self.safety_stop_logged = False
        self.caution_active = False
        
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
        
        # LIDAR safety subscriptions
        self.lidar_stop_subscription = self.create_subscription(
            Bool, self.lidar_stop_topic, self.lidar_stop_callback, 10)
        self.lidar_caution_subscription = self.create_subscription(
            Bool, self.lidar_caution_topic, self.lidar_caution_callback, 10)
        
        # Create timer for serial communication
        self.serial_timer = self.create_timer(0.05, self.process_serial)
        
        self.get_logger().info('Hardware node started')
    
    def connect_to_arduino(self):
        """Connect to Arduino (from your track.py)"""
        if self._shutdown_requested:
            return False

        # Close existing connection if open
        if self.ser and self.ser.is_open:
            self.get_logger().info("Closing existing serial connection...")
            try:
                self.ser.close()
            except Exception:
                pass
            time.sleep(0.1)

        # Build list of candidate ports
        if self.serial_port:
            candidate_ports = [self.serial_port]
        else:
            candidate_ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if not candidate_ports:
                self.get_logger().error("No serial devices matching /dev/ttyACM* or /dev/ttyUSB* found")
                return False
            self.get_logger().info(f"Available ports: {candidate_ports}")

        last_error = None
        for port in candidate_ports:
            if self._shutdown_requested:
                return False
            try:
                self.get_logger().info(f"Attempting to connect on {port} ...")
                new_ser = serial.Serial(
                    port,
                    self.serial_baud,
                    timeout=1,
                    write_timeout=2,
                    rtscts=False,
                    dsrdtr=False,
                    exclusive=True,
                )
                new_ser.reset_input_buffer()
                new_ser.reset_output_buffer()
                new_ser.setDTR(False)
                new_ser.setRTS(False)
                time.sleep(0.05)
                new_ser.setDTR(True)
                new_ser.setRTS(True)
                time.sleep(0.1)

                self.ser = new_ser
                self.serial_port = port  # remember working port
                self.get_logger().info(f"Connected to Arduino on {port}")

                # Reset last sent caches so we always issue the first commands
                self.last_sent_pwms = None
                self.last_steering_sent = None
                
                # Initialize steppers to 0 degrees
                self.get_logger().info("[Init] Setting wheels to 0°...")
                self.send_steer_command([0, 0, 0, 0], force=True)
                time.sleep(0.3)
                
                # Calibrate positions
                self.get_logger().info("[Init] Calibrating stepper positions...")
                self.calibrate_steppers()

                # Ensure drive motors are stopped
                self.send_drive_command([1500, 1500, 1500, 1500], force=True)
                return True
            except Exception as exc:
                last_error = exc
                self.get_logger().warn(f"Port {port} unavailable: {exc}")
                try:
                    if 'new_ser' in locals() and new_ser and new_ser.is_open:
                        new_ser.close()
                except Exception:
                    pass
                continue

        self.get_logger().error(f"Could not connect to Arduino: {last_error}")
        self.get_logger().info("Will retry connection in background...")
        self.ser = None
        return False
    
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
            self.pending_steering_retries = max(0, self.steering_retries)
            # Log with module numbers for easier diagnosis
            self.get_logger().info(
                f"[ROS] Steering command: Module[0]={angles[0]}°, "
                f"Module[1]={angles[1]}°, Module[2]={angles[2]}°, Module[3]={angles[3]}°"
            )
            self.get_logger().debug(f"Received steering command: {angles}")
    
    def lidar_stop_callback(self, msg: Bool):
        """Handle emergency stop from LIDAR guard."""
        self.safety_stop_active = bool(msg.data)
        if self.safety_stop_active:
            if not self.safety_stop_logged:
                self.get_logger().warn("LIDAR stop active - halting drive commands")
            self.safety_stop_logged = True
            # Issue immediate stop if we are currently moving
            if self.last_sent_pwms != [1500, 1500, 1500, 1500]:
                self.send_drive_command([1500, 1500, 1500, 1500], force=True)
        else:
            if self.safety_stop_logged:
                self.get_logger().info("LIDAR stop cleared - drive enabled")
            self.safety_stop_logged = False
    
    def lidar_caution_callback(self, msg: Bool):
        self.caution_active = bool(msg.data)
        if self.caution_active:
            self.get_logger().debug("LIDAR caution active - limiting speed")
        
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
        if self.safety_stop_active:
            # Ignore incoming commands while stop is active
            if self.last_sent_pwms != [1500, 1500, 1500, 1500]:
                self.get_logger().debug(f"[{source}] Command blocked due to LIDAR stop")
                self.send_drive_command([1500, 1500, 1500, 1500], force=True)
            return

        if abs(msg.linear.x) > 0.1:
            speed = int(msg.linear.x)
            # Clamp speed
            speed = max(-self.max_speed, min(self.max_speed, speed))

            if self.caution_active and speed != 0:
                scaled = int(round(speed * self.caution_speed_scale))
                if scaled == 0:
                    scaled = 1 if speed > 0 else -1
                if scaled != speed:
                    self.get_logger().debug(
                        f"[{source}] Speed {speed} limited to {scaled} due to LIDAR caution"
                    )
                speed = scaled
            
            # Convert to PWM values
            pwms = [1500 + self.wheel_drive_dir[i] * speed for i in range(4)]
            self.get_logger().info(f"[{source}] Sending drive command: {pwms}")
            self.send_drive_command(pwms)
        else:
            # Stop
            if self.last_sent_pwms != [1500, 1500, 1500, 1500]:
                self.get_logger().info(f"[{source}] Sending drive command: [1500, 1500, 1500, 1500]")
            self.send_drive_command([1500, 1500, 1500, 1500])
    
    def send_drive_command(self, speeds, force=False):
        """Send drive command to Arduino (from your track.py)"""
        speeds = [int(min(2000, max(1000, round(v)))) for v in speeds]
        
        # Only send if values changed (unless forced)
        if not force and self.last_sent_pwms is not None and speeds == self.last_sent_pwms:
            return True  # Command already active
        
        cmd = f"D {speeds[0]} {speeds[1]} {speeds[2]} {speeds[3]}\n"
        
        need_reconnect = False
        with self.serial_lock:
            if not (self.ser and self.ser.is_open):
                need_reconnect = True
            else:
                try:
                    self.ser.write(cmd.encode())
                    self.last_sent_pwms = speeds[:]  # Store what we sent
                    self.get_logger().info(f"[D] {cmd.strip()}")  # Like track.py!
                    return True
                except serial.SerialTimeoutException as e:
                    self.get_logger().warn(f"Drive command timeout: {e}")
                    self.last_sent_pwms = None
                    need_reconnect = True
                except serial.SerialException as e:
                    self.get_logger().error(f"Drive command failed: {e}")
                    need_reconnect = True
                except Exception as e:
                    self.get_logger().error(f"Unexpected drive command error: {e}")
                    need_reconnect = True

        if need_reconnect:
            self.attempt_reconnect()
        return False
    
    def send_steer_command(self, angles, force=False):
        """Send steering command to Arduino (from your track.py)"""
        angles = [int(round(v)) for v in angles]
        
        # Only send if values changed (unless forced)
        if not force and self.last_steering_sent is not None and angles == self.last_steering_sent:
            return True  # Already sent this command
        
        cmd = f"S {angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
        
        need_reconnect = False
        with self.serial_lock:
            if not (self.ser and self.ser.is_open):
                need_reconnect = True
            else:
                try:
                    self.ser.write(cmd.encode())
                    self.last_steering_sent = angles[:]  # Store what we sent
                    self.get_logger().info(f"[S] {cmd.strip()}")  # Like track.py!
                    return True
                except serial.SerialTimeoutException as e:
                    self.get_logger().warn(f"Steer command timeout: {e}")
                    need_reconnect = True
                except serial.SerialException as e:
                    self.get_logger().error(f"Steer command failed: {e}")
                    need_reconnect = True
                except Exception as e:
                    self.get_logger().error(f"Unexpected steering command error: {e}")
                    need_reconnect = True

        if need_reconnect:
            self.attempt_reconnect()
        return False
    
    def attempt_reconnect(self):
        """Attempt to reconnect to Arduino in background"""
        if self._shutdown_requested:
            return
        if self._reconnect_active:
            return

        def _loop():
            try:
                while rclpy.ok() and not self._shutdown_requested:
                    with self.serial_lock:
                        try:
                            if self.ser:
                                self.ser.close()
                        except Exception:
                            pass
                        self.ser = None

                    self.get_logger().warn("Attempting to reconnect to Arduino...")
                    if self.connect_to_arduino():
                        self.get_logger().info("Reconnected to Arduino")
                        return

                    if not rclpy.ok() or self._shutdown_requested:
                        break
                    time.sleep(self.serial_retry_delay)
            finally:
                self._reconnect_active = False

        self._reconnect_active = True
        threading.Thread(target=_loop, daemon=True).start()
    
    def process_serial(self):
        """Process serial communication and send pending commands"""
        # Reconnect if disconnected
        if self.ser is None or not self.ser.is_open:
            self.connect_to_arduino()
            return
        
        # Send pending steering commands (with retries)
        if self.pending_steering is not None:
            angles = self.pending_steering
            force = self.pending_steering_retries > 0
            if self.send_steer_command(angles, force=force):
                self.get_logger().info(f"[SERIAL] Sent steering command: {angles} (force={force})")
                self.current_steering_angles = angles
                if self.pending_steering_retries > 0:
                    self.pending_steering_retries -= 1
                    # keep pending to resend next cycle if retries remain
                    if self.pending_steering_retries == 0:
                        self.pending_steering = None
                else:
                    self.pending_steering = None
    
    def destroy_node(self):
        """Clean up resources"""
        self._shutdown_requested = True
        if self.ser and self.ser.is_open:
            try:
                # Stop all motors
                self.send_drive_command([1500, 1500, 1500, 1500], force=True)
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
        node.get_logger().info("Shutting down - stopping motors and steering...")
        # Send stop commands
        node.send_drive_command([1500, 1500, 1500, 1500], force=True)
        time.sleep(0.1)
    finally:
        # Close serial connection
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
