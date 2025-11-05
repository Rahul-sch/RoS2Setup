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
        self.declare_parameter('steering_port', '')  # Empty = auto-detect
        self.declare_parameter('drive_port', '')     # Empty = auto-detect
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
        self.declare_parameter('steering_retries', 0)
        
        # Track last sent commands to avoid redundant writes
        self.last_sent_pwms = None
        
        # Get parameters
        self.steering_port = self.get_parameter('steering_port').value
        self.drive_port = self.get_parameter('drive_port').value
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
        
        # Serial communication - TWO separate Arduinos
        self.serial_lock = threading.Lock()
        self.ser_steering = None  # Arduino #1: Steering control
        self.ser_drive = None     # Arduino #2: Drive control
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
        """Connect to BOTH Arduinos - Steering and Drive"""
        if self._shutdown_requested:
            return False

        # Close existing connections if open
        if self.ser_steering and self.ser_steering.is_open:
            self.get_logger().info("Closing existing steering connection...")
            try:
                self.ser_steering.close()
            except Exception:
                pass
            time.sleep(0.1)

        if self.ser_drive and self.ser_drive.is_open:
            self.get_logger().info("Closing existing drive connection...")
            try:
                self.ser_drive.close()
            except Exception:
                pass
            time.sleep(0.1)

        # Build list of candidate ports
        # Check if ports are manually specified
        manual_steering_port = self.steering_port if self.steering_port else None
        manual_drive_port = self.drive_port if self.drive_port else None

        if manual_steering_port and manual_drive_port:
            # Both ports manually specified - connect directly
            self.get_logger().info(f"Using manual port config: steering={manual_steering_port}, drive={manual_drive_port}")
            try:
                # Connect to steering Arduino
                self.ser_steering = serial.Serial(
                    manual_steering_port, self.serial_baud, timeout=1, write_timeout=2,
                    rtscts=False, dsrdtr=False, exclusive=True
                )
                self.ser_steering.reset_input_buffer()
                self.ser_steering.reset_output_buffer()
                time.sleep(0.2)
                self.get_logger().info(f"✓ Connected to STEERING Arduino on {manual_steering_port}")

                # Connect to drive Arduino
                self.ser_drive = serial.Serial(
                    manual_drive_port, self.serial_baud, timeout=1, write_timeout=2,
                    rtscts=False, dsrdtr=False, exclusive=True
                )
                self.ser_drive.reset_input_buffer()
                self.ser_drive.reset_output_buffer()
                time.sleep(0.2)
                self.get_logger().info(f"✓ Connected to DRIVE Arduino on {manual_drive_port}")

                # Initialize both Arduinos
                self.last_sent_pwms = None
                self.last_steering_sent = None
                self.get_logger().info("[Init] Setting wheels to 0°...")
                self.send_steer_command([0, 0, 0, 0], force=True)
                time.sleep(0.3)
                self.get_logger().info("[Init] Calibrating stepper positions...")
                self.calibrate_steppers()
                self.get_logger().info("[Init] Stopping drive motors...")
                self.send_drive_command([1500, 1500, 1500, 1500], force=True)
                self.get_logger().info("✓ Both Arduinos initialized successfully!")
                return True

            except Exception as e:
                self.get_logger().error(f"Failed to connect with manual ports: {e}")
                return False

        # Auto-detect mode
        candidate_ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        if not candidate_ports:
            self.get_logger().error("No serial devices matching /dev/ttyACM* or /dev/ttyUSB* found")
            return False

        self.get_logger().info(f"Auto-detecting Arduinos from ports: {candidate_ports}")

        # Try to connect to both Arduinos by detecting which is which
        steering_connected = False
        drive_connected = False

        for port in candidate_ports:
            if self._shutdown_requested:
                return False

            if steering_connected and drive_connected:
                break

            try:
                self.get_logger().info(f"Attempting to connect on {port}...")
                test_ser = serial.Serial(
                    port,
                    self.serial_baud,
                    timeout=1,
                    write_timeout=2,
                    rtscts=False,
                    dsrdtr=False,
                    exclusive=True,
                )
                test_ser.reset_input_buffer()
                test_ser.reset_output_buffer()
                test_ser.setDTR(False)
                test_ser.setRTS(False)
                time.sleep(0.05)
                test_ser.setDTR(True)
                test_ser.setRTS(True)
                time.sleep(0.5)  # Wait for Arduino startup message

                # Read startup message to identify Arduino
                lines = []
                while test_ser.in_waiting:
                    try:
                        line = test_ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            lines.append(line)
                    except:
                        pass

                startup_msg = ' '.join(lines).lower()
                self.get_logger().info(f"Port {port} says: {startup_msg}")

                # Identify Arduino by its startup message
                if 'steering' in startup_msg and not steering_connected:
                    self.ser_steering = test_ser
                    self.steering_port = port
                    self.get_logger().info(f"✓ Connected to STEERING Arduino on {port}")
                    steering_connected = True
                elif 'drive' in startup_msg and not drive_connected:
                    self.ser_drive = test_ser
                    self.drive_port = port
                    self.get_logger().info(f"✓ Connected to DRIVE Arduino on {port}")
                    drive_connected = True
                else:
                    # Can't identify or already connected to this type
                    self.get_logger().warn(f"Port {port} - unknown or duplicate Arduino type")
                    test_ser.close()

            except Exception as exc:
                self.get_logger().warn(f"Port {port} unavailable: {exc}")
                try:
                    if 'test_ser' in locals() and test_ser and test_ser.is_open:
                        test_ser.close()
                except Exception:
                    pass
                continue

        # Check if both connected
        if not steering_connected:
            self.get_logger().error("Could not connect to STEERING Arduino!")
            self.ser_steering = None

        if not drive_connected:
            self.get_logger().error("Could not connect to DRIVE Arduino!")
            self.ser_drive = None

        if not (steering_connected and drive_connected):
            self.get_logger().error("Failed to connect to both Arduinos. Will retry...")
            return False

        # Reset last sent caches
        self.last_sent_pwms = None
        self.last_steering_sent = None

        # Initialize steering Arduino
        self.get_logger().info("[Init] Setting wheels to 0°...")
        self.send_steer_command([0, 0, 0, 0], force=True)
        time.sleep(0.3)

        self.get_logger().info("[Init] Calibrating stepper positions...")
        self.calibrate_steppers()

        # Initialize drive Arduino
        self.get_logger().info("[Init] Stopping drive motors...")
        self.send_drive_command([1500, 1500, 1500, 1500], force=True)

        self.get_logger().info("✓ Both Arduinos initialized successfully!")
        return True
    
    def calibrate_steppers(self):
        """Calibrate stepper positions (from your track.py)"""
        if self.ser_steering and self.ser_steering.is_open:
            try:
                self.ser_steering.write(b"C 0 0\nC 1 0\nC 2 0\nC 3 0\n")
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

            # Skip if identical to current target to avoid hissing/holding noise
            if self.current_steering_angles == angles:
                return

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
        if self.safety_stop_active:
            # Ignore incoming commands while stop is active
            if self.last_sent_pwms != [1500, 1500, 1500, 1500]:
                self.get_logger().debug(f"[{source}] Command blocked due to LIDAR stop")
                self.send_drive_command([1500, 1500, 1500, 1500], force=True)
            return

        # Extract velocity components
        vx = msg.linear.x      # Forward/backward
        vy = msg.linear.y      # Strafe left/right
        omega = msg.angular.z  # Rotation

        # Check if any movement commanded
        if abs(vx) < 0.1 and abs(vy) < 0.1 and abs(omega) < 0.1:
            # Stop all motors
            if self.last_sent_pwms != [1500, 1500, 1500, 1500]:
                self.get_logger().info(f"[{source}] Stopping all motors")
            self.send_drive_command([1500, 1500, 1500, 1500])
            return

        # For pure rotation, set wheels to X-pattern and rotate
        if abs(vx) < 0.1 and abs(vy) < 0.1 and abs(omega) > 0.1:
            # Rotation in place - set wheels to 45° angles for X-pattern
            angles = [45, 135, 45, 135]  # X-pattern for rotation
            self.pending_steering = angles

            # All wheels rotate in same direction for in-place rotation
            speed = int(omega * self.max_speed)
            speed = max(-self.max_speed, min(self.max_speed, speed))

            if self.caution_active and speed != 0:
                scaled = int(round(speed * self.caution_speed_scale))
                if scaled == 0:
                    scaled = 1 if speed > 0 else -1
                speed = scaled

            # All wheels same speed for rotation
            pwms = [1500 + self.wheel_drive_dir[i] * speed for i in range(4)]
            self.get_logger().info(f"[{source}] Rotation mode: angles={angles}, pwms={pwms}")
            self.send_drive_command(pwms)
            return

        # For forward/backward movement
        if abs(vx) > 0.1:
            # Point all wheels forward
            if self.pending_steering != [0, 0, 0, 0]:
                self.pending_steering = [0, 0, 0, 0]

            speed = int(vx)
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
            self.get_logger().info(f"[{source}] Forward/backward: speed={speed}, pwms={pwms}")
            self.send_drive_command(pwms)
            return

        # For strafe movement
        if abs(vy) > 0.1:
            # Point wheels to 90° for strafing
            angles = [90, 90, 90, 90]
            if self.pending_steering != angles:
                self.pending_steering = angles

            speed = int(vy)
            speed = max(-self.max_speed, min(self.max_speed, speed))

            if self.caution_active and speed != 0:
                scaled = int(round(speed * self.caution_speed_scale))
                if scaled == 0:
                    scaled = 1 if speed > 0 else -1
                speed = scaled

            pwms = [1500 + self.wheel_drive_dir[i] * speed for i in range(4)]
            self.get_logger().info(f"[{source}] Strafe mode: angles={angles}, pwms={pwms}")
            self.send_drive_command(pwms)
    
    def send_drive_command(self, speeds, force=False):
        """Send drive command to Drive Arduino"""
        speeds = [int(min(2000, max(1000, round(v)))) for v in speeds]

        # Only send if values changed (unless forced)
        if not force and self.last_sent_pwms is not None and speeds == self.last_sent_pwms:
            return True  # Command already active

        # arduino_drive.ino expects 'S' command, not 'D'
        cmd = f"S {speeds[0]} {speeds[1]} {speeds[2]} {speeds[3]}\n"

        need_reconnect = False
        with self.serial_lock:
            if not (self.ser_drive and self.ser_drive.is_open):
                need_reconnect = True
            else:
                try:
                    self.ser_drive.write(cmd.encode())
                    self.last_sent_pwms = speeds[:]  # Store what we sent
                    self.get_logger().info(f"[DRIVE] {cmd.strip()}")
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
        """Send steering command to Steering Arduino"""
        angles = [int(round(v)) for v in angles]

        # Only send if values changed (unless forced)
        if not force and self.last_steering_sent is not None and angles == self.last_steering_sent:
            return True  # Already sent this command

        cmd = f"S {angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"

        need_reconnect = False
        with self.serial_lock:
            if not (self.ser_steering and self.ser_steering.is_open):
                need_reconnect = True
            else:
                try:
                    self.ser_steering.write(cmd.encode())
                    self.last_steering_sent = angles[:]  # Store what we sent
                    self.get_logger().info(f"[STEERING] {cmd.strip()}")
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
        """Attempt to reconnect to both Arduinos in background"""
        if self._shutdown_requested:
            return
        if self._reconnect_active:
            return

        def _loop():
            try:
                while rclpy.ok() and not self._shutdown_requested:
                    with self.serial_lock:
                        try:
                            if self.ser_steering:
                                self.ser_steering.close()
                        except Exception:
                            pass
                        try:
                            if self.ser_drive:
                                self.ser_drive.close()
                        except Exception:
                            pass
                        self.ser_steering = None
                        self.ser_drive = None

                    self.get_logger().warn("Attempting to reconnect to Arduinos...")
                    if self.connect_to_arduino():
                        self.get_logger().info("Reconnected to both Arduinos")
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
        # Reconnect if either Arduino is disconnected
        if not (self.ser_steering and self.ser_steering.is_open) or \
           not (self.ser_drive and self.ser_drive.is_open):
            if not self._reconnect_active:
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

        # Stop all motors before disconnecting
        try:
            self.send_drive_command([1500, 1500, 1500, 1500], force=True)
        except Exception:
            pass

        # Close both serial connections
        if self.ser_steering and self.ser_steering.is_open:
            try:
                self.ser_steering.close()
            except Exception:
                pass

        if self.ser_drive and self.ser_drive.is_open:
            try:
                self.ser_drive.close()
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
        # Close both serial connections
        if node.ser_steering and node.ser_steering.is_open:
            node.ser_steering.close()
        if node.ser_drive and node.ser_drive.is_open:
            node.ser_drive.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
