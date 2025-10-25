import evdev
import numpy as np
import math
import serial
import time
import threading
import cv2
import glob

SERIAL_PORT = None  # If None, auto-detect first /dev/ttyACM* or /dev/ttyUSB*
SERIAL_BAUD = 115200
SERIAL_RETRY_DELAY = 2.0

DEBUG = True
def dlog(msg):
    if DEBUG:
        print(msg)

commandCounter = 0
maxSpeed = 100
drive_mode_auto = False
last_y_press_time = 0
controllerInput = None
ser = None
prevSpeed = 0
last_drive_pwms = [1500, 1500, 1500, 1500]
JOYSTICK_DEADZONE = 0.18
manual_axes = {"x": 0.0, "y": 0.0}
trigger_axes = {"left": 0.0, "right": 0.0}
WHEEL_DRIVE_DIR = [1, 1, 1, 1]  # All wheels drive same direction for swerve
STEER_OFFSETS = [0, 0, 0, 0]
STEER_DIR = [1, 1, 1, 1]  # Every module follows the same command
current_manual_angle = None
manual_target_angle = 0
last_sent_angle = None
SIDE_STEER_ANGLE = 90.0  # Keep wheels sideways for lateral-only motion
SIDE_LATERAL_DEADZONE = 15.0  # px window around center before we drive
SIDE_MAX_OFFSET_RATIO = 0.35  # Fraction of frame width that maps to max speed
SIDE_MIN_DELTA = 20  # Minimum µs offset once we decide to move
SIDE_MAX_DELTA = 60  # Peak µs offset (=> 1500 ± 60)
ANGLE_UPDATE_THRESHOLD = 15.0  # Degrees change before re-steering (increased to reduce jitter)
TRIGGER_THRESHOLD = 0.5
TRIGGER_LEFT_CODES = {code for code in [getattr(evdev.ecodes,"ABS_Z",None),getattr(evdev.ecodes,"ABS_GAS",None),getattr(evdev.ecodes,"ABS_THROTTLE",None)] if code is not None}
TRIGGER_RIGHT_CODES = {code for code in [getattr(evdev.ecodes,"ABS_RZ",None),getattr(evdev.ecodes,"ABS_BRAKE",None)] if code is not None}
trigger_prev = {"left": False, "right": False}

frame_lock = threading.Lock()
serial_lock = threading.Lock()
latest_frame = None
frame_width = 640
frame_height = 480

tracking_active = False
last_status = None

# Optical Flow setup from new code
lk_params = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03)
)
prev_gray = None
prev_points = None
tracking_state = "idle"
lost_frames = 0
REACQUIRE_INTERVAL = 10
REACQUIRE_RADIUS = 20

# Globals for display from thread
current_points = None
reacquire_box = None

# Globals
original_center = None

# Track object position for velocity calculation
last_center = None
vel_smoothed = np.array([0.0, 0.0])
VEL_ALPHA = 0.3  # Smoothing factor for velocity

# Smoothed angle to reduce jitter
smoothed_angle = None
ANGLE_ALPHA = 0.3  # Smoothing factor for angle

# State machine for steering before driving
steering_ready = True  # True when wheels are at target angle
last_steer_command_time = 0
last_drive_direction = 1  # 1 = forward, -1 = backward


try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

class CameraSource:
    def __init__(self):
        self.picam2 = None
        self.cap = None
        if Picamera2 is not None:
            try:
                self.picam2 = Picamera2()
                config = self.picam2.create_video_configuration(main={"size": (frame_width, frame_height), "format": "RGB888"})
                self.picam2.configure(config)
                self.picam2.start()
                print("[✓] Picamera2 initialized successfully.")
                return
            except Exception as exc:
                print(f"[!] Picamera2 init failed: {exc}")
                try:
                    self.picam2.close()
                except Exception:
                    pass
                self.picam2 = None
        for index in range(0, 4):
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            if cap.isOpened():
                self.cap = cap
                print(f"[✓] USB/V4L2 camera initialized at index {index}.")
                return
            cap.release()
        raise RuntimeError("Unable to access any /dev/video* camera")

    def read(self):
        if self.picam2 is not None:
            try:
                frame = self.picam2.capture_array("main")
            except Exception:
                return False, None
            if frame is None:
                return False, None
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return True, frame_bgr
        if self.cap is None:
            return False, None
        return self.cap.read()

    def release(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
            try:
                self.picam2.close()
            except Exception:
                pass
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

camera = CameraSource()

def map_value(value, in_min, in_max, out_min, out_max):
    return np.interp(value, [in_min, in_max], [out_min, out_max])

def connect_to_arduino(port, baudrate):
    if port is None:
        candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        if not candidates:
            raise FileNotFoundError("No serial devices matching /dev/ttyACM* or /dev/ttyUSB* found")
        port = candidates[0]
    print(f"[Serial] Connecting to {port} ...")
    with serial_lock:
        try:
            s = serial.Serial(
                port,
                baudrate,
                timeout=1,
                write_timeout=2,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
            try:
                s.reset_input_buffer()
                s.reset_output_buffer()
                s.setDTR(False)
                s.setRTS(False)
                time.sleep(0.05)
                s.setDTR(True)
                s.setRTS(True)
            except Exception:
                pass
            time.sleep(0.1)
            print(f"[✓] Connected to Arduino on {port}")
            return s
        except Exception as e:
            print(f"[!] Could not connect to Arduino: {e}")
            return None

def send_drive_command(s, speeds):
    global ser
    speeds = [int(min(2000, max(1000, round(v)))) for v in speeds]
    cmd = f"D {speeds[0]} {speeds[1]} {speeds[2]} {speeds[3]}\n"
    need_reconnect = False
    with serial_lock:
        if not (s and s.is_open):
            need_reconnect = True
        else:
            try:
                s.write(cmd.encode())
                dlog(f"[D] {cmd.strip()}")
                return True
            except serial.SerialException as exc:
                print(f"[!] Drive write failed: {exc}")
                need_reconnect = True
    if need_reconnect:
        attempt_serial_reconnect()
    return False

def send_steer_command(s, angles):
    global ser
    angles = [int(round(v)) for v in angles]
    cmd = f"S {angles[0]} {angles[1]} {angles[2]} {angles[3]}\n"
    need_reconnect = False
    with serial_lock:
        dlog(f"[S] {cmd.strip()}")
        if not (s and s.is_open):
            need_reconnect = True
        else:
            try:
                s.write(cmd.encode())
                return True
            except serial.SerialException as exc:
                print(f"[!] Steer write failed: {exc}")
                need_reconnect = True
    if need_reconnect:
        attempt_serial_reconnect()
    return False

def attempt_serial_reconnect():
    global ser
    with serial_lock:
        try:
            if ser:
                ser.close()
        except Exception:
            pass
        ser = None
    print("[Serial] Attempting reconnect...")
    time.sleep(SERIAL_RETRY_DELAY)
    while True:
        try:
            candidate = connect_to_arduino(SERIAL_PORT, SERIAL_BAUD)
        except FileNotFoundError as exc:
            print(f"[Serial] {exc}")
            candidate = None
        if candidate is not None:
            with serial_lock:
                ser = candidate
            break
        print(f"[Serial] Retry in {SERIAL_RETRY_DELAY:.1f}s")
        time.sleep(SERIAL_RETRY_DELAY)

def stop_all_driving():
    try:
        send_drive_command(ser, [1500, 1500, 1500, 1500])
    except Exception:
        pass

def shutdown_robot():
    stop_all_driving()
    try:
        send__command(ser, compose_wheel_angles(0))
    except Exception:
        pass
    try:
        if ser and ser.is_open:
            ser.close()
    except Exception:
        pass

def shortest_angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

def moveRobot(center):
    global last_sent_angle, last_drive_pwms, last_center, vel_smoothed
    global original_center, steering_ready, last_steer_command_time, smoothed_angle
    global last_center_print_time
    
    cx, cy = center
    
    # Initialize on first call
    if last_center is None:
        last_center = (cx, cy)
        vel_smoothed = np.array([0.0, 0.0])
        if original_center is None:
            original_center = (cx, cy)
        print("[INIT] Tracking started - will follow object")
        return

    # Use original_center as reference for relative degree calculation
    frame_cx = original_center[0]
    frame_cy = original_center[1]
    frame_half = frame_width / 2.0  # Keep for normalization

    dx = cx - frame_cx
    dy = cy - frame_cy
    error = dx / frame_half if frame_half > 0 else 0.0
    deadzone_norm = SIDE_LATERAL_DEADZONE / frame_half if frame_half > 0 else 0.0

    vector_mag = math.hypot(dx, dy)
    
    # Don't calculate angle if object is within 30 pixels of original center
    CENTER_THRESHOLD = 30.0  # pixels - no rotation within this distance
    if vector_mag < CENTER_THRESHOLD:
        # Skip debug print to reduce spam
        # Keep current steering, don't send new commands
        last_center = (cx, cy)
        return
    
    # Calculate raw angle only when object has moved away from center
    raw_angle = None
    if vector_mag > CENTER_THRESHOLD:
        raw_angle = (math.degrees(math.atan2(-dx, dy)) + 360.0) % 360.0
    
    # Smooth angle to reduce jitter
    if raw_angle is not None:
        if smoothed_angle is None:
            smoothed_angle = raw_angle
        else:
            # Handle angle wrapping (e.g., 359° -> 1°)
            angle_diff = shortest_angle_diff(raw_angle, smoothed_angle)
            smoothed_angle = (smoothed_angle + ANGLE_ALPHA * angle_diff) % 360.0
        target_angle = smoothed_angle
    elif last_sent_angle is not None:
        target_angle = last_sent_angle
    else:
        target_angle = None

    if target_angle is not None:
        angle_diff = abs(shortest_angle_diff(target_angle, last_sent_angle)) if last_sent_angle is not None else 999
        
        # Send steering only if angle changed by 5 degrees or more
        ANGLE_SEND_THRESHOLD = 5.0  # Only send every 5 degrees
        if angle_diff > ANGLE_SEND_THRESHOLD:
            if send_steer_command(ser, compose_wheel_angles(target_angle)):
                last_sent_angle = target_angle
                last_steer_command_time = time.time()
                steering_ready = False
            # Stop wheels while turning
            target_pwms = [1500] * 4
            if target_pwms != last_drive_pwms:
                send_drive_command(ser, target_pwms)
                last_drive_pwms = target_pwms[:]
            last_center = (cx, cy)
            return

    # Wait for steppers to reach position
    if not steering_ready:
        if time.time() - last_steer_command_time >= 0.3:
            steering_ready = True
        else:
            # Still waiting for steppers to turn
            last_center = (cx, cy)
            return

    # Draw original center marker
    cv2.circle(latest_frame, (int(original_center[0]), int(original_center[1])), 8, (0, 0, 255), -1)
    cv2.putText(latest_frame, "ORIGINAL", (int(original_center[0]) - 30, int(original_center[1]) - 15), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Stop if object is at center (within deadzone)
    if vector_mag < SIDE_LATERAL_DEADZONE:
        target_pwms = [1500] * 4
        if target_pwms != last_drive_pwms:
            send_drive_command(ser, target_pwms)
            last_drive_pwms = target_pwms[:]
        last_center = (cx, cy)
        return
    
    # Drive forward proportional to distance from center
    MAX_DISTANCE = 200.0  # Max distance for speed scaling
    speed_factor = min(vector_mag / MAX_DISTANCE, 1.0)  # 0.0 to 1.0
    drive_speed = int(round(speed_factor * maxSpeed))
    
    # Drive forward in direction wheels are pointing (with per-wheel calibration)
    target_pwms = [1500 + WHEEL_DRIVE_DIR[i] * drive_speed for i in range(4)]
    
    # Only send if PWM changed by at least 5 (reduces serial spam)
    max_pwm_change = max(abs(target_pwms[i] - last_drive_pwms[i]) for i in range(4))
    if max_pwm_change >= 20:
        send_drive_command(ser, target_pwms)
        last_drive_pwms = target_pwms[:]
    
    last_center = (cx, cy)

def normalize_axis(device, code, value):
    try:
        info = device.absinfo(code)
        lo = info.min if info.min is not None else 0
        hi = info.max if info.max is not None else 65535
    except Exception:
        lo, hi = 0, 65535
    if hi == lo:
        return 0.0
    mid = (lo + hi) / 2.0
    span = (hi - lo) / 2.0
    norm = (value - mid) / span
    if abs(norm) < JOYSTICK_DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, norm))

def normalize_trigger(device, code, value):
    try:
        info = device.absinfo(code)
        lo = info.min if info.min is not None else 0
        hi = info.max if info.max is not None else 1023
    except Exception:
        lo, hi = 0, 1023
    if hi <= lo:
        return 0.0
    norm = (value - lo) / (hi - lo)
    return float(max(0.0, min(1.0, norm)))

def manual_drive_from_joystick(forward_axis):
    global last_drive_pwms
    neutral = [1500] * 4
    if abs(forward_axis) < JOYSTICK_DEADZONE:
        if last_drive_pwms != neutral:
            send_drive_command(ser, neutral)
            last_drive_pwms = neutral[:]
        return
    delta = int(round(np.clip(forward_axis, -1.0, 1.0) * maxSpeed))
    pwms = [1500 + WHEEL_DRIVE_DIR[i] * delta for i in range(4)]
    if pwms != last_drive_pwms:
        send_drive_command(ser, pwms)
        last_drive_pwms = pwms[:]

def compose_wheel_angles(target_angle):
    angles = []
    base = int(round(target_angle)) % 360
    for i in range(4):
        ang = (STEER_DIR[i] * base + STEER_OFFSETS[i]) % 360
        angles.append(int(round(ang)))
    return angles

def apply_manual_steer(angle):
    global current_manual_angle, manual_target_angle
    angle = angle % 360
    manual_target_angle = angle
    if current_manual_angle != angle:
        send_steer_command(ser, compose_wheel_angles(angle))
        current_manual_angle = angle

def update_manual_trigger_steer():
    global manual_target_angle, trigger_prev
    if drive_mode_auto:
        return
    left_active = trigger_axes["left"] >= TRIGGER_THRESHOLD
    right_active = trigger_axes["right"] >= TRIGGER_THRESHOLD
    if left_active and not trigger_prev["left"]:
        manual_target_angle = manual_target_angle - 45
        apply_manual_steer(manual_target_angle)
    if right_active and not trigger_prev["right"]:
        manual_target_angle = manual_target_angle + 45
        apply_manual_steer(manual_target_angle)
    trigger_prev["left"] = left_active
    trigger_prev["right"] = right_active

def trigger_motor_control(direction, userInput, prevSpeed):
    global maxSpeed, ser, commandCounter
    if userInput < 100 and prevSpeed != 0:
        send_drive_command(ser, [1500, 1500, 1500, 1500])
        return 0
    else:
        speedMultiplier = int(round(map_value(userInput, 0, 1023, 0, direction * maxSpeed)))
        if abs(speedMultiplier - prevSpeed) > 5:
            send_drive_command(ser, [1500 + speedMultiplier] * 4)
            commandCounter += 1
        return speedMultiplier

def select_point(event, x, y, flags, params):
    global tracking_active, prev_gray, prev_points, last_center, vel_smoothed, steering_ready, last_sent_angle, last_drive_direction, original_center, tracking_state, lost_frames, current_points, reacquire_box, smoothed_angle
    if event == cv2.EVENT_LBUTTONDOWN:
        with frame_lock:
            if latest_frame is None:
                return
            frame = latest_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prev_gray = gray.copy()
        half = 5
        x1, y1 = max(0, x - half), max(0, y - half)
        x2, y2 = min(gray.shape[1] - 1, x + half), min(gray.shape[0] - 1, y + half)
        mask = np.zeros_like(gray)
        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
        prev_points = cv2.goodFeaturesToTrack(
            gray, mask=mask, maxCorners=30, qualityLevel=0.15, minDistance=5, blockSize=10
        )
        if prev_points is not None:
            tracking_active = True
            last_center = None
            vel_smoothed = np.array([0.0, 0.0])
            smoothed_angle = None
            steering_ready = True
            last_sent_angle = None
            last_drive_direction = 1
            original_center = (x, y)
            tracking_state = "tracking"
            lost_frames = 0
            current_points = None
            reacquire_box = None
            print(f"[Camera] Selected {len(prev_points)} tracking points at ({x},{y})")
        else:
            print("[Camera] No good features found.")

def camera_loop():
    global latest_frame
    while True:
        ret, frame = camera.read()
        if not ret:
            print("[WARN] Frame not received")
            time.sleep(0.05)
            continue
        with frame_lock:
            latest_frame = frame.copy()
        time.sleep(0.01)

def auto_tracking_loop():
    global prev_gray, prev_points, tracking_active, last_center, tracking_state, lost_frames, current_points, reacquire_box, last_status
    print("[System] Auto-tracking thread started.")
    while True:
        if not drive_mode_auto or not tracking_active:
            time.sleep(0.05)
            continue

        with frame_lock:
            if latest_frame is None:
                time.sleep(0.01)
                continue
            frame = latest_frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if prev_points is not None and prev_gray is not None:
            next_points, status, err = cv2.calcOpticalFlowPyrLK(
                prev_gray, gray, prev_points, None, **lk_params
            )
            if next_points is None or status is None or err is None:
                tracking_active = False
                continue
            next_points = next_points.reshape(-1, 2)
            status = status.reshape(-1)
            err = err.reshape(-1)
            good_new = next_points[status == 1]
            good_old = prev_points[status == 1]
            good_err = err[status == 1]
            if len(good_new) == 0:
                lost_frames += 1
                tracking_state = "frozen"
                last_status = "lost"
                stop_all_driving()
                current_points = None
                reacquire_box = None
                continue
            avg_err = np.mean(good_err)
            motion_mag = np.mean(np.linalg.norm(good_new - good_old, axis=1))
            if avg_err > 25.0 or motion_mag > 40.0:
                print(f"[OCCLUDED] AvgErr={avg_err:.1f}, Motion={motion_mag:.1f} → freeze")
                stop_all_driving()
                tracking_state = "frozen"
                last_status = "lost"
                lost_frames += 1
                current_points = None
                reacquire_box = None
                continue
            reliable_mask = good_err < 25.0
            reliable_points = good_new[reliable_mask]
            if len(reliable_points) >= 1:
                tracking_state = "tracking"
                last_status = "tracking"
                lost_frames = 0
                prev_points = reliable_points.reshape(-1, 1, 2)
                prev_gray = gray.copy()
                current_points = reliable_points
                reacquire_box = None
                avg_new = np.mean(reliable_points, axis=0)
                cx, cy = int(avg_new[0]), int(avg_new[1])
                last_center = (cx, cy)
                moveRobot((cx, cy))
            else:
                lost_frames += 1
                tracking_state = "frozen"
                last_status = "lost"
                stop_all_driving()
                current_points = None
                reacquire_box = None
                if lost_frames % REACQUIRE_INTERVAL == 0 and last_center is not None:
                    expansion = min(3 * REACQUIRE_RADIUS, REACQUIRE_RADIUS + int(lost_frames * 1.5))
                    x, y = map(int, last_center)
                    mask = np.zeros_like(gray)
                    x1, y1 = max(0, x - expansion), max(0, y - expansion)
                    x2, y2 = min(gray.shape[1] - 1, x + expansion), min(gray.shape[0] - 1, y + expansion)
                    reacquire_box = (x1, y1, x2, y2)
                    cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
                    new_features = cv2.goodFeaturesToTrack(
                        gray, mask=mask, maxCorners=40, qualityLevel=0.15, minDistance=5, blockSize=10
                    )
                    if new_features is not None:
                        print(f"[AUTO] Reacquired {len(new_features)} features.")
                        prev_points = new_features
                        prev_gray = gray.copy()
                        tracking_state = "tracking"
                        lost_frames = 0
                        current_points = new_features.reshape(-1, 2)
                        reacquire_box = None
        else:
            prev_gray = gray.copy()
        time.sleep(0.1)  # Slower loop = less serial commands

def controller_loop():
    global controllerInput, drive_mode_auto, last_y_press_time, current_manual_angle, manual_target_angle, trigger_prev, last_center, vel_smoothed, steering_ready, tracking_active, prev_gray, prev_points, tracking_state, lost_frames, current_points, reacquire_box
    while True:
        try:
            if controllerInput is None:
                devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
                for d in devices:
                    if "Xbox" in d.name:
                        controllerInput = evdev.InputDevice(d.path)
                        print(f"[✓] Xbox controller connected: {d.name}")
                        break
                if controllerInput is None:
                    print("[!] No Xbox controller found. Retrying...")
                    time.sleep(2)
                    continue
            for event in controllerInput.read_loop():
                if event.type == evdev.ecodes.EV_KEY:
                    if event.code == evdev.ecodes.BTN_WEST and event.value == 1:
                        if time.time() - last_y_press_time > 0.5:
                            drive_mode_auto = not drive_mode_auto
                            mode = "AUTO-TRACKING" if drive_mode_auto else "MANUAL DRIVE"
                            print(f"[Controller] Y pressed → {mode}")
                            last_y_press_time = time.time()
                            manual_axes["x"] = manual_axes["y"] = 0.0
                            trigger_axes["left"] = trigger_axes["right"] = 0.0
                            manual_drive_from_joystick(0.0)
                            manual_target_angle = 0.0
                            trigger_prev["left"] = trigger_prev["right"] = False
                            apply_manual_steer(manual_target_angle)
                            current_manual_angle = manual_target_angle
                            last_center = None
                            vel_smoothed = np.array([0.0, 0.0])
                            steering_ready = True
                            if not drive_mode_auto:
                                tracking_active = False
                                prev_gray = None
                                prev_points = None
                                tracking_state = "idle"
                                lost_frames = 0
                                current_points = None
                                reacquire_box = None
                                original_center = None
                                stop_all_driving()
                elif event.type == evdev.ecodes.EV_ABS and not drive_mode_auto:
                    if event.code == evdev.ecodes.ABS_Y:
                        manual_axes["y"] = -normalize_axis(controllerInput, evdev.ecodes.ABS_Y, event.value)
                        manual_drive_from_joystick(manual_axes["y"])
                    elif event.code in TRIGGER_LEFT_CODES:
                        trigger_axes["left"] = normalize_trigger(controllerInput, event.code, event.value)
                        update_manual_trigger_steer()
                    elif event.code in TRIGGER_RIGHT_CODES:
                        trigger_axes["right"] = normalize_trigger(controllerInput, event.code, event.value)
                        update_manual_trigger_steer()
        except (OSError, IOError):
            print("[!] Controller disconnected. Retrying...")
            controllerInput = None
            time.sleep(2)

if __name__ == "__main__":
    ser = connect_to_arduino(SERIAL_PORT, SERIAL_BAUD)

    # --- auto-zero once after connect (before threads) ---
    if ser:
        time.sleep(0.2)  # let MCU settle
        
        # Send steering command to 0° (function handles its own locking)
        print("[Init] Setting wheels to 0°...")
        send_steer_command(ser, [0, 0, 0, 0])
        time.sleep(0.3)  # Wait for command to process
        
        # Calibrate current position as 0° for all modules
        print("[Init] Calibrating stepper positions...")
        with serial_lock:
            if ser and ser.is_open:
                try:
                    ser.write(b"C 0 0\nC 1 0\nC 2 0\nC 3 0\n")
                    time.sleep(0.1)
                    print("[✓] Auto-calibration complete")
                except Exception as e:
                    print(f"[!] Calibration write failed: {e}")
    # -----------------------------------------------------

    threading.Thread(target=camera_loop, daemon=True).start()
    threading.Thread(target=auto_tracking_loop, daemon=True).start()
    threading.Thread(target=controller_loop, daemon=True).start()

    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", select_point)
    print("[System] Running. Click to select target. Press Y to toggle auto-tracking.")
    try:
        while True:
            with frame_lock:
                if latest_frame is None:
                    time.sleep(0.01)
                    continue
                display = latest_frame.copy()
            if tracking_active:
                if current_points is not None:
                    for p in current_points:
                        px, py = p.ravel()
                        cv2.circle(display, (int(px), int(py)), 4, (0, 255, 0), -1)
                if last_center is not None:
                    cv2.circle(display, (int(last_center[0]), int(last_center[1])), 6, (0, 0, 255), 2)
                if reacquire_box is not None:
                    cv2.rectangle(display, (reacquire_box[0], reacquire_box[1]), (reacquire_box[2], reacquire_box[3]), (255, 0, 0), 2)
                cv2.putText(display, tracking_state if tracking_state else "", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            mode_text = "AUTO" if drive_mode_auto else "MANUAL"
            color = (0, 255, 0) if drive_mode_auto else (0, 0, 255)
            cv2.putText(display, f"Mode: {mode_text}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow("Frame", display)
            if cv2.waitKey(1) & 0xFF == 27:
                print("Exiting...")
                break
            time.sleep(0.01)
    finally:
        shutdown_robot()
        try:
            camera.release()
        except Exception:
            pass
        cv2.destroyAllWindows()