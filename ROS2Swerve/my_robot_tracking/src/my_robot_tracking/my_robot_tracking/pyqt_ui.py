#!/usr/bin/env python3
"""
PyQt5 implementation of the MedRa robot UI so it can run without a web browser.

The UI connects directly to ROS2 topics - no Flask server needed!

The UI includes:
    - Login screen with MedRa branding and Login/Register buttons
    - Dashboard with Manual Mode and Camera entry points plus doctor name
    - Manual control page with directional pad and top-half controls
    - Camera page showing the tracking feed and status indicators
"""

from __future__ import annotations

import base64
import sys
import threading
from typing import Callable, Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray


class RobotAPI(Node, QObject):
    """Direct ROS2 connection - publishes/subscribes to topics."""
    
    status_updated = pyqtSignal(dict)  # Signal for status updates
    camera_frame_updated = pyqtSignal(str)  # Signal for camera frames
    
    def __init__(self) -> None:
        Node.__init__(self, 'pyqt_ui_node')
        QObject.__init__(self)
        
        self.bridge = CvBridge()
        self.tracking_active = False
        self.safety_stop = False
        self.latest_camera_frame = None
        
        # Publishers
        self.manual_cmd_vel_pub = self.create_publisher(Twist, '/manual/cmd_vel', 10)
        self.manual_steering_pub = self.create_publisher(Float64MultiArray, '/manual/steering_angles', 10)
        
        # Subscribers
        self.tracking_sub = self.create_subscription(Bool, '/tracking/active', self.tracking_callback, 10)
        self.safety_sub = self.create_subscription(Bool, '/safety/stop', self.safety_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image', self.camera_callback, 10)
        
        self.get_logger().info('PyQt UI ROS2 node started')

    def tracking_callback(self, msg: Bool) -> None:
        self.tracking_active = msg.data
        self._emit_status()
    
    def safety_callback(self, msg: Bool) -> None:
        self.safety_stop = msg.data
        self._emit_status()
    
    def camera_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_camera_frame = cv_image
            # Convert to base64 for QLabel
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            self.camera_frame_updated.emit(f'data:image/jpeg;base64,{img_base64}')
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")
    
    def _emit_status(self) -> None:
        self.status_updated.emit({
            'tracking_active': self.tracking_active,
            'safety_stop': self.safety_stop
        })
    
    def move(self, direction: str, speed: int = 50) -> None:
        twist = Twist()
        if direction == 'up':
            twist.linear.x = float(speed)
        elif direction == 'down':
            twist.linear.x = float(-speed)
        elif direction == 'left':
            twist.angular.z = float(speed)
        elif direction == 'right':
            twist.angular.z = float(-speed)
        self.manual_cmd_vel_pub.publish(twist)
    
    def stop(self) -> None:
        twist = Twist()  # All zeros
        self.manual_cmd_vel_pub.publish(twist)
    
    def control_top_half(self, action: str) -> None:
        # TODO: Implement top half control via ROS2 topic
        self.get_logger().info(f"Top half control: {action}")
    
    def status(self) -> Optional[dict]:
        return {
            'tracking_active': self.tracking_active,
            'safety_stop': self.safety_stop
        }
    
    def camera_frame(self) -> Optional[str]:
        # This is handled via signal now, but keeping for compatibility
        return None


def build_title_label(text: str, size: int) -> QLabel:
    label = QLabel(text)
    label.setAlignment(Qt.AlignCenter)
    label.setFont(QFont("Arial", size, QFont.Bold))
    return label


class LoginPage(QWidget):
    def __init__(self, on_enter: Callable[[], None]) -> None:
        super().__init__()
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)

        title = build_title_label("MedRa", 48)
        subtitle = QLabel("Medical Robot Control System")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setFont(QFont("Arial", 16))

        button_box = QVBoxLayout()
        button_box.setSpacing(20)

        login_btn = QPushButton("Login")
        register_btn = QPushButton("Register")
        for btn in (login_btn, register_btn):
            btn.setFixedHeight(60)
            btn.setFont(QFont("Arial", 16, QFont.Bold))
            btn.clicked.connect(on_enter)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        button_box.addWidget(login_btn)
        button_box.addWidget(register_btn)

        card = QVBoxLayout()
        card.setSpacing(30)
        card.addWidget(title)
        card.addWidget(subtitle)
        card.addLayout(button_box)

        wrapper = QFrame()
        wrapper.setLayout(card)
        wrapper.setObjectName("card")
        wrapper.setFixedWidth(360)

        layout.addWidget(wrapper)
        self.setLayout(layout)


class DashboardPage(QWidget):
    def __init__(self, on_manual: Callable[[], None], on_camera: Callable[[], None]) -> None:
        super().__init__()

        outer = QVBoxLayout()
        outer.setContentsMargins(40, 40, 40, 40)
        outer.setSpacing(40)

        header = QHBoxLayout()
        header.addStretch()
        doctor_label = QLabel("Dr. Smith")
        doctor_label.setObjectName("doctorLabel")
        doctor_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        doctor_label.setFont(QFont("Arial", 16, QFont.Bold))
        header.addWidget(doctor_label)

        cards = QHBoxLayout()
        cards.setSpacing(40)

        manual_btn = self._build_card_button("🎮", "Manual Mode", "Direct robot control", on_manual)
        camera_btn = self._build_card_button("📷", "Camera", "Live tracking view", on_camera)

        cards.addWidget(manual_btn)
        cards.addWidget(camera_btn)

        outer.addLayout(header)
        outer.addStretch()
        outer.addLayout(cards)
        outer.addStretch()

        self.setLayout(outer)

    @staticmethod
    def _build_card_button(icon: str, title: str, subtitle: str, handler: Callable[[], None]) -> QPushButton:
        button = QPushButton()
        button.setObjectName("cardButton")
        button.setCursor(Qt.PointingHandCursor)
        button.setMinimumSize(280, 320)
        button.clicked.connect(handler)

        layout = QVBoxLayout(button)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(20)

        icon_label = QLabel(icon)
        icon_label.setAlignment(Qt.AlignCenter)
        icon_label.setFont(QFont("Arial", 64))

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 26, QFont.Bold))

        subtitle_label = QLabel(subtitle)
        subtitle_label.setAlignment(Qt.AlignCenter)
        subtitle_label.setFont(QFont("Arial", 14))

        layout.addWidget(icon_label)
        layout.addWidget(title_label)
        layout.addWidget(subtitle_label)

        return button


class ManualPage(QWidget):
    def __init__(self, api: RobotAPI, on_back: Callable[[], None]) -> None:
        super().__init__()
        self.api = api
        self.current_direction: Optional[str] = None
        self.move_timer = QTimer(self)
        self.move_timer.setInterval(100)
        self.move_timer.timeout.connect(self._send_move_command)

        outer = QVBoxLayout()
        outer.setContentsMargins(30, 30, 30, 30)
        outer.setSpacing(20)

        header = QHBoxLayout()
        back_btn = QPushButton("← Back")
        back_btn.setFixedWidth(120)
        back_btn.clicked.connect(on_back)
        back_btn.setObjectName("backButton")

        title = QLabel("Manual Control")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 24, QFont.Bold))

        header.addWidget(back_btn)
        header.addStretch()
        header.addWidget(title)
        header.addStretch()

        content = QHBoxLayout()
        content.setSpacing(40)

        movement_card = self._build_movement_card()
        top_half_card = self._build_top_half_card()

        content.addWidget(movement_card, stretch=1)
        content.addWidget(top_half_card, stretch=1)

        outer.addLayout(header)
        outer.addLayout(content)
        self.setLayout(outer)

    def _build_movement_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("controlCard")
        layout = QVBoxLayout(card)
        layout.setSpacing(20)

        heading = QLabel("Robot Movement")
        heading.setAlignment(Qt.AlignCenter)
        heading.setFont(QFont("Arial", 20, QFont.Bold))

        grid = QGridLayout()
        grid.setSpacing(20)

        up_btn = self._arrow_button("↑", "up")
        down_btn = self._arrow_button("↓", "down")
        left_btn = self._arrow_button("←", "left")
        right_btn = self._arrow_button("→", "right")

        grid.addWidget(up_btn, 0, 1)
        grid.addWidget(left_btn, 1, 0)
        grid.addWidget(right_btn, 1, 2)
        grid.addWidget(down_btn, 2, 1)

        layout.addWidget(heading)
        layout.addStretch()
        layout.addLayout(grid)
        layout.addStretch()
        return card

    def _build_top_half_card(self) -> QFrame:
        card = QFrame()
        card.setObjectName("controlCard")
        layout = QVBoxLayout(card)
        layout.setSpacing(20)

        heading = QLabel("Top Half Control")
        heading.setAlignment(Qt.AlignCenter)
        heading.setFont(QFont("Arial", 20, QFont.Bold))

        plus_row = QHBoxLayout()
        plus_row.setSpacing(10)
        plus_symbol = QLabel("↑")
        plus_symbol.setFont(QFont("Arial", 32))
        plus_symbol.setAlignment(Qt.AlignCenter)
        plus_row.addWidget(plus_symbol)
        plus_row.addSpacing(10)
        plus_row.addWidget(self._small_button("+", lambda: self.api.control_top_half("up_plus")))
        plus_row.addWidget(self._small_button("-", lambda: self.api.control_top_half("up_minus")))

        up_row = QHBoxLayout()
        up_row.setSpacing(20)
        up_row.addWidget(self._top_half_button("↑", "up"))
        up_row.addWidget(self._top_half_button("↑", "up"))

        down_row = QHBoxLayout()
        down_row.setSpacing(20)
        down_row.addWidget(self._top_half_button("↓", "down"))
        down_row.addWidget(self._top_half_button("↓", "down"))

        minus_row = QHBoxLayout()
        minus_row.setSpacing(10)
        minus_symbol = QLabel("↓")
        minus_symbol.setFont(QFont("Arial", 32))
        minus_symbol.setAlignment(Qt.AlignCenter)
        minus_row.addWidget(minus_symbol)
        minus_row.addSpacing(10)
        minus_row.addWidget(self._small_button("+", lambda: self.api.control_top_half("down_plus")))
        minus_row.addWidget(self._small_button("-", lambda: self.api.control_top_half("down_minus")))

        layout.addWidget(heading)
        layout.addLayout(plus_row)
        layout.addLayout(up_row)
        layout.addLayout(down_row)
        layout.addLayout(minus_row)
        return card

    def _arrow_button(self, label: str, direction: str) -> QPushButton:
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 32, QFont.Bold))
        btn.setFixedSize(120, 120)
        btn.setObjectName("arrowButton")
        btn.pressed.connect(lambda d=direction: self.start_move(d))
        btn.released.connect(self.stop_move)
        return btn

    def _top_half_button(self, label: str, action: str) -> QPushButton:
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 26, QFont.Bold))
        btn.setFixedSize(160, 100)
        btn.setObjectName("topHalfButton")
        btn.pressed.connect(lambda a=action: self.api.control_top_half(a))
        btn.released.connect(lambda: self.api.control_top_half("stop"))
        return btn

    def _small_button(self, label: str, handler: Callable[[], None]) -> QPushButton:
        btn = QPushButton(label)
        btn.setFont(QFont("Arial", 20, QFont.Bold))
        btn.setFixedSize(60, 60)
        btn.setObjectName("smallControlButton")
        btn.clicked.connect(handler)
        return btn

    def start_move(self, direction: str) -> None:
        self.current_direction = direction
        self.api.move(direction)
        self.move_timer.start()

    def _send_move_command(self) -> None:
        if self.current_direction:
            self.api.move(self.current_direction)

    def stop_move(self) -> None:
        self.move_timer.stop()
        self.api.stop()
        self.current_direction = None


class CameraPage(QWidget):
    def __init__(self, api: RobotAPI, on_back: Callable[[], None]) -> None:
        super().__init__()
        self.api = api

        outer = QVBoxLayout()
        outer.setContentsMargins(30, 30, 30, 30)
        outer.setSpacing(20)

        header = QHBoxLayout()
        back_btn = QPushButton("← Back")
        back_btn.setFixedWidth(120)
        back_btn.clicked.connect(on_back)
        back_btn.setObjectName("backButton")

        title = QLabel("Camera Tracking")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 24, QFont.Bold))

        header.addWidget(back_btn)
        header.addStretch()
        header.addWidget(title)
        header.addStretch()

        status_box = QHBoxLayout()
        status_box.setSpacing(12)
        status_box.addStretch()
        self.status_indicator = QLabel("●")
        self.status_indicator.setObjectName("statusIndicator")
        self.status_indicator.setFont(QFont("Arial", 24))
        self.status_text = QLabel("Tracking: Inactive")
        self.status_text.setFont(QFont("Arial", 16, QFont.Bold))
        status_box.addWidget(self.status_indicator)
        status_box.addWidget(self.status_text)
        status_box.addStretch()

        image_frame = QFrame()
        image_frame.setObjectName("cameraFrame")
        image_layout = QVBoxLayout(image_frame)
        image_layout.setAlignment(Qt.AlignCenter)
        image_layout.setContentsMargins(20, 20, 20, 20)

        self.image_label = QLabel("Loading camera feed...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setObjectName("cameraLabel")

        instruction = QLabel("Click the feed to select a tracking target (placeholder)")
        instruction.setAlignment(Qt.AlignCenter)
        instruction.setFont(QFont("Arial", 12))

        image_layout.addWidget(self.image_label)
        image_layout.addSpacing(10)
        image_layout.addWidget(instruction)

        outer.addLayout(header)
        outer.addLayout(status_box)
        outer.addWidget(image_frame)
        self.setLayout(outer)

        # Connect to ROS2 signals (no polling needed!)
        self.api.status_updated.connect(self.update_status)
        self.api.camera_frame_updated.connect(self.update_camera_frame)
        
        # Initial status update
        self.update_status()

    def update_status(self, data: Optional[dict] = None) -> None:
        if data is None:
            data = self.api.status()
        
        if not data:
            self.status_indicator.setStyleSheet("color: #ff4d4f;")
            self.status_text.setText("Tracking: Unknown (ROS2 offline)")
            return

        active = data.get("tracking_active", False)
        safety_stop = data.get("safety_stop", False)
        self.status_indicator.setStyleSheet("color: #16a34a;" if active else "color: #ef4444;")
        text = "Tracking: Active" if active else "Tracking: Inactive"
        if safety_stop:
            text += " — STOPPED"
        self.status_text.setText(text)

    def update_camera_frame(self, frame: str) -> None:
        if not frame:
            return

        if frame.startswith("data:image"):
            _, _, encoded = frame.partition(",")
        else:
            encoded = frame

        try:
            raw = base64.b64decode(encoded)
        except Exception as exc:
            print("[CameraPage] Failed to decode frame:", exc)
            return

        image = QImage.fromData(raw)
        if image.isNull():
            return

        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        # The signal will update the camera frame when a new one arrives


class MedRaWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("MedRa Robot Control")
        self.resize(1200, 800)

        self.api = RobotAPI()

        self.stack = QStackedWidget()

        self.login_page = LoginPage(self.show_dashboard)
        self.dashboard_page = DashboardPage(self.show_manual, self.show_camera)
        self.manual_page = ManualPage(self.api, self.show_dashboard)
        self.camera_page = CameraPage(self.api, self.show_dashboard)

        for page in (self.login_page, self.dashboard_page, self.manual_page, self.camera_page):
            self.stack.addWidget(page)

        layout = QVBoxLayout(self)
        layout.addWidget(self.stack)

        self.apply_styles()

    def apply_styles(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #667eea, stop:1 #764ba2);
                color: #1f2937;
            }
            #card {
                background: white;
                border-radius: 24px;
                padding: 40px 60px;
            }
            QPushButton {
                background-color: #4f46e5;
                color: white;
                border: none;
                border-radius: 16px;
                padding: 18px 32px;
            }
            QPushButton:hover {
                background-color: #4338ca;
            }
            QPushButton:pressed {
                background-color: #3730a3;
            }
            #cardButton {
                background-color: white;
                border-radius: 32px;
                border: 3px solid rgba(79, 70, 229, 0.15);
            }
            #cardButton:hover {
                border-color: rgba(79, 70, 229, 0.4);
                background-color: rgba(255, 255, 255, 0.9);
            }
            #doctorLabel {
                background: rgba(255, 255, 255, 0.25);
                padding: 12px 32px;
                border-radius: 999px;
            }
            #controlCard {
                background: white;
                border-radius: 24px;
                padding: 30px;
            }
            #arrowButton {
                background: white;
                border: 4px solid #4f46e5;
                border-radius: 20px;
                color: #4f46e5;
            }
            #arrowButton:pressed {
                background: #4f46e5;
                color: white;
            }
            #topHalfButton {
                background: white;
                border: 4px solid #ec4899;
                border-radius: 18px;
                color: #ec4899;
            }
            #topHalfButton:pressed {
                background: #ec4899;
                color: white;
            }
            #smallControlButton {
                background: #ec4899;
                border-radius: 14px;
            }
            #smallControlButton:hover {
                background: #db2777;
            }
            #backButton {
                background: #4f46e5;
                border-radius: 12px;
            }
            #cameraFrame {
                background: rgba(17, 24, 39, 0.85);
                border-radius: 24px;
            }
            #cameraLabel {
                background: rgba(17, 24, 39, 0.4);
                border-radius: 18px;
                padding: 20px;
                color: white;
            }
            #statusIndicator {
                color: #ef4444;
            }
            """
        )

    def show_dashboard(self) -> None:
        self.stack.setCurrentWidget(self.dashboard_page)

    def show_manual(self) -> None:
        self.stack.setCurrentWidget(self.manual_page)

    def show_camera(self) -> None:
        self.stack.setCurrentWidget(self.camera_page)


def main() -> None:
    # Initialize ROS2
    try:
        rclpy.init()
    except RuntimeError:
        # ROS2 already initialized, that's fine
        pass
    
    app = QApplication(sys.argv)
    
    # Create window (this initializes RobotAPI which needs ROS2)
    window = MedRaWindow()
    
    # Start ROS2 spinning in a separate thread
    def spin_ros():
        try:
            rclpy.spin(window.api)
        except Exception as e:
            print(f"ROS2 spin error: {e}", file=sys.stderr)
    
    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
