#!/usr/bin/env python3
"""
PyQt5 implementation of the MedRa robot UI so it can run without a web browser.

The UI mirrors the existing Flask/HTML layout:
    - Login screen with MedRa branding and Login/Register buttons
    - Dashboard with Manual Mode and Camera entry points plus doctor name
    - Manual control page with directional pad and top-half controls
    - Camera page showing the tracking feed placeholder and status indicators

All actions call the existing Flask API (default http://localhost:8080) so the
ROS2 bridge and nodes can remain unchanged.
"""

from __future__ import annotations

import base64
import os
import sys
from typing import Callable, Optional

import requests
from PyQt5.QtCore import Qt, QTimer
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


API_BASE_URL = os.getenv("MEDRA_API_URL", "http://localhost:8080").rstrip("/")


class RobotAPI:
    """Thin wrapper around the Flask endpoints with short timeouts."""

    def __init__(self, base_url: str = API_BASE_URL) -> None:
        self.base_url = base_url

    def _post(self, route: str, payload: Optional[dict] = None) -> None:
        try:
            requests.post(
                f"{self.base_url}{route}",
                json=payload,
                timeout=0.4,
            )
        except requests.RequestException as exc:
            print(f"[RobotAPI] POST {route} failed:", exc)

    def _get(self, route: str) -> Optional[dict]:
        try:
            response = requests.get(f"{self.base_url}{route}", timeout=0.4)
            response.raise_for_status()
            return response.json()
        except requests.RequestException as exc:
            print(f"[RobotAPI] GET {route} failed:", exc)
            return None

    def move(self, direction: str, speed: int = 50) -> None:
        self._post("/api/move", {"direction": direction, "speed": speed})

    def stop(self) -> None:
        self._post("/api/stop")

    def control_top_half(self, action: str) -> None:
        self._post("/api/top_half", {"action": action})

    def status(self) -> Optional[dict]:
        return self._get("/api/status")

    def camera_frame(self) -> Optional[str]:
        data = self._get("/api/camera_frame")
        if not data:
            return None
        return data.get("frame")


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

        # Periodic polling for status/camera frames
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(1000)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start()

        self.camera_timer = QTimer(self)
        self.camera_timer.setInterval(200)
        self.camera_timer.timeout.connect(self.update_camera)
        self.camera_timer.start()

    def update_status(self) -> None:
        data = self.api.status()
        if not data:
            self.status_indicator.setStyleSheet("color: #ff4d4f;")
            self.status_text.setText("Tracking: Unknown (API offline)")
            return

        active = data.get("tracking_active", False)
        safety_stop = data.get("safety_stop", False)
        self.status_indicator.setStyleSheet("color: #16a34a;" if active else "color: #ef4444;")
        text = "Tracking: Active" if active else "Tracking: Inactive"
        if safety_stop:
            text += " — STOPPED"
        self.status_text.setText(text)

    def update_camera(self) -> None:
        frame = self.api.camera_frame()
        if not frame:
            return

        if frame.startswith("data:image"):
            _, _, encoded = frame.partition(",")
        else:
            encoded = frame

        try:
            raw = base64.b64decode(encoded)
        except base64.binascii.Error as exc:
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
        if self.image_label.pixmap():
            self.update_camera()


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
    app = QApplication(sys.argv)
    window = MedRaWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
