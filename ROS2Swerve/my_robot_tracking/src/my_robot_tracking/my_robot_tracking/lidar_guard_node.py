#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class LidarGuardNode(Node):
    """Monitor a LaserScan for near-field obstacles and raise stop/caution flags."""

    def __init__(self) -> None:
        super().__init__('lidar_guard')

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_distance', 0.45)          # meters
        self.declare_parameter('caution_distance', 1.2)        # meters
        self.declare_parameter('release_margin', 0.05)         # meters added for hysteresis
        self.declare_parameter('min_valid_fraction', 0.05)     # fraction of samples that must be valid
        self.declare_parameter('scan_timeout', 0.5)            # seconds without scans => stop
        self.declare_parameter('publish_min_distance', True)
        self.declare_parameter('stop_topic', '/safety/stop')
        self.declare_parameter('caution_topic', '/safety/caution')
        self.declare_parameter('distance_topic', '/safety/min_distance')

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.caution_distance = float(self.get_parameter('caution_distance').value)
        self.release_margin = float(self.get_parameter('release_margin').value)
        self.min_valid_fraction = float(self.get_parameter('min_valid_fraction').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.publish_min_distance = bool(self.get_parameter('publish_min_distance').value)
        self.stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value
        self.caution_topic = self.get_parameter('caution_topic').get_parameter_value().string_value
        self.distance_topic = self.get_parameter('distance_topic').get_parameter_value().string_value

        if self.stop_distance <= 0.0:
            self.get_logger().warn('stop_distance must be > 0; clamping to 0.3m')
            self.stop_distance = 0.3
        if self.caution_distance <= self.stop_distance:
            self.get_logger().warn('caution_distance <= stop_distance; increasing caution_distance')
            self.caution_distance = self.stop_distance + 0.3
        if self.release_margin < 0.0:
            self.release_margin = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.stop_pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.caution_pub = self.create_publisher(Bool, self.caution_topic, 10)
        self.distance_pub: Optional[Publisher] = None
        if self.publish_min_distance:
            self.distance_pub = self.create_publisher(Float32, self.distance_topic, 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos,
        )

        self.last_scan_time = time.monotonic()
        self.stop_active = False
        self.caution_active = False
        self.last_min_distance: Optional[float] = None

        # Watchdog for stale scans
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info(
            f'LIDAR guard active on topic "{self.scan_topic}" '
            f'(stop<{self.stop_distance:.2f} m, caution<{self.caution_distance:.2f} m)'
        )

    def scan_callback(self, scan: LaserScan) -> None:
        """Process incoming scan and update stop/caution flags."""
        self.last_scan_time = time.monotonic()

        valid_ranges = [
            r for r in scan.ranges
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max
        ]

        if not valid_ranges:
            # No usable data; hold previous state and warn periodically via watchdog.
            return

        valid_fraction = len(valid_ranges) / max(1, len(scan.ranges))
        if valid_fraction < self.min_valid_fraction:
            self.get_logger().warn_throttle(
                1.0,
                f'Only {valid_fraction*100:.1f}% of LIDAR samples valid; keeping previous state',
            )
            return

        min_distance = min(valid_ranges)
        self.last_min_distance = min_distance

        if self.distance_pub:
            msg = Float32()
            msg.data = float(min_distance)
            self.distance_pub.publish(msg)

        # Hysteresis thresholds
        stop_enter = self.stop_distance
        stop_release = self.stop_distance + self.release_margin
        caution_enter = self.caution_distance
        caution_release = self.caution_distance + self.release_margin

        # Stop zone logic
        if min_distance <= stop_enter:
            if not self.stop_active:
                self.get_logger().warn(
                    f'LIDAR stop triggered (min distance {min_distance:.2f} m)'
                )
            self.stop_active = True
        elif self.stop_active and min_distance > stop_release:
            self.get_logger().info('LIDAR stop cleared')
            self.stop_active = False

        # Caution logic (only if not stopped)
        if self.stop_active:
            self.caution_active = True
        else:
            if min_distance <= caution_enter:
                if not self.caution_active:
                    self.get_logger().info(
                        f'LIDAR caution triggered (min distance {min_distance:.2f} m)'
                    )
                self.caution_active = True
            elif self.caution_active and min_distance > caution_release:
                self.get_logger().info('LIDAR caution cleared')
                self.caution_active = False

        self.publish_flags()

    def watchdog_check(self) -> None:
        """Ensure we stop if scans go stale or disappear."""
        elapsed = time.monotonic() - self.last_scan_time
        if elapsed > self.scan_timeout:
            if not self.stop_active:
                self.get_logger().error(
                    f'No LIDAR data for {elapsed:.2f}s (timeout {self.scan_timeout:.2f}s); forcing stop'
                )
            self.stop_active = True
            self.caution_active = True
            self.publish_flags()
            self.last_min_distance = None
            self.get_logger().warn('LIDAR data stale; forcing stop')

    def publish_flags(self) -> None:
        stop_msg = Bool()
        stop_msg.data = self.stop_active
        self.stop_pub.publish(stop_msg)

        caution_msg = Bool()
        caution_msg.data = self.caution_active
        self.caution_pub.publish(caution_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
