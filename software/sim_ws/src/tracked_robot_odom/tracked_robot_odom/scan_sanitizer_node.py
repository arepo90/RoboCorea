#!/usr/bin/env python3
"""Stage A LaserScan sanitizer.

Gazebo GPU lidar can publish finite max-range endpoints and near-field
self-returns from robot geometry. Nav2 should consume obstacle hits, not those
simulation artifacts, so this node republishes /scan_raw as /scan after replacing
those artifacts with +inf.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):
    def __init__(self):
        super().__init__('scan_sanitizer_node')
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('self_clearance_range', 0.65)
        self.declare_parameter('max_range_epsilon', 0.05)
        self.declare_parameter('drop_max_range_returns', True)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.self_clearance_range = float(self.get_parameter('self_clearance_range').value)
        self.max_range_epsilon = float(self.get_parameter('max_range_epsilon').value)
        self.drop_max_range_returns = bool(self.get_parameter('drop_max_range_returns').value)

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(LaserScan, input_topic, self.scan_callback, 10)
        self.get_logger().info(
            f'Sanitizing LaserScan {input_topic} -> {output_topic}; '
            f'self_clearance={self.self_clearance_range:.2f} m, '
            f'drop_max_range_returns={self.drop_max_range_returns}'
        )

    def scan_callback(self, msg: LaserScan):
        clean = LaserScan()
        clean.header = msg.header
        clean.angle_min = msg.angle_min
        clean.angle_max = msg.angle_max
        clean.angle_increment = msg.angle_increment
        clean.time_increment = msg.time_increment
        clean.scan_time = msg.scan_time
        clean.range_min = msg.range_min
        clean.range_max = msg.range_max
        clean.intensities = list(msg.intensities)

        max_threshold = msg.range_max - self.max_range_epsilon
        ranges = []
        for r in msg.ranges:
            if not math.isfinite(r):
                ranges.append(r)
                continue
            if r <= self.self_clearance_range:
                ranges.append(math.inf)
                continue
            if self.drop_max_range_returns and r >= max_threshold:
                ranges.append(math.inf)
                continue
            ranges.append(r)

        clean.ranges = ranges
        self.publisher.publish(clean)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
