#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from ping360_sonar_msgs.msg import SonarEcho


class Ping360DistanceBridge(Node):
    def __init__(self) -> None:
        super().__init__("ping360_distance_bridge")

        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("scan_echo_topic", "scan_echo")
        self.declare_parameter("scan_distance_topic", "/ping360/scan/distance")
        self.declare_parameter("scan_echo_distance_topic", "/ping360/scan_echo/distance")
        self.declare_parameter("echo_threshold", 200)
        self.declare_parameter("scan_echo_min_distance_m", 0.20)
        self.declare_parameter("min_valid_distance_m", 0.0)
        self.declare_parameter("max_valid_distance_m", -1.0)

        scan_topic = str(self.get_parameter("scan_topic").value)
        scan_echo_topic = str(self.get_parameter("scan_echo_topic").value)
        scan_distance_topic = str(self.get_parameter("scan_distance_topic").value)
        scan_echo_distance_topic = str(self.get_parameter("scan_echo_distance_topic").value)

        self.echo_threshold = int(self.get_parameter("echo_threshold").value)
        self.scan_echo_min_distance_m = float(
            self.get_parameter("scan_echo_min_distance_m").value
        )
        self.min_valid_distance_m = float(self.get_parameter("min_valid_distance_m").value)
        self.max_valid_distance_m = float(self.get_parameter("max_valid_distance_m").value)

        self.scan_distance_pub = self.create_publisher(Float32, scan_distance_topic, 10)
        self.scan_echo_distance_pub = self.create_publisher(Float32, scan_echo_distance_topic, 10)

        self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SonarEcho,
            scan_echo_topic,
            self._scan_echo_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Listening on '%s' and '%s', publishing '%s' and '%s'"
            % (scan_topic, scan_echo_topic, scan_distance_topic, scan_echo_distance_topic)
        )

    def _distance_is_valid(self, distance_m: float) -> bool:
        if not math.isfinite(distance_m):
            return False
        if distance_m <= 0.0:
            return False
        if distance_m < self.min_valid_distance_m:
            return False
        if self.max_valid_distance_m > 0.0 and distance_m > self.max_valid_distance_m:
            return False
        return True

    def _publish_distance(self, publisher, distance_m: float) -> None:
        msg = Float32()
        msg.data = float(distance_m)
        publisher.publish(msg)

    def _scan_callback(self, msg: LaserScan) -> None:
        valid_ranges = [
            distance_m
            for distance_m in msg.ranges
            if self._distance_is_valid(distance_m)
        ]

        if not valid_ranges:
            return

        self._publish_distance(self.scan_distance_pub, min(valid_ranges))

    def _scan_echo_callback(self, msg: SonarEcho) -> None:
        sample_count = int(msg.number_of_samples)
        range_max_m = float(msg.range)

        if sample_count <= 0 or range_max_m <= 0.0:
            return

        first_hit_distance_m: Optional[float] = None
        for sample_index, intensity in enumerate(msg.intensities):
            if int(intensity) < self.echo_threshold:
                continue

            distance_m = ((sample_index + 1) * range_max_m) / sample_count
            if distance_m < self.scan_echo_min_distance_m:
                continue
            if self._distance_is_valid(distance_m):
                first_hit_distance_m = distance_m
                break

        if first_hit_distance_m is None:
            return

        self._publish_distance(self.scan_echo_distance_pub, first_hit_distance_m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ping360DistanceBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
