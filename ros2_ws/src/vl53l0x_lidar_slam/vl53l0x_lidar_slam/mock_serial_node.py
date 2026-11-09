#!/usr/bin/env python3
"""
Mock LaserScan Publisher for VL53L0X LiDAR.

Simulates a 180-degree sweeping ToF sensor in a simulated rectangular room,
publishing directly to /scan for testing ROS 2 SLAM Toolbox without physical hardware.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros


class MockVL53L0XLidar(Node):
    def __init__(self):
        super().__init__('mock_vl53l0x_lidar')

        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 0.5) # 1 scan every 2 seconds (matches physical sweep)
        self.declare_parameter('num_points', 91)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.num_points = self.get_parameter('num_points').get_parameter_value().integer_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # Simulated rectangular room dimensions (meters) centered at origin
        # Room bounds: x in [-1.5, 1.5], y in [-1.5, 1.5]
        self.walls = [
            # ((x1, y1), (x2, y2))
            ((-1.5, -1.5), (1.5, -1.5)),
            ((1.5, -1.5), (1.5, 1.5)),
            ((1.5, 1.5), (-1.5, 1.5)),
            ((-1.5, 1.5), (-1.5, -1.5)),
            # An obstacle box in front of the sensor
            ((0.3, 0.5), (0.7, 0.5)),
            ((0.7, 0.5), (0.7, 0.8)),
            ((0.7, 0.8), (0.3, 0.8)),
            ((0.3, 0.8), (0.3, 0.5)),
        ]

        self.timer = self.create_timer(1.0 / rate_hz, self.publish_mock_scan)
        self.get_logger().info("Mock VL53L0X LiDAR node started. Publishing simulated 180° scans to /scan.")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def ray_segment_intersect(self, p, ray_dir, a, b):
        """Ray-segment intersection test."""
        # Ray: p + t * ray_dir (t > 0)
        # Segment: a + u * (b - a) (0 <= u <= 1)
        v1 = p - a
        v2 = b - a
        v3 = np.array([-ray_dir[1], ray_dir[0]])

        dot = np.dot(v2, v3)
        if abs(dot) < 1e-6:
            return float('inf')

        t = (v2[0] * v1[1] - v2[1] * v1[0]) / dot
        u = np.dot(v1, v3) / dot

        if t > 0.03 and 0.0 <= u <= 1.0:
            return t
        return float('inf')

    def publish_mock_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id

        angle_min = -math.pi / 2.0
        angle_max = math.pi / 2.0
        angles = np.linspace(angle_min, angle_max, self.num_points)

        ranges = []
        origin = np.array([0.0, 0.0])

        for ang in angles:
            ray_dir = np.array([math.cos(ang), math.sin(ang)])
            min_dist = float('inf')
            for (p1, p2) in self.walls:
                d = self.ray_segment_intersect(origin, ray_dir, np.array(p1), np.array(p2))
                if d < min_dist:
                    min_dist = d
            
            # Max range of VL53L0X is 2.0m, add minor realistic sensor noise
            if min_dist <= 2.0:
                noisy_dist = float(min_dist + np.random.normal(0, 0.015))
                ranges.append(max(0.03, min(2.0, noisy_dist)))
            else:
                ranges.append(float('inf'))

        scan_msg.angle_min = angle_min
        scan_msg.angle_max = angle_max
        scan_msg.angle_increment = (angle_max - angle_min) / (self.num_points - 1)
        scan_msg.time_increment = 2.0 / self.num_points
        scan_msg.scan_time = 2.0
        scan_msg.range_min = 0.03
        scan_msg.range_max = 2.0
        scan_msg.ranges = ranges

        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockVL53L0XLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
