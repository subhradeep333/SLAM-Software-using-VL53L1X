#!/usr/bin/env python3
"""
ROS 2 Driver Node for VL53L0X + SG90 Servo LiDAR on ESP32.

Receives serial stream from ESP32, reconstructs full 180° sweeps,
and publishes sensor_msgs/msg/LaserScan messages to /scan with TF transforms.
"""

import math
import time
import threading
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros


class VL53L0XSerialDriver(Node):
    def __init__(self):
        super().__init__('vl53l0x_serial_driver')

        # Declare ROS 2 parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('laser_x', 0.1)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.1)
        self.declare_parameter('angle_min_deg', -90.0)
        self.declare_parameter('angle_max_deg', 90.0)
        self.declare_parameter('range_min', 0.03)
        self.declare_parameter('range_max', 2.0)

        # Get parameter values
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.laser_x = self.get_parameter('laser_x').get_parameter_value().double_value
        self.laser_y = self.get_parameter('laser_y').get_parameter_value().double_value
        self.laser_z = self.get_parameter('laser_z').get_parameter_value().double_value
        self.angle_min_deg = self.get_parameter('angle_min_deg').get_parameter_value().double_value
        self.angle_max_deg = self.get_parameter('angle_max_deg').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        # LaserScan publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Static TF Broadcaster (base_link -> laser_frame)
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            self.publish_static_transform()

        # Serial reader thread state
        self.running = True
        self.serial_conn = None
        self.thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(
            f"VL53L0X Serial Driver initialized. Target port: {self.port} @ {self.baudrate}"
        )

    def publish_static_transform(self):
        """Broadcasts static transform from base_link to laser_frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = self.laser_x
        t.transform.translation.y = self.laser_y
        t.transform.translation.z = self.laser_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def open_serial(self):
        """Attempts to open the serial port with auto-discovery fallback."""
        try:
            self.get_logger().info(f"Connecting to serial port {self.port}...")
            return serial.Serial(self.port, self.baudrate, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().warn(f"Failed to open {self.port}: {e}")
            # Try finding any USB serial port
            ports = list(serial.tools.list_ports.comports())
            for p in ports:
                if 'usb' in p.device.lower() or 'acm' in p.device.lower():
                    try:
                        self.get_logger().info(f"Trying detected port: {p.device}")
                        return serial.Serial(p.device, self.baudrate, timeout=1.0)
                    except Exception:
                        continue
        return None

    def serial_loop(self):
        """Background thread continuously reading scans from ESP32."""
        current_readings = []
        in_scan = False
        sweep_direction = 1
        expected_points = 91
        scan_start_time = time.time()

        while self.running and rclpy.ok():
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.serial_conn = self.open_serial()
                if self.serial_conn is None:
                    time.sleep(2.0)
                    continue
                else:
                    self.get_logger().info(f"Connected to {self.serial_conn.name}")

            try:
                line_bytes = self.serial_conn.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode('utf-8', errors='ignore').strip()

                if not line:
                    continue

                if line.startswith('$SCAN'):
                    # Frame Start: $SCAN,<direction>,<count>
                    parts = line.split(',')
                    if len(parts) >= 3:
                        sweep_direction = int(parts[1])
                        expected_points = int(parts[2])
                    in_scan = True
                    current_readings = []
                    scan_start_time = time.time()

                elif line.startswith('$END') and in_scan:
                    # Frame End: publish LaserScan
                    in_scan = False
                    scan_duration = time.time() - scan_start_time
                    if len(current_readings) >= 10:
                        self.publish_scan(current_readings, sweep_direction, scan_duration)
                    current_readings = []

                elif in_scan and ',' in line:
                    # Data point: <angle_deg>,<distance_mm>
                    try:
                        angle_str, dist_str = line.split(',')[:2]
                        angle_deg = float(angle_str)
                        dist_mm = float(dist_str)
                        current_readings.append((angle_deg, dist_mm))
                    except ValueError:
                        pass

                elif line.startswith('#'):
                    # Comment / log from ESP32
                    self.get_logger().info(f"[ESP32] {line[1:].strip()}")

            except serial.SerialException as e:
                self.get_logger().error(f"Serial connection lost: {e}")
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                self.serial_conn = None
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().warn(f"Error parsing serial stream: {e}")

    def publish_scan(self, readings, direction, scan_duration):
        """Constructs and publishes a sensor_msgs/LaserScan."""
        # When sweeping backwards (180° -> 0°), reverse readings to keep ascending angles
        if direction == -1:
            readings = list(reversed(readings))

        if not readings:
            return

        num_points = len(readings)
        ranges = []

        for _, dist_mm in readings:
            dist_m = dist_mm / 1000.0
            if self.range_min <= dist_m <= self.range_max:
                ranges.append(dist_m)
            else:
                # In ROS LaserScan, out-of-range is represented by infinity
                ranges.append(float('inf'))

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id

        scan_msg.angle_min = math.radians(self.angle_min_deg)
        scan_msg.angle_max = math.radians(self.angle_max_deg)
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / max(1, (num_points - 1))
        scan_msg.time_increment = scan_duration / max(1, num_points)
        scan_msg.scan_time = scan_duration
        scan_msg.range_min = float(self.range_min)
        scan_msg.range_max = float(self.range_max)
        scan_msg.ranges = ranges

        self.scan_pub.publish(scan_msg)

    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VL53L0XSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

