"""
Virtual Environment and LiDAR Simulator for VL53L0X.

Generates realistic 180° sweeping ToF distance measurements with noise
inside a configurable virtual room with walls and obstacle pillars.
"""

import math
import time
import queue
import numpy as np


class LidarSimulator:
    def __init__(self, room_size=4.0):
        self.room_size = room_size
        half = room_size / 2.0

        # Virtual room boundaries and obstacles: line segments ((x1, y1), (x2, y2))
        self.walls = [
            # Outer room boundaries
            ((-half, -half), (half, -half)),
            ((half, -half), (half, half)),
            ((half, half), (-half, half)),
            ((-half, half), (-half, -half)),

            # Interior Obstacle 1: Rectangular box in upper right
            ((0.6, 0.4), (1.2, 0.4)),
            ((1.2, 0.4), (1.2, 1.0)),
            ((1.2, 1.0), (0.6, 1.0)),
            ((0.6, 1.0), (0.6, 0.4)),

            # Interior Obstacle 2: Square column in lower left
            ((-1.1, -0.9), (-0.5, -0.9)),
            ((-0.5, -0.9), (-0.5, -0.3)),
            ((-0.5, -0.3), (-1.1, -0.3)),
            ((-1.1, -0.3), (-1.1, -0.9)),

            # Interior Obstacle 3: Small barrier in upper left
            ((-1.2, 0.8), (-0.4, 1.2)),
        ]

        # Robot state: x, y, theta
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Motion velocities
        self.v = 0.0      # Linear velocity (m/s)
        self.w = 0.0      # Angular velocity (rad/s)

        # Servo & sensor properties
        self.angle_min = 0.0     # 0 degrees
        self.angle_max = 180.0   # 180 degrees
        self.angle_step = 2.0    # 2-degree step
        self.sensor_min_m = 0.03
        self.sensor_max_m = 2.00 # VL53L0X 2-meter limit
        self.noise_std = 0.015   # 1.5 cm measurement noise

        # Sweep timing
        self.sweep_direction = 1 # 1 = forward (0->180), -1 = backward (180->0)
        self.sweep_duration = 1.8 # Seconds per 180° sweep
        self.last_sweep_time = time.time()

        self.scan_queue = queue.Queue(maxsize=10)
        self.running = True

    def set_velocity(self, v, w):
        """Sets linear and angular command velocities."""
        self.v = float(v)
        self.w = float(w)

    def update_motion(self, dt):
        """Updates robot position based on commanded velocities with boundary clamping."""
        self.robot_theta += self.w * dt
        # Normalize theta to [-pi, pi]
        self.robot_theta = (self.robot_theta + math.pi) % (2.0 * math.pi) - math.pi

        dx = self.v * math.cos(self.robot_theta) * dt
        dy = self.v * math.sin(self.robot_theta) * dt

        # Collision clamping against outer walls
        half = (self.room_size / 2.0) - 0.25
        self.robot_x = max(-half, min(half, self.robot_x + dx))
        self.robot_y = max(-half, min(half, self.robot_y + dy))

    def ray_segment_intersect(self, p, ray_dir, a, b):
        """Computes ray-segment intersection distance."""
        v1 = p - a
        v2 = b - a
        v3 = np.array([-ray_dir[1], ray_dir[0]])

        dot = np.dot(v2, v3)
        if abs(dot) < 1e-6:
            return float('inf')

        t = (v2[0] * v1[1] - v2[1] * v1[0]) / dot
        u = np.dot(v1, v3) / dot

        if t > self.sensor_min_m and 0.0 <= u <= 1.0:
            return t
        return float('inf')

    def generate_sweep(self):
        """Generates a complete 180-degree scan frame."""
        now = time.time()
        angles_deg = np.arange(self.angle_min, self.angle_max + 1e-4, self.angle_step)
        if self.sweep_direction == -1:
            angles_deg = angles_deg[::-1]

        points = []
        origin = np.array([self.robot_x, self.robot_y])

        for ang_deg in angles_deg:
            # 0° on servo corresponds to -90° (right), 90° is straight ahead, 180° is +90° (left)
            rel_rad = math.radians(ang_deg - 90.0)
            world_rad = self.robot_theta + rel_rad
            ray_dir = np.array([math.cos(world_rad), math.sin(world_rad)])

            min_dist = float('inf')
            for (p1, p2) in self.walls:
                d = self.ray_segment_intersect(origin, ray_dir, np.array(p1), np.array(p2))
                if d < min_dist:
                    min_dist = d

            # Apply sensor range and noise
            if min_dist <= self.sensor_max_m:
                noisy_dist_m = min_dist + np.random.normal(0, self.noise_std)
                dist_mm = max(self.sensor_min_m * 1000.0, min(self.sensor_max_m * 1000.0, noisy_dist_m * 1000.0))
            else:
                dist_mm = 0.0 # Out of range / no reflection

            points.append((ang_deg, dist_mm))

        # Reorder points so that index 0 is always 0 degrees
        ordered_points = list(reversed(points)) if self.sweep_direction == -1 else list(points)
        self.sweep_direction *= -1

        if self.scan_queue.full():
            try:
                self.scan_queue.get_nowait()
            except queue.Empty:
                pass
        self.scan_queue.put((now, self.sweep_direction, ordered_points))

    def step(self, dt=0.05):
        """Simulation step: updates motion and checks sweep generation."""
        self.update_motion(dt)
        now = time.time()
        if now - self.last_sweep_time >= self.sweep_duration:
            self.generate_sweep()
            self.last_sweep_time = now

    def get_latest_scan(self):
        """Pulls the latest scan frame from the queue."""
        try:
            return self.scan_queue.get_nowait()
        except queue.Empty:
            return None
