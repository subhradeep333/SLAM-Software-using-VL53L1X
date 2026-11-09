"""
Main SLAM Coordinator and Application Entry Point.

Supports running with real ESP32 hardware via serial port
or inside a simulated 2D virtual environment with full GUI visualization.
"""

import os
import sys
import math
import time
import argparse
import tkinter as tk

# Support direct script execution: python3 standalone_slam/main.py
if __package__ is None or __package__ == "":
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    from standalone_slam.occupancy_grid import OccupancyGridMap
    from standalone_slam.scan_matcher import ScanMatcher
    from standalone_slam.serial_reader import SerialLidarReader
    from standalone_slam.simulator import LidarSimulator
    from standalone_slam.gui import LidarSlamGUI
else:
    from .occupancy_grid import OccupancyGridMap
    from .scan_matcher import ScanMatcher
    from .serial_reader import SerialLidarReader
    from .simulator import LidarSimulator
    from .gui import LidarSlamGUI


class SlamEngine:
    def __init__(self, sim_mode=True, serial_port=None, baudrate=115200):
        self.sim_mode = sim_mode
        self.serial_port = serial_port
        self.baudrate = baudrate

        # SLAM Data structures
        self.grid_map = OccupancyGridMap(width_m=6.0, height_m=6.0, resolution=0.03)
        self.scan_matcher = ScanMatcher()

        # Robot state: [x (m), y (m), theta (rad)]
        self.robot_pose = [0.0, 0.0, 0.0]

        # Drivers
        self.simulator = None
        self.serial_reader = None

        if self.sim_mode:
            self.simulator = LidarSimulator(room_size=4.0)
        else:
            self.serial_reader = SerialLidarReader(port=self.serial_port, baudrate=self.baudrate)
            self.serial_reader.start()

        # Telemetry & stats
        self.scan_count = 0
        self.latest_cartesian_hits = []
        self.last_frame_time = time.time()
        self.fps = 0.0
        self.frame_counter = 0

    @property
    def is_connected(self):
        if self.sim_mode:
            return True
        return self.serial_reader.is_connected if self.serial_reader else False

    @property
    def status_text(self):
        if self.sim_mode:
            return "Simulation Mode active. Drive robot with WASD."
        return self.serial_reader.status_message if self.serial_reader else "Idle"

    def switch_mode(self, is_sim=True, port=None):
        """Switches between simulation and physical serial connection."""
        self.sim_mode = is_sim
        if is_sim:
            if self.serial_reader:
                self.serial_reader.stop()
                self.serial_reader = None
            if self.simulator is None:
                self.simulator = LidarSimulator(room_size=4.0)
        else:
            self.simulator = None
            if self.serial_reader:
                self.serial_reader.stop()
            self.serial_port = port
            self.serial_reader = SerialLidarReader(port=port, baudrate=self.baudrate)
            self.serial_reader.start()

    def step(self):
        """
        Executes one SLAM cycle:
        Pulls completed scan, matches pose against map, and updates occupancy grid.
        """
        now = time.time()
        self.frame_counter += 1
        if now - self.last_frame_time >= 1.0:
            self.fps = self.frame_counter / (now - self.last_frame_time)
            self.frame_counter = 0
            self.last_frame_time = now

        scan_packet = None
        if self.sim_mode and self.simulator:
            self.simulator.step(dt=0.033)
            scan_packet = self.simulator.get_latest_scan()
        elif self.serial_reader:
            scan_packet = self.serial_reader.get_latest_scan()

        if scan_packet is None:
            return False

        timestamp, direction, points = scan_packet
        if not points:
            return False

        # In simulation mode, ground-truth motion provides initial guess
        rx, ry, r_th = self.robot_pose
        if self.sim_mode and self.simulator:
            initial_guess = (self.simulator.robot_x, self.simulator.robot_y, self.simulator.robot_theta)
        else:
            initial_guess = (rx, ry, r_th)

        # Convert polar points to world hit coordinates
        valid_polar_hits = []
        map_update_points = []
        cartesian_hits_for_gui = []

        for (angle_deg, dist_mm) in points:
            # 0° on servo corresponds to -90° (right), 90° is straight ahead (0°), 180° is +90° (left)
            rel_rad = math.radians(angle_deg - 90.0)

            if 30.0 <= dist_mm <= 2000.0:
                dist_m = dist_mm / 1000.0
                valid_polar_hits.append((rel_rad, dist_m))

        # Scan-to-Map Matching (refine robot pose estimate)
        if self.scan_count > 0 and len(valid_polar_hits) >= 10:
            best_pose, score = self.scan_matcher.match(initial_guess, valid_polar_hits, self.grid_map)
            self.robot_pose = list(best_pose)
        else:
            self.robot_pose = list(initial_guess)

        # Recalculate hit points with refined pose
        curr_x, curr_y, curr_th = self.robot_pose
        for (angle_deg, dist_mm) in points:
            rel_rad = math.radians(angle_deg - 90.0)
            world_rad = curr_th + rel_rad

            if 30.0 <= dist_mm <= 2000.0:
                dist_m = dist_mm / 1000.0
                hx = curr_x + dist_m * math.cos(world_rad)
                hy = curr_y + dist_m * math.sin(world_rad)
                map_update_points.append((hx, hy, True))
                cartesian_hits_for_gui.append((hx, hy))
            else:
                # Out-of-range ray: free space up to 2.0 meters
                max_d = 2.0
                hx = curr_x + max_d * math.cos(world_rad)
                hy = curr_y + max_d * math.sin(world_rad)
                map_update_points.append((hx, hy, False))

        # Update 2D Occupancy Grid Map
        self.grid_map.update_scan(self.robot_pose, map_update_points, max_range=2.0)

        self.latest_cartesian_hits = cartesian_hits_for_gui
        self.scan_count += 1
        return True


def main():
    parser = argparse.ArgumentParser(description="VL53L0X 2D LiDAR SLAM Studio")
    parser.add_argument(
        "--sim", action="store_true", default=False,
        help="Run in virtual simulation room mode (default if no hardware port specified)"
    )
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port of ESP32 (e.g. /dev/ttyUSB0 or /dev/cu.usbserial-110)"
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Serial baud rate (default: 115200)"
    )
    args = parser.parse_args()

    # Default to simulation mode if no explicit port is supplied
    is_sim = args.sim or (args.port is None)

    slam_engine = SlamEngine(sim_mode=is_sim, serial_port=args.port, baudrate=args.baud)

    root = tk.Tk()
    gui = LidarSlamGUI(root, slam_engine)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if slam_engine.serial_reader:
            slam_engine.serial_reader.stop()


if __name__ == "__main__":
    main()
