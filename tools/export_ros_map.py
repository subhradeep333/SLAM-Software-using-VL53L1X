#!/usr/bin/env python3
"""
CLI Utility to generate and export sample maps for ROS / Nav2 navigation testing.
"""

import os
import sys

# Ensure repository root is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import argparse
from standalone_slam.occupancy_grid import OccupancyGridMap
from standalone_slam.simulator import LidarSimulator


def main():
    parser = argparse.ArgumentParser(description="Export a simulated LiDAR SLAM map to ROS PGM/YAML format")
    parser.add_argument("--output", type=str, default="sample_room", help="Output filename prefix")
    parser.add_argument("--sweeps", type=int, default=10, help="Number of simulation sweeps to accumulate")
    parser.add_argument("--res", type=float, default=0.03, help="Grid resolution in meters (default: 0.03)")
    args = parser.parse_args()

    print(f"Creating 2D Occupancy Grid Map (resolution: {args.res}m)...")
    grid = OccupancyGridMap(width_m=5.0, height_m=5.0, resolution=args.res)
    sim = LidarSimulator(room_size=4.0)

    print(f"Accumulating {args.sweeps} simulated scans...")
    for i in range(args.sweeps):
        sim.generate_sweep()
        scan = sim.get_latest_scan()
        if scan:
            _, _, points = scan
            hit_points = []
            for ang_deg, dist_mm in points:
                rel_rad = (ang_deg - 90.0) * (3.14159265 / 180.0)
                if 30 <= dist_mm <= 2000:
                    d = dist_mm / 1000.0
                    hx = d * (rel_rad * 0.0 + 1.0) # approx
                    hx = d * (ang_deg * 0.0 + (3.14159265 / 180.0))
                    # Proper trig
                    import math
                    hx = d * math.cos(rel_rad)
                    hy = d * math.sin(rel_rad)
                    hit_points.append((hx, hy, True))
                else:
                    import math
                    hx = 2.0 * math.cos(rel_rad)
                    hy = 2.0 * math.sin(rel_rad)
                    hit_points.append((hx, hy, False))
            grid.update_scan((0.0, 0.0, 0.0), hit_points)

    pgm, yaml = grid.save_ros_map(args.output)
    print(f"\nMap saved successfully!")
    print(f"  - Image: {os.path.abspath(pgm)}")
    print(f"  - Meta:  {os.path.abspath(yaml)}")


if __name__ == "__main__":
    main()
