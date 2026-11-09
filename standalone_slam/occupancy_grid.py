"""
2D Occupancy Grid Map Implementation with Log-Odds Representation.

Uses Bresenham ray-casting to update free space along the beam and
occupied state at the obstacle reflection point.
"""

import math
import numpy as np


class OccupancyGridMap:
    def __init__(self, width_m=6.0, height_m=6.0, resolution=0.03):
        """
        Parameters:
            width_m: Map width in meters (e.g. 6.0m)
            height_m: Map height in meters (e.g. 6.0m)
            resolution: Grid cell size in meters (e.g. 0.03 = 3cm per cell)
        """
        self.resolution = float(resolution)
        self.width_m = float(width_m)
        self.height_m = float(height_m)

        self.cols = int(round(self.width_m / self.resolution))
        self.rows = int(round(self.height_m / self.resolution))

        # Origin in world coordinates (center of map)
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # Log-odds parameters
        self.l_occ = 0.85     # Increment for obstacle hit
        self.l_free = -0.40   # Decrement for traversed free space
        self.l_min = -5.0     # Lower clamp
        self.l_max = 5.0      # Upper clamp

        # Log-odds grid initialized to 0.0 (probability 0.5 = unknown)
        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)

    def world_to_grid(self, wx, wy):
        """Convert world coordinates (meters) to grid indices (col, row)."""
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        return col, row

    def grid_to_world(self, col, row):
        """Convert grid indices (col, row) to world coordinates (meters)."""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (row + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, col, row):
        """Check if grid cell is inside the map bounds."""
        return 0 <= col < self.cols and 0 <= row < self.rows

    def bresenham_line(self, x0, y0, x1, y1):
        """Standard 2D Bresenham line algorithm yielding integer grid coordinates."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def update_scan(self, robot_pose, hit_points, max_range=2.0):
        """
        Updates the occupancy grid given robot pose (rx, ry, r_theta)
        and an array of endpoint Cartesian coordinates [(hx, hy), ...].
        """
        rx, ry, _ = robot_pose
        r_col, r_row = self.world_to_grid(rx, ry)

        if not self.in_bounds(r_col, r_row):
            return

        for (hx, hy, is_hit) in hit_points:
            h_col, h_row = self.world_to_grid(hx, hy)

            # Trace ray cells from robot to endpoint
            line_cells = list(self.bresenham_line(r_col, r_row, h_col, h_row))
            if not line_cells:
                continue

            # Free cells along the ray (excluding the endpoint)
            free_cells = line_cells[:-1]
            for fc, fr in free_cells:
                if self.in_bounds(fc, fr):
                    self.log_odds[fr, fc] = max(self.l_min, self.log_odds[fr, fc] + self.l_free)

            # Endpoint cell (occupied if a valid obstacle reflection was detected)
            if is_hit and self.in_bounds(h_col, h_row):
                self.log_odds[h_row, h_col] = min(self.l_max, self.log_odds[h_row, h_col] + self.l_occ)

    def get_probability_grid(self):
        """Returns cell probabilities in range [0.0, 1.0]. 0.5 is unknown."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def reset(self):
        """Resets the entire map to unknown (0.0 log-odds)."""
        self.log_odds.fill(0.0)

    def save_ros_map(self, file_prefix="vl53l0x_map"):
        """
        Exports the occupancy map to standard ROS format:
          1. file_prefix.pgm (portable graymap image)
          2. file_prefix.yaml (ROS map metadata)
        """
        probs = self.get_probability_grid()

        # ROS map standard encoding:
        # Unknown (p == 0.5) -> 205 (gray)
        # Free (p < 0.25)     -> 254 (white)
        # Occupied (p > 0.65) -> 0   (black)
        pgm_pixels = np.full(probs.shape, 205, dtype=np.uint8)
        pgm_pixels[probs < 0.35] = 254
        pgm_pixels[probs > 0.65] = 0

        # ROS maps are typically saved with Y pointing up (flipped vertically relative to array index 0 at top)
        pgm_flipped = np.flipud(pgm_pixels)

        # Write PGM (P5 binary)
        pgm_filename = f"{file_prefix}.pgm"
        with open(pgm_filename, "wb") as f:
            header = f"P5\n{self.cols} {self.rows}\n255\n"
            f.write(header.encode("ascii"))
            f.write(pgm_flipped.tobytes())

        # Write YAML metadata
        yaml_filename = f"{file_prefix}.yaml"
        with open(yaml_filename, "w") as f:
            f.write(f"image: {pgm_filename}\n")
            f.write(f"resolution: {self.resolution:.4f}\n")
            f.write(f"origin: [{self.origin_x:.3f}, {self.origin_y:.3f}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.25\n")

        return pgm_filename, yaml_filename
