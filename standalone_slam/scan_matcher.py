"""
Correlative Scan-to-Map Matcher for 2D LiDAR Pose Tracking.

Aligns incoming 180° scan points with the built occupancy grid map
to estimate robot displacement (dx, dy, dtheta) without wheel odometry.
"""

import math
import numpy as np


class ScanMatcher:
    def __init__(self, search_window_xy=0.15, search_window_theta=math.radians(15.0),
                 step_xy=0.02, step_theta=math.radians(2.0)):
        """
        Parameters:
            search_window_xy: Search radius in meters for translational shift
            search_window_theta: Search radius in radians for angular shift
            step_xy: Grid search step size for position
            step_theta: Grid search step size for angle
        """
        self.search_window_xy = search_window_xy
        self.search_window_theta = search_window_theta
        self.step_xy = step_xy
        self.step_theta = step_theta

    def match(self, initial_pose, scan_polar_points, grid_map):
        """
        Refines initial_pose (x, y, theta) by maximizing alignment with grid_map.

        Parameters:
            initial_pose: tuple (rx, ry, r_theta)
            scan_polar_points: list of (rel_angle_rad, distance_m) for valid obstacle hits
            grid_map: OccupancyGridMap instance

        Returns:
            best_pose: tuple (best_x, best_y, best_theta)
            best_score: float alignment score
        """
        if not scan_polar_points:
            return initial_pose, 0.0

        init_x, init_y, init_th = initial_pose

        # Pre-filter hit points
        valid_hits = [(ang, dist) for ang, dist in scan_polar_points if 0.05 <= dist <= 2.0]
        if len(valid_hits) < 8:
            return initial_pose, 0.0

        angles = np.array([p[0] for p in valid_hits])
        dists = np.array([p[1] for p in valid_hits])

        dx_range = np.arange(-self.search_window_xy, self.search_window_xy + 1e-5, self.step_xy)
        dy_range = np.arange(-self.search_window_xy, self.search_window_xy + 1e-5, self.step_xy)
        dth_range = np.arange(-self.search_window_theta, self.search_window_theta + 1e-5, self.step_theta)

        best_score = -float('inf')
        best_pose = initial_pose

        log_odds_grid = grid_map.log_odds
        res = grid_map.resolution
        orig_x = grid_map.origin_x
        orig_y = grid_map.origin_y
        cols = grid_map.cols
        rows = grid_map.rows

        # Grid search over candidate poses
        for dth in dth_range:
            cand_th = init_th + dth
            total_angles = cand_th + angles
            cos_a = np.cos(total_angles)
            sin_a = np.sin(total_angles)
            px_rel = dists * cos_a
            py_rel = dists * sin_a

            for dx in dx_range:
                cand_x = init_x + dx
                cand_cols = ((cand_x + px_rel - orig_x) / res).astype(np.int32)
                
                # Check column bounds early
                col_valid = (cand_cols >= 0) & (cand_cols < cols)
                if not np.any(col_valid):
                    continue

                for dy in dy_range:
                    cand_y = init_y + dy
                    cand_rows = ((cand_y + py_rel - orig_y) / res).astype(np.int32)

                    valid_mask = col_valid & (cand_rows >= 0) & (cand_rows < rows)
                    if not np.any(valid_mask):
                        continue

                    # Sum log-odds of occupied map cells matching current scan
                    score = np.sum(log_odds_grid[cand_rows[valid_mask], cand_cols[valid_mask]])

                    # Penalty for moving far from initial guess
                    penalty = 0.5 * (dx * dx + dy * dy) + 0.2 * (dth * dth)
                    score -= penalty

                    if score > best_score:
                        best_score = score
                        best_pose = (cand_x, cand_y, cand_th)

        return best_pose, best_score

