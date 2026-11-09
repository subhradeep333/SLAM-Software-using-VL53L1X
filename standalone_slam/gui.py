"""
Interactive Desktop GUI for VL53L0X LiDAR SLAM.

Provides real-time occupancy grid visualization, laser beam sweeping display,
robot pose trajectory, hardware serial connection management, and simulated teleoperation.
"""

import math
import os
import time
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np


class LidarSlamGUI:
    def __init__(self, root, slam_engine):
        self.root = root
        self.slam = slam_engine
        self.root.title("VL53L0X 2D LiDAR SLAM Studio")
        self.root.geometry("1080x740")
        self.root.minsize(960, 640)

        # UI Color Theme
        self.bg_color = "#1e1e24"
        self.panel_bg = "#282830"
        self.text_color = "#e6edf3"
        self.accent_color = "#38bdf8"
        self.root.configure(bg=self.bg_color)

        # Canvas parameters
        self.canvas_size = 640
        self.scale = 80.0 # Pixels per meter
        self.pan_x = self.canvas_size / 2.0
        self.pan_y = self.canvas_size / 2.0

        # Trajectory history: list of (x, y)
        self.trajectory = []
        self.latest_scan_hits = []
        self.active_beam = None

        self._build_ui()
        self._bind_keys()

        # Start periodic GUI refresh (30 FPS)
        self.root.after(33, self._gui_loop)

    def _build_ui(self):
        # Top Header Bar
        header = tk.Frame(self.root, bg="#121216", height=45)
        header.pack(fill=tk.X, side=tk.TOP)
        title_lbl = tk.Label(
            header,
            text="⚡ VL53L0X LiDAR SLAM Engine",
            font=("Helvetica", 14, "bold"),
            fg=self.accent_color,
            bg="#121216"
        )
        title_lbl.pack(side=tk.LEFT, padx=15, pady=8)

        self.status_badge = tk.Label(
            header,
            text="● INITIALIZING",
            font=("Helvetica", 10, "bold"),
            fg="#eab308",
            bg="#121216"
        )
        self.status_badge.pack(side=tk.RIGHT, padx=15)

        # Main container
        main_frame = tk.Frame(self.root, bg=self.bg_color)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left: Map Canvas
        canvas_card = tk.Frame(main_frame, bg="#15151a", bd=1, relief=tk.SOLID)
        canvas_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        self.canvas = tk.Canvas(
            canvas_card,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="#0d1117",
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Right: Control & Telemetry Sidebar
        sidebar = tk.Frame(main_frame, bg=self.panel_bg, width=320, padx=15, pady=15)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)
        sidebar.pack_propagate(False)

        # Hardware / Mode Section
        mode_box = tk.LabelFrame(
            sidebar, text=" Operation Mode ", font=("Helvetica", 10, "bold"),
            fg=self.text_color, bg=self.panel_bg, padx=10, pady=8
        )
        mode_box.pack(fill=tk.X, pady=(0, 10))

        self.mode_var = tk.StringVar(value="SIM" if self.slam.sim_mode else "HW")
        rb_sim = tk.Radiobutton(
            mode_box, text="Virtual Simulation", variable=self.mode_var,
            value="SIM", command=self._on_mode_change,
            fg=self.text_color, bg=self.panel_bg, selectcolor="#374151"
        )
        rb_sim.pack(anchor=tk.W)

        rb_hw = tk.Radiobutton(
            mode_box, text="Physical ESP32 Serial", variable=self.mode_var,
            value="HW", command=self._on_mode_change,
            fg=self.text_color, bg=self.panel_bg, selectcolor="#374151"
        )
        rb_hw.pack(anchor=tk.W)

        # Serial Port Box
        self.port_box = tk.LabelFrame(
            sidebar, text=" Serial Connection ", font=("Helvetica", 10, "bold"),
            fg=self.text_color, bg=self.panel_bg, padx=10, pady=8
        )
        self.port_box.pack(fill=tk.X, pady=(0, 10))

        port_row = tk.Frame(self.port_box, bg=self.panel_bg)
        port_row.pack(fill=tk.X, pady=4)

        self.port_combo = ttk.Combobox(port_row, width=16)
        self.port_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._refresh_serial_ports()

        btn_refresh = tk.Button(
            port_row, text="🔄", command=self._refresh_serial_ports,
            bg="#374151", fg=self.text_color, width=3
        )
        btn_refresh.pack(side=tk.RIGHT, padx=(4, 0))

        self.btn_connect = tk.Button(
            self.port_box, text="Connect Hardware",
            command=self._toggle_connection,
            bg="#0284c7", fg="#ffffff", font=("Helvetica", 9, "bold"), pady=4
        )
        self.btn_connect.pack(fill=tk.X, pady=(6, 0))

        # Live Robot Telemetry
        tele_box = tk.LabelFrame(
            sidebar, text=" Live Pose & Scans ", font=("Helvetica", 10, "bold"),
            fg=self.text_color, bg=self.panel_bg, padx=10, pady=8
        )
        tele_box.pack(fill=tk.X, pady=(0, 10))

        self.lbl_pose = tk.Label(
            tele_box, text="Pose: X: 0.00m  Y: 0.00m  θ: 0.0°",
            font=("Courier", 10), fg="#a5f3fc", bg=self.panel_bg, anchor=tk.W
        )
        self.lbl_pose.pack(fill=tk.X, pady=2)

        self.lbl_scans = tk.Label(
            tele_box, text="Scans: 0  |  FPS: 0.0",
            font=("Courier", 10), fg="#a5f3fc", bg=self.panel_bg, anchor=tk.W
        )
        self.lbl_scans.pack(fill=tk.X, pady=2)

        self.lbl_status = tk.Label(
            tele_box, text="Ready",
            font=("Helvetica", 9), fg="#94a3b8", bg=self.panel_bg, wraplength=270, justify=tk.LEFT
        )
        self.lbl_status.pack(fill=tk.X, pady=2)

        # Simulation Teleop Controls
        self.teleop_box = tk.LabelFrame(
            sidebar, text=" Virtual Robot Teleop ", font=("Helvetica", 10, "bold"),
            fg=self.text_color, bg=self.panel_bg, padx=10, pady=8
        )
        self.teleop_box.pack(fill=tk.X, pady=(0, 10))

        teleop_grid = tk.Frame(self.teleop_box, bg=self.panel_bg)
        teleop_grid.pack(pady=4)

        btn_fwd = tk.Button(teleop_grid, text="▲ (W)", width=6, command=lambda: self._cmd_vel(0.2, 0.0))
        btn_fwd.grid(row=0, column=1, pady=2)

        btn_left = tk.Button(teleop_grid, text="◄ (A)", width=6, command=lambda: self._cmd_vel(0.0, 0.5))
        btn_left.grid(row=1, column=0, padx=2)

        btn_stop = tk.Button(teleop_grid, text="■ Stop", width=6, bg="#ef4444", fg="white", command=lambda: self._cmd_vel(0.0, 0.0))
        btn_stop.grid(row=1, column=1, padx=2)

        btn_right = tk.Button(teleop_grid, text="► (D)", width=6, command=lambda: self._cmd_vel(0.0, -0.5))
        btn_right.grid(row=1, column=2, padx=2)

        btn_back = tk.Button(teleop_grid, text="▼ (S)", width=6, command=lambda: self._cmd_vel(-0.2, 0.0))
        btn_back.grid(row=2, column=1, pady=2)

        teleop_tip = tk.Label(
            self.teleop_box, text="Keyboard: W/A/S/D or Arrow keys",
            font=("Helvetica", 8), fg="#94a3b8", bg=self.panel_bg
        )
        teleop_tip.pack(pady=(2, 0))

        # Action Buttons
        btn_save = tk.Button(
            sidebar, text="💾 Save Map (ROS PGM / YAML)",
            command=self._save_map,
            bg="#10b981", fg="#ffffff", font=("Helvetica", 10, "bold"), pady=6
        )
        btn_save.pack(fill=tk.X, pady=(6, 4))

        btn_reset = tk.Button(
            sidebar, text="🗑️ Reset Map",
            command=self._reset_map,
            bg="#475569", fg="#ffffff", font=("Helvetica", 9), pady=4
        )
        btn_reset.pack(fill=tk.X, pady=(0, 6))

    def _bind_keys(self):
        # Keyboard teleop binding
        self.root.bind("<w>", lambda e: self._cmd_vel(0.2, 0.0))
        self.root.bind("<s>", lambda e: self._cmd_vel(-0.2, 0.0))
        self.root.bind("<a>", lambda e: self._cmd_vel(0.0, 0.5))
        self.root.bind("<d>", lambda e: self._cmd_vel(0.0, -0.5))
        self.root.bind("<space>", lambda e: self._cmd_vel(0.0, 0.0))
        self.root.bind("<Up>", lambda e: self._cmd_vel(0.2, 0.0))
        self.root.bind("<Down>", lambda e: self._cmd_vel(-0.2, 0.0))
        self.root.bind("<Left>", lambda e: self._cmd_vel(0.0, 0.5))
        self.root.bind("<Right>", lambda e: self._cmd_vel(0.0, -0.5))

    def _cmd_vel(self, v, w):
        if self.slam.simulator:
            self.slam.simulator.set_velocity(v, w)

    def _refresh_serial_ports(self):
        try:
            from .serial_reader import SerialLidarReader
        except ImportError:
            from standalone_slam.serial_reader import SerialLidarReader
        ports = SerialLidarReader.list_available_ports()
        self.port_combo['values'] = ports
        if ports and not self.port_combo.get():
            self.port_combo.set(ports[0])

    def _on_mode_change(self):
        is_sim = (self.mode_var.get() == "SIM")
        self.slam.switch_mode(is_sim=is_sim, port=self.port_combo.get())
        if is_sim:
            self.teleop_box.pack(fill=tk.X, pady=(0, 10))
            self.status_badge.config(text="● SIMULATION RUNNING", fg="#38bdf8")
        else:
            self.status_badge.config(text="● HARDWARE CONNECTING", fg="#eab308")

    def _toggle_connection(self):
        port = self.port_combo.get()
        self.mode_var.set("HW")
        self.slam.switch_mode(is_sim=False, port=port)

    def _reset_map(self):
        self.slam.grid_map.reset()
        self.trajectory = []
        self.latest_scan_hits = []
        self.lbl_status.config(text="Map reset to initial state.")

    def _save_map(self):
        out_dir = os.path.expanduser("~/SLAM/maps")
        os.makedirs(out_dir, exist_ok=True)
        prefix = os.path.join(out_dir, f"map_{int(time.time())}")
        pgm, yaml = self.slam.grid_map.save_ros_map(prefix)
        msg = f"Map exported successfully!\n\nPGM: {os.path.basename(pgm)}\nYAML: {os.path.basename(yaml)}\n\nFolder: {out_dir}"
        self.lbl_status.config(text=f"Saved map to {os.path.basename(pgm)}")
        messagebox.showinfo("Map Saved", msg)

    def world_to_screen(self, wx, wy):
        """Converts world coordinates (meters) to canvas pixel coordinates."""
        # Canvas center is (pan_x, pan_y)
        # World X -> Screen Right (+), World Y -> Screen Up (-)
        sx = self.pan_x + wx * self.scale
        sy = self.pan_y - wy * self.scale
        return sx, sy

    def _render_occupancy_grid(self):
        """Draws probability grid cells on Canvas."""
        self.canvas.delete("all")
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw > 50:
            self.pan_x = cw / 2.0
            self.pan_y = ch / 2.0

        # Draw grid background lines
        for m in range(-4, 5):
            sx, _ = self.world_to_screen(m, 0)
            _, sy = self.world_to_screen(0, m)
            self.canvas.create_line(sx, 0, sx, ch, fill="#1c1f26", dash=(2, 4))
            self.canvas.create_line(0, sy, cw, sy, fill="#1c1f26", dash=(2, 4))

        # Draw occupancy cells (subsampled for smooth 30fps canvas rendering)
        probs = self.slam.grid_map.get_probability_grid()
        rows, cols = probs.shape
        res = self.slam.grid_map.resolution
        orig_x = self.slam.grid_map.origin_x
        orig_y = self.slam.grid_map.origin_y

        cell_px = max(2, int(res * self.scale))

        # Identify free and occupied cells
        occ_indices = np.argwhere(probs > 0.65)
        free_indices = np.argwhere(probs < 0.35)

        # Draw free space (light gray)
        # Subsample free cells if too dense
        stride = 2 if len(free_indices) > 5000 else 1
        for fr, fc in free_indices[::stride]:
            wx = orig_x + (fc + 0.5) * res
            wy = orig_y + (fr + 0.5) * res
            sx, sy = self.world_to_screen(wx, wy)
            self.canvas.create_rectangle(
                sx - cell_px, sy - cell_px, sx + cell_px, sy + cell_px,
                fill="#2e3440", outline=""
            )

        # Draw occupied obstacles (bright white / cyan)
        for fr, fc in occ_indices:
            wx = orig_x + (fc + 0.5) * res
            wy = orig_y + (fr + 0.5) * res
            sx, sy = self.world_to_screen(wx, wy)
            self.canvas.create_rectangle(
                sx - cell_px, sy - cell_px, sx + cell_px, sy + cell_px,
                fill="#f8fafc", outline=""
            )

        # Draw virtual walls if in simulation mode
        if self.slam.sim_mode and self.slam.simulator:
            for (p1, p2) in self.slam.simulator.walls:
                s1x, s1y = self.world_to_screen(p1[0], p1[1])
                s2x, s2y = self.world_to_screen(p2[0], p2[1])
                self.canvas.create_line(s1x, s1y, s2x, s2y, fill="#475569", width=2)

        # Draw trajectory
        if len(self.trajectory) > 1:
            for i in range(len(self.trajectory) - 1):
                p1 = self.world_to_screen(self.trajectory[i][0], self.trajectory[i][1])
                p2 = self.world_to_screen(self.trajectory[i+1][0], self.trajectory[i+1][1])
                self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill="#38bdf8", width=2)

        # Draw current LiDAR scan hit points (red dots)
        rx, ry, r_th = self.slam.robot_pose
        rsx, rsy = self.world_to_screen(rx, ry)

        for (hx, hy) in self.latest_scan_hits:
            hsx, hsy = self.world_to_screen(hx, hy)
            self.canvas.create_oval(hsx - 3, hsy - 3, hsx + 3, hsy + 3, fill="#ef4444", outline="")
            # Faint beam ray
            self.canvas.create_line(rsx, rsy, hsx, hsy, fill="#ef4444", dash=(1, 5))

        # Draw Robot Base
        radius_px = int(0.12 * self.scale) # 12cm robot radius
        self.canvas.create_oval(
            rsx - radius_px, rsy - radius_px, rsx + radius_px, rsy + radius_px,
            fill="#0284c7", outline="#bae6fd", width=2
        )

        # Draw Heading Arrow
        arrow_len = radius_px * 1.6
        head_x = rsx + arrow_len * math.cos(r_th)
        head_y = rsy - arrow_len * math.sin(r_th)
        self.canvas.create_line(rsx, rsy, head_x, head_y, fill="#facc15", width=3, arrow=tk.LAST)

        # Draw 180° Sweep Sector Envelope
        # Left edge (+90° = r_th + pi/2), Right edge (-90° = r_th - pi/2)
        sweep_radius_px = 2.0 * self.scale
        ang_left = r_th + (math.pi / 2.0)
        ang_right = r_th - (math.pi / 2.0)
        lx = rsx + sweep_radius_px * math.cos(ang_left)
        ly = rsy - sweep_radius_px * math.sin(ang_left)
        rx_edge = rsx + sweep_radius_px * math.cos(ang_right)
        ry_edge = rsy - sweep_radius_px * math.sin(ang_right)
        self.canvas.create_line(rsx, rsy, lx, ly, fill="#0284c7", dash=(2, 6))
        self.canvas.create_line(rsx, rsy, rx_edge, ry_edge, fill="#0284c7", dash=(2, 6))

    def _gui_loop(self):
        # Update SLAM engine step
        has_new_scan = self.slam.step()

        if has_new_scan:
            # Append trajectory
            rx, ry, r_th = self.slam.robot_pose
            if not self.trajectory or math.hypot(rx - self.trajectory[-1][0], ry - self.trajectory[-1][1]) > 0.03:
                self.trajectory.append((rx, ry))

            # Store hit points for rendering
            self.latest_scan_hits = self.slam.latest_cartesian_hits

        # Update telemetry labels
        rx, ry, r_th = self.slam.robot_pose
        self.lbl_pose.config(
            text=f"Pose: X:{rx:+.2f}m  Y:{ry:+.2f}m  θ:{math.degrees(r_th):+.1f}°"
        )
        self.lbl_scans.config(
            text=f"Scans: {self.slam.scan_count}  |  FPS: {self.slam.fps:.1f}"
        )

        status_text = self.slam.status_text
        if self.slam.sim_mode:
            self.status_badge.config(text="● SIMULATION RUNNING", fg="#38bdf8")
        elif self.slam.is_connected:
            self.status_badge.config(text="● HARDWARE CONNECTED", fg="#10b981")
        else:
            self.status_badge.config(text="● SEARCHING ESP32...", fg="#eab308")

        self.lbl_status.config(text=status_text)

        # Redraw Canvas
        self._render_occupancy_grid()

        # Schedule next tick
        self.root.after(33, self._gui_loop)
