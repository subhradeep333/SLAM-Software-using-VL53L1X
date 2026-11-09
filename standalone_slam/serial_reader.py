"""
Serial Port Interface for ESP32 VL53L0X LiDAR Scanner.

Features auto-port discovery, reconnection logic, and streaming frame assembly.
"""

import time
import threading
import queue
import serial
import serial.tools.list_ports


class SerialLidarReader:
    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_conn = None
        self.thread = None
        self.scan_queue = queue.Queue(maxsize=10)
        self.status_message = "Not connected"
        self.is_connected = False

    @staticmethod
    def list_available_ports():
        """Returns a list of device strings for detected serial ports."""
        ports = serial.tools.list_ports.comports()
        detected = []
        for p in ports:
            # Common ESP32 USB-UART chip descriptions / names
            desc = p.description.lower()
            dev = p.device
            if any(k in dev.lower() or k in desc for k in ['usb', 'acm', 'uart', 'wch', 'cp210', 'ftdi']):
                detected.append(dev)
            else:
                detected.append(dev)
        return detected

    def start(self):
        """Starts the background reading thread."""
        self.running = True
        self.thread = threading.Thread(target=self._read_worker, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops reading and closes the serial port."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        self.is_connected = False
        self.status_message = "Disconnected"

    def get_latest_scan(self, block=False, timeout=0.1):
        """Retrieves the latest completed scan frame from the queue."""
        try:
            return self.scan_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    def _open_connection(self):
        candidate_ports = [self.port] if self.port else self.list_available_ports()
        for p in candidate_ports:
            if not p:
                continue
            try:
                self.status_message = f"Connecting to {p}..."
                conn = serial.Serial(p, self.baudrate, timeout=1.0)
                self.port = p
                self.is_connected = True
                self.status_message = f"Connected to {p} @ {self.baudrate}"
                return conn
            except Exception as e:
                self.status_message = f"Failed to connect to {p}: {e}"
        return None

    def _read_worker(self):
        current_readings = []
        in_scan = False
        sweep_direction = 1

        while self.running:
            if self.serial_conn is None or not self.serial_conn.is_open:
                self.serial_conn = self._open_connection()
                if self.serial_conn is None:
                    time.sleep(2.0)
                    continue

            try:
                line_bytes = self.serial_conn.readline()
                if not line_bytes:
                    continue
                line = line_bytes.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('$SCAN'):
                    parts = line.split(',')
                    if len(parts) >= 2:
                        sweep_direction = int(parts[1])
                    in_scan = True
                    current_readings = []

                elif line.startswith('$END') and in_scan:
                    in_scan = False
                    if len(current_readings) >= 10:
                        # Sweep direction 1 is 0°->180°, -1 is 180°->0°
                        # Reverse list on direction -1 to always maintain 0° to 180° order
                        ordered_points = list(reversed(current_readings)) if sweep_direction == -1 else list(current_readings)
                        
                        # Drop oldest if queue full to stay real-time
                        if self.scan_queue.full():
                            try:
                                self.scan_queue.get_nowait()
                            except queue.Empty:
                                pass
                        self.scan_queue.put((time.time(), sweep_direction, ordered_points))
                    current_readings = []

                elif in_scan and ',' in line:
                    try:
                        ang_str, dist_str = line.split(',')[:2]
                        angle_deg = float(ang_str)
                        dist_mm = float(dist_str)
                        current_readings.append((angle_deg, dist_mm))
                    except ValueError:
                        pass

                elif line.startswith('#'):
                    # Firmware status/log message
                    self.status_message = line[1:].strip()

            except serial.SerialException as e:
                self.status_message = f"Serial error: {e}"
                self.is_connected = False
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except Exception:
                        pass
                self.serial_conn = None
                time.sleep(1.0)
            except Exception as e:
                self.status_message = f"Parser error: {e}"
