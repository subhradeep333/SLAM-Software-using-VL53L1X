#!/usr/bin/env python3
"""
Hardware Verification & Serial Diagnostic Tool for VL53L0X ESP32 LiDAR.

Tests serial connection, prints live sensor readings, and measures sweep rate.
"""

import sys
import time
import argparse
import serial
import serial.tools.list_ports


def main():
    parser = argparse.ArgumentParser(description="Test ESP32 VL53L0X Serial Output")
    parser.add_argument("--port", type=str, default=None, help="Serial port to open")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    ports = list(serial.tools.list_ports.comports())
    print("=" * 60)
    print("Detected Serial Ports:")
    for p in ports:
        print(f"  - {p.device} ({p.description})")
    print("=" * 60)

    target_port = args.port
    if not target_port:
        if not ports:
            print("No serial ports detected! Please connect your ESP32 via USB.")
            sys.exit(1)
        # Choose first matching port
        target_port = ports[0].device

    print(f"Opening port {target_port} at {args.baud} baud...")
    try:
        ser = serial.Serial(target_port, args.baud, timeout=2.0)
    except Exception as e:
        print(f"Error opening port: {e}")
        sys.exit(1)

    print("Connected! Listening for scan packets (Ctrl+C to quit)...")
    print("-" * 60)

    scan_count = 0
    point_count = 0
    scan_start_time = time.time()

    try:
        while True:
            raw_line = ser.readline()
            if not raw_line:
                continue
            line = raw_line.decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if line.startswith('$SCAN'):
                scan_count += 1
                point_count = 0
                scan_start_time = time.time()
                parts = line.split(',')
                direction = "0° -> 180°" if len(parts) > 1 and parts[1] == '1' else "180° -> 0°"
                print(f"\n[Scan #{scan_count}] Starting sweep ({direction})")

            elif line.startswith('$END'):
                elapsed = time.time() - scan_start_time
                rate = point_count / max(0.001, elapsed)
                print(f"[Scan #{scan_count}] Completed in {elapsed:.2f}s | {point_count} points ({rate:.1f} pts/sec)")

            elif line.startswith('#'):
                print(f"  [ESP32 LOG] {line}")

            elif ',' in line:
                point_count += 1
                try:
                    angle, dist = line.split(',')[:2]
                    # Print every 10th reading to avoid terminal flooding
                    if point_count % 10 == 1:
                        print(f"  Angle: {float(angle):>5.1f}° | Distance: {float(dist):>5.0f} mm")
                except ValueError:
                    pass

    except KeyboardInterrupt:
        print("\nExiting diagnostic tool.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

