#!/usr/bin/env python3
"""
Magnetometer calibration (hard-iron and soft-iron) for the
SparkFun 9DoF Razor IMU M0.

What it does:
  1. Captures raw mx, my, mz samples for N seconds while moving the IMU
     in figure-8 patterns in the air, covering as many orientations
     as possible (up/down, sideways, rotating on all axes).
  2. Computes the hard-iron offset: offset = (max + min) / 2 per axis.
  3. Computes a simple soft-iron scale factor (normalizes the "radius"
     of each axis so the point cloud approximates a sphere rather than an ellipsoid).
  4. Prints values ready to use as ROS 2 parameters:
       mag_offset_x, mag_offset_y, mag_offset_z

Usage:
    python3 calibrate_mag.py --port /dev/ttyACM0 --seconds 40

During capture: move the IMU SLOWLY in figure-8 motions, aiming it
in as many 3D directions as possible (not just rotating in a single 2D plane).
"""

import argparse
import time
import serial


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 10:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=40.0)
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    print(f"\nCapturing for {args.seconds:.0f} seconds.")
    print("Move the IMU SLOWLY drawing figure-8 motions in the air,")
    print("covering as many distinct 3D orientations as possible.\n")
    print("Starting in 3...")
    time.sleep(1)
    print("2...")
    time.sleep(1)
    print("1...")
    time.sleep(1)
    print("GO! Move the IMU now.\n")

    mx_min = my_min = mz_min = float("inf")
    mx_max = my_max = mz_max = float("-inf")
    count = 0

    t_end = time.time() + args.seconds
    try:
        while time.time() < t_end:
            raw = ser.readline().decode(errors="ignore")
            parsed = parse_line(raw)
            if parsed is None:
                continue
            _, _, _, _, _, _, _, mx, my, mz = parsed

            mx_min, mx_max = min(mx_min, mx), max(mx_max, mx)
            my_min, my_max = min(my_min, my), max(my_max, my)
            mz_min, mz_max = min(mz_min, mz), max(mz_max, mz)
            count += 1

            remaining = t_end - time.time()
            print(f"Samples: {count}   Remaining time: {remaining:4.1f}s", end="\r")
    except KeyboardInterrupt:
        print("\nCapture interrupted by user.")
    finally:
        ser.close()

    print("\n\n--- Capture finished ---")
    print(f"Total samples: {count}")

    if count < 100:
        print("\n⚠ Too few samples. Please repeat calibration moving for longer / slower.")
        return

    print(f"\nRaw ranges:")
    print(f"  mx: [{mx_min:.3f}, {mx_max:.3f}]")
    print(f"  my: [{my_min:.3f}, {my_max:.3f}]")
    print(f"  mz: [{mz_min:.3f}, {mz_max:.3f}]")

    # Hard-iron offset: midpoint of each range
    offset_x = (mx_max + mx_min) / 2.0
    offset_y = (my_max + my_min) / 2.0
    offset_z = (mz_max + mz_min) / 2.0

    # Soft-iron: radius (semi-range) of each axis, normalized against
    # the average radius of all three axes.
    radius_x = (mx_max - mx_min) / 2.0
    radius_y = (my_max - my_min) / 2.0
    radius_z = (mz_max - mz_min) / 2.0
    avg_radius = (radius_x + radius_y + radius_z) / 3.0

    scale_x = avg_radius / radius_x if radius_x > 1e-6 else 1.0
    scale_y = avg_radius / radius_y if radius_y > 1e-6 else 1.0
    scale_z = avg_radius / radius_z if radius_z > 1e-6 else 1.0

    print("\n=== HARD-IRON OFFSETS (Use as ROS 2 parameters) ===")
    print(f"  mag_offset_x: {offset_x:.6f}")
    print(f"  mag_offset_y: {offset_y:.6f}")
    print(f"  mag_offset_z: {offset_z:.6f}")

    print("\n=== SOFT-IRON SCALE FACTORS (Optional) ===")
    print(f"  mag_scale_x: {scale_x:.6f}")
    print(f"  mag_scale_y: {scale_y:.6f}")
    print(f"  mag_scale_z: {scale_z:.6f}")

    print("\nTo pass these offsets to your ROS 2 driver node:")
    print("  ros2 run pladypos driver_imu.py --ros-args \\")
    print(f"    -p mag_offset_x:={offset_x:.6f} \\")
    print(f"    -p mag_offset_y:={offset_y:.6f} \\")
    print(f"    -p mag_offset_z:={offset_z:.6f}")
    print("\n(or update DEFAULT_MAG_OFFSET in driver_imu.py or your YAML parameters)")


if __name__ == "__main__":
    main()