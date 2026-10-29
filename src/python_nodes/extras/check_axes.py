#!/usr/bin/env python3
"""
Verifies the physical orientation of the IMU axes.

Prints live raw ax, ay, az readings (in g) so you can:
  1. Place the IMU flat on a table and observe which axis reads ~ +/- 1g (that is Z).
  2. Turn it upside down and confirm that same axis flips its sign.
  3. Stand it on its edges to identify the X and Y axes.

Usage:
    python3 check_axes.py --port /dev/ttyACM0
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
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    print("Reading... Press Ctrl+C to exit.")
    print("Move the IMU: flat / upside down / on each edge, and note which axis reads ~1g.\n")

    try:
        while True:
            raw = ser.readline().decode(errors="ignore")
            parsed = parse_line(raw)
            if parsed is None:
                continue
            _, ax, ay, az, *_ = parsed
            mag = (ax**2 + ay**2 + az**2) ** 0.5
            print(f"ax={ax:+.3f}  ay={ay:+.3f}  az={az:+.3f}   |mag|={mag:.3f}", end="\r")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()