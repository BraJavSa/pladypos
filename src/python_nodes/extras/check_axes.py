#!/usr/bin/env python3
"""
Verifica la orientacion fisica de los ejes de la IMU.

Imprime en vivo ax, ay, az crudos (en g) para que puedas:
  1. Poner la IMU plana sobre la mesa y ver que eje marca ~+/-1g (ese es Z).
  2. Voltearla boca abajo y confirmar que ese mismo eje invierte signo.
  3. Pararla de canto en cada lado para identificar X e Y.

Uso:
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

    print(f"Abriendo {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    print("Leyendo... Ctrl+C para salir.")
    print("Mueve la IMU: plana / boca abajo / de canto en cada lado, y observa cual eje marca ~1g.\n")

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
        print("\nSaliendo.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()