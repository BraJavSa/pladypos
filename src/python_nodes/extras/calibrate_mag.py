#!/usr/bin/env python3
"""
Calibracion de magnetometro (hard-iron y soft-iron) para la
SparkFun 9DoF Razor IMU M0.

Que hace:
  1. Captura mx,my,mz crudos durante N segundos mientras mueves la IMU
     dibujando figuras de "8" en el aire, cubriendo tantas orientaciones
     como puedas (arriba/abajo, de lado, girando en todos los ejes).
  2. Calcula el offset de hard-iron: offset = (max + min) / 2 por eje.
  3. Calcula un factor de escala de soft-iron simple (normaliza el
     "radio" de cada eje para que la nube de puntos se aproxime a una
     esfera en vez de un elipsoide).
  4. Imprime los valores listos para usar como parametros ROS2:
       mag_offset_x, mag_offset_y, mag_offset_z

Uso:
    python3 calibrate_mag.py --port /dev/ttyACM0 --seconds 40

Durante la captura: mueve la IMU LENTO y dibujando figuras de 8,
tratando de apuntarla en tantas direcciones distintas como puedas
(no solo girar en un plano).
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

    print(f"Abriendo {args.port} @ {args.baud}...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    print(f"\nCapturando durante {args.seconds:.0f} segundos.")
    print("Mueve la IMU LENTO dibujando figuras de 8 en el aire,")
    print("cubriendo tantas orientaciones distintas como puedas.\n")
    print("Iniciando en 3...")
    time.sleep(1)
    print("2...")
    time.sleep(1)
    print("1...")
    time.sleep(1)
    print("YA! Mueve la IMU ahora.\n")

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
            print(f"Muestras: {count}   Tiempo restante: {remaining:4.1f}s", end="\r")
    except KeyboardInterrupt:
        print("\nCaptura interrumpida por el usuario.")
    finally:
        ser.close()

    print("\n\n--- Captura terminada ---")
    print(f"Muestras totales: {count}")

    if count < 100:
        print("\n⚠ Muy pocas muestras. Repite la calibracion moviendo mas tiempo/lento.")
        return

    print(f"\nRangos crudos:")
    print(f"  mx: [{mx_min:.3f}, {mx_max:.3f}]")
    print(f"  my: [{my_min:.3f}, {my_max:.3f}]")
    print(f"  mz: [{mz_min:.3f}, {mz_max:.3f}]")

    # Hard-iron offset: centro de cada rango
    offset_x = (mx_max + mx_min) / 2.0
    offset_y = (my_max + my_min) / 2.0
    offset_z = (mz_max + mz_min) / 2.0

    # Soft-iron: radio (semi-rango) de cada eje, normalizado contra
    # el promedio de los tres radios. Esto da un factor de escala
    # opcional para que la nube de puntos se aproxime a una esfera.
    radius_x = (mx_max - mx_min) / 2.0
    radius_y = (my_max - my_min) / 2.0
    radius_z = (mz_max - mz_min) / 2.0
    avg_radius = (radius_x + radius_y + radius_z) / 3.0

    scale_x = avg_radius / radius_x if radius_x > 1e-6 else 1.0
    scale_y = avg_radius / radius_y if radius_y > 1e-6 else 1.0
    scale_z = avg_radius / radius_z if radius_z > 1e-6 else 1.0

    print("\n=== HARD-IRON OFFSETS (usar como parametros ROS2) ===")
    print(f"  mag_offset_x: {offset_x:.6f}")
    print(f"  mag_offset_y: {offset_y:.6f}")
    print(f"  mag_offset_z: {offset_z:.6f}")

    print("\n=== SOFT-IRON SCALE FACTORS (opcional, no soportado aun por el nodo) ===")
    print(f"  mag_scale_x: {scale_x:.6f}")
    print(f"  mag_scale_y: {scale_y:.6f}")
    print(f"  mag_scale_z: {scale_z:.6f}")

    print("\nPara usar los offsets con tu nodo ROS2:")
    print("  ros2 run <paquete> imu_ros2_node.py --ros-args \\")
    print(f"    -p mag_offset_x:={offset_x:.6f} \\")
    print(f"    -p mag_offset_y:={offset_y:.6f} \\")
    print(f"    -p mag_offset_z:={offset_z:.6f}")
    print("\n(o agrega esos valores a un archivo YAML de parametros)")


if __name__ == "__main__":
    main()