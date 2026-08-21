#!/usr/bin/env python3
"""
Test stream + detección de AprilTag tag16h5 ID 8.
Muestra la imagen con FPS y un punto en el centro del tag detectado.

Uso:
    python3 test_stream.py                    # cámara por defecto (0354)
    python3 test_stream.py --camera 9835      # cámara específica
"""

import argparse
import json
import os
import time
import urllib.request
import cv2
import numpy as np
from pupil_apriltags import Detector

import rclpy
from geometry_msgs.msg import PoseStamped


class _SuppressStderr:
    """Silencia mensajes de la librería C (pose estimation warnings)."""
    def __enter__(self):
        self._fd = os.dup(2)
        self._devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self._devnull, 2)
    def __exit__(self, *_):
        os.dup2(self._fd, 2)
        os.close(self._fd)
        os.close(self._devnull)

CAMERAS = {
    "9835": "camera_9835",
    "0352": "camera_0352",
    "0353": "camera_0353",
    "0354": "camera_0354",
}

TARGET_TAG_ID = 8
TAG_SIZE = 0.25  # metros (25 cm)


def build_url(host: str, port: int, camera_id: str) -> str:
    cam = CAMERAS[camera_id]
    return f"http://{host}:{port}/stream?topic=/{cam}/{cam}/image_raw"


def fetch_calibration(host: str, port: int, camera_id: str):
    """Obtiene fx, fy, cx, cy y resolución de calibración desde el API."""
    url = f"http://{host}:{port}/calibration?camera={camera_id}"
    try:
        resp = urllib.request.urlopen(url, timeout=5)
        cal = json.loads(resp.read())
        K = cal["camera_matrix"]["data"]  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        w = float(cal.get("width", 2048))
        h = float(cal.get("height", 2048))
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        return fx, fy, cx, cy, w, h
    except Exception as e:
        print(f"[WARN] No se pudo obtener calibración: {e}")
        print("[WARN] Usando valores por defecto (cámara 0354)")
        return 1109.2076, 1114.4800, 1019.4268, 1033.4869, 2048.0, 2048.0


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream + AprilTag tag16h5 ID 8 Pose Publisher")
    parser.add_argument("--camera", "-c", default="0354", choices=list(CAMERAS.keys()))
    parser.add_argument("--host", default="10.250.253.1")
    parser.add_argument("--port", type=int, default=8083)
    args = parser.parse_args()

    # Inicializar ROS 2 Node y Publisher
    rclpy.init(args=None)
    ros_node = rclpy.create_node("stream_viewer")
    pose_pub = ros_node.create_publisher(PoseStamped, "/usv5/pose", 10)

    url = build_url(args.host, args.port, args.camera)

    # Obtener calibración desde el servidor
    calib_fx, calib_fy, calib_cx, calib_cy, calib_w, calib_h = \
        fetch_calibration(args.host, args.port, args.camera)

    # Inicializar detector
    detector = Detector(families="tag16h5", nthreads=4, quad_decimate=1.0)

    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[ERROR] No se pudo abrir el stream.")
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # Detectar AprilTags en escala de grises
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape[:2]
            sx, sy = w / calib_w, h / calib_h
            cam_params = [calib_fx * sx, calib_fy * sy, calib_cx * sx, calib_cy * sy]
            with _SuppressStderr():
                detections = detector.detect(gray, estimate_tag_pose=True,
                                             camera_params=cam_params, tag_size=TAG_SIZE)

            # Filtrar ID 8: hamming ≤ 1 y margin ≥ 5
            for det in detections:
                if det.tag_id != TARGET_TAG_ID:
                    continue
                if det.hamming > 1:
                    continue
                if det.decision_margin < 5.0:
                    continue

                if det.pose_t is not None:
                    t = det.pose_t.flatten()
                    pose_x, pose_y = t[0], t[1]

                    # Publicar posición en /usv5/pose (sin cálculo extra de orientación)
                    msg = PoseStamped()
                    msg.header.stamp = ros_node.get_clock().now().to_msg()
                    msg.header.frame_id = "odom"
                    msg.pose.position.x = float(pose_x)
                    msg.pose.position.y = float(pose_y)
                    msg.pose.position.z = 0.0
                    msg.pose.orientation.w = 1.0  # Identidad fija (cero costo CPU)
                    msg.pose.orientation.x = 0.0
                    msg.pose.orientation.y = 0.0
                    msg.pose.orientation.z = 0.0
                    pose_pub.publish(msg)

            rclpy.spin_once(ros_node, timeout_sec=0.001)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
