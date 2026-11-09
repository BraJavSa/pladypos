#!/usr/bin/env python3
"""
visual_tag_estimator.py
Node 2 of localization pipeline:
Processes overhead camera frames (directly via ROS 2 sensor_msgs/Image topic on the
server at full 25.5 Hz, or via HTTP MJPEG fallback), detects AprilTag 285, and
publishes the visual 3D position (x, y, z) and visual orientation (qw, qx, qy, qz)
on /usv5/visual_odom.
"""

import json
import math
import os
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

DEFAULT_STREAM_URL         = "http://10.250.253.1:8083/stream?topic=/camera_0354/camera_0354/image_raw"
DEFAULT_CALIB_URL          = "http://10.250.253.1:8083/calibration?camera=0354"
DEFAULT_CALIB_FILE         = "/config/calibration_536890354.yaml"
DEFAULT_IMAGE_TOPIC        = "/camera_0354/camera_0354/image_raw"
DEFAULT_TAG_ID             = 285
DEFAULT_TAG_SIZE           = 0.25
DEFAULT_ODOM_TOPIC         = "/usv5/visual_odom"
DEFAULT_TAG_DETECTED_TOPIC = "/usv5/tag_detected"
DEFAULT_CAMERA_FRAME       = "camera"
DEFAULT_USV_FRAME          = "usv5"
DEFAULT_RATE_HZ            = 30.0

DEFAULT_FX = 1109.207609712391
DEFAULT_FY = 1114.480046638150
DEFAULT_CX = 1019.426843076671
DEFAULT_CY = 1033.486904196278
DEFAULT_DIST = [-0.1268312444881338, 0.04624447483590838, -0.0006651708835886797, -0.0003698069893855444, 0.0]


def fetch_calibration(calib_url: str, calib_file: str = DEFAULT_CALIB_FILE):
    """Loads intrinsic camera matrix and distortion coefficients from YAML file or HTTP endpoint."""
    fx, fy, cx, cy = DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY
    dist = np.array(DEFAULT_DIST, dtype=np.float64).reshape(-1, 1)

    if calib_file and os.path.exists(calib_file):
        try:
            with open(calib_file, "r") as f:
                data = yaml.safe_load(f)
            m = data["camera_matrix"]["data"]
            fx, cx = float(m[0]), float(m[2])
            fy, cy = float(m[4]), float(m[5])
            if "distortion_coefficients" in data and "data" in data["distortion_coefficients"]:
                d = data["distortion_coefficients"]["data"]
                dist = np.array(d, dtype=np.float64).reshape(-1, 1)
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
            return K, dist
        except Exception:
            pass

    try:
        with urllib.request.urlopen(calib_url, timeout=2) as r:
            data = json.load(r)
        m = data["camera_matrix"]["data"]
        fx, cx = float(m[0]), float(m[2])
        fy, cy = float(m[4]), float(m[5])
        if "distortion_coefficients" in data and "data" in data["distortion_coefficients"]:
            d = data["distortion_coefficients"]["data"]
            dist = np.array(d, dtype=np.float64).reshape(-1, 1)
    except Exception:
        pass

    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    return K, dist


class MJPEGReader:
    """Non-blocking background reader for HTTP MJPEG stream (fallback when not using ROS Image topic)."""
    def __init__(self, url: str):
        self._url = url
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._new_frame = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._new_frame.set()
        self._t.join(timeout=2)

    def wait_new_frame(self, timeout=1.0):
        if not self._new_frame.wait(timeout=timeout):
            return None
        self._new_frame.clear()
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def _run(self):
        while not self._stop.is_set():
            try:
                req = urllib.request.urlopen(self._url, timeout=10)
                buf = b""
                while not self._stop.is_set():
                    chunk = req.read(16384)
                    if not chunk:
                        break
                    buf += chunk
                    a = buf.find(b"\xff\xd8")
                    b_ = buf.find(b"\xff\xd9")
                    if a != -1 and b_ != -1 and b_ > a:
                        jpg = buf[a:b_+2]
                        buf = buf[b_+2:]
                        img = cv2.imdecode(
                            np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
                        if img is not None:
                            with self._lock:
                                self._frame = img
                            self._new_frame.set()
            except Exception:
                if not self._stop.is_set():
                    time.sleep(1)


def build_detector():
    """Initializes high-performance pupil_apriltags Detector with fallback to OpenCV ArUco."""
    try:
        from pupil_apriltags import Detector
        detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        return "pupil", detector
    except ImportError:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters_create()
        return "opencv", (aruco_dict, params)


def extract_gray_patch(raw_img: np.ndarray, encoding: str, roi_box=None):
    """
    Converts raw image (or only the ROI sub-region for speed) to 8-bit grayscale.
    Ensures even Bayer alignment when cropping Bayer images.
    """
    h_img, w_img = raw_img.shape[:2]
    if roi_box is not None:
        rx0, ry0, rx1, ry1 = roi_box
        rx0 = max(0, min(w_img - 2, int(rx0))) & ~1
        ry0 = max(0, min(h_img - 2, int(ry0))) & ~1
        rx1 = max(rx0 + 16, min(w_img, int(rx1) + (int(rx1) & 1)))
        ry1 = max(ry0 + 16, min(h_img, int(ry1) + (int(ry1) & 1)))
        sub = raw_img[ry0:ry1, rx0:rx1]
        offset_x, offset_y = rx0, ry0
    else:
        sub = raw_img
        offset_x, offset_y = 0, 0

    enc = encoding.lower()
    if "bayer_gbrg" in enc:
        gray_patch = cv2.cvtColor(sub, cv2.COLOR_BayerGB2GRAY)
    elif "bayer_grbg" in enc:
        gray_patch = cv2.cvtColor(sub, cv2.COLOR_BayerGR2GRAY)
    elif "bayer_rggb" in enc:
        gray_patch = cv2.cvtColor(sub, cv2.COLOR_BayerRG2GRAY)
    elif "bayer_bggr" in enc:
        gray_patch = cv2.cvtColor(sub, cv2.COLOR_BayerBG2GRAY)
    elif sub.ndim == 3:
        gray_patch = cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)
    else:
        gray_patch = sub

    return gray_patch, offset_x, offset_y


def detect_tag_in_patch(detector_info, img_patch: np.ndarray, target_id: int, offset_x: int = 0, offset_y: int = 0):
    backend, det = detector_info
    if backend == "pupil":
        tags = det.detect(img_patch)
        for t in tags:
            if int(t.tag_id) == target_id:
                raw_corners = t.corners[[1, 0, 3, 2]].copy()
                raw_corners[:, 0] += offset_x
                raw_corners[:, 1] += offset_y
                center = np.array([t.center[0] + offset_x, t.center[1] + offset_y], dtype=np.float64)
                return raw_corners, center
        return None, None
    else:
        aruco_dict, params = det
        corners, ids, _ = cv2.aruco.detectMarkers(img_patch, aruco_dict, parameters=params)
        if ids is not None:
            for i, id_val in enumerate(ids.flatten()):
                if int(id_val) == target_id:
                    c = corners[i][0].copy()
                    c[:, 0] += offset_x
                    c[:, 1] += offset_y
                    center = np.mean(c, axis=0)
                    return c, center
        return None, None


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Converts 3x3 rotation matrix to unit quaternion (qw, qx, qy, qz)."""
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return qw / norm, qx / norm, qy / norm, qz / norm


def estimate_pose(corners_px: np.ndarray, tag_size: float, K: np.ndarray, dist: np.ndarray):
    """
    Computes visual 3D position (x, y, z) and visual quaternion (qw, qx, qy, qz)
    from 2D image corners using IPPE.
    """
    half = tag_size / 2.0
    obj_pts = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0]
    ], dtype=np.float64)

    try:
        retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj_pts, corners_px.astype(np.float64),
            K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not retval or len(rvecs) == 0:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corners_px.astype(np.float64),
                K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not ok:
                return None
            rvecs, tvecs = [rvec], [tvec]

        best_idx = 0
        best_vz = 1.0
        for i, rv in enumerate(rvecs):
            R_cand, _ = cv2.Rodrigues(rv)
            vz = R_cand[2, 2]
            if vz < best_vz:
                best_vz = vz
                best_idx = i

        rvec = rvecs[best_idx]
        tvec = tvecs[best_idx]

        x = float(tvec.flat[0])
        y = float(tvec.flat[1])
        z = float(tvec.flat[2])

        R, _ = cv2.Rodrigues(rvec)
        qw, qx, qy, qz = rotation_matrix_to_quaternion(R)

        return x, y, z, qw, qx, qy, qz
    except Exception:
        return None


def make_pose_covariance(distance: float):
    var_pos = max(0.001, 0.01 * (distance ** 2))
    var_rot = max(0.005, 0.03 * (distance ** 2))
    cov = [0.0] * 36
    cov[0]  = var_pos
    cov[7]  = var_pos
    cov[14] = var_pos
    cov[21] = var_rot
    cov[28] = var_rot
    cov[35] = var_rot
    return cov


def make_twist_covariance():
    cov = [0.0] * 36
    cov[0]  = 0.05
    cov[7]  = 0.05
    cov[14] = 0.05
    cov[21] = 0.1
    cov[28] = 0.1
    cov[35] = 0.1
    return cov


class MovingAverageFilter2D:
    def __init__(self, window_size: int = 2):
        self.window_size = window_size
        self.buf_x = deque(maxlen=window_size)
        self.buf_y = deque(maxlen=window_size)

    def reset(self):
        self.buf_x.clear()
        self.buf_y.clear()

    def update(self, x: float, y: float):
        self.buf_x.append(x)
        self.buf_y.append(y)
        x_avg = sum(self.buf_x) / len(self.buf_x)
        y_avg = sum(self.buf_y) / len(self.buf_y)
        return x_avg, y_avg


class AlphaBeta1D:
    def __init__(self, alpha: float = 0.75, beta: float = 0.35):
        self.alpha = alpha
        self.beta = beta
        self.x = None
        self.v = 0.0

    def reset(self):
        self.x = None
        self.v = 0.0

    def update(self, z: float, dt: float):
        if self.x is None:
            self.x = z
            self.v = 0.0
            return self.x, self.v

        x_pred = self.x + self.v * dt
        v_pred = self.v

        res = z - x_pred
        self.x = x_pred + self.alpha * res
        self.v = v_pred + (self.beta / dt) * res

        return self.x, self.v


class VisualTagEstimatorNode(Node):
    def __init__(self):
        super().__init__("visual_tag_estimator")

        self.declare_parameter("use_ros_topic", True)
        self.declare_parameter("image_topic", DEFAULT_IMAGE_TOPIC)
        self.declare_parameter("calib_file", DEFAULT_CALIB_FILE)
        self.declare_parameter("stream_url", DEFAULT_STREAM_URL)
        self.declare_parameter("calib_url", DEFAULT_CALIB_URL)
        self.declare_parameter("tag_id", DEFAULT_TAG_ID)
        self.declare_parameter("tag_size", DEFAULT_TAG_SIZE)
        self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
        self.declare_parameter("tag_detected_topic", DEFAULT_TAG_DETECTED_TOPIC)
        self.declare_parameter("camera_frame_id", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("usv_frame_id", DEFAULT_USV_FRAME)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("publish_rate", DEFAULT_RATE_HZ)
        self.declare_parameter("use_filter", False)
        self.declare_parameter("window_size", 2)
        self.declare_parameter("ab_alpha", 0.75)
        self.declare_parameter("ab_beta", 0.35)

        self.use_ros_topic = bool(self.get_parameter("use_ros_topic").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.calib_file = self.get_parameter("calib_file").value
        self.stream_url = self.get_parameter("stream_url").value
        self.calib_url = self.get_parameter("calib_url").value
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.tag_detected_topic = self.get_parameter("tag_detected_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.usv_frame_id = self.get_parameter("usv_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.use_filter = bool(self.get_parameter("use_filter").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.ab_alpha = float(self.get_parameter("ab_alpha").value)
        self.ab_beta = float(self.get_parameter("ab_beta").value)

        self._pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self._pub_tag_detected = self.create_publisher(Bool, self.tag_detected_topic, 10)
        self._tf_br = TransformBroadcaster(self)

        self._K, self._dist = fetch_calibration(self.calib_url, self.calib_file)
        self._detector = build_detector()

        self._frame_lock = threading.Lock()
        self._latest_raw = None
        self._latest_enc = "mono8"
        self._new_ros_frame = threading.Event()

        self._reader = None
        if self.use_ros_topic:
            self.create_subscription(
                Image,
                self.image_topic,
                self._ros_image_cb,
                qos_profile_sensor_data
            )
        else:
            self._reader = MJPEGReader(self.stream_url)

        self._stop = threading.Event()
        self._detection_thread = threading.Thread(target=self._detection_loop, daemon=True)

        self.ma_filter = MovingAverageFilter2D(window_size=self.window_size)
        self.ab_x = AlphaBeta1D(alpha=self.ab_alpha, beta=self.ab_beta)
        self.ab_y = AlphaBeta1D(alpha=self.ab_alpha, beta=self.ab_beta)
        self._last_detection_time = None
        self._prev_x = 0.0
        self._prev_y = 0.0

        # ROI Tracking State (bounding box crop around last known tag center)
        self._last_tag_center = None
        self._roi_half_size = 260
        self._roi_consecutive_misses = 0
        self._max_roi_misses = 2

        self.get_logger().info(
            f"VisualTagEstimatorNode initialized:\n"
            f"  Mode: {'Direct ROS 2 Image Topic (' + self.image_topic + ')' if self.use_ros_topic else 'HTTP MJPEG (' + self.stream_url + ')'}\n"
            f"  Tag ID: {self.tag_id} ({self.tag_size} m)\n"
            f"  Visual Odometry Published: {self.odom_topic}\n"
            f"  Publish TF: {self.publish_tf}"
        )

    def _ros_image_cb(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            if arr.shape[2] == 1:
                arr = arr[:, :, 0]
            with self._frame_lock:
                self._latest_raw = arr
                self._latest_enc = msg.encoding
            self._new_ros_frame.set()
        except Exception:
            pass

    def start(self):
        if self._reader is not None:
            self._reader.start()
        self._detection_thread.start()

    def stop(self):
        self._stop.set()
        self._new_ros_frame.set()
        if self._reader is not None:
            self._reader.stop()
        self._detection_thread.join(timeout=3)

    def _detection_loop(self):
        while not self._stop.is_set():
            if self.use_ros_topic:
                if not self._new_ros_frame.wait(timeout=0.5):
                    continue
                self._new_ros_frame.clear()
                with self._frame_lock:
                    raw_img = self._latest_raw
                    enc = self._latest_enc
                if raw_img is None:
                    continue
            else:
                raw_img = self._reader.wait_new_frame(timeout=0.5)
                enc = "mono8"
                if raw_img is None:
                    continue

            corners = None
            center = None

            # 1. Fast ROI search (only debayers the 520x520 patch around the boat!)
            if self._last_tag_center is not None and self._roi_consecutive_misses < self._max_roi_misses:
                cx, cy = self._last_tag_center
                roi_box = (
                    cx - self._roi_half_size,
                    cy - self._roi_half_size,
                    cx + self._roi_half_size,
                    cy + self._roi_half_size
                )
                patch, ox, oy = extract_gray_patch(raw_img, enc, roi_box=roi_box)
                corners, center = detect_tag_in_patch(self._detector, patch, self.tag_id, ox, oy)
                if corners is None:
                    self._roi_consecutive_misses += 1

            # 2. Fall back to full frame if ROI not active or missed
            if corners is None:
                patch, ox, oy = extract_gray_patch(raw_img, enc, roi_box=None)
                corners, center = detect_tag_in_patch(self._detector, patch, self.tag_id, ox, oy)

            if corners is None:
                continue

            self._last_tag_center = center
            self._roi_consecutive_misses = 0

            res = estimate_pose(corners, self.tag_size, self._K, self._dist)
            if res is None:
                continue

            x, y, z, qw, qx, qy, qz = res
            z = 5.1
            now_sec = time.monotonic()

            if self.use_filter:
                if self._last_detection_time is None or (now_sec - self._last_detection_time) > 0.5:
                    self.ma_filter.reset()
                    self.ab_x.reset()
                    self.ab_y.reset()
                    dt = 0.04
                else:
                    dt = max(0.001, now_sec - self._last_detection_time)

                x_avg, y_avg = self.ma_filter.update(x, y)
                x_filt, vx = self.ab_x.update(x_avg, dt)
                y_filt, vy = self.ab_y.update(y_avg, dt)
            else:
                x_filt, y_filt = x, y
                vx, vy = 0.0, 0.0
                if self._last_detection_time is not None:
                    dt = now_sec - self._last_detection_time
                    if 0.002 < dt < 0.5:
                        vx = (x - self._prev_x) / dt
                        vy = (y - self._prev_y) / dt
                self._prev_x = x
                self._prev_y = y

            vz = 0.0
            self._last_detection_time = now_sec

            self._publish_data(x_filt, y_filt, z, qw, qx, qy, qz, vx, vy, vz)

    def _publish_tf(self, stamp, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float):
        if not self.publish_tf:
            return

        t_usv = TransformStamped()
        t_usv.header.stamp = stamp
        t_usv.header.frame_id = self.camera_frame_id
        t_usv.child_frame_id = self.usv_frame_id
        t_usv.transform.translation.x = x
        t_usv.transform.translation.y = y
        t_usv.transform.translation.z = z
        t_usv.transform.rotation.w = qw
        t_usv.transform.rotation.x = qx
        t_usv.transform.rotation.y = qy
        t_usv.transform.rotation.z = qz

        self._tf_br.sendTransform(t_usv)

    def _publish_data(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float, vx: float, vy: float, vz: float):
        stamp = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.camera_frame_id
        odom_msg.child_frame_id = self.usv_frame_id

        dist = math.sqrt(x * x + y * y + z * z)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        odom_msg.pose.covariance = make_pose_covariance(dist)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz
        odom_msg.twist.covariance = make_twist_covariance()

        self._pub_odom.publish(odom_msg)

        tag_detected_msg = Bool()
        tag_detected_msg.data = True
        self._pub_tag_detected.publish(tag_detected_msg)

        if self.publish_tf:
            self._publish_tf(stamp, x, y, z, qw, qx, qy, qz)


def main(args=None):
    rclpy.init(args=args)
    node = VisualTagEstimatorNode()
    node.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
