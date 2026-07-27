#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

def rotation_matrix_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw

class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')

        # Declare parameters
        self.declare_parameter('scale_factor', 0.003)      # Scale translation based on pixel motion
        self.declare_parameter('min_displacement', 1.0)    # Min pixel displacement to register motion
        self.declare_parameter('yaw_mismatch_threshold', 0.05)  # Max allowed yaw diff between VO and IMU (rad)
        self.declare_parameter('min_features', 60)         # Redetect if tracked features drop below this
        self.declare_parameter('max_features', 150)        # Max features to detect
        self.declare_parameter('publish_tf', True)         # Publish odom -> base_link TF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Camera extrinsics parameters (translation from base_link to camera)
        self.declare_parameter('cam_x', 0.10)
        self.declare_parameter('cam_y', 0.0)
        self.declare_parameter('cam_z', 0.30)

        # Get parameter values
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_displacement = self.get_parameter('min_displacement').value
        self.yaw_mismatch_threshold = self.get_parameter('yaw_mismatch_threshold').value
        self.min_features = self.get_parameter('min_features').value
        self.max_features = self.get_parameter('max_features').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        cam_x = self.get_parameter('cam_x').value
        cam_y = self.get_parameter('cam_y').value
        cam_z = self.get_parameter('cam_z').value

        # Define T_b_c (transformation from base_link to camera frame)
        # Orientation matches Kinect camera optical frame: Z-forward, X-right, Y-down
        # Relative to base_link: X-forward, Y-left, Z-up
        self.R_b_c = np.array([
            [ 0.0,  0.0,  1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0, -1.0,  0.0]
        ])
        self.t_b_c = np.array([cam_x, cam_y, cam_z])
        
        self.T_b_c = np.eye(4)
        self.T_b_c[0:3, 0:3] = self.R_b_c
        self.T_b_c[0:3, 3] = self.t_b_c
        self.T_c_b = np.linalg.inv(self.T_b_c)

        # Camera intrinsics matrix (will be loaded from camera_info or initialized to default qhd values)
        self.K = np.array([
            [540.686, 0.0,     479.75],
            [0.0,     540.686, 269.75],
            [0.0,     0.0,     1.0]
        ])
        self.intrinsics_loaded = False

        # State variables
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_pts = None
        
        # Absolute camera pose in world (odom) frame, initialized at camera's offset
        self.T_world_cam = np.eye(4)
        self.T_world_cam[0:3, 0:3] = self.R_b_c
        self.T_world_cam[0:3, 3] = self.t_b_c

        self.last_time = None
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        
        self.last_imu_yaw = None
        self.prev_imu_yaw = None

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'camera/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'camera/pose', 10)
        self.vis_pub = self.create_publisher(Image, 'camera/odom_visualization', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Optical flow configuration
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        self.get_logger().info("Lightweight Visual Odometry Node Initialized.")

    def camera_info_callback(self, msg):
        if not self.intrinsics_loaded:
            # Load K from camera_info
            self.K = np.array(msg.k).reshape((3, 3))
            self.intrinsics_loaded = True
            self.get_logger().info(f"Loaded camera intrinsics from camera_info: fx={self.K[0,0]:.2f}, cx={self.K[0,2]:.2f}")

    def imu_callback(self, msg):
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.last_imu_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        try:
            # Convert image to grayscale
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        current_time = self.get_clock().now()
        dt = 0.0
        if self.last_time is not None:
            dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time

        # If it's the first frame or we have too few features, detect new ones
        if self.prev_gray is None or self.prev_pts is None or len(self.prev_pts) < self.min_features:
            self.prev_pts = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=self.max_features,
                qualityLevel=0.01,
                minDistance=15
            )
            self.prev_gray = gray.copy()
            
            # Publish visualization even if we don't have enough tracked points yet
            self.publish_visualization(cv_image, [], [])
            return

        # Track features using Lucas-Kanade optical flow
        curr_pts, status, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray,
            gray,
            self.prev_pts,
            None,
            **self.lk_params
        )

        # Filter out invalid points
        if status is not None:
            valid = (status == 1).reshape(-1)
            good_prev = self.prev_pts[valid].reshape(-1, 2)
            good_curr = curr_pts[valid].reshape(-1, 2)
        else:
            good_prev = np.empty((0, 2))
            good_curr = np.empty((0, 2))

        if len(good_curr) >= 15:
            try:
                # Estimate Relative Motion using Essential Matrix RANSAC
                E, mask = cv2.findEssentialMat(
                    good_prev,
                    good_curr,
                    self.K,
                    method=cv2.RANSAC,
                    prob=0.999,
                    threshold=1.0
                )
                
                if E is not None and E.shape == (3, 3):
                    _, R, t, mask_pose = cv2.recoverPose(E, good_prev, good_curr, self.K, mask=mask)
                    
                    # Compute mean displacement of inliers
                    inliers_prev = good_prev[mask_pose.ravel() > 0]
                    inliers_curr = good_curr[mask_pose.ravel() > 0]
                    
                    if len(inliers_curr) > 0:
                        displacements = np.linalg.norm(inliers_curr - inliers_prev, axis=1)
                        mean_displacement = np.mean(displacements)
                    else:
                        mean_displacement = 0.0

                    # If displacement is above threshold, update motion; otherwise keep stationary
                    if mean_displacement > self.min_displacement:
                        scale = mean_displacement * self.scale_factor
                        
                        # Transform camera rotation to base_link frame to extract base_link yaw change
                        R_base = self.R_b_c @ R @ self.R_b_c.T
                        delta_yaw_vo = np.arctan2(R_base[1, 0], R_base[0, 0])
                        
                        # Calculate IMU yaw change
                        delta_yaw_imu = 0.0
                        is_mismatch = False
                        if self.last_imu_yaw is not None and self.prev_imu_yaw is not None:
                            diff = self.last_imu_yaw - self.prev_imu_yaw
                            delta_yaw_imu = np.arctan2(np.sin(diff), np.cos(diff))
                            yaw_discrepancy = np.abs(np.arctan2(np.sin(delta_yaw_vo - delta_yaw_imu), np.cos(delta_yaw_vo - delta_yaw_imu)))
                            
                            if yaw_discrepancy > self.yaw_mismatch_threshold:
                                is_mismatch = True
                                self.get_logger().warn(
                                    f"VO update rejected: yaw mismatch with IMU. VO yaw change: {delta_yaw_vo*180/np.pi:.2f} deg, "
                                    f"IMU yaw change: {delta_yaw_imu*180/np.pi:.2f} deg. Discrepancy: {yaw_discrepancy*180/np.pi:.2f} deg"
                                )
                        
                        if self.last_imu_yaw is not None:
                            self.prev_imu_yaw = self.last_imu_yaw

                        if not is_mismatch:
                            # Update camera pose using calculated motion
                            T_prev_curr = np.eye(4)
                            T_prev_curr[0:3, 0:3] = R
                            T_prev_curr[0:3, 3] = t.ravel() * scale
                            
                            # Integrate pose
                            self.T_world_cam = self.T_world_cam @ T_prev_curr
                            
                            # Estimate velocities
                            if dt > 0.0:
                                self.linear_velocity = (t.ravel() * scale) / dt
                                # Calculate angular velocities from rotation matrix
                                sy = np.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
                                if sy > 1e-6:
                                    self.angular_velocity = np.array([
                                        np.arctan2(R[2,1], R[2,2]),
                                        np.arctan2(-R[2,0], sy),
                                        np.arctan2(R[1,0], R[0,0])
                                    ]) / dt
                                else:
                                    self.angular_velocity = np.zeros(3)
                        else:
                            self.linear_velocity = np.zeros(3)
                            self.angular_velocity = np.zeros(3)
                    else:
                        self.linear_velocity = np.zeros(3)
                        self.angular_velocity = np.zeros(3)

                # Keep successfully tracked points for next iteration
                self.prev_pts = good_curr.reshape(-1, 1, 2)
                
            except Exception as e:
                self.get_logger().debug(f"Pose recovery failed: {e}")
                # Fallback: re-detect features
                self.prev_pts = None
        else:
            self.prev_pts = None

        self.prev_gray = gray.copy()

        # Calculate base_link pose in world (odom) frame
        # T_world_base = T_world_cam * T_c_b
        T_world_base = self.T_world_cam @ self.T_c_b
        
        # Publish messages and TF
        self.publish_outputs(msg.header.stamp, T_world_base)
        self.publish_visualization(cv_image, good_prev, good_curr)

    def publish_outputs(self, stamp, T_world_base):
        # Extract translation
        t_base = T_world_base[0:3, 3]
        # Extract rotation
        R_base = T_world_base[0:3, 0:3]
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_base)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = float(t_base[0])
        pose_msg.pose.position.y = float(t_base[1])
        pose_msg.pose.position.z = float(t_base[2])
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pose_pub.publish(pose_msg)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose

        # Non-zero covariances for robot_localization EKF fusion
        # Low variance for tracked variables (X, Y, Yaw), high variance for untracked (Z, Roll, Pitch)
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.01   # X variance
        pose_cov[7] = 0.01   # Y variance
        pose_cov[14] = 999.0 # Z variance
        pose_cov[21] = 999.0 # Roll variance
        pose_cov[28] = 999.0 # Pitch variance
        pose_cov[35] = 0.05  # Yaw variance
        odom_msg.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.02   # X velocity variance
        twist_cov[7] = 0.02   # Y velocity variance
        twist_cov[14] = 999.0 # Z velocity variance
        twist_cov[21] = 999.0 # Roll velocity variance
        twist_cov[28] = 999.0 # Pitch velocity variance
        twist_cov[35] = 0.1   # Yaw velocity variance
        odom_msg.twist.covariance = twist_cov
        
        # Express velocity in the boat's base_link frame
        # v_base = R_c_b * v_camera
        v_base = self.R_b_c.T @ self.linear_velocity
        omega_base = self.R_b_c.T @ self.angular_velocity

        odom_msg.twist.twist.linear.x = float(v_base[0])
        odom_msg.twist.twist.linear.y = float(v_base[1])
        odom_msg.twist.twist.linear.z = float(v_base[2])
        odom_msg.twist.twist.angular.x = float(omega_base[0])
        odom_msg.twist.twist.angular.y = float(omega_base[1])
        odom_msg.twist.twist.angular.z = float(omega_base[2])
        
        self.odom_pub.publish(odom_msg)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(t_base[0])
            t.transform.translation.y = float(t_base[1])
            t.transform.translation.z = float(t_base[2])
            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(t)

    def publish_visualization(self, image, prev_pts, curr_pts):
        vis_image = image.copy()
        
        # Draw tracked features and motion lines
        for p, c in zip(prev_pts, curr_pts):
            px, py = int(p[0]), int(p[1])
            cx, cy = int(c[0]), int(c[1])
            cv2.line(vis_image, (px, py), (cx, cy), (0, 255, 0), 2)
            cv2.circle(vis_image, (cx, cy), 3, (0, 0, 255), -1)

        # Print statistics overlay
        num_features = len(curr_pts)
        t_base = (self.T_world_cam @ self.T_c_b)[0:3, 3]
        
        cv2.putText(vis_image, f"Features tracked: {num_features}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(vis_image, f"Pose X: {t_base[0]:.3f} m, Y: {t_base[1]:.3f} m", (15, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Publish the visualization image
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            self.vis_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish visualization: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
