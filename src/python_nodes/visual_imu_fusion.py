#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw_camera(qw: float, qx: float, qy: float, qz: float) -> float:
    r10 = 2.0 * (qx * qy + qw * qz)
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(-r10, r00)


def quaternion_to_yaw_imu(qw: float, qx: float, qy: float, qz: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class VisualImuFusionNode(Node):
    def __init__(self):
        super().__init__('visual_imu_fusion')

        self.declare_parameter('apriltag_odom_topic', '/usv5/odom')
        self.declare_parameter('imu_topic', '/usv5/imu/data')
        self.declare_parameter('odom_topic', '/usv5/imubased_odom')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('child_frame_id', 'imubased_usv5')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('refine_interval_sec', 2.0)

        self.apriltag_odom_topic = self.get_parameter('apriltag_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.refine_interval = float(self.get_parameter('refine_interval_sec').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.apriltag_odom_topic, self.apriltag_odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)

        self.latest_pos = None
        self.prev_pos = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.latest_ori = (0.0, 1.0, 0.0, 0.0)
        self.latest_raw_imu_yaw = None
        self.yaw_offset = None
        self.last_refine_time = 0.0
        self.latest_imu_time = None
        self.last_imu_step_time = None
        self.raw_integrated_yaw = 0.0
        self.angular_vel_z = 0.0

        timer_period = 1.0 / self.rate
        self.create_timer(timer_period, self.timer_callback)

    def update_aligned_orientation(self):
        if self.latest_raw_imu_yaw is None:
            return

        if self.yaw_offset is not None:
            psi_corr = self.latest_raw_imu_yaw + self.yaw_offset
        else:
            psi_corr = self.latest_raw_imu_yaw

        half_psi = psi_corr * 0.5
        self.latest_ori = (0.0, math.cos(half_psi), -math.sin(half_psi), 0.0)

    def apriltag_odom_cb(self, msg: Odometry):
        now_sec = time.time()
        pos = msg.pose.pose.position
        x, y, z = pos.x, pos.y, 5.1

        if self.prev_pos is not None:
            px, py, pz, pt = self.prev_pos
            dt = now_sec - pt
            if dt > 0.001:
                self.vx = (x - px) / dt
                self.vy = (y - py) / dt
                self.vz = (z - pz) / dt

        self.prev_pos = (x, y, z, now_sec)
        self.latest_pos = (x, y, z, now_sec)

        ori_tag = msg.pose.pose.orientation
        if not (ori_tag.w == 0.0 and ori_tag.x == 0.0 and ori_tag.y == 0.0 and ori_tag.z == 0.0):
            yaw_tag = quaternion_to_yaw_camera(ori_tag.w, ori_tag.x, ori_tag.y, ori_tag.z)

            if self.latest_raw_imu_yaw is not None:
                diff = yaw_tag - self.latest_raw_imu_yaw
                self.yaw_offset = math.atan2(math.sin(diff), math.cos(diff))
                self.update_aligned_orientation()

    def imu_cb(self, msg: Imu):
        now_sec = time.time()
        ori = msg.orientation
        self.angular_vel_z = msg.angular_velocity.z

        if not (ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0):
            raw_qw, raw_qx, raw_qy, raw_qz = ori.w, ori.x, ori.y, ori.z
            self.latest_raw_imu_yaw = quaternion_to_yaw_imu(raw_qw, raw_qx, raw_qy, raw_qz)
        else:
            if self.last_imu_step_time is not None:
                dt = now_sec - self.last_imu_step_time
                if 0.0001 < dt < 1.0:
                    self.raw_integrated_yaw += msg.angular_velocity.z * dt
                    self.raw_integrated_yaw = (self.raw_integrated_yaw + math.pi) % (2 * math.pi) - math.pi

            self.latest_raw_imu_yaw = self.raw_integrated_yaw

        self.update_aligned_orientation()
        self.last_imu_step_time = now_sec
        self.latest_imu_time = now_sec

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()
        now_sec = time.time()

        if self.latest_pos is not None and (now_sec - self.latest_pos[3]) <= 1.0:
            x, y, z, _ = self.latest_pos
            vx, vy, vz = self.vx, self.vy, self.vz
        else:
            x, y, z = 0.0, 0.0, 0.0
            vx, vy, vz = 0.0, 0.0, 0.0

        qw, qx, qy, qz = self.latest_ori

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 5.1

        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz
        odom_msg.twist.twist.angular.z = self.angular_vel_z

        odom_msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id

            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z

            tf_msg.transform.rotation.w = qw
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz

            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualImuFusionNode()
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
