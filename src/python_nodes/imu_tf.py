#!/usr/bin/env python3
"""
Publishes the TF transform odom -> imu_link from the orientation fused
by imu_filter_madgwick (topic imu/data), relative to the initial startup pose.

This node does NOT hardcode the namespace: it subscribes to "imu/data" relatively.
If launched within the usv5 namespace (via launch or with --ros-args -r __ns:=/usv5),
it automatically listens to /usv5/imu/data.

Parameters:
    frame_id        (string, default 'odom')
    child_frame_id  (string, default 'imu_link')
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_mul(q1, q2):
    """Quaternion multiplication (x,y,z,w) * (x,y,z,w) -> (x,y,z,w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


class ImuTfNode(Node):
    def __init__(self):
        super().__init__("imu_tf_broadcaster")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "imu_link")

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.q0_inv = None  # inverse of initial orientation (zero reference pose)

        # Relative topic subscription: respects node namespace (e.g. /usv5 -> /usv5/imu/data)
        self.sub = self.create_subscription(Imu, "imu/data", self.on_imu, 10)
        resolved_topic = self.sub.topic_name
        self.get_logger().info(f"Waiting for {resolved_topic} (fused by Madgwick)...")

    def on_imu(self, msg: Imu):
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        if self.q0_inv is None:
            self.q0_inv = quat_conj(q)
            self.get_logger().info("Initial pose captured (set as reference zero).")
            return

        # Orientation relative to initial pose: q_rel = q0^-1 * q
        qx, qy, qz, qw = quat_mul(self.q0_inv, q)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ImuTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
