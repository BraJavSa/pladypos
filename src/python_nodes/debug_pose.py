#!/usr/bin/env python3
"""
Publica TF odom→usv5 usando posición XY del AprilTag y orientación de la IMU.
Para verificar visualmente en RViz que ambos sensores están alineados.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TagImuTF(Node):
    def __init__(self):
        super().__init__("tag_imu_tf")
        self._br = TransformBroadcaster(self)
        self._tag_x = 0.0
        self._tag_y = 0.0
        self._imu_q = None
        self.create_subscription(Odometry, "/usv5/odom", self._tag_cb, 10)
        self.create_subscription(Imu, "/usv5/imu/data", self._imu_cb, 10)

    def _tag_cb(self, msg):
        self._tag_x = msg.pose.pose.position.x
        self._tag_y = msg.pose.pose.position.y
        self._publish()

    def _imu_cb(self, msg):
        self._imu_q = msg.orientation

    def _publish(self):
        if self._imu_q is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "imubased"
        t.transform.translation.x = self._tag_x
        t.transform.translation.y = self._tag_y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._imu_q
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TagImuTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
