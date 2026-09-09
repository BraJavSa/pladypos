#!/usr/bin/env python3
"""
Publica el TF odom -> imu_link a partir de la orientacion ya fusionada
por imu_filter_madgwick (topic imu/data), relativa a la pose inicial
al arrancar.

Este nodo NO hardcodea el namespace: se suscribe a "imu/data" (relativo).
Si lo corres dentro del namespace usv5 (por launch o con --ros-args -r
__ns:=/usv5), automaticamente escucha /usv5/imu/data.

Uso manual (fuera de launch), especificando el namespace:
    source /opt/ros/jazzy/setup.bash
    python3 imu_tf.py --ros-args -r __ns:=/usv5

Uso dentro de un launch file (recomendado, mismo patron que el resto
del proyecto): agregar como Node(package='pladypos',
executable='imu_tf.py', namespace=ns, ...)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_mul(q1, q2):
    """Multiplicacion de cuaterniones (x,y,z,w) * (x,y,z,w) -> (x,y,z,w)."""
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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.q0_inv = None  # inversa de la orientacion inicial (pose cero)

        # Suscripcion relativa: respeta el namespace con el que se
        # lance el nodo (ej. dentro de /usv5 -> escucha /usv5/imu/data)
        self.sub = self.create_subscription(Imu, "imu/data", self.on_imu, 10)
        resolved_topic = self.sub.topic_name
        self.get_logger().info(f"Esperando {resolved_topic} (ya fusionado por Madgwick)...")

    def on_imu(self, msg: Imu):
        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        if self.q0_inv is None:
            self.q0_inv = quat_conj(q)
            self.get_logger().info("Pose inicial capturada (sera el cero).")
            return

        # Orientacion relativa a la pose inicial: q_rel = q0^-1 * q
        qx, qy, qz, qw = quat_mul(self.q0_inv, q)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "imu_link"
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