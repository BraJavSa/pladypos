#!/usr/bin/env python3
"""
pool_boundary_publisher.py
Publishes visualization markers for the pool perimeter and safe navigable area
in the 'odom' frame so it can be monitored in RViz.

Corner coordinates provided from pool calibration:
  P1 (Bottom-Right): ( 2.225, -4.631, 0.0)
  P2 (Top-Right):    ( 2.335,  1.432, 0.0)
  P3 (Top-Left):     (-4.320,  1.827, 0.0)
  P4 (Bottom-Left):  (-4.694, -4.420, 0.0)
"""

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class PoolBoundaryPublisher(Node):
    def __init__(self):
        super().__init__('pool_boundary_publisher')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('marker_topic', '/pool_boundary')
        self.declare_parameter('safety_margin', 0.40)  # 40 cm safety margin from walls
        self.declare_parameter('center_x', -0.5713)
        self.declare_parameter('center_y', -1.2731)
        self.declare_parameter('ax_70', 1.7602)
        self.declare_parameter('ay_70', 1.6902)

        self.frame_id = self.get_parameter('frame_id').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.ax_70 = float(self.get_parameter('ax_70').value)
        self.ay_70 = float(self.get_parameter('ay_70').value)

        self.pub_marker = self.create_publisher(Marker, self.marker_topic, 10)

        # 4 Pool boundary corners (in 'odom' surface frame)
        self.corners = [
            ( 2.2248, -4.6310, 0.0),  # P1
            ( 2.3348,  1.4320, 0.0),  # P2
            (-4.3201,  1.8269, 0.0),  # P3
            (-4.6942, -4.4201, 0.0),  # P4
        ]

        # Precompute 70% centered Figure-8 points
        self.fig8_points = []
        n_pts = 360
        for i in range(n_pts + 1):
            theta = 2.0 * math.pi * (i / n_pts)
            p = Point()
            p.x = self.center_x + self.ax_70 * math.sin(theta)
            p.y = self.center_y + self.ay_70 * math.sin(2.0 * theta)
            p.z = -0.05
            self.fig8_points.append(p)

        # Publish at 0.5 Hz (every 2 seconds) for minimal ROS overhead
        self.create_timer(2.0, self.publish_markers)
        self.publish_markers()
        self.get_logger().info(f"PoolBoundaryPublisher started on {self.marker_topic} (frame: {self.frame_id})")

    def publish_markers(self):
        # 1. Outer Pool Walls (Cyan / Bright Blue Line Strip)
        wall_marker = Marker()
        wall_marker.header.frame_id = self.frame_id
        wall_marker.ns = "pool_outer_wall"
        wall_marker.id = 0
        wall_marker.type = Marker.LINE_STRIP
        wall_marker.action = Marker.ADD
        wall_marker.scale.x = 0.08  # 8 cm thick line
        wall_marker.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.9)  # Bright Cyan
        wall_marker.pose.orientation.w = 1.0

        for pt in self.corners + [self.corners[0]]:
            p = Point()
            p.x, p.y, p.z = pt
            wall_marker.points.append(p)

        self.pub_marker.publish(wall_marker)

        # 2. 70% Centered Figure-8 Reference Trajectory (Crimson Red Line Strip)
        fig8_marker = Marker()
        fig8_marker.header.frame_id = self.frame_id
        fig8_marker.ns = "figure8_reference"
        fig8_marker.id = 10
        fig8_marker.type = Marker.LINE_STRIP
        fig8_marker.action = Marker.ADD
        fig8_marker.scale.x = 0.045
        fig8_marker.color = ColorRGBA(r=0.95, g=0.22, b=0.22, a=0.95)
        fig8_marker.pose.orientation.w = 1.0
        fig8_marker.points = self.fig8_points
        self.pub_marker.publish(fig8_marker)

        # Delete legacy start sphere & executed trail markers if present in RViz
        for ns_del, id_del in (("figure8_start", 11), ("usv_executed_trail", 13)):
            del_marker = Marker()
            del_marker.header.frame_id = self.frame_id
            del_marker.ns = ns_del
            del_marker.id = id_del
            del_marker.action = Marker.DELETE
            self.pub_marker.publish(del_marker)


def main(args=None):
    rclpy.init(args=args)
    node = PoolBoundaryPublisher()
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
