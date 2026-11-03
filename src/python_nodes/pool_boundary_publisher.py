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

        self.frame_id = self.get_parameter('frame_id').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.safety_margin = float(self.get_parameter('safety_margin').value)

        self.pub_marker = self.create_publisher(Marker, self.marker_topic, 10)

        # 4 Pool boundary corners (in 'odom' surface frame)
        self.corners = [
            ( 2.2248, -4.6310, 0.0),  # P1
            ( 2.3348,  1.4320, 0.0),  # P2
            (-4.3201,  1.8269, 0.0),  # P3
            (-4.6942, -4.4201, 0.0),  # P4
        ]

        # Publish at 1 Hz (latched visual markers)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f"PoolBoundaryPublisher started on {self.marker_topic} (frame: {self.frame_id})")

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()

        # 1. Outer Pool Walls (Cyan / Bright Blue Line Strip)
        wall_marker = Marker()
        wall_marker.header.stamp = stamp
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
