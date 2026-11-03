#!/usr/bin/env python3
"""Republish FAST_LIO_MULTI outputs onto Go2 nav stack topic names."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class MultiLioNavBridge(Node):
    def __init__(self) -> None:
        super().__init__('multi_lio_nav_bridge')
        self.declare_parameter('odom_in', '/Odometry')
        self.declare_parameter('odom_out', '/baselink2map')
        self.declare_parameter('cloud_in', '/cloud_registered')
        self.declare_parameter('cloud_out', '/cloud_registered_1')
        self.declare_parameter('cloud_body_in', '/cloud_registered_body')
        self.declare_parameter('cloud_body_out', '/cloud_registered_body_1')

        odom_in = self.get_parameter('odom_in').value
        odom_out = self.get_parameter('odom_out').value
        cloud_in = self.get_parameter('cloud_in').value
        cloud_out = self.get_parameter('cloud_out').value
        body_in = self.get_parameter('cloud_body_in').value
        body_out = self.get_parameter('cloud_body_out').value

        self.odom_pub = self.create_publisher(Odometry, odom_out, 20)
        self.cloud_pub = self.create_publisher(PointCloud2, cloud_out, 20)
        self.body_pub = self.create_publisher(PointCloud2, body_out, 20)
        self.create_subscription(Odometry, odom_in, self.on_odom, 20)
        self.create_subscription(PointCloud2, cloud_in, self.on_cloud, 20)
        self.create_subscription(PointCloud2, body_in, self.on_body, 20)
        self.get_logger().info(
            f'nav bridge: {odom_in}->{odom_out}, {cloud_in}->{cloud_out}, {body_in}->{body_out}'
        )

    def on_odom(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        # Prefer map frame name expected by downstream if empty
        if not out.header.frame_id:
            out.header.frame_id = 'map'
        out.child_frame_id = msg.child_frame_id or 'base_link'
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

    def on_cloud(self, msg: PointCloud2) -> None:
        self.cloud_pub.publish(msg)

    def on_body(self, msg: PointCloud2) -> None:
        self.body_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MultiLioNavBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
