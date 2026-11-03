#!/usr/bin/env python3
"""Split deskewed /cloud_registered into Mid360 / Odin topics for RViz color compare.

Uses LIO output (motion-compensated + pose-correct). Do NOT transform raw clouds
with the latest /Odometry — that smears as soon as the robot moves.
"""
from __future__ import annotations

import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def make_cloud(header: Header, xyz: np.ndarray) -> PointCloud2:
    if xyz.size == 0:
        xyz = np.zeros((0, 3), dtype=np.float32)
    else:
        xyz = xyz.astype(np.float32)
    return point_cloud2.create_cloud_xyz32(header, xyz)


class DualLidarColorViz(Node):
    def __init__(self) -> None:
        super().__init__('dual_lidar_color_viz')
        self.declare_parameter('registered_topic', '/cloud_registered')
        self.declare_parameter('odin_intensity_thresh', 100.0)
        self.declare_parameter('mid_stride', 1)
        self.declare_parameter('odin_stride', 1)

        self.reg_topic = self.get_parameter('registered_topic').value
        self.odin_thr = float(self.get_parameter('odin_intensity_thresh').value)
        self.mid_stride = max(1, int(self.get_parameter('mid_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.create_subscription(PointCloud2, self.reg_topic, self.on_registered, 5)
        self.get_logger().info(
            f'dual color viz: split {self.reg_topic} -> '
            '/viz/cloud_mid360 (cyan) /viz/cloud_odin (orange)'
        )

    def on_registered(self, msg: PointCloud2) -> None:
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return
        has_i = 'intensity' in offs
        n = msg.width * msg.height
        mid: list[list[float]] = []
        odin: list[list[float]] = []
        step = msg.point_step
        data = msg.data
        mi = oi = 0
        for i in range(n):
            b = i * step
            x, y, z = struct.unpack_from('fff', data, b + offs['x'])
            inten = (
                struct.unpack_from('f', data, b + offs['intensity'])[0]
                if has_i
                else 10.0
            )
            if inten >= self.odin_thr:
                if (oi % self.odin_stride) == 0:
                    odin.append([x, y, z])
                oi += 1
            else:
                if (mi % self.mid_stride) == 0:
                    mid.append([x, y, z])
                mi += 1

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or 'map'
        self.pub_mid.publish(make_cloud(header, np.asarray(mid, dtype=np.float64)))
        self.pub_odin.publish(make_cloud(header, np.asarray(odin, dtype=np.float64)))


def main() -> None:
    rclpy.init()
    node = DualLidarColorViz()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
