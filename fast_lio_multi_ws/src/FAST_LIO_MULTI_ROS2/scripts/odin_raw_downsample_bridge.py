#!/usr/bin/env python3
"""Voxel-downsample Odin1 /odin1/cloud_raw for FAST_LIO_MULTI second lidar input."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np


class OdinRawDownsampleBridge(Node):
    def __init__(self) -> None:
        super().__init__('odin_raw_downsample_bridge')
        self.declare_parameter('input_topic', '/odin1/cloud_raw')
        self.declare_parameter('output_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_points', 20000)

        self.voxel = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(PointCloud2, out, 10)
        self.sub = self.create_subscription(PointCloud2, inn, self.cb, 10)
        self.get_logger().info(
            f'downsample {inn} -> {out} voxel={self.voxel}m max_points={self.max_points}'
        )

    def cb(self, msg: PointCloud2) -> None:
        pts = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if not pts:
            return
        arr = np.asarray([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        if self.voxel > 0:
            keys = np.floor(arr / self.voxel).astype(np.int32)
            # unique voxels keep first point
            _, idx = np.unique(keys, axis=0, return_index=True)
            arr = arr[np.sort(idx)]
        if len(arr) > self.max_points:
            step = int(np.ceil(len(arr) / self.max_points))
            arr = arr[::step]
        out = point_cloud2.create_cloud_xyz32(msg.header, arr.tolist())
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdinRawDownsampleBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
