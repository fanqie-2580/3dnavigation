#!/usr/bin/env python3
"""Voxel-downsample Odin1 /odin1/cloud_raw for FAST_LIO_MULTI lidar2.

Preserves intensity, confidence (optional), and offset_time (seconds) so
preprocess xyz_handler can write per-point curvature in ms.

IMPORTANT for tight fusion:
  - Do NOT restamp with Mid /Odometry (that forges time and breaks IEKF sync).
  - Keep Odin header.stamp (after host-time alignment via use_host_ros_time=2).
  - Optional fixed stamp_offset_sec for a measured constant delay.
"""
from __future__ import annotations

import struct
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _stamp_add(stamp: TimeMsg, offset_sec: float) -> TimeMsg:
    if abs(offset_sec) < 1e-12:
        return stamp
    total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) + int(round(offset_sec * 1e9))
    out = TimeMsg()
    out.sec = total_ns // 1_000_000_000
    out.nanosec = total_ns % 1_000_000_000
    return out


class OdinRawDownsampleBridge(Node):
    def __init__(self) -> None:
        super().__init__('odin_raw_downsample_bridge')
        self.declare_parameter('input_topic', '/odin1/cloud_raw')
        self.declare_parameter('output_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('conf_min', 30.0)
        self.declare_parameter('stamp_offset_sec', 0.0)
        # Kept for compatibility; default false for MULTI fusion.
        self.declare_parameter('restamp_from_odom', False)

        self.voxel = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.conf_min = float(self.get_parameter('conf_min').value)
        self.stamp_offset_sec = float(self.get_parameter('stamp_offset_sec').value)
        if bool(self.get_parameter('restamp_from_odom').value):
            self.get_logger().warn(
                'restamp_from_odom=true is NOT recommended for FAST_LIO_MULTI fusion; '
                'keeping Odin stamp + stamp_offset_sec instead.'
            )

        inn = str(self.get_parameter('input_topic').value)
        out = str(self.get_parameter('output_topic').value)
        from rclpy.qos import qos_profile_sensor_data
        self.pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, inn, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'downsample {inn} -> {out} voxel={self.voxel}m max_points={self.max_points} '
            f'conf_min={self.conf_min} stamp_offset_sec={self.stamp_offset_sec}'
        )

    def cb(self, msg: PointCloud2) -> None:
        names = [f.name for f in msg.fields]
        fields: List[str] = ['x', 'y', 'z']
        if 'intensity' in names:
            fields.append('intensity')
        if 'confidence' in names:
            fields.append('confidence')
        if 'offset_time' in names:
            fields.append('offset_time')

        pts = list(point_cloud2.read_points(msg, field_names=tuple(fields), skip_nans=True))
        if not pts:
            return

        idx = {n: i for i, n in enumerate(fields)}
        rows = []
        for p in pts:
            x = float(p[idx['x']]); y = float(p[idx['y']]); z = float(p[idx['z']])
            inten = float(p[idx['intensity']]) if 'intensity' in idx else 0.0
            conf = float(p[idx['confidence']]) if 'confidence' in idx else 1e9
            ot = float(p[idx['offset_time']]) if 'offset_time' in idx else 0.0
            if self.conf_min > 0.0 and conf < self.conf_min:
                continue
            if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                continue
            rows.append((x, y, z, inten, ot))
        if not rows:
            return

        arr = np.asarray(rows, dtype=np.float32)
        if self.voxel > 0:
            keys = np.floor(arr[:, :3] / self.voxel).astype(np.int32)
            _, keep = np.unique(keys, axis=0, return_index=True)
            arr = arr[np.sort(keep)]
        if len(arr) > self.max_points:
            step = int(np.ceil(len(arr) / self.max_points))
            arr = arr[::step]

        header = Header()
        header.stamp = _stamp_add(msg.header.stamp, self.stamp_offset_sec)
        header.frame_id = msg.header.frame_id
        out_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='offset_time', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        data = b''.join(
            struct.pack('<fffff', float(x), float(y), float(z), float(i), float(t))
            for x, y, z, i, t in arr
        )
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = len(arr)
        out.fields = out_fields
        out.is_bigendian = False
        out.point_step = 20
        out.row_step = 20 * len(arr)
        out.is_dense = True
        out.data = data
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
