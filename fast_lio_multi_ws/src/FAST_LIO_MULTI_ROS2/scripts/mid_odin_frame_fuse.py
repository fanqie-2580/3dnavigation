#!/usr/bin/env python3
"""Fuse Mid + Odin without crossing clock domains (fixes mode0 fan root cause).

Bad (mode0 fan):
  Odin cloud_raw(stamp=device) × Mid /Odometry(stamp=ROS)  → wrong pose on turn

Good (this node):
  Mid points already in map (FAST-LIO clock)
  Odin points already in odin_map (Odin clock / cloud_slam or Odin-odom×raw)
  Transform Odin → map via TF map→odin_map (extrinsic lock), then merge

Never multiplies Odin points by Mid odometry.
"""
from __future__ import annotations

import math
import struct
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def make_cloud(header: Header, xyz: np.ndarray) -> PointCloud2:
    if xyz.size == 0:
        xyz = np.zeros((0, 3), dtype=np.float32)
    else:
        xyz = xyz.astype(np.float32)
    return point_cloud2.create_cloud_xyz32(header, xyz)


def xyz_from_msg(msg: PointCloud2, stride: int = 1) -> np.ndarray:
    offs = {f.name: f.offset for f in msg.fields}
    if not all(k in offs for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float64)
    pts = []
    n = msg.width * msg.height
    k = 0
    for i in range(n):
        b = i * msg.point_step
        x, y, z = struct.unpack_from('fff', msg.data, b + offs['x'])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if (k % max(1, stride)) == 0:
            pts.append([x, y, z])
        k += 1
    return np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 3), dtype=np.float64)


def tf_to_mat(tf) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class MidOdinFrameFuse(Node):
    def __init__(self) -> None:
        super().__init__('mid_odin_frame_fuse')
        self.declare_parameter('mid_scan_topic', '/cloud_registered')
        self.declare_parameter('odin_scan_topic', '/odin1/cloud_in_odom')
        self.declare_parameter('mid_accum_topic', '/viz/accum_mid360')
        self.declare_parameter('odin_accum_topic', '/viz/accum_odin')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odin_frame', 'odin_map')
        self.declare_parameter('stride', 1)
        self.declare_parameter('accum_period', 0.5)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odin_frame = str(self.get_parameter('odin_frame').value)
        self.stride = max(1, int(self.get_parameter('stride').value))

        self._mid_scan: Optional[np.ndarray] = None
        self._odin_scan: Optional[np.ndarray] = None
        self._mid_accum: Optional[np.ndarray] = None
        self._odin_accum: Optional[np.ndarray] = None
        self._stamp_scan = None
        self._stamp_accum = None

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self.pub_scan = self.create_publisher(PointCloud2, '/cloud_fused', 5)
        self.pub_accum = self.create_publisher(PointCloud2, '/viz/accum_fused', 1)

        self.create_subscription(
            PointCloud2, self.get_parameter('mid_scan_topic').value, self.on_mid_scan, 5
        )
        self.create_subscription(
            PointCloud2, self.get_parameter('odin_scan_topic').value, self.on_odin_scan, 5
        )
        self.create_subscription(
            PointCloud2, self.get_parameter('mid_accum_topic').value, self.on_mid_accum, 1
        )
        self.create_subscription(
            PointCloud2, self.get_parameter('odin_accum_topic').value, self.on_odin_accum, 1
        )
        period = float(self.get_parameter('accum_period').value)
        self.create_timer(max(0.2, period), self.publish_accum)

        self.get_logger().info(
            'Frame-domain fuse: Mid(map) + TF(map←odin_map)×Odin(odin_map) '
            '— never Mid-odom × Odin-stamp'
        )

    def _lookup_T(self) -> Optional[np.ndarray]:
        try:
            tf = self._tf_buf.lookup_transform(
                self.map_frame, self.odin_frame, rclpy.time.Time()
            )
            return tf_to_mat(tf)
        except Exception:
            return None

    def _odin_to_map(self, pts: np.ndarray) -> Optional[np.ndarray]:
        if pts is None or pts.size == 0:
            return None
        T = self._lookup_T()
        if T is None:
            return None
        R, t = T[:3, :3], T[:3, 3]
        return (pts @ R.T) + t

    def on_mid_scan(self, msg: PointCloud2) -> None:
        self._mid_scan = xyz_from_msg(msg, self.stride)
        self._stamp_scan = msg.header.stamp
        self._publish_scan()

    def on_odin_scan(self, msg: PointCloud2) -> None:
        self._odin_scan = xyz_from_msg(msg, self.stride)
        if self._stamp_scan is None:
            self._stamp_scan = msg.header.stamp
        self._publish_scan()

    def on_mid_accum(self, msg: PointCloud2) -> None:
        self._mid_accum = xyz_from_msg(msg, 1)
        self._stamp_accum = msg.header.stamp

    def on_odin_accum(self, msg: PointCloud2) -> None:
        self._odin_accum = xyz_from_msg(msg, 1)
        if self._stamp_accum is None:
            self._stamp_accum = msg.header.stamp

    def _publish_scan(self) -> None:
        parts = []
        if self._mid_scan is not None and self._mid_scan.size:
            parts.append(self._mid_scan)
        odin_m = self._odin_to_map(self._odin_scan) if self._odin_scan is not None else None
        if odin_m is not None and odin_m.size:
            parts.append(odin_m)
        if not parts or self._stamp_scan is None:
            return
        fused = np.vstack(parts) if len(parts) > 1 else parts[0]
        h = Header()
        h.stamp = self._stamp_scan
        h.frame_id = self.map_frame
        self.pub_scan.publish(make_cloud(h, fused))

    def publish_accum(self) -> None:
        parts = []
        if self._mid_accum is not None and self._mid_accum.size:
            parts.append(self._mid_accum)
        odin_m = self._odin_to_map(self._odin_accum) if self._odin_accum is not None else None
        if odin_m is not None and odin_m.size:
            parts.append(odin_m)
        if not parts:
            return
        fused = np.vstack(parts) if len(parts) > 1 else parts[0]
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.map_frame
        self.pub_accum.publish(make_cloud(h, fused))


def main() -> None:
    rclpy.init()
    node = MidOdinFrameFuse()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
