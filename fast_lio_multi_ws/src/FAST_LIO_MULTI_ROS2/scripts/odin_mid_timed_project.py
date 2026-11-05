#!/usr/bin/env python3
"""Project delayed Odin raw cloud into Mid map using Mid TF at Odin stamp.

After host-time align, Odin stamps are on the same timeline as Mid, but arrive
~3s late over TCP. Using Odin's own odom builds a second drifting map → ghost/fan
when fused. Instead:

  T_map_odin_lidar(t) = T_map_body(t) * T_body_L1 * T_L1_L2

Lookup Mid TF at the cloud's stamp (needs long TF cache). Drop if |dt| too large.
"""
from __future__ import annotations

import math
import struct
from typing import Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def mat_from_tf(tf: TransformStamped) -> np.ndarray:
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


def se3(Rflat, Txyz) -> np.ndarray:
    R = np.asarray(Rflat, dtype=np.float64).reshape(3, 3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(Txyz, dtype=np.float64).reshape(3)
    return T


class OdinMidTimedProject(Node):
    def __init__(self) -> None:
        super().__init__('odin_mid_timed_project')
        self.declare_parameter('cloud_topic', '/odin1/cloud_raw')
        self.declare_parameter('out_topic', '/odin1/cloud_in_map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('tf_cache_sec', 12.0)
        self.declare_parameter('max_stamp_delay_sec', 6.0)
        self.declare_parameter('stride', 2)
        self.declare_parameter('blind', 0.3)
        self.declare_parameter('min_confidence', 30.0)
        # Mid IMU(body) → Mid lidar, Odin lidar in Mid lidar (same as mid360_odin1.yaml)
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [0.150, 0.0, 0.035])
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [
                0.97317887, 0.0, -0.23004974,
                0.0, 1.0, 0.0,
                0.23004974, 0.0, 0.97317887,
            ],
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.body_frame = str(self.get_parameter('body_frame').value)
        self.max_delay = float(self.get_parameter('max_stamp_delay_sec').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.blind2 = float(self.get_parameter('blind').value) ** 2
        self.min_conf = float(self.get_parameter('min_confidence').value)
        cache = float(self.get_parameter('tf_cache_sec').value)

        T_body_L1 = se3(
            self.get_parameter('extrinsic_R').value,
            self.get_parameter('extrinsic_T').value,
        )
        T_L1_L2 = se3(
            self.get_parameter('extrinsic_R_L2_wrt_L1').value,
            self.get_parameter('extrinsic_T_L2_wrt_L1').value,
        )
        self.T_body_odin = T_body_L1 @ T_L1_L2

        self._tf = Buffer(cache_time=Duration(seconds=cache))
        self._tf_listener = TransformListener(self._tf, self)
        self._drop = 0
        self._ok = 0

        from rclpy.qos import qos_profile_sensor_data

        out = str(self.get_parameter('out_topic').value)
        inn = str(self.get_parameter('cloud_topic').value)
        self.pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, inn, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(2.0, self._stats)
        self.get_logger().info(
            f'Odin→Mid-map timed project: {inn} → {out}, '
            f'TF {self.map_frame}←{self.body_frame} cache={cache}s max_delay={self.max_delay}s'
        )

    def _stats(self) -> None:
        total = self._ok + self._drop
        if total == 0:
            return
        self.get_logger().info(
            f'timed_project ok={self._ok} drop={self._drop} '
            f'({100.0 * self._ok / total:.0f}% hit)'
        )
        self._ok = 0
        self._drop = 0

    def _xyz(self, msg: PointCloud2) -> np.ndarray:
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return np.zeros((0, 3), dtype=np.float64)
        has_conf = 'confidence' in offs
        pts = []
        n = msg.width * msg.height
        k = 0
        for i in range(n):
            b = i * msg.point_step
            x, y, z = struct.unpack_from('fff', msg.data, b + offs['x'])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if x * x + y * y + z * z <= self.blind2:
                continue
            if has_conf and self.min_conf > 0:
                try:
                    conf = struct.unpack_from('f', msg.data, b + offs['confidence'])[0]
                except Exception:
                    conf = float(msg.data[b + offs['confidence']])
                if conf < self.min_conf:
                    continue
            if (k % self.stride) == 0:
                pts.append([x, y, z])
            k += 1
        return np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 3), dtype=np.float64)

    def on_cloud(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp
        now = self.get_clock().now()
        delay = (now - Time.from_msg(stamp)).nanoseconds * 1e-9
        if delay < -0.5 or delay > self.max_delay:
            self._drop += 1
            return
        try:
            tf = self._tf.lookup_transform(
                self.map_frame,
                self.body_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            self._drop += 1
            return

        arr = self._xyz(msg)
        if arr.size == 0:
            return
        T = mat_from_tf(tf) @ self.T_body_odin
        world = (arr @ T[:3, :3].T) + T[:3, 3]
        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, world.astype(np.float32)))
        self._ok += 1


def main() -> None:
    rclpy.init()
    node = OdinMidTimedProject()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
