#!/usr/bin/env python3
"""Fuse Mid360 + Odin1 into map-frame scan / accum (mid360_odin1 path).

Mid360 FAST-LIO already publishes map-fixed /cloud_registered and (via
dual_lidar_color_viz) /viz/accum_mid360.

Odin1 with enable_odin_timed goes through odin_mid_timed_project →
/odin1/cloud_in_map (already in map). color_viz intentionally runs
odin_mode=off so it does NOT own Odin accum; this node therefore:

  - merges current-frame Mid + Odin → /cloud_fused
  - voxel-accumulates Odin from odin_scan_topic → /viz/accum_odin
  - merges Mid accum + Odin accum → /viz/accum_fused

Never multiplies Odin by Mid odometry (that caused mode0 fan).
"""
from __future__ import annotations

import math
import struct
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
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


class VoxelAccum:
    """Simple voxel grid accumulator (same idea as dual_lidar_color_viz)."""

    def __init__(self, voxel: float, max_points: int) -> None:
        self.voxel = max(1e-3, float(voxel))
        self.max_points = max(1000, int(max_points))
        self._grid: Dict[Tuple[int, int, int], np.ndarray] = {}

    def clear(self) -> None:
        self._grid.clear()

    def __len__(self) -> int:
        return len(self._grid)

    def add(self, pts: np.ndarray) -> bool:
        if pts is None or pts.size == 0:
            return False
        inv = 1.0 / self.voxel
        changed = False
        for p in pts:
            key = (int(math.floor(p[0] * inv)), int(math.floor(p[1] * inv)), int(math.floor(p[2] * inv)))
            if key not in self._grid:
                if len(self._grid) >= self.max_points:
                    continue
                self._grid[key] = p.astype(np.float64, copy=False)
                changed = True
        return changed

    def numpy(self) -> np.ndarray:
        if not self._grid:
            return np.zeros((0, 3), dtype=np.float64)
        return np.vstack(list(self._grid.values()))


class MidOdinFrameFuse(Node):
    def __init__(self) -> None:
        super().__init__('mid_odin_frame_fuse')
        self.declare_parameter('mid_scan_topic', '/cloud_registered')
        self.declare_parameter('odin_scan_topic', '/odin1/cloud_in_map')
        self.declare_parameter('mid_accum_topic', '/viz/accum_mid360')
        self.declare_parameter('odin_accum_topic', '/viz/accum_odin')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odin_frame', 'odin_map')
        # true: Odin scan already in map (odin_mid_timed_project for mid360+odin1)
        self.declare_parameter('odin_already_in_map', True)
        # Build Odin voxel accum from odin_scan (needed when color_viz odin_mode=off).
        self.declare_parameter('accumulate_odin_from_scan', True)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_skip_sec', 2.0)
        # Skip Odin voxel accum while Mid yaw rate (from /Odometry pose delta) is high.
        self.declare_parameter('mid_odom_topic', '/Odometry')
        self.declare_parameter('accum_gyr_gate', 0.35)
        # Do not voxel-add Odin scans older than this (host_now - stamp).
        # With dock residual lag ~1–2s, keep this tight so delayed/wrong paints
        # never enter the voxel map (live topic /odin1/cloud_in_map is separate).
        self.declare_parameter('max_scan_age_sec', 0.25)
        # After |yaw_rate| exceeds gate, keep blocking accum for this long.
        self.declare_parameter('accum_gyr_cooldown_sec', 1.0)
        self.declare_parameter('stride', 1)
        self.declare_parameter('accum_period', 0.5)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.odin_frame = str(self.get_parameter('odin_frame').value)
        self.odin_already_in_map = bool(self.get_parameter('odin_already_in_map').value)
        self.accumulate_odin = bool(self.get_parameter('accumulate_odin_from_scan').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self.accum_gyr_gate = float(self.get_parameter('accum_gyr_gate').value)
        self.max_scan_age_sec = float(self.get_parameter('max_scan_age_sec').value)
        self.accum_gyr_cooldown_sec = float(self.get_parameter('accum_gyr_cooldown_sec').value)
        self._mid_gyr_norm = 0.0
        self._prev_mid_yaw: Optional[float] = None
        self._prev_mid_t: Optional[float] = None
        self._gyr_block_until = 0.0
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)

        self._mid_scan: Optional[np.ndarray] = None
        self._odin_scan: Optional[np.ndarray] = None
        self._mid_accum: Optional[np.ndarray] = None
        self._odin_accum_ext: Optional[np.ndarray] = None
        self._stamp_scan = None
        self._t0 = time.time()
        self._odin_voxel = VoxelAccum(voxel, max_n)
        self._dirty_odin = False

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        from rclpy.qos import qos_profile_sensor_data

        self.pub_scan = self.create_publisher(PointCloud2, '/cloud_fused', 5)
        self.pub_accum = self.create_publisher(PointCloud2, '/viz/accum_fused', 1)
        # Only advertise /viz/accum_odin when we own densify (else independent_accum).
        self.pub_accum_odin = None
        if self.accumulate_odin:
            self.pub_accum_odin = self.create_publisher(
                PointCloud2, str(self.get_parameter('odin_accum_topic').value), 1
            )

        self.create_subscription(
            PointCloud2,
            self.get_parameter('mid_scan_topic').value,
            self.on_mid_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('odin_scan_topic').value,
            self.on_odin_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, self.get_parameter('mid_accum_topic').value, self.on_mid_accum, 1
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('mid_odom_topic').value),
            self.on_mid_odom,
            50,
        )
        # External Odin accum only when we are NOT the publisher (avoid self-echo).
        if not self.accumulate_odin:
            self.create_subscription(
                PointCloud2,
                self.get_parameter('odin_accum_topic').value,
                self.on_odin_accum_ext,
                1,
            )
        period = float(self.get_parameter('accum_period').value)
        self.create_timer(max(0.2, period), self.publish_accum)

        mode = 'already-in-map' if self.odin_already_in_map else 'TF(map←odin_map)'
        self.get_logger().info(
            f'Frame fuse Mid360+Odin1: Mid(map) + Odin via {mode}; '
            f'odin_scan_accum={self.accumulate_odin} voxel={voxel} '
            f'accum_gyr_gate={self.accum_gyr_gate} cooldown={self.accum_gyr_cooldown_sec:.2f}s '
            f'max_scan_age={self.max_scan_age_sec:.2f}s'
        )

    def _accum_ready(self) -> bool:
        return (time.time() - self._t0) >= self.accum_skip_sec

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
        if self.odin_already_in_map:
            return pts
        T = self._lookup_T()
        if T is None:
            return None
        R, t = T[:3, :3], T[:3, 3]
        return (pts @ R.T) + t

    def on_mid_scan(self, msg: PointCloud2) -> None:
        self._mid_scan = xyz_from_msg(msg, self.stride)
        self._stamp_scan = msg.header.stamp
        self._publish_scan()

    def on_mid_odom(self, msg: Odometry) -> None:
        # FAST_LIO /Odometry often has empty twist — estimate yaw rate from pose.
        q = msg.pose.pose.orientation
        # yaw from quaternion (ZXY body)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self._prev_mid_t is not None and t > self._prev_mid_t + 1e-4:
            dyaw = yaw - self._prev_mid_yaw
            while dyaw > math.pi:
                dyaw -= 2.0 * math.pi
            while dyaw < -math.pi:
                dyaw += 2.0 * math.pi
            self._mid_gyr_norm = abs(dyaw) / (t - self._prev_mid_t)
            if self._mid_gyr_norm >= self.accum_gyr_gate:
                self._gyr_block_until = time.time() + max(0.0, self.accum_gyr_cooldown_sec)
        self._prev_mid_yaw = yaw
        self._prev_mid_t = t

    def on_odin_scan(self, msg: PointCloud2) -> None:
        pts = xyz_from_msg(msg, self.stride)
        # Always refresh live fused scan from /odin1/cloud_in_map (already map-projected).
        self._odin_scan = pts
        if self._stamp_scan is None:
            self._stamp_scan = msg.header.stamp
        self._publish_scan()

        if not self.accumulate_odin or not self._accum_ready():
            return
        stamp_age = (
            self.get_clock().now().nanoseconds * 1e-9
            - (float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9)
        )
        # Stale stamp / turn cooldown: never voxel-add (prevents wrong map stains).
        if self.max_scan_age_sec > 0.0 and stamp_age > self.max_scan_age_sec:
            return
        if self._mid_gyr_norm >= self.accum_gyr_gate or time.time() < self._gyr_block_until:
            return
        odin_m = self._odin_to_map(pts)
        if odin_m is None:
            return
        if self._odin_voxel.add(odin_m):
            self._dirty_odin = True

    def on_mid_accum(self, msg: PointCloud2) -> None:
        self._mid_accum = xyz_from_msg(msg, 1)

    def on_odin_accum_ext(self, msg: PointCloud2) -> None:
        # Ignore our own republish echo if DDS loops; size check is enough for now.
        self._odin_accum_ext = xyz_from_msg(msg, 1)

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

    def _odin_accum_cloud(self) -> Optional[np.ndarray]:
        # Prefer local voxel from timed-project scans (mid360+odin1 default).
        if self.accumulate_odin and len(self._odin_voxel):
            return self._odin_voxel.numpy()
        if self._odin_accum_ext is not None and self._odin_accum_ext.size:
            return self._odin_to_map(self._odin_accum_ext)
        return None

    def publish_accum(self) -> None:
        odin_m = self._odin_accum_cloud()
        stamp = self.get_clock().now().to_msg()
        h = Header()
        h.stamp = stamp
        h.frame_id = self.map_frame

        if (
            self.accumulate_odin
            and self.pub_accum_odin is not None
            and odin_m is not None
            and odin_m.size
        ):
            self.pub_accum_odin.publish(make_cloud(h, odin_m))
            self._dirty_odin = False

        parts = []
        if self._mid_accum is not None and self._mid_accum.size:
            parts.append(self._mid_accum)
        if odin_m is not None and odin_m.size:
            parts.append(odin_m)
        if not parts:
            return
        fused = np.vstack(parts) if len(parts) > 1 else parts[0]
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
