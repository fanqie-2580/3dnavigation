#!/usr/bin/env python3
"""Prepare Odin1 cloud for FAST_LIO_MULTI L2 (Mid360+Odin1 tight coupling).

Pipeline:
  1) If frame_id is odom/map (dock often forwards cloud_slam as cloud_raw),
     unproject to lidar frame with /odin1/odometry * T_imu_L.
  2) Confidence filter when field exists.
  3) Voxel downsample; keep offset_time (s) when present (else 0).
  4) Restamp only to a Mid /Odometry stamp within mid_restamp_max_dt of the
     *cloud* stamp (never latest Mid while geometry is seconds old).
     Always enforce monotonic stamps to avoid L2 "loop back".
  5) Rate-limit ~one cloud per Mid odom tick when Mid odom is flowing;
     otherwise publish every Nth input (~10 Hz).
  6) Drop clouds older than max_cloud_age_sec vs newest Mid (stale TCP backlog).
"""
from __future__ import annotations

import math
import struct
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _field_map(msg: PointCloud2) -> Dict[str, PointField]:
    return {f.name: f for f in msg.fields}


def _stamp_key(stamp) -> Tuple[int, int]:
    return (int(stamp.sec), int(stamp.nanosec))


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _mat_from_odom(msg: Odometry) -> np.ndarray:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
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
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def _unpack_points(msg: PointCloud2, min_conf: float) -> np.ndarray:
    """Nx5: x,y,z,intensity,offset_time_sec (vectorized)."""
    fm = _field_map(msg)
    if not all(k in fm for k in ('x', 'y', 'z')):
        return np.zeros((0, 5), dtype=np.float32)

    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return np.zeros((0, 5), dtype=np.float32)
    step = int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < n * step:
        n = raw.size // step
    if n <= 0:
        return np.zeros((0, 5), dtype=np.float32)
    arr = raw[: n * step].reshape(n, step)

    def col_f32(name: str) -> np.ndarray:
        o = fm[name].offset
        return arr[:, o : o + 4].copy().view(np.float32).reshape(-1)

    xyz = np.column_stack([col_f32('x'), col_f32('y'), col_f32('z')])
    finite = np.isfinite(xyz).all(axis=1)

    intensity = np.zeros(n, dtype=np.float32)
    if 'intensity' in fm:
        f = fm['intensity']
        o = f.offset
        if f.datatype == PointField.FLOAT32:
            intensity = arr[:, o : o + 4].copy().view(np.float32).reshape(-1)
        elif f.datatype == PointField.UINT8:
            intensity = arr[:, o].astype(np.float32)
        elif f.datatype == PointField.UINT16:
            intensity = arr[:, o : o + 2].copy().view(np.uint16).reshape(-1).astype(np.float32)

    if 'confidence' in fm and min_conf > 0.0:
        f = fm['confidence']
        o = f.offset
        if f.datatype == PointField.FLOAT32:
            conf = arr[:, o : o + 4].copy().view(np.float32).reshape(-1)
        elif f.datatype == PointField.UINT16:
            conf = arr[:, o : o + 2].copy().view(np.uint16).reshape(-1).astype(np.float32)
        elif f.datatype == PointField.UINT8:
            conf = arr[:, o].astype(np.float32)
        else:
            conf = np.full(n, 1e9, dtype=np.float32)
        finite &= conf >= min_conf

    offset_time = np.zeros(n, dtype=np.float32)
    tname = next((k for k in ('offset_time', 'time', 't') if k in fm), None)
    if tname is not None:
        f = fm[tname]
        o = f.offset
        if f.datatype == PointField.FLOAT32:
            offset_time = arr[:, o : o + 4].copy().view(np.float32).reshape(-1)
        elif f.datatype == PointField.UINT32:
            offset_time = (
                arr[:, o : o + 4].copy().view(np.uint32).reshape(-1).astype(np.float32) * 1e-9
            )

    xyz = xyz[finite]
    intensity = intensity[finite]
    offset_time = offset_time[finite]
    if xyz.size == 0:
        return np.zeros((0, 5), dtype=np.float32)
    return np.column_stack([xyz, intensity, offset_time]).astype(np.float32)


class OdinRawDownsampleBridge(Node):
    def __init__(self) -> None:
        super().__init__('odin_raw_downsample_bridge')
        self.declare_parameter('input_topic', '/odin1/cloud_raw')
        self.declare_parameter('output_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('min_confidence', 30.0)
        self.declare_parameter('restamp_from_odom', True)
        self.declare_parameter('one_cloud_per_odom', True)
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('odin_odom_topic', '/odin1/odometry')
        self.declare_parameter('odin_odom_max_dt', 1.0)
        # Only restamp when |t_cloud - t_mid| <= this (matched historical Mid).
        self.declare_parameter('mid_restamp_max_dt', 0.05)
        self.declare_parameter('max_cloud_age_sec', 0.35)
        self.declare_parameter('fallback_period', 0.1)
        # Official Odin IMU→lidar (wiki), R≈I.
        self.declare_parameter('extrinsic_T_imu_L', [-0.02663, 0.03447, 0.02174])

        self.voxel = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.restamp_from_odom = bool(self.get_parameter('restamp_from_odom').value)
        self.one_cloud_per_odom = bool(self.get_parameter('one_cloud_per_odom').value)
        self.odin_odom_max_dt = float(self.get_parameter('odin_odom_max_dt').value)
        self.mid_restamp_max_dt = float(self.get_parameter('mid_restamp_max_dt').value)
        self.max_cloud_age_sec = float(self.get_parameter('max_cloud_age_sec').value)
        self.fallback_period = float(self.get_parameter('fallback_period').value)
        t_il = list(self.get_parameter('extrinsic_T_imu_L').value)
        self.T_imu_L = np.eye(4, dtype=np.float64)
        self.T_imu_L[:3, 3] = np.asarray(t_il, dtype=np.float64).reshape(3)

        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self._mid_odom_stamp = None
        self._mid_odom_key: Optional[Tuple[int, int]] = None
        self._last_pub_key: Optional[Tuple[int, int]] = None
        self._last_pub_sec = -1.0
        self._pending: Optional[PointCloud2] = None
        self._odin_odom: Deque[Tuple[float, np.ndarray]] = deque(maxlen=2000)
        # (t_sec, stamp_msg)
        self._mid_stamps: Deque[Tuple[float, object]] = deque(maxlen=500)
        self._warned_odom_cloud = False
        self._n_cloud = 0
        self._n_mid = 0
        self._n_odin = 0
        self._n_pub = 0
        self._n_skip_unproj = 0
        self._n_skip_empty = 0
        self._n_skip_age = 0

        reliable = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(PointCloud2, out, reliable)
        self.sub = self.create_subscription(PointCloud2, inn, self.cb, reliable)
        self.create_subscription(
            Odometry, self.get_parameter('odin_odom_topic').value, self.on_odin_odom, reliable
        )
        if self.restamp_from_odom or self.one_cloud_per_odom:
            self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value, self.on_mid_odom, reliable
            )
        self.create_timer(2.0, self._status)
        self.get_logger().info(
            f'downsample {inn} -> {out} voxel={self.voxel}m max={self.max_points} '
            f'conf>={self.min_confidence} restamp={self.restamp_from_odom} '
            f'mid_dt<={self.mid_restamp_max_dt:.3f}s max_age={self.max_cloud_age_sec:.2f}s '
            f'one_per_odom={self.one_cloud_per_odom} qos=RELIABLE '
            f'(unproject odom-frame if needed; match Mid by cloud stamp)'
        )

    def _status(self) -> None:
        self.get_logger().info(
            f'stats cloud={self._n_cloud} odin_odom={self._n_odin} mid_odom={self._n_mid} '
            f'pub={self._n_pub} skip_unproj={self._n_skip_unproj} skip_empty={self._n_skip_empty} '
            f'skip_age={self._n_skip_age} pending={"Y" if self._pending is not None else "N"}'
        )

    def on_odin_odom(self, msg: Odometry) -> None:
        self._n_odin += 1
        t = _stamp_sec(msg.header.stamp)
        self._odin_odom.append((t, _mat_from_odom(msg)))

    def _nearest_mid_stamp(self, t_cloud: float):
        best = None
        best_dt = 1e9
        for t, st in self._mid_stamps:
            dt = abs(t - t_cloud)
            if dt < best_dt:
                best_dt = dt
                best = (t, st, dt)
        return best

    def on_mid_odom(self, msg: Odometry) -> None:
        self._n_mid += 1
        key = _stamp_key(msg.header.stamp)
        t = _stamp_sec(msg.header.stamp)
        self._mid_odom_stamp = msg.header.stamp
        self._mid_odom_key = key
        self._mid_stamps.append((t, msg.header.stamp))
        if self.one_cloud_per_odom and self._pending is not None and self._last_pub_key != key:
            # Match Mid stamp to *pending cloud* time — not blindly latest Mid.
            if self._publish_from_msg(self._pending, None):
                self._last_pub_key = key
            self._pending = None

    def _lookup_odin_odom(self, stamp) -> Optional[np.ndarray]:
        if not self._odin_odom:
            return None
        t = _stamp_sec(stamp)
        best = None
        best_dt = 1e9
        for ot, T in self._odin_odom:
            dt = abs(ot - t)
            if dt < best_dt:
                best_dt = dt
                best = T
        if best is None or best_dt > self.odin_odom_max_dt:
            return None
        return best

    def _cloud_in_odom(self, msg: PointCloud2) -> bool:
        fid = (msg.header.frame_id or '').strip().lower()
        return fid in ('odom', 'map', 'odin_odom', 'odin_map')

    def cb(self, msg: PointCloud2) -> None:
        self._n_cloud += 1
        t_cloud = _stamp_sec(msg.header.stamp)
        if self._mid_stamps and self.max_cloud_age_sec > 0.0:
            newest_mid = self._mid_stamps[-1][0]
            if (newest_mid - t_cloud) > self.max_cloud_age_sec:
                self._n_skip_age += 1
                return
        # Prefer Mid-odom gated publish when Mid is alive; otherwise publish on a
        # fallback period so LIO can bootstrap without chicken-egg deadlock.
        if self.one_cloud_per_odom and self._mid_odom_stamp is not None:
            self._pending = msg
            if self._mid_odom_key is not None and self._last_pub_key != self._mid_odom_key:
                if self._publish_from_msg(msg, None):
                    self._last_pub_key = self._mid_odom_key
                self._pending = None
            return

        # No Mid odom yet: rate-limit by wall time / cloud stamp.
        stamp = msg.header.stamp
        pub_sec = _stamp_sec(stamp)
        if self._last_pub_sec >= 0.0 and (pub_sec - self._last_pub_sec) < self.fallback_period:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if (now_sec - self._last_pub_sec) < self.fallback_period:
                self._pending = msg
                return
            stamp = self.get_clock().now().to_msg()
        self._publish_from_msg(msg, stamp)

    def _publish_from_msg(self, msg: PointCloud2, stamp) -> bool:
        t_cloud = _stamp_sec(msg.header.stamp)
        if self._mid_stamps and self.max_cloud_age_sec > 0.0:
            newest_mid = self._mid_stamps[-1][0]
            if (newest_mid - t_cloud) > self.max_cloud_age_sec:
                self._n_skip_age += 1
                return False

        arr = _unpack_points(msg, self.min_confidence)
        if arr.size == 0:
            self._n_skip_empty += 1
            return False

        if self._cloud_in_odom(msg):
            if not self._warned_odom_cloud:
                self.get_logger().warn(
                    'Input looks like odom-frame cloud_slam (frame_id=%s, fields=%s). '
                    'Unprojecting with /odin1/odometry for L2; prefer mode0 true raw '
                    'with offset_time for better undistortion.'
                    % (msg.header.frame_id, [f.name for f in msg.fields])
                )
                self._warned_odom_cloud = True
            T_oi = self._lookup_odin_odom(msg.header.stamp)
            if T_oi is None:
                self._n_skip_unproj += 1
                return False
            T_oL = T_oi @ self.T_imu_L
            R = T_oL[:3, :3]
            t = T_oL[:3, 3]
            xyz = arr[:, :3].astype(np.float64)
            # p_L = R^T (p_o - t); row vectors => (p - t) @ R
            xyz = (xyz - t) @ R
            arr = np.column_stack([xyz, arr[:, 3], arr[:, 4]]).astype(np.float32)

        if self.voxel > 0:
            keys = np.floor(arr[:, :3] / self.voxel).astype(np.int32)
            _, idx = np.unique(keys, axis=0, return_index=True)
            arr = arr[np.sort(idx)]
        if len(arr) > self.max_points:
            step = int(np.ceil(len(arr) / self.max_points))
            arr = arr[::step]

        # Match Mid by cloud stamp; never force latest Mid onto old geometry.
        if stamp is None:
            stamp = msg.header.stamp
        if self.restamp_from_odom and self._mid_stamps:
            mid = self._nearest_mid_stamp(t_cloud)
            if mid is not None and mid[2] <= self.mid_restamp_max_dt:
                stamp = mid[1]
            # else keep cloud stamp (better than lying with latest Mid)
        pub_sec = _stamp_sec(stamp)
        # Enforce monotonic stamps for L2 buffer.
        if pub_sec + 1e-6 < self._last_pub_sec:
            stamp = self.get_clock().now().to_msg()
            pub_sec = _stamp_sec(stamp)
        if pub_sec < self._last_pub_sec + 1e-4:
            # Nudge forward by 1ms to keep strictly increasing.
            pub_sec = self._last_pub_sec + 1e-3
            sec = int(pub_sec)
            nsec = int((pub_sec - sec) * 1e9)
            stamp = type(stamp)(sec=sec, nanosec=nsec)
        self._last_pub_sec = pub_sec

        header = Header()
        header.stamp = stamp
        header.frame_id = 'odin_lidar'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='offset_time', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        packed = b''.join(
            struct.pack('<fffff', float(r[0]), float(r[1]), float(r[2]), float(r[3]), float(r[4]))
            for r in arr
        )
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = len(arr)
        out.fields = fields
        out.is_bigendian = False
        out.point_step = 20
        out.row_step = 20 * len(arr)
        out.is_dense = True
        out.data = packed
        self.pub.publish(out)
        self._n_pub += 1
        return True


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
