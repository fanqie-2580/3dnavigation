#!/usr/bin/env python3
"""Prepare Odin1 cloud for FAST_LIO_MULTI L2 (Mid360+Odin1 tight coupling).

Pipeline:
  1) If frame_id is odom/map (dock often forwards cloud_slam as cloud_raw),
     unproject to lidar frame with /odin1/odometry * T_imu_L.
  2) Confidence filter when field exists.
  3) Voxel downsample; keep offset_time (s) when present (else 0).
  4) Optional ground-only keep (RANSAC plane in lidar frame) for L2 densify.
  5) Restamp only to a Mid /Odometry stamp within mid_restamp_max_dt of the
     *cloud* stamp (never latest Mid while geometry is seconds old).
     Always enforce monotonic stamps to avoid L2 "loop back".
  6) Rate-limit ~one cloud per Mid odom tick when Mid odom is flowing;
     otherwise publish every Nth input (~10 Hz).
  7) Drop clouds older than max_cloud_age_sec vs newest Mid (stale TCP backlog).
"""
from __future__ import annotations

import math
import struct
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _extract_ground(
    arr: np.ndarray,
    distance: float,
    max_angle_deg: float,
    ransac_iters: int,
    min_inliers: int,
    fit_z_span: float,
    z_band: float,
    rng: np.random.Generator,
) -> Tuple[np.ndarray, str]:
    """Keep ground points from Nx5 (xyz,intensity,offset_time) in lidar frame.

    Primary: RANSAC plane with normal near +Z. Fallback: low-Z band around the
    10th percentile (keeps L2 fed when plane fit fails indoors).
    """
    if arr.size == 0 or len(arr) < 3:
        return arr, 'empty'

    xyz = arr[:, :3].astype(np.float64)
    z = xyz[:, 2]
    z_lo = float(np.percentile(z, 10))
    # Fit on near-floor band so walls/ceiling do not dominate.
    fit_mask = z <= (z_lo + max(0.2, float(fit_z_span)))
    cand = xyz[fit_mask]
    if len(cand) < 3:
        cand = xyz
        fit_mask = np.ones(len(xyz), dtype=bool)

    cos_max = math.cos(math.radians(float(max_angle_deg)))
    best_count = 0
    best_n = None
    best_d = 0.0
    n_cand = len(cand)
    for _ in range(max(1, int(ransac_iters))):
        idx = rng.choice(n_cand, size=3, replace=False)
        p0, p1, p2 = cand[idx]
        n = np.cross(p1 - p0, p2 - p0)
        nn = float(np.linalg.norm(n))
        if nn < 1e-9:
            continue
        n = n / nn
        if n[2] < 0.0:
            n = -n
        if float(n[2]) < cos_max:
            continue
        d = -float(n.dot(p0))
        dist = np.abs(cand @ n + d)
        count = int(np.count_nonzero(dist < distance))
        if count > best_count:
            best_count = count
            best_n = n
            best_d = d

    if best_n is not None and best_count >= int(min_inliers):
        dist_all = np.abs(xyz @ best_n + best_d)
        keep = dist_all < distance
        # Prefer the lower side of the slab (reject ceiling parallel misfits).
        signed = xyz @ best_n + best_d
        keep &= signed < (distance * 1.5)
        if int(np.count_nonzero(keep)) >= max(20, int(min_inliers) // 2):
            return arr[keep], f'ransac:{int(np.count_nonzero(keep))}/{len(arr)}'

    # Fallback: keep a thin band above the low-Z percentile.
    keep = (z >= (z_lo - 0.05)) & (z <= (z_lo + max(0.05, float(z_band))))
    if not np.any(keep):
        keep = z <= (z_lo + max(0.05, float(z_band)))
    return arr[keep], f'zband:{int(np.count_nonzero(keep))}/{len(arr)}'


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
        # Plan A: keep only Odin ground for L2 / densify.
        self.declare_parameter('ground_only', False)
        self.declare_parameter('ground_topic', '/odin1/cloud_ground')
        self.declare_parameter('publish_ground_topic', True)
        self.declare_parameter('publish_full_topic', '')  # empty = off; e.g. /odin1/cloud_raw_ds_full
        self.declare_parameter('ground_distance', 0.08)
        self.declare_parameter('ground_max_angle_deg', 25.0)
        self.declare_parameter('ground_ransac_iters', 60)
        self.declare_parameter('ground_min_inliers', 80)
        self.declare_parameter('ground_fit_z_span', 1.5)
        self.declare_parameter('ground_z_band', 0.20)

        self.voxel = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.restamp_from_odom = bool(self.get_parameter('restamp_from_odom').value)
        self.one_cloud_per_odom = bool(self.get_parameter('one_cloud_per_odom').value)
        self.odin_odom_max_dt = float(self.get_parameter('odin_odom_max_dt').value)
        self.mid_restamp_max_dt = float(self.get_parameter('mid_restamp_max_dt').value)
        self.max_cloud_age_sec = float(self.get_parameter('max_cloud_age_sec').value)
        self.fallback_period = float(self.get_parameter('fallback_period').value)
        self.ground_only = bool(self.get_parameter('ground_only').value)
        self.publish_ground_topic = bool(self.get_parameter('publish_ground_topic').value)
        self.publish_full_topic = str(self.get_parameter('publish_full_topic').value or '').strip()
        self.ground_distance = float(self.get_parameter('ground_distance').value)
        self.ground_max_angle_deg = float(self.get_parameter('ground_max_angle_deg').value)
        self.ground_ransac_iters = int(self.get_parameter('ground_ransac_iters').value)
        self.ground_min_inliers = int(self.get_parameter('ground_min_inliers').value)
        self.ground_fit_z_span = float(self.get_parameter('ground_fit_z_span').value)
        self.ground_z_band = float(self.get_parameter('ground_z_band').value)
        self._rng = np.random.default_rng(0)
        t_il = list(self.get_parameter('extrinsic_T_imu_L').value)
        self.T_imu_L = np.eye(4, dtype=np.float64)
        self.T_imu_L[:3, 3] = np.asarray(t_il, dtype=np.float64).reshape(3)

        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        ground_topic = str(self.get_parameter('ground_topic').value)
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
        self._n_ground_pub = 0
        self._n_skip_unproj = 0
        self._n_skip_empty = 0
        self._n_skip_age = 0
        self._n_skip_ground = 0
        self._last_ground_mode = '-'
        self._last_n_full = 0
        self._last_n_ground = 0
        self._last_mid_wall = -1.0
        # If Mid /Odometry stalls (bundle waiting on L2), keep publishing L2 with
        # cloud stamps so laserMapping can exit the Mid↔L2 chicken-egg deadlock.
        self.declare_parameter('mid_stall_fallback_sec', 0.5)
        self.mid_stall_fallback_sec = float(self.get_parameter('mid_stall_fallback_sec').value)

        reliable = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(PointCloud2, out, reliable)
        self.pub_ground = None
        if self.publish_ground_topic and ground_topic and ground_topic != out:
            self.pub_ground = self.create_publisher(PointCloud2, ground_topic, reliable)
        self.pub_full = None
        if self.publish_full_topic:
            self.pub_full = self.create_publisher(PointCloud2, self.publish_full_topic, reliable)
        self.sub = self.create_subscription(PointCloud2, inn, self.cb, reliable)
        self.create_subscription(
            Odometry, self.get_parameter('odin_odom_topic').value, self.on_odin_odom, reliable
        )
        if self.restamp_from_odom or self.one_cloud_per_odom:
            self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value, self.on_mid_odom, reliable
            )
        # Latest-cloud publisher: avoid processing a backlog of stale frames.
        self.create_timer(0.02, self._poll_pending)
        self.create_timer(2.0, self._status)
        self.get_logger().info(
            f'downsample {inn} -> {out} voxel={self.voxel}m max={self.max_points} '
            f'conf>={self.min_confidence} restamp={self.restamp_from_odom} '
            f'mid_dt<={self.mid_restamp_max_dt:.3f}s max_age={self.max_cloud_age_sec:.2f}s '
            f'one_per_odom={self.one_cloud_per_odom} ground_only={self.ground_only} '
            f'ground_topic={ground_topic if self.publish_ground_topic else "off"} '
            f'full_topic={self.publish_full_topic or "off"} qos=RELIABLE '
            f'(unproject odom-frame if needed; match Mid by cloud stamp)'
        )

    def _status(self) -> None:
        self.get_logger().info(
            f'stats cloud={self._n_cloud} odin_odom={self._n_odin} mid_odom={self._n_mid} '
            f'pub={self._n_pub} ground_pub={self._n_ground_pub} '
            f'last_full={self._last_n_full} last_ground={self._last_n_ground} '
            f'mode={self._last_ground_mode} skip_ground={self._n_skip_ground} '
            f'skip_unproj={self._n_skip_unproj} skip_empty={self._n_skip_empty} '
            f'skip_age={self._n_skip_age} pending={"Y" if self._pending is not None else "N"}'
        )

    def _make_cloud(self, header: Header, arr: np.ndarray) -> PointCloud2:
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='offset_time', offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        # Vectorized pack — Python struct.pack-per-point was ~100ms on 8k pts.
        if arr.size == 0:
            packed = b''
            width = 0
        else:
            pts = np.ascontiguousarray(arr[:, :5], dtype=np.float32)
            packed = pts.tobytes(order='C')
            width = int(pts.shape[0])
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = width
        out.fields = fields
        out.is_bigendian = False
        out.point_step = 20
        out.row_step = 20 * width
        out.is_dense = True
        out.data = packed
        return out

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
        self._last_mid_wall = self.get_clock().now().nanoseconds * 1e-9
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

    def _mid_stalled(self) -> bool:
        if self._last_mid_wall < 0.0:
            return True
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - self._last_mid_wall) > self.mid_stall_fallback_sec

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
        if best is not None and best_dt <= self.odin_odom_max_dt:
            return best
        # Restamped cloud/odom can desync briefly after TCP stalls; better to
        # use the newest odom than drop the whole scan (which looked like lag).
        return self._odin_odom[-1][1]

    def _cloud_in_odom(self, msg: PointCloud2) -> bool:
        fid = (msg.header.frame_id or '').strip().lower()
        return fid in ('odom', 'map', 'odin_odom', 'odin_map')

    def cb(self, msg: PointCloud2) -> None:
        """Keep only the newest cloud. Processing backlog was the main Odin lag
        (~250ms): each frame ~80ms, so a queued 3-frame backlog looked like delay
        vs Mid/robot motion. Overwrite pending and let the poll timer publish.
        """
        self._n_cloud += 1
        self._pending = msg

    def _poll_pending(self) -> None:
        msg = self._pending
        if msg is None:
            return
        # Drop if we are still inside fallback_period (rate limit).
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_pub_sec >= 0.0 and (now_sec - self._last_pub_sec) < self.fallback_period:
            return
        self._pending = None

        t_cloud = _stamp_sec(msg.header.stamp)
        if self._mid_stamps and self.max_cloud_age_sec > 0.0:
            newest_mid = self._mid_stamps[-1][0]
            if (newest_mid - t_cloud) > self.max_cloud_age_sec:
                self._n_skip_age += 1
                return

        if (
            self.one_cloud_per_odom
            and self._mid_odom_stamp is not None
            and not self._mid_stalled()
        ):
            # Align publish cadence to Mid odom when requested.
            if self._mid_odom_key is None or self._last_pub_key == self._mid_odom_key:
                self._pending = msg  # wait for a new Mid tick
                return
            if self._publish_from_msg(msg, None):
                self._last_pub_key = self._mid_odom_key
            return

        self._publish_from_msg(msg, msg.header.stamp)

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

        # Pre-stride before unique: np.unique on 35k×3 int keys was ~100ms.
        if len(arr) > self.max_points * 3:
            step0 = int(np.ceil(len(arr) / float(self.max_points * 3)))
            arr = arr[::step0]
        if self.voxel > 0 and len(arr) > 0:
            keys = np.ascontiguousarray(
                np.floor(arr[:, :3] / self.voxel).astype(np.int32)
            )
            # Structured-view unique is much faster than unique(axis=0).
            flat = keys.view(
                [('', keys.dtype), ('', keys.dtype), ('', keys.dtype)]
            ).reshape(-1)
            _, idx = np.unique(flat, return_index=True)
            arr = arr[np.sort(idx)]
        if len(arr) > self.max_points:
            step = int(np.ceil(len(arr) / self.max_points))
            arr = arr[::step]

        self._last_n_full = int(len(arr))
        ground = arr
        ground_mode = 'off'
        if self.ground_only or self.pub_ground is not None:
            ground, ground_mode = _extract_ground(
                arr,
                distance=self.ground_distance,
                max_angle_deg=self.ground_max_angle_deg,
                ransac_iters=self.ground_ransac_iters,
                min_inliers=self.ground_min_inliers,
                fit_z_span=self.ground_fit_z_span,
                z_band=self.ground_z_band,
                rng=self._rng,
            )
        self._last_ground_mode = ground_mode
        self._last_n_ground = int(len(ground))

        out_arr = ground if self.ground_only else arr
        if out_arr.size == 0 or len(out_arr) == 0:
            self._n_skip_ground += 1
            return False

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

        # L2 input keeps Mid-restamped time (laserMapping sync).
        self.pub.publish(self._make_cloud(header, out_arr))
        self._n_pub += 1
        if self.pub_ground is not None and len(ground) > 0:
            # B1 densify pairs with /odin1/odometry → keep original cloud stamp.
            g_header = Header()
            g_header.stamp = msg.header.stamp
            g_header.frame_id = 'odin_lidar'
            self.pub_ground.publish(self._make_cloud(g_header, ground))
            self._n_ground_pub += 1
        if self.pub_full is not None and len(arr) > 0:
            self.pub_full.publish(self._make_cloud(header, arr))
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
