#!/usr/bin/env python3
"""Project delayed Odin cloud into Mid map with *exact* Mid /Odometry stamps.

Dock often forwards cloud_slam as /odin1/cloud_raw (frame_id=odom). Pipeline:

  p_L   = inv(T_odin_odom(t*) * T_imu_L) * p_odom     # unproject at t*
  p_map = T_map_body(t*) * T_body_L1 * T_L1_L2 * p_L  # Mid pose at same t*

t* is the Mid /Odometry stamp paired to the cloud — not TF interpolation at a
loose Odin stamp. Pairing rule (strict):

  |t_cloud - t_mid|  <= mid_pair_max_dt
  |t_cloud - t_odin| <= odin_pair_max_dt   (only if cloud is odom-frame)

Output header.stamp is *identical* to the Mid /Odometry stamp used. Clouds that
cannot be paired within the dt gates are dropped (no wrong-pose paint on yaw).
"""
from __future__ import annotations

import math
import struct
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def mat_from_odom(msg: Odometry) -> np.ndarray:
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


def se3(Rflat, Txyz) -> np.ndarray:
    R = np.asarray(Rflat, dtype=np.float64).reshape(3, 3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(Txyz, dtype=np.float64).reshape(3)
    return T


def yaw_only_pose(T: np.ndarray) -> np.ndarray:
    yaw = math.atan2(T[1, 0], T[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.eye(4, dtype=np.float64)
    out[0, 0] = c
    out[0, 1] = -s
    out[1, 0] = s
    out[1, 1] = c
    out[:3, 3] = T[:3, 3]
    return out


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_key(stamp) -> Tuple[int, int]:
    return (int(stamp.sec), int(stamp.nanosec))


class OdinMidTimedProject(Node):
    def __init__(self) -> None:
        super().__init__('odin_mid_timed_project')
        self.declare_parameter('cloud_topic', '/odin1/cloud_raw')
        self.declare_parameter('out_topic', '/odin1/cloud_in_map')
        self.declare_parameter('odin_odom_topic', '/odin1/odometry')
        self.declare_parameter('mid_odom_topic', '/Odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'body')
        # Strict pairing gates (seconds). Mid is ~10 Hz → half-period ≈ 50 ms.
        # Pose + Odin-odom + output stamp all use the *same* Mid /Odometry time.
        self.declare_parameter('mid_pair_max_dt', 0.05)
        self.declare_parameter('odin_pair_max_dt', 0.05)
        # Drop only extreme backlog vs newest Mid. Residual dock lag is ~1–2s;
        # do NOT set this to 0.25 (that kills EXACT viz). Accum age-gate is separate.
        self.declare_parameter('max_cloud_age_sec', 3.0)
        self.declare_parameter('cloud_buffer_sec', 8.0)
        self.declare_parameter('odom_buffer_sec', 10.0)
        self.declare_parameter('stride', 2)
        self.declare_parameter('blind', 0.3)
        self.declare_parameter('min_confidence', 30.0)
        self.declare_parameter('cloud_in_odom', 'auto')
        self.declare_parameter('use_yaw_only_body', False)
        self.declare_parameter('use_yaw_only_odin_odom', False)
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [0.12157888, 0.00549610, 0.03169932])
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [
                0.97437006, 0.0, -0.22495105,
                0.0, 1.0, 0.0,
                0.22495105, 0.0, 0.97437006,
            ],
        )
        self.declare_parameter('extrinsic_T_imu_L', [-0.02663, 0.03447, 0.02174])
        self.declare_parameter(
            'extrinsic_R_imu_L', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.mid_pair_max_dt = float(self.get_parameter('mid_pair_max_dt').value)
        self.odin_pair_max_dt = float(self.get_parameter('odin_pair_max_dt').value)
        self.max_cloud_age_sec = float(self.get_parameter('max_cloud_age_sec').value)
        self.cloud_buffer_sec = float(self.get_parameter('cloud_buffer_sec').value)
        self.odom_buffer_sec = float(self.get_parameter('odom_buffer_sec').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.blind2 = float(self.get_parameter('blind').value) ** 2
        self.min_conf = float(self.get_parameter('min_confidence').value)
        self.use_yaw_only = bool(self.get_parameter('use_yaw_only_body').value)
        self.use_yaw_only_odin = bool(self.get_parameter('use_yaw_only_odin_odom').value)
        mode = str(self.get_parameter('cloud_in_odom').value).strip().lower()
        self.cloud_in_odom_mode = mode

        T_body_L1 = se3(
            self.get_parameter('extrinsic_R').value,
            self.get_parameter('extrinsic_T').value,
        )
        T_L1_L2 = se3(
            self.get_parameter('extrinsic_R_L2_wrt_L1').value,
            self.get_parameter('extrinsic_T_L2_wrt_L1').value,
        )
        self.T_body_odin = T_body_L1 @ T_L1_L2
        self.T_imu_L = se3(
            self.get_parameter('extrinsic_R_imu_L').value,
            self.get_parameter('extrinsic_T_imu_L').value,
        )

        # (t_sec, stamp_msg, T_map_body)
        self._mid_odom: Deque[Tuple[float, object, np.ndarray]] = deque()
        # (t_sec, T_odin_odom)
        self._odin_odom: Deque[Tuple[float, np.ndarray]] = deque()
        # (t_sec, stamp_msg, PointCloud2)
        self._cloud_buf: Deque[Tuple[float, object, PointCloud2]] = deque()
        self._used_cloud_keys: Deque[Tuple[int, int]] = deque(maxlen=500)
        self._used_mid_keys: Deque[Tuple[int, int]] = deque(maxlen=500)

        self._ok = 0
        self._drop_mid = 0
        self._drop_odin = 0
        self._drop_empty = 0
        self._drop_age = 0
        self._sum_dt_mid = 0.0
        self._sum_dt_odin = 0.0

        reliable = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        out = str(self.get_parameter('out_topic').value)
        inn = str(self.get_parameter('cloud_topic').value)
        odin_odom_topic = str(self.get_parameter('odin_odom_topic').value)
        mid_odom_topic = str(self.get_parameter('mid_odom_topic').value)
        self.pub = self.create_publisher(PointCloud2, out, reliable)
        self.create_subscription(PointCloud2, inn, self.on_cloud, reliable)
        self.create_subscription(Odometry, odin_odom_topic, self.on_odin_odom, reliable)
        self.create_subscription(Odometry, mid_odom_topic, self.on_mid_odom, reliable)
        self.create_timer(2.0, self._stats)
        self.get_logger().info(
            f'Odin→Mid-map EXACT-stamp project: {inn} → {out}, '
            f'mid_odom={mid_odom_topic} odin_odom={odin_odom_topic} '
            f'mid_dt<={self.mid_pair_max_dt:.3f}s odin_dt<={self.odin_pair_max_dt:.3f}s '
            f'max_age={self.max_cloud_age_sec:.2f}s cloud_in_odom={self.cloud_in_odom_mode} '
            f'(output stamp == Mid /Odometry stamp)'
        )

    def _trim(self, buf: deque, newest_t: float, horizon: float) -> None:
        while buf and (newest_t - buf[0][0]) > horizon:
            buf.popleft()

    def _stats(self) -> None:
        total = self._ok + self._drop_mid + self._drop_odin + self._drop_empty + self._drop_age
        if total == 0:
            return
        avg_mid = self._sum_dt_mid / self._ok if self._ok else 0.0
        avg_odin = self._sum_dt_odin / self._ok if self._ok else 0.0
        self.get_logger().info(
            f'timed_project ok={self._ok} drop_mid={self._drop_mid} '
            f'drop_odin={self._drop_odin} drop_age={self._drop_age} empty={self._drop_empty} '
            f'avg|dt_mid|={avg_mid * 1000:.1f}ms avg|dt_odin|={avg_odin * 1000:.1f}ms '
            f'buf_cloud={len(self._cloud_buf)} buf_mid={len(self._mid_odom)}'
        )
        self._ok = 0
        self._drop_mid = 0
        self._drop_odin = 0
        self._drop_empty = 0
        self._drop_age = 0
        self._sum_dt_mid = 0.0
        self._sum_dt_odin = 0.0

    def on_odin_odom(self, msg: Odometry) -> None:
        t = stamp_sec(msg.header.stamp)
        self._odin_odom.append((t, mat_from_odom(msg)))
        self._trim(self._odin_odom, t, self.odom_buffer_sec)

    def on_mid_odom(self, msg: Odometry) -> None:
        t = stamp_sec(msg.header.stamp)
        self._mid_odom.append((t, msg.header.stamp, mat_from_odom(msg)))
        self._trim(self._mid_odom, t, self.odom_buffer_sec)
        # Mid tick drives matching so delayed Odin can catch up to historical Mid.
        self._try_match()

    def on_cloud(self, msg: PointCloud2) -> None:
        t = stamp_sec(msg.header.stamp)
        self._cloud_buf.append((t, msg.header.stamp, msg))
        self._trim(self._cloud_buf, t, self.cloud_buffer_sec)
        self._try_match()

    def _nearest_mid(self, t_cloud: float):
        best = None
        best_dt = 1e9
        for t, stamp, T in self._mid_odom:
            dt = abs(t - t_cloud)
            if dt < best_dt:
                best_dt = dt
                best = (t, stamp, T, dt)
        return best

    def _nearest_odin(self, t_cloud: float):
        best = None
        best_dt = 1e9
        for t, T in self._odin_odom:
            dt = abs(t - t_cloud)
            if dt < best_dt:
                best_dt = dt
                best = (t, T, dt)
        return best

    def _cloud_is_in_odom(self, msg: PointCloud2) -> bool:
        if self.cloud_in_odom_mode in ('1', 'true', 'yes', 'on'):
            return True
        if self.cloud_in_odom_mode in ('0', 'false', 'no', 'off'):
            return False
        fid = (msg.header.frame_id or '').strip().lower()
        return fid in ('odom', 'map', 'odin_odom', 'odin_map')

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

    def _try_match(self) -> None:
        if not self._cloud_buf or not self._mid_odom:
            return

        # Process oldest clouds first so delayed frames still bind to historical Mid.
        pending = list(self._cloud_buf)
        self._cloud_buf.clear()
        newest_mid = self._mid_odom[-1][0]
        for t_cloud, cloud_stamp, msg in pending:
            ckey = stamp_key(cloud_stamp)
            if ckey in self._used_cloud_keys:
                continue

            # Too old vs live Mid → would paint a delayed turn; drop.
            if self.max_cloud_age_sec > 0.0 and (newest_mid - t_cloud) > self.max_cloud_age_sec:
                self._drop_age += 1
                continue

            mid = self._nearest_mid(t_cloud)
            if mid is None or mid[3] > self.mid_pair_max_dt:
                newest_mid = self._mid_odom[-1][0]
                oldest_mid = self._mid_odom[0][0]
                # Cloud still in the future relative to Mid → wait.
                if t_cloud > newest_mid + self.mid_pair_max_dt:
                    self._cloud_buf.append((t_cloud, cloud_stamp, msg))
                    continue
                # Mid has already passed this cloud with no tight pair → drop.
                if t_cloud < oldest_mid - self.mid_pair_max_dt or newest_mid > t_cloud + self.mid_pair_max_dt:
                    self._drop_mid += 1
                    continue
                self._cloud_buf.append((t_cloud, cloud_stamp, msg))
                continue

            t_mid, mid_stamp, T_mb, dt_mid = mid
            mkey = stamp_key(mid_stamp)
            if mkey in self._used_mid_keys:
                # Already painted one Odin cloud for this Mid tick; drop extras.
                self._drop_mid += 1
                continue

            need_unproj = self._cloud_is_in_odom(msg)
            dt_odin = 0.0
            T_oi = None
            if need_unproj:
                # cloud_slam was generated at t_cloud → unproject with Odin odom @ t_cloud.
                # Mid pose is the paired Mid odom (@ t_mid≈t_cloud). Both gates ≤50ms.
                od = self._nearest_odin(t_cloud)
                if od is None or od[2] > self.odin_pair_max_dt:
                    if self._odin_odom and t_cloud > self._odin_odom[-1][0] + self.odin_pair_max_dt:
                        self._cloud_buf.append((t_cloud, cloud_stamp, msg))
                        continue
                    if (not self._odin_odom) or t_cloud > self._odin_odom[-1][0] - 0.5:
                        self._cloud_buf.append((t_cloud, cloud_stamp, msg))
                        continue
                    self._drop_odin += 1
                    continue
                _t_o, T_oi, dt_odin = od
                if abs(t_mid - _t_o) > self.mid_pair_max_dt:
                    self._drop_odin += 1
                    continue

            arr = self._xyz(msg)
            if arr.size == 0:
                self._drop_empty += 1
                continue

            if need_unproj and T_oi is not None:
                if self.use_yaw_only_odin:
                    T_oi = yaw_only_pose(T_oi)
                T_oL = T_oi @ self.T_imu_L
                R = T_oL[:3, :3]
                t = T_oL[:3, 3]
                # p_L = R^T (p_o - t); row vectors => (p - t) @ R
                arr = (arr - t) @ R

            if self.use_yaw_only:
                T_mb = yaw_only_pose(T_mb)
            T = T_mb @ self.T_body_odin
            world = (arr @ T[:3, :3].T) + T[:3, 3]

            header = Header()
            # Identical to Mid /Odometry stamp used for the pose.
            header.stamp = mid_stamp
            header.frame_id = self.map_frame
            self.pub.publish(
                point_cloud2.create_cloud_xyz32(header, world.astype(np.float32))
            )
            self._used_cloud_keys.append(ckey)
            self._used_mid_keys.append(mkey)
            self._ok += 1
            self._sum_dt_mid += dt_mid
            self._sum_dt_odin += dt_odin


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
