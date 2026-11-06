#!/usr/bin/env python3
"""Project Odin cloud_slam into Mid map for *live* viz (no catch-up lag).

Dock Odin stamps are typically ~4s behind host, but the cloud that just arrived
is effectively current. Pairing Mid /Odometry at the raw Odin stamp therefore
uses a Mid pose from seconds ago → live cloud lags the robot, then "catches up"
and can overshoot / jump as a backlog of historical Mid poses is flushed.

Live policy (this node):
  p_L   = inv(T_odin_odom(t_cloud) * T_imu_L) * p_odom   # Odin timeline
  p_map = T_map_body_LATEST * T_body_odin * p_L          # newest Mid pose
  header.stamp = Mid_LATEST.stamp                        # identical Mid stamp

Only the newest Odin cloud is kept (no multi-second backlog). One publish per
Mid /Odometry tick. Accumulation gating (no paint while turning / jumping) is
in mid_odin_frame_fuse.
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
        self.declare_parameter('odin_pair_max_dt', 0.15)
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
        self.odin_pair_max_dt = float(self.get_parameter('odin_pair_max_dt').value)
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

        self._mid_latest: Optional[Tuple[object, np.ndarray]] = None  # stamp, T
        self._odin_odom: Deque[Tuple[float, np.ndarray]] = deque()
        self._pending_cloud: Optional[PointCloud2] = None
        self._last_pub_mid_key: Optional[Tuple[int, int]] = None

        self._ok = 0
        self._drop_mid = 0
        self._drop_odin = 0
        self._drop_empty = 0

        reliable = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        out = str(self.get_parameter('out_topic').value)
        inn = str(self.get_parameter('cloud_topic').value)
        self.pub = self.create_publisher(PointCloud2, out, reliable)
        self.create_subscription(PointCloud2, inn, self.on_cloud, reliable)
        self.create_subscription(
            Odometry, str(self.get_parameter('odin_odom_topic').value), self.on_odin_odom, reliable
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('mid_odom_topic').value), self.on_mid_odom, reliable
        )
        self.create_timer(2.0, self._stats)
        self.get_logger().info(
            f'Odin→Mid-map LIVE project: {inn} → {out} '
            f'(latest Mid pose × newest Odin cloud; no historical catch-up)'
        )

    def _trim_odin(self, newest_t: float) -> None:
        while self._odin_odom and (newest_t - self._odin_odom[0][0]) > self.odom_buffer_sec:
            self._odin_odom.popleft()

    def _stats(self) -> None:
        total = self._ok + self._drop_mid + self._drop_odin + self._drop_empty
        if total == 0:
            return
        self.get_logger().info(
            f'timed_project ok={self._ok} drop_mid={self._drop_mid} '
            f'drop_odin={self._drop_odin} empty={self._drop_empty} '
            f'pending={"Y" if self._pending_cloud is not None else "N"}'
        )
        self._ok = 0
        self._drop_mid = 0
        self._drop_odin = 0
        self._drop_empty = 0

    def on_odin_odom(self, msg: Odometry) -> None:
        t = stamp_sec(msg.header.stamp)
        self._odin_odom.append((t, mat_from_odom(msg)))
        self._trim_odin(t)

    def on_mid_odom(self, msg: Odometry) -> None:
        self._mid_latest = (msg.header.stamp, mat_from_odom(msg))
        self._try_publish()

    def on_cloud(self, msg: PointCloud2) -> None:
        # Keep only the newest cloud — never flush a seconds-long backlog onto
        # historical Mid poses (that caused lag → catch-up → overshoot/jump).
        self._pending_cloud = msg
        self._try_publish()

    def _nearest_odin(self, t_odin: float):
        best = None
        best_dt = 1e9
        for t, T in self._odin_odom:
            dt = abs(t - t_odin)
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

    def _try_publish(self) -> None:
        if self._pending_cloud is None or self._mid_latest is None:
            if self._pending_cloud is not None and self._mid_latest is None:
                self._drop_mid += 1
            return

        mid_stamp, T_mb = self._mid_latest
        mkey = stamp_key(mid_stamp)
        if mkey == self._last_pub_mid_key:
            return  # wait for a new Mid tick

        msg = self._pending_cloud
        need_unproj = self._cloud_is_in_odom(msg)
        T_oi = None
        if need_unproj:
            od = self._nearest_odin(stamp_sec(msg.header.stamp))
            if od is None or od[2] > self.odin_pair_max_dt:
                self._drop_odin += 1
                return
            _t_o, T_oi, _dt = od

        arr = self._xyz(msg)
        if arr.size == 0:
            self._drop_empty += 1
            self._pending_cloud = None
            return

        if need_unproj and T_oi is not None:
            if self.use_yaw_only_odin:
                T_oi = yaw_only_pose(T_oi)
            T_oL = T_oi @ self.T_imu_L
            R = T_oL[:3, :3]
            t = T_oL[:3, 3]
            arr = (arr - t) @ R

        if self.use_yaw_only:
            T_mb = yaw_only_pose(T_mb)
        T = T_mb @ self.T_body_odin
        world = (arr @ T[:3, :3].T) + T[:3, 3]

        header = Header()
        header.stamp = mid_stamp
        header.frame_id = self.map_frame
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, world.astype(np.float32)))

        self._last_pub_mid_key = mkey
        self._pending_cloud = None  # consumed; next cloud replaces
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
