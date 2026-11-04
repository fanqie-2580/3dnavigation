#!/usr/bin/env python3
"""World-fixed dual-color accumulation for Mid360 + Odin.

Mid360 (cyan): /cloud_registered (FAST-LIO map cloud) — already map-fixed.

Odin1 (orange): default /cloud_odin_registered — same Mid-rate, IMU-undistorted
  map cloud as Mid. Do NOT re-pose /odin1/cloud_raw_ds with Mid odom: Odin is
  ~30 Hz while Mid odom is ~10 Hz; restamping many scans to one odom stamp paints
  the same pose onto rotating body-frame walls → orange fan on yaw.

Optional odin_mode:=raw keeps the old sensor×interp-odom path for debugging.
"""
from __future__ import annotations

import math
import struct
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger


def make_cloud(header: Header, xyz: np.ndarray) -> PointCloud2:
    if xyz.size == 0:
        xyz = np.zeros((0, 3), dtype=np.float32)
    else:
        xyz = xyz.astype(np.float32)
    return point_cloud2.create_cloud_xyz32(header, xyz)


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def se3_from_pose(pose: Pose) -> np.ndarray:
    p = pose.position
    q = pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def se3_from_rt(R9, t3) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R9, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t3, dtype=np.float64).reshape(3)
    return T


def slerp_R(R0: np.ndarray, R1: np.ndarray, a: float) -> np.ndarray:
    a = float(np.clip(a, 0.0, 1.0))
    if a <= 0.0:
        return R0.copy()
    if a >= 1.0:
        return R1.copy()
    Rrel = R0.T @ R1
    c = float(np.clip((np.trace(Rrel) - 1.0) * 0.5, -1.0, 1.0))
    ang = math.acos(c)
    if abs(ang) < 1e-9:
        return R0.copy()
    K = Rrel - Rrel.T
    axis = np.array([K[2, 1], K[0, 2], K[1, 0]], dtype=np.float64)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return R0.copy()
    axis /= n
    a2 = ang * a
    x, y, z = axis
    Kx = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)
    Rinc = np.eye(3) + math.sin(a2) * Kx + (1 - math.cos(a2)) * (Kx @ Kx)
    return R0 @ Rinc


class VoxelAccum:
    def __init__(self, voxel: float, max_points: int) -> None:
        self.voxel = float(voxel)
        self.max_points = int(max_points)
        self._grid: Dict[Tuple[int, int, int], Tuple[float, float, float]] = {}

    def add(self, pts: np.ndarray) -> int:
        if pts.size == 0:
            return 0
        v = self.voxel
        added = 0
        for x, y, z in pts:
            key = (int(np.floor(x / v)), int(np.floor(y / v)), int(np.floor(z / v)))
            if key in self._grid:
                continue
            self._grid[key] = (float(x), float(y), float(z))
            added += 1
            if len(self._grid) >= self.max_points:
                break
        return added

    def clear(self) -> None:
        self._grid.clear()

    def numpy(self) -> np.ndarray:
        if not self._grid:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray(list(self._grid.values()), dtype=np.float64)

    def __len__(self) -> int:
        return len(self._grid)


class DualLidarColorViz(Node):
    def __init__(self) -> None:
        super().__init__('dual_lidar_color_viz')
        self.declare_parameter('mid_topic', '/cloud_registered')
        # registered = map-fixed LIO split (default). raw = sensor × Mid odom (debug).
        self.declare_parameter('odin_mode', 'registered')
        self.declare_parameter('odin_registered_topic', '/cloud_odin_registered')
        self.declare_parameter('odin_raw_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mid_stride', 1)
        self.declare_parameter('odin_stride', 1)
        self.declare_parameter('odin_blind', 0.3)
        # Drop near-ground wedge so side-view does not look like a pitched cloud.
        # Level forward depth cams always paint a slanted floor sheet in XZ.
        self.declare_parameter('odin_z_min', 0.25)
        self.declare_parameter('odin_z_max', 100.0)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        self.declare_parameter('accum_skip_sec', 3.0)
        self.declare_parameter('odom_buffer_sec', 2.0)
        self.declare_parameter('max_odom_cloud_dt', 0.15)
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [0.152932, -0.032606, 0.080137])
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [
                0.96905401, -0.00098037, -0.24684685,
                0.00160252, 0.99999603, 0.00231950,
                0.24684359, -0.00264330, 0.96905173,
            ],
        )

        self.mid_topic = self.get_parameter('mid_topic').value
        self.odin_mode = str(self.get_parameter('odin_mode').value).strip().lower()
        self.odin_registered_topic = self.get_parameter('odin_registered_topic').value
        self.odin_raw_topic = self.get_parameter('odin_raw_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.mid_stride = max(1, int(self.get_parameter('mid_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))
        self.odin_blind2 = float(self.get_parameter('odin_blind').value) ** 2
        self.odin_z_min = float(self.get_parameter('odin_z_min').value)
        self.odin_z_max = float(self.get_parameter('odin_z_max').value)
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self.odom_buffer_sec = float(self.get_parameter('odom_buffer_sec').value)
        self.max_odom_cloud_dt = float(self.get_parameter('max_odom_cloud_dt').value)
        self._t0 = time.time()

        T_imu_mid = se3_from_rt(
            self.get_parameter('extrinsic_R').value,
            self.get_parameter('extrinsic_T').value,
        )
        T_mid_odin = se3_from_rt(
            self.get_parameter('extrinsic_R_L2_wrt_L1').value,
            self.get_parameter('extrinsic_T_L2_wrt_L1').value,
        )
        self.T_imu_odin = T_imu_mid @ T_mid_odin

        self._accum_mid = VoxelAccum(voxel, max_n)
        self._accum_odin = VoxelAccum(voxel, max_n)
        self._dirty_mid = False
        self._dirty_odin = False
        self._odom_buf: Deque[Tuple[float, np.ndarray]] = deque()

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.pub_accum_mid = self.create_publisher(PointCloud2, '/viz/accum_mid360', 1)
        self.pub_accum_odin = self.create_publisher(PointCloud2, '/viz/accum_odin', 1)

        self.create_subscription(PointCloud2, self.mid_topic, self.on_mid, 5)
        self.create_timer(max(0.2, period), self.publish_accum)
        self.create_service(Trigger, '~/clear_accum', self.on_clear_accum)

        if self.odin_mode == 'raw':
            self.create_subscription(Odometry, self.odom_topic, self.on_odom, 100)
            self.create_subscription(PointCloud2, self.odin_raw_topic, self.on_odin_raw, 5)
            self.get_logger().info(
                f'Mid={self.mid_topic}; Odin RAW={self.odin_raw_topic} x interp '
                f'{self.odom_topic} (debug path)'
            )
        else:
            self.create_subscription(
                PointCloud2, self.odin_registered_topic, self.on_odin_registered, 5
            )
            self.get_logger().info(
                f'Mid={self.mid_topic}; Odin REGISTERED={self.odin_registered_topic} '
                f'(map-fixed, no re-pose), voxel={voxel}'
            )

    def _accum_ready(self) -> bool:
        return (time.time() - self._t0) >= self.accum_skip_sec

    def on_clear_accum(self, _req, resp):
        self._accum_mid.clear()
        self._accum_odin.clear()
        self._dirty_mid = self._dirty_odin = True
        self._t0 = time.time()
        resp.success = True
        resp.message = 'cleared Mid/Odin voxel accum'
        return resp

    def _header(self, stamp, frame_id: str | None = None) -> Header:
        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id or self.map_frame
        return h

    def on_odom(self, msg: Odometry) -> None:
        t = stamp_sec(msg.header.stamp)
        T = se3_from_pose(msg.pose.pose)
        self._odom_buf.append((t, T))
        cutoff = t - self.odom_buffer_sec
        while self._odom_buf and self._odom_buf[0][0] < cutoff:
            self._odom_buf.popleft()

    def _interp_odom(self, t_cloud: float) -> Optional[np.ndarray]:
        if len(self._odom_buf) < 1:
            return None
        if t_cloud <= self._odom_buf[0][0]:
            if abs(t_cloud - self._odom_buf[0][0]) > self.max_odom_cloud_dt:
                return None
            return self._odom_buf[0][1].copy()
        if t_cloud >= self._odom_buf[-1][0]:
            if abs(t_cloud - self._odom_buf[-1][0]) > self.max_odom_cloud_dt:
                return None
            return self._odom_buf[-1][1].copy()
        for i in range(len(self._odom_buf) - 1):
            t0, T0 = self._odom_buf[i]
            t1, T1 = self._odom_buf[i + 1]
            if t0 <= t_cloud <= t1:
                if (t1 - t0) < 1e-6:
                    return T0.copy()
                a = (t_cloud - t0) / (t1 - t0)
                T = np.eye(4)
                T[:3, :3] = slerp_R(T0[:3, :3], T1[:3, :3], a)
                T[:3, 3] = (1 - a) * T0[:3, 3] + a * T1[:3, 3]
                return T
        return None

    def _xyz_stride(self, msg: PointCloud2, stride: int, blind2: float = 0.0) -> np.ndarray:
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return np.zeros((0, 3), dtype=np.float64)
        pts: list[list[float]] = []
        n = msg.width * msg.height
        k = 0
        for i in range(n):
            b = i * msg.point_step
            x, y, z = struct.unpack_from('fff', msg.data, b + offs['x'])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if blind2 > 0.0 and (x * x + y * y + z * z) < blind2:
                continue
            if (k % stride) == 0:
                pts.append([x, y, z])
            k += 1
        return np.asarray(pts, dtype=np.float64)

    def on_mid(self, msg: PointCloud2) -> None:
        arr = self._xyz_stride(msg, self.mid_stride)
        self.pub_mid.publish(
            make_cloud(self._header(msg.header.stamp, msg.header.frame_id), arr)
        )
        if self._accum_ready() and self._accum_mid.add(arr):
            self._dirty_mid = True

    def on_odin_registered(self, msg: PointCloud2) -> None:
        # Already in map frame (same pipeline as Mid /cloud_registered).
        arr = self._xyz_stride(msg, self.odin_stride)
        if arr.size == 0:
            return
        if self.odin_z_min > -50.0 or self.odin_z_max < 50.0:
            arr = arr[(arr[:, 2] >= self.odin_z_min) & (arr[:, 2] <= self.odin_z_max)]
            if arr.size == 0:
                return
        self.pub_odin.publish(
            make_cloud(self._header(msg.header.stamp, msg.header.frame_id or self.map_frame), arr)
        )
        if self._accum_ready() and self._accum_odin.add(arr):
            self._dirty_odin = True

    def on_odin_raw(self, msg: PointCloud2) -> None:
        t_cloud = stamp_sec(msg.header.stamp)
        T_map_imu = self._interp_odom(t_cloud)
        if T_map_imu is None:
            return
        arr = self._xyz_stride(msg, self.odin_stride, self.odin_blind2)
        if arr.size == 0:
            return

        T_map_odin = T_map_imu @ self.T_imu_odin
        R = T_map_odin[:3, :3]
        t = T_map_odin[:3, 3]
        world = (arr @ R.T) + t

        self.pub_odin.publish(make_cloud(self._header(msg.header.stamp), world))
        if self._accum_ready() and self._accum_odin.add(world):
            self._dirty_odin = True

    def publish_accum(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if self._dirty_mid or len(self._accum_mid):
            self.pub_accum_mid.publish(
                make_cloud(self._header(stamp), self._accum_mid.numpy())
            )
            self._dirty_mid = False
        if self._dirty_odin or len(self._accum_odin):
            self.pub_accum_odin.publish(
                make_cloud(self._header(stamp), self._accum_odin.numpy())
            )
            self._dirty_odin = False


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
