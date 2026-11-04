#!/usr/bin/env python3
"""World-fixed dual-color accumulation for Mid360 + Odin.

Mid360 (cyan): /cloud_registered — already map-fixed by FAST-LIO.

Odin1 (orange): default odin_mode:=imu_prop
  Soft-sync for mismatched clocks (Odin uptime vs Mid/ROS time):
  - ignore Odin header stamp for posing
  - use receive-time on the Mid/ROS clock
  - take latest Mid /Odometry, then propagate with /livox/imu gyro to receive time
  - apply L2 extrinsic once, voxel-accum
  This is the usual soft-sync idea (MILIOM / MA-LIO style), without needing PPS.

Other modes:
  odin_own  — pose with /odin1/odometry (same as odin_odom_mapper), lock into Mid map once
  odom_sync — handshake Mid odom × raw (old; fails when clocks disagree)
  registered / raw — debug
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
from sensor_msgs.msg import Imu, PointCloud2
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


def rotvec_to_R(w: np.ndarray) -> np.ndarray:
    """Rodrigues: rotation vector (rad) -> R."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array(
        [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], dtype=np.float64
    )
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


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
        # imu_prop | odin_own | odom_sync | registered | raw
        self.declare_parameter('odin_mode', 'imu_prop')
        self.declare_parameter('odin_registered_topic', '/cloud_odin_registered')
        self.declare_parameter('odin_raw_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('odin_odom_topic', '/odin1/odometry')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mid_stride', 1)
        self.declare_parameter('odin_stride', 1)
        self.declare_parameter('odin_blind', 0.3)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        self.declare_parameter('accum_skip_sec', 3.0)
        self.declare_parameter('max_imu_prop_sec', 0.25)
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
        # Odin body extrinsic (same defaults as odin_odom_mapper) for odin_own mode
        self.declare_parameter('odin_imu_to_lidar_xyz', [-0.004, 0.013, 0.026])
        self.declare_parameter('odin_imu_to_lidar_rpy_deg', [0.66, -0.11, -0.29])

        self.mid_topic = self.get_parameter('mid_topic').value
        self.odin_mode = str(self.get_parameter('odin_mode').value).strip().lower()
        self.odin_registered_topic = self.get_parameter('odin_registered_topic').value
        self.odin_raw_topic = self.get_parameter('odin_raw_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odin_odom_topic = self.get_parameter('odin_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.mid_stride = max(1, int(self.get_parameter('mid_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))
        self.odin_blind2 = float(self.get_parameter('odin_blind').value) ** 2
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self.max_imu_prop_sec = float(self.get_parameter('max_imu_prop_sec').value)
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

        xyz = list(self.get_parameter('odin_imu_to_lidar_xyz').value)
        rpy = [math.radians(float(v)) for v in self.get_parameter('odin_imu_to_lidar_rpy_deg').value]
        cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
        cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        self.T_odin_imu_lidar = np.eye(4)
        self.T_odin_imu_lidar[:3, :3] = Rz @ Ry @ Rx
        self.T_odin_imu_lidar[:3, 3] = xyz

        self._accum_mid = VoxelAccum(voxel, max_n)
        self._accum_odin = VoxelAccum(voxel, max_n)
        self._dirty_mid = False
        self._dirty_odin = False
        self._posed_count = 0

        self._mid_odom: Deque[Tuple[float, np.ndarray]] = deque(maxlen=200)
        self._imu: Deque[Tuple[float, np.ndarray]] = deque(maxlen=2000)
        self._odin_odom: Optional[np.ndarray] = None
        self._T_align: Optional[np.ndarray] = None  # map <- odin_odom

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.pub_accum_mid = self.create_publisher(PointCloud2, '/viz/accum_mid360', 1)
        self.pub_accum_odin = self.create_publisher(PointCloud2, '/viz/accum_odin', 1)

        self.create_subscription(PointCloud2, self.mid_topic, self.on_mid, 5)
        self.create_timer(max(0.2, period), self.publish_accum)
        self.create_service(Trigger, '~/clear_accum', self.on_clear_accum)

        if self.odin_mode == 'imu_prop':
            self.create_subscription(Odometry, self.odom_topic, self.on_mid_odom, 100)
            self.create_subscription(Imu, self.imu_topic, self.on_imu, 200)
            self.create_subscription(PointCloud2, self.odin_raw_topic, self.on_odin_imu_prop, 10)
            self.get_logger().info(
                f'Mid={self.mid_topic}; Odin IMU_PROP raw={self.odin_raw_topic} '
                f'+ {self.odom_topic} + {self.imu_topic} (soft-sync, ignore Odin stamp)'
            )
        elif self.odin_mode == 'odin_own':
            self.create_subscription(Odometry, self.odom_topic, self.on_mid_odom, 50)
            self.create_subscription(Odometry, self.odin_odom_topic, self.on_odin_odom, 50)
            self.create_subscription(PointCloud2, self.odin_raw_topic, self.on_odin_own, 10)
            self.get_logger().info(
                f'Mid={self.mid_topic}; Odin OWN_ODOM raw×{self.odin_odom_topic}, '
                f'lock into Mid map once'
            )
        elif self.odin_mode == 'registered':
            self.create_subscription(
                PointCloud2, self.odin_registered_topic, self.on_odin_registered, 5
            )
            self.get_logger().info(f'Odin REGISTERED={self.odin_registered_topic}')
        else:
            self.get_logger().warn(f'odin_mode={self.odin_mode} falling back to imu_prop wiring')
            self.create_subscription(Odometry, self.odom_topic, self.on_mid_odom, 100)
            self.create_subscription(Imu, self.imu_topic, self.on_imu, 200)
            self.create_subscription(PointCloud2, self.odin_raw_topic, self.on_odin_imu_prop, 10)

    def _accum_ready(self) -> bool:
        return (time.time() - self._t0) >= self.accum_skip_sec

    def on_clear_accum(self, _req, resp):
        self._accum_mid.clear()
        self._accum_odin.clear()
        self._dirty_mid = self._dirty_odin = True
        self._T_align = None
        self._t0 = time.time()
        resp.success = True
        resp.message = f'cleared (posed={self._posed_count})'
        return resp

    def _header(self, stamp, frame_id: str | None = None) -> Header:
        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id or self.map_frame
        return h

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

    def _publish_world(self, world: np.ndarray, stamp) -> None:
        self.pub_odin.publish(make_cloud(self._header(stamp), world))
        self._posed_count += 1
        if self._accum_ready() and self._accum_odin.add(world):
            self._dirty_odin = True

    def on_mid(self, msg: PointCloud2) -> None:
        arr = self._xyz_stride(msg, self.mid_stride)
        self.pub_mid.publish(
            make_cloud(self._header(msg.header.stamp, msg.header.frame_id), arr)
        )
        if self._accum_ready() and self._accum_mid.add(arr):
            self._dirty_mid = True

    def on_mid_odom(self, msg: Odometry) -> None:
        self._mid_odom.append((stamp_sec(msg.header.stamp), se3_from_pose(msg.pose.pose)))

    def on_imu(self, msg: Imu) -> None:
        w = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float64,
        )
        self._imu.append((stamp_sec(msg.header.stamp), w))

    def _prop_mid_pose(self, t_target: float) -> Optional[np.ndarray]:
        """Latest Mid odom at/before t_target, gyro-propagate rotation to t_target."""
        if not self._mid_odom:
            return None
        # pick latest odom with stamp <= target; else earliest
        T0 = None
        t0 = None
        for t, T in reversed(self._mid_odom):
            if t <= t_target + 1e-3:
                t0, T0 = t, T
                break
        if T0 is None:
            t0, T0 = self._mid_odom[0]
        dt = t_target - t0
        if abs(dt) > self.max_imu_prop_sec:
            # too far — still return odom (better than skip entirely when clocks skew)
            return T0.copy()
        if abs(dt) < 1e-4 or len(self._imu) < 2:
            return T0.copy()
        # integrate gyro in [t0, t_target]
        samples = [(t, w) for t, w in self._imu if t0 - 0.01 <= t <= t_target + 0.01]
        if len(samples) < 2:
            return T0.copy()
        R = T0[:3, :3].copy()
        for i in range(len(samples) - 1):
            t_a, w_a = samples[i]
            t_b, w_b = samples[i + 1]
            if t_b <= t0:
                continue
            if t_a >= t_target:
                break
            ta = max(t_a, t0)
            tb = min(t_b, t_target)
            if tb <= ta:
                continue
            w = 0.5 * (w_a + w_b)
            R = R @ rotvec_to_R(w * (tb - ta))
        T = T0.copy()
        T[:3, :3] = R
        # translation: freeze (short dt); Mid odom already carries vel poorly
        return T

    def _level_yaw_pose(self, T: np.ndarray) -> np.ndarray:
        """Keep Mid xy + yaw only. Odin raw is near gravity-level; applying Mid
        roll/pitch (~IMU mount tilt) makes the whole orange map look slanted."""
        R = T[:3, :3]
        yaw = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        out = np.eye(4, dtype=np.float64)
        out[0, 0], out[0, 1] = c, -s
        out[1, 0], out[1, 1] = s, c
        out[0, 3], out[1, 3], out[2, 3] = T[0, 3], T[1, 3], T[2, 3]
        return out

    def on_odin_imu_prop(self, msg: PointCloud2) -> None:
        # Critical: pose by receive time on Mid/ROS clock, NOT Odin device stamp.
        t_recv = self.get_clock().now().nanoseconds * 1e-9
        T_map_imu = self._prop_mid_pose(t_recv)
        if T_map_imu is None:
            return
        arr = self._xyz_stride(msg, self.odin_stride, self.odin_blind2)
        if arr.size == 0:
            return
        # Odin raw ≈ gravity-horizontal; Mid IMU/body has mount pitch. Using full
        # Mid R tilts the whole orange map. Keep Mid yaw+xy only + L2 translation.
        T_yaw = self._level_yaw_pose(T_map_imu)
        R = T_yaw[:3, :3]
        t = T_yaw[:3, 3] + R @ self.T_imu_odin[:3, 3]
        world = (arr @ R.T) + t
        self._publish_world(world, self.get_clock().now().to_msg())

    def on_odin_odom(self, msg: Odometry) -> None:
        self._odin_odom = se3_from_pose(msg.pose.pose)

    def on_odin_own(self, msg: PointCloud2) -> None:
        if self._odin_odom is None:
            return
        arr = self._xyz_stride(msg, self.odin_stride, self.odin_blind2)
        if arr.size == 0:
            return
        T_od = self._odin_odom @ self.T_odin_imu_lidar
        world_od = (arr @ T_od[:3, :3].T) + T_od[:3, 3]
        if self._T_align is None:
            if not self._mid_odom:
                return
            # Lock: Mid map pose × Mid-chain extrinsic ≈ align × Odin odom pose
            T_map_mid = self._mid_odom[-1][1]
            self._T_align = (T_map_mid @ self.T_imu_odin) @ np.linalg.inv(T_od)
            self.get_logger().info('Locked T_map<-odin_odom from first Mid/Odin pair')
        world = (world_od @ self._T_align[:3, :3].T) + self._T_align[:3, 3]
        self._publish_world(world, self.get_clock().now().to_msg())

    def on_odin_registered(self, msg: PointCloud2) -> None:
        arr = self._xyz_stride(msg, self.odin_stride)
        if arr.size == 0:
            return
        self._publish_world(arr, msg.header.stamp)

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
