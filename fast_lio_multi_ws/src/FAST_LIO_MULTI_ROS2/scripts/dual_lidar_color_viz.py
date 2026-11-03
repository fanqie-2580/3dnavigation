#!/usr/bin/env python3
"""World-fixed dual-color accumulation for Mid360 + Odin.

Mid360 (cyan): /cloud_registered (already in map from FAST-LIO)

Odin1 (orange): same pattern as ws_odin_pvlio/bridge/odin_odom_mapper.py
  - Input is sensor-frame /odin1/cloud_raw_ds (NOT cloud_slam, NOT LIO-undistorted)
  - Apply Mid /Odometry pose ONCE with L2_wrt_L1 + Mid IMU extrinsic
  - Voxel-accumulate in map

Standalone Odin mapping worked because cloud_slam was already in odom and
the mapper never re-multiplied pose (re-applying odom caused "follows robot"
smearing). Fusion must likewise apply pose exactly once to raw sensor cloud.
"""
from __future__ import annotations

import math
import struct
import time
from typing import Dict, Optional, Tuple

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


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
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
    R = np.asarray(R9, dtype=np.float64).reshape(3, 3)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t3, dtype=np.float64).reshape(3)
    return T


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
        # Sensor-frame Odin (mode0 raw). Do NOT use cloud_slam here.
        self.declare_parameter('odin_raw_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mid_stride', 1)
        self.declare_parameter('odin_stride', 2)
        self.declare_parameter('odin_blind', 0.3)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        self.declare_parameter('accum_skip_sec', 8.0)
        # Mid360 IMU -> Mid lidar (mid360_odin1.yaml)
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        # Odin -> Mid lidar
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
        self.odin_raw_topic = self.get_parameter('odin_raw_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.mid_stride = max(1, int(self.get_parameter('mid_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))
        self.odin_blind2 = float(self.get_parameter('odin_blind').value) ** 2
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self._t0 = time.time()

        T_imu_mid = se3_from_rt(
            self.get_parameter('extrinsic_R').value,
            self.get_parameter('extrinsic_T').value,
        )
        T_mid_odin = se3_from_rt(
            self.get_parameter('extrinsic_R_L2_wrt_L1').value,
            self.get_parameter('extrinsic_T_L2_wrt_L1').value,
        )
        # p_imu = T_imu_mid * T_mid_odin * p_odin
        self.T_imu_odin = T_imu_mid @ T_mid_odin

        self._accum_mid = VoxelAccum(voxel, max_n)
        self._accum_odin = VoxelAccum(voxel, max_n)
        self._dirty_mid = False
        self._dirty_odin = False
        self._latest_odom: Optional[Odometry] = None

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.pub_accum_mid = self.create_publisher(PointCloud2, '/viz/accum_mid360', 1)
        self.pub_accum_odin = self.create_publisher(PointCloud2, '/viz/accum_odin', 1)

        self.create_subscription(PointCloud2, self.mid_topic, self.on_mid, 5)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(PointCloud2, self.odin_raw_topic, self.on_odin_raw, 5)
        self.create_timer(max(0.2, period), self.publish_accum)
        self.create_service(Trigger, '~/clear_accum', self.on_clear_accum)

        self.get_logger().info(
            f'Mid accum from {self.mid_topic}; Odin from SENSOR {self.odin_raw_topic} '
            f'x Mid {self.odom_topic} once (odin_odom_mapper style), voxel={voxel}, '
            f'skip={self.accum_skip_sec}s'
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
        self._latest_odom = msg

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

    def on_odin_raw(self, msg: PointCloud2) -> None:
        """Pose sensor-frame Odin with Mid odom exactly once → map (world-fixed)."""
        odom = self._latest_odom
        if odom is None:
            return
        arr = self._xyz_stride(msg, self.odin_stride, self.odin_blind2)
        if arr.size == 0:
            return

        T_map_imu = se3_from_pose(odom.pose.pose)
        T_map_odin = T_map_imu @ self.T_imu_odin
        R = T_map_odin[:3, :3]
        t = T_map_odin[:3, 3]
        world = (arr @ R.T) + t

        stamp = odom.header.stamp
        self.pub_odin.publish(make_cloud(self._header(stamp), world))
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
