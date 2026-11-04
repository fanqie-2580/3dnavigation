#!/usr/bin/env python3
"""Independent Odin1 map accumulation — NO Mid360 association.

Uses only:
  /odin1/odometry  +  /odin1/cloud_raw  (lidar frame → odom via Odin pose)

Publishes:
  /odin1/cloud_in_odom   current scan in odin_map
  /viz/accum_odin       voxel accum in odin_map
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


class OdinIndependentAccum(Node):
    def __init__(self) -> None:
        super().__init__('odin_independent_accum')
        self.declare_parameter('cloud_topic', '/odin1/cloud_raw')
        self.declare_parameter('odom_topic', '/odin1/odometry')
        self.declare_parameter('map_frame', 'odin_map')
        # true: cloud already in odom (e.g. /odin1/cloud_slam); do not multiply odom
        self.declare_parameter('cloud_already_in_odom', False)
        self.declare_parameter('stride', 2)
        self.declare_parameter('blind', 0.3)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        self.declare_parameter('accum_skip_sec', 2.0)
        self.declare_parameter('odom_buffer_sec', 2.0)
        self.declare_parameter('max_odom_cloud_dt', 0.25)

        self.cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.cloud_already_in_odom = bool(self.get_parameter('cloud_already_in_odom').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.blind2 = float(self.get_parameter('blind').value) ** 2
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self.odom_buffer_sec = float(self.get_parameter('odom_buffer_sec').value)
        self.max_odom_cloud_dt = float(self.get_parameter('max_odom_cloud_dt').value)
        self._t0 = time.time()

        self._accum = VoxelAccum(voxel, max_n)
        self._dirty = False
        self._odom_buf: Deque[Tuple[float, np.ndarray]] = deque()

        from rclpy.qos import qos_profile_sensor_data

        self.pub_scan = self.create_publisher(PointCloud2, '/odin1/cloud_in_odom', 5)
        self.pub_accum = self.create_publisher(PointCloud2, '/viz/accum_odin', 1)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data
        )
        self.create_timer(max(0.2, period), self.publish_accum)
        self.create_service(Trigger, '~/clear_accum', self.on_clear)

        self.get_logger().info(
            f'INDEPENDENT Odin accum: cloud={self.cloud_topic} odom={self.odom_topic} '
            f'frame={self.map_frame} already_in_odom={self.cloud_already_in_odom} '
            f'(no Mid360 association)'
        )

    def on_clear(self, _req, resp):
        self._accum.clear()
        self._dirty = True
        self._t0 = time.time()
        resp.success = True
        resp.message = 'cleared independent Odin accum'
        return resp

    def on_odom(self, msg: Odometry) -> None:
        t = stamp_sec(msg.header.stamp)
        T = se3_from_pose(msg.pose.pose)
        self._odom_buf.append((t, T))
        cutoff = t - self.odom_buffer_sec
        while self._odom_buf and self._odom_buf[0][0] < cutoff:
            self._odom_buf.popleft()

    def _nearest_odom(self, t_cloud: float) -> Optional[np.ndarray]:
        if not self._odom_buf:
            return None
        # Prefer latest odom if cloud stamp is device-uptime (far from ROS odom time)
        t_odom_last = self._odom_buf[-1][0]
        if abs(t_cloud - t_odom_last) > 1.0:
            return self._odom_buf[-1][1].copy()
        best = None
        best_dt = 1e9
        for t, T in self._odom_buf:
            dt = abs(t - t_cloud)
            if dt < best_dt:
                best_dt = dt
                best = T
        if best is None or best_dt > self.max_odom_cloud_dt:
            return self._odom_buf[-1][1].copy()
        return best.copy()

    def _xyz_stride(self, msg: PointCloud2) -> np.ndarray:
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
            if (x * x + y * y + z * z) < self.blind2:
                continue
            if (k % self.stride) == 0:
                pts.append([x, y, z])
            k += 1
        return np.asarray(pts, dtype=np.float64)

    def on_cloud(self, msg: PointCloud2) -> None:
        arr = self._xyz_stride(msg)
        if arr.size == 0:
            return

        if self.cloud_already_in_odom:
            world = arr
            stamp = msg.header.stamp
        else:
            T = self._nearest_odom(stamp_sec(msg.header.stamp))
            if T is None:
                return
            R, t = T[:3, :3], T[:3, 3]
            world = (arr @ R.T) + t
            # Stamp with odom time when device cloud stamp is unusable
            if self._odom_buf:
                last_t = self._odom_buf[-1][0]
                if abs(stamp_sec(msg.header.stamp) - last_t) > 1.0:
                    sec = int(last_t)
                    nsec = int((last_t - sec) * 1e9)
                    stamp = msg.header.stamp
                    stamp.sec = sec
                    stamp.nanosec = nsec
                else:
                    stamp = msg.header.stamp
            else:
                stamp = msg.header.stamp

        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        self.pub_scan.publish(make_cloud(header, world))

        if (time.time() - self._t0) >= self.accum_skip_sec:
            if self._accum.add(world):
                self._dirty = True

    def publish_accum(self) -> None:
        if not (self._dirty or len(self._accum)):
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        self.pub_accum.publish(make_cloud(header, self._accum.numpy()))
        self._dirty = False


def main() -> None:
    rclpy.init()
    node = OdinIndependentAccum()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
