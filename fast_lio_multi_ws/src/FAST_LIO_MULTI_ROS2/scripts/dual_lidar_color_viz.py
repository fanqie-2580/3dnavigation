#!/usr/bin/env python3
"""World-fixed dual-color accumulation for Mid360 + Odin.

Mid360 (cyan):  /cloud_registered      -> /viz/accum_mid360
Odin1  (orange): /cloud_odin_registered -> /viz/accum_odin

Both topics are already in map, posed by the same FAST-LIO Mid360 state_point.
Do not transform raw Odin with a separate /Odometry path — that drifts when moving.
"""
from __future__ import annotations

import struct
import time
from typing import Dict, Tuple

import numpy as np
import rclpy
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
        self.declare_parameter('odin_topic', '/cloud_odin_registered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odin_intensity_thresh', 100.0)
        self.declare_parameter('mid_stride', 1)
        self.declare_parameter('odin_stride', 1)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        # Skip early frames while LIO gravity/zero-start settles — otherwise
        # permanent VoxelAccum keeps ghost walls forever.
        self.declare_parameter('accum_skip_sec', 8.0)

        self.mid_topic = self.get_parameter('mid_topic').value
        self.odin_topic = self.get_parameter('odin_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odin_thr = float(self.get_parameter('odin_intensity_thresh').value)
        self.mid_stride = max(1, int(self.get_parameter('mid_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self._t0 = time.time()

        self._accum_mid = VoxelAccum(voxel, max_n)
        self._accum_odin = VoxelAccum(voxel, max_n)
        self._dirty_mid = False
        self._dirty_odin = False

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.pub_accum_mid = self.create_publisher(PointCloud2, '/viz/accum_mid360', 1)
        self.pub_accum_odin = self.create_publisher(PointCloud2, '/viz/accum_odin', 1)

        self.create_subscription(PointCloud2, self.mid_topic, self.on_mid, 5)
        self.create_subscription(PointCloud2, self.odin_topic, self.on_odin, 5)
        self.create_timer(max(0.2, period), self.publish_accum)
        self.create_service(Trigger, '~/clear_accum', self.on_clear_accum)

        self.get_logger().info(
            f'dual accum from LIO map clouds: Mid {self.mid_topic}, '
            f'Odin {self.odin_topic}, voxel={voxel}, skip={self.accum_skip_sec}s'
        )

    def _accum_ready(self) -> bool:
        return (time.time() - self._t0) >= self.accum_skip_sec

    def on_clear_accum(self, _req, resp):
        self._accum_mid = VoxelAccum(self._accum_mid.voxel, self._accum_mid.max_points)
        self._accum_odin = VoxelAccum(self._accum_odin.voxel, self._accum_odin.max_points)
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

    def _xyz_from_cloud(
        self, msg: PointCloud2, stride: int, require_odin: bool | None
    ) -> np.ndarray:
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return np.zeros((0, 3), dtype=np.float64)
        has_i = 'intensity' in offs
        pts: list[list[float]] = []
        n = msg.width * msg.height
        k = 0
        for i in range(n):
            b = i * msg.point_step
            x, y, z = struct.unpack_from('fff', msg.data, b + offs['x'])
            if has_i and require_odin is not None:
                inten = struct.unpack_from('f', msg.data, b + offs['intensity'])[0]
                is_odin = inten >= self.odin_thr
                if require_odin and not is_odin:
                    continue
                if (not require_odin) and is_odin:
                    continue
            if (k % stride) == 0:
                pts.append([x, y, z])
            k += 1
        return np.asarray(pts, dtype=np.float64)

    def on_mid(self, msg: PointCloud2) -> None:
        # Mid registered may occasionally contain tagged Odin in bundle mode; filter.
        arr = self._xyz_from_cloud(msg, self.mid_stride, require_odin=False)
        self.pub_mid.publish(
            make_cloud(self._header(msg.header.stamp, msg.header.frame_id), arr)
        )
        if self._accum_ready() and self._accum_mid.add(arr):
            self._dirty_mid = True

    def on_odin(self, msg: PointCloud2) -> None:
        arr = self._xyz_from_cloud(msg, self.odin_stride, require_odin=None)
        self.pub_odin.publish(
            make_cloud(self._header(msg.header.stamp, msg.header.frame_id), arr)
        )
        if self._accum_ready() and self._accum_odin.add(arr):
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
