#!/usr/bin/env python3
"""Dual-color viz for Mid360 + Odin without making the world map spin.

Mid360  -> /viz/cloud_mid360  from deskewed /cloud_registered (world-fixed LIO)
Odin1   -> /viz/cloud_odin    from /odin1/cloud_raw_ds + /Odometry (live overlay)

When mapping.lidar2_update_en=false, Odin is not in /cloud_registered, so it must
be published on its own topic. Keep RViz Odin Decay Time = 0 (current frame only).
"""
from __future__ import annotations

import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def quat_to_R(q) -> np.ndarray:
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def make_cloud(header: Header, xyz: np.ndarray) -> PointCloud2:
    if xyz.size == 0:
        xyz = np.zeros((0, 3), dtype=np.float32)
    else:
        xyz = xyz.astype(np.float32)
    return point_cloud2.create_cloud_xyz32(header, xyz)


class DualLidarColorViz(Node):
    def __init__(self) -> None:
        super().__init__('dual_lidar_color_viz')
        self.declare_parameter('registered_topic', '/cloud_registered')
        self.declare_parameter('odin_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odin_intensity_thresh', 100.0)
        self.declare_parameter('mid_stride', 1)
        self.declare_parameter('odin_stride', 2)
        self.declare_parameter(
            'mapping.extrinsic_T_L2_wrt_L1', [0.154713, 0.022738, 0.000055]
        )
        self.declare_parameter(
            'mapping.extrinsic_R_L2_wrt_L1',
            [
                0.99603821, 0.00426344, -0.08882406,
                -0.00656408, 0.99965007, -0.02562515,
                0.08868373, 0.02610668, 0.99571765,
            ],
        )

        self.reg_topic = self.get_parameter('registered_topic').value
        self.odin_topic = self.get_parameter('odin_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odin_thr = float(self.get_parameter('odin_intensity_thresh').value)
        self.mid_stride = max(1, int(self.get_parameter('mid_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))
        self.t_w = np.asarray(
            self.get_parameter('mapping.extrinsic_T_L2_wrt_L1').value, dtype=np.float64
        ).reshape(3)
        self.R_yaml = np.asarray(
            self.get_parameter('mapping.extrinsic_R_L2_wrt_L1').value, dtype=np.float64
        ).reshape(3, 3)

        self._pose: Pose | None = None
        self._R_L2: np.ndarray | None = None
        self._t_L2: np.ndarray | None = None

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 20)
        self.create_subscription(PointCloud2, self.reg_topic, self.on_registered, 5)
        self.create_subscription(PointCloud2, self.odin_topic, self.on_odin, 5)
        self.get_logger().info(
            f'dual color viz: Mid from {self.reg_topic}, '
            f'Odin live from {self.odin_topic} -> /viz/cloud_odin'
        )

    def on_odom(self, msg: Odometry) -> None:
        self._pose = msg.pose.pose
        if self._R_L2 is None:
            R_wi0 = quat_to_R(msg.pose.pose.orientation)
            self._R_L2 = R_wi0.T @ self.R_yaml
            self._t_L2 = R_wi0.T @ self.t_w
            self.get_logger().info('captured R_wi0; Odin L2 extrinsic converted')

    def on_registered(self, msg: PointCloud2) -> None:
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return
        has_i = 'intensity' in offs
        n = msg.width * msg.height
        mid: list[list[float]] = []
        step = msg.point_step
        data = msg.data
        mi = 0
        for i in range(n):
            b = i * step
            x, y, z = struct.unpack_from('fff', data, b + offs['x'])
            inten = (
                struct.unpack_from('f', data, b + offs['intensity'])[0]
                if has_i
                else 10.0
            )
            if inten >= self.odin_thr:
                continue
            if (mi % self.mid_stride) == 0:
                mid.append([x, y, z])
            mi += 1
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or self.map_frame
        self.pub_mid.publish(make_cloud(header, np.asarray(mid, dtype=np.float64)))

    def on_odin(self, msg: PointCloud2) -> None:
        if self._pose is None or self._R_L2 is None or self._t_L2 is None:
            return
        pts = list(
            point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        )
        if not pts:
            return
        arr = np.array(
            [[p[0], p[1], p[2]] for p in pts[:: self.odin_stride]], dtype=np.float64
        )
        if arr.size == 0:
            return
        body = (self._R_L2 @ arr.T).T + self._t_L2
        R = quat_to_R(self._pose.orientation)
        t = np.array(
            [self._pose.position.x, self._pose.position.y, self._pose.position.z],
            dtype=np.float64,
        )
        world = (R @ body.T).T + t
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.map_frame
        self.pub_odin.publish(make_cloud(header, world))


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
