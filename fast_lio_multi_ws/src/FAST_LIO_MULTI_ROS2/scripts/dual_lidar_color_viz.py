#!/usr/bin/env python3
"""Publish Mid360 and Odin clouds in map frame on separate topics for RViz color compare.

Mid360  -> /viz/cloud_mid360
Odin1   -> /viz/cloud_odin

Uses /Odometry pose; Odin gravity-frame extrinsic (R,T) is converted with first
IMU-aligned orientation R_wi0^T, matching laserMapping_adaptive compensation.
"""
from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
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
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('mid360_stride', 8)
        self.declare_parameter('odin_stride', 2)
        # Prefer keys from mid360_odin1.yaml when parameters=[config_file]
        self.declare_parameter('mapping.extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'mapping.extrinsic_R',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
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

        self.map_frame = self.get_parameter('map_frame').value
        self.mid_stride = max(1, int(self.get_parameter('mid360_stride').value))
        self.odin_stride = max(1, int(self.get_parameter('odin_stride').value))
        self.t_il = np.asarray(
            self.get_parameter('mapping.extrinsic_T').value, dtype=np.float64
        ).reshape(3)
        self.R_il = np.asarray(
            self.get_parameter('mapping.extrinsic_R').value, dtype=np.float64
        ).reshape(3, 3)
        self.t_w = np.asarray(
            self.get_parameter('mapping.extrinsic_T_L2_wrt_L1').value, dtype=np.float64
        ).reshape(3)
        self.R_yaml = np.asarray(
            self.get_parameter('mapping.extrinsic_R_L2_wrt_L1').value, dtype=np.float64
        ).reshape(3, 3)

        self._pose: Pose | None = None
        self._R_wi0: np.ndarray | None = None
        self._R_L2: np.ndarray | None = None
        self._t_L2: np.ndarray | None = None

        self.pub_mid = self.create_publisher(PointCloud2, '/viz/cloud_mid360', 5)
        self.pub_odin = self.create_publisher(PointCloud2, '/viz/cloud_odin', 5)
        self.create_subscription(Odometry, '/Odometry', self.on_odom, 20)
        self.create_subscription(CustomMsg, '/livox/lidar', self.on_mid, 5)
        self.create_subscription(PointCloud2, '/odin1/cloud_raw_ds', self.on_odin, 5)
        self.get_logger().info(
            'dual color viz: Mid360=/viz/cloud_mid360 Odin=/viz/cloud_odin'
        )

    def on_odom(self, msg: Odometry) -> None:
        self._pose = msg.pose.pose
        if self._R_wi0 is None:
            self._R_wi0 = quat_to_R(msg.pose.pose.orientation)
            self._R_L2 = self._R_wi0.T @ self.R_yaml
            self._t_L2 = self._R_wi0.T @ self.t_w
            self.get_logger().info('captured R_wi0; Odin L2 extrinsic converted')

    def _body_to_world(self, pts_body: np.ndarray) -> np.ndarray:
        assert self._pose is not None
        R = quat_to_R(self._pose.orientation)
        t = np.array(
            [self._pose.position.x, self._pose.position.y, self._pose.position.z],
            dtype=np.float64,
        )
        return (R @ pts_body.T).T + t

    def on_mid(self, msg: CustomMsg) -> None:
        if self._pose is None or not msg.points:
            return
        pts = np.asarray(
            [[p.x, p.y, p.z] for p in msg.points[:: self.mid_stride]], dtype=np.float64
        )
        if pts.size == 0:
            return
        body = (self.R_il @ pts.T).T + self.t_il
        world = self._body_to_world(body)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.map_frame
        self.pub_mid.publish(make_cloud(header, world))

    def on_odin(self, msg: PointCloud2) -> None:
        if self._pose is None or self._R_L2 is None or self._t_L2 is None:
            return
        pts = list(
            point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        )
        if not pts:
            return
        # read_points yields structured tuples; force plain Nx3 float64
        arr = np.array([[p[0], p[1], p[2]] for p in pts[:: self.odin_stride]], dtype=np.float64)
        if arr.size == 0:
            return
        body = (self._R_L2 @ arr.T).T + self._t_L2
        world = self._body_to_world(body)
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
