#!/usr/bin/env python3
"""Publish Mid360 + Odin1 lidar frames in map for RViz TF axes.

Mid360: full Mid body pose * L1_wrt_IMU (same as FAST-LIO lidar).
Odin1 (gravity_frame): Mid yaw+pos only * L2_wrt_L1 — Mid pitch/roll do NOT enter.
"""
from __future__ import annotations

import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
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


def R_to_quat(R: np.ndarray):
    m = R
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def mid_yaw_R(R: np.ndarray) -> np.ndarray:
    yaw = math.atan2(R[1, 0], R[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


class DualLidarFrames(Node):
    def __init__(self) -> None:
        super().__init__('dual_lidar_frames')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('mid_frame', 'mid360_lidar')
        self.declare_parameter('odin_frame', 'odin1_lidar')
        self.declare_parameter('gravity_frame', True)
        # Mid IMU -> Mid lidar
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter('extrinsic_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        # Odin lidar -> Mid lidar
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [-0.030, -0.0371, 0.208])
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.mid_frame = str(self.get_parameter('mid_frame').value)
        self.odin_frame = str(self.get_parameter('odin_frame').value)
        self.gravity_frame = bool(self.get_parameter('gravity_frame').value)

        t1: List[float] = list(self.get_parameter('extrinsic_T').value)
        r1: List[float] = list(self.get_parameter('extrinsic_R').value)
        t2: List[float] = list(self.get_parameter('extrinsic_T_L2_wrt_L1').value)
        r2: List[float] = list(self.get_parameter('extrinsic_R_L2_wrt_L1').value)
        self.R_L1_I = np.asarray(r1, dtype=np.float64).reshape(3, 3)
        self.t_L1_I = np.asarray(t1, dtype=np.float64).reshape(3)
        self.R_L2_L1 = np.asarray(r2, dtype=np.float64).reshape(3, 3)
        self.t_L2_L1 = np.asarray(t2, dtype=np.float64).reshape(3)

        self.br = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self.on_odom,
            20,
        )
        self.get_logger().info(
            f'Publishing TF {self.map_frame}->{{{self.mid_frame},{self.odin_frame}}} '
            f'(gravity_frame={self.gravity_frame})'
        )

    def _send(self, stamp, parent: str, child: str, R: np.ndarray, t: np.ndarray) -> None:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        x, y, z, w = R_to_quat(R)
        msg.transform.rotation.x = float(x)
        msg.transform.rotation.y = float(y)
        msg.transform.rotation.z = float(z)
        msg.transform.rotation.w = float(w)
        self.br.sendTransform(msg)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        R_wb = quat_to_R(q.x, q.y, q.z, q.w)
        t_wb = np.array([p.x, p.y, p.z], dtype=np.float64)

        # Mid360 lidar in map (full pose)
        R_wm = R_wb @ self.R_L1_I
        t_wm = R_wb @ self.t_L1_I + t_wb
        self._send(msg.header.stamp, self.map_frame, self.mid_frame, R_wm, t_wm)

        # Odin1 lidar in map
        if self.gravity_frame:
            R_yaw = mid_yaw_R(R_wb)
            R_wo = R_yaw @ self.R_L2_L1
            t_wo = R_yaw @ self.t_L2_L1 + t_wb
        else:
            R_wo = R_wb @ self.R_L1_I @ self.R_L2_L1
            t_wo = R_wb @ (self.R_L1_I @ self.t_L2_L1 + self.t_L1_I) + t_wb
        self._send(msg.header.stamp, self.map_frame, self.odin_frame, R_wo, t_wo)


def main() -> None:
    rclpy.init()
    node = DualLidarFrames()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
