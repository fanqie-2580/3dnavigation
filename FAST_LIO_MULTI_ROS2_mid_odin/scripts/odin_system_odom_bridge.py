#!/usr/bin/env python3
"""B2: publish system /Odometry + map→body TF from /odin1/odometry.

Mid FAST-LIO keeps estimating on /Odometry_mid (no TF). This node owns the
pose used by RViz robot model and nav (/baselink2map via multi_lio_nav_bridge).

Pipeline:
  1) One-shot extrinsic lock vs Mid /Odometry_mid (same CAD path as B1 densify)
  2) Live: T_map_imu = T_align @ T_odin_odom_imu
  3) T_map_base = T_map_imu @ inv(T_base_odinIMU)
  4) T_map_body = T_map_base @ inv(T_body_base)   # keep existing body→base_link static TF
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


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


def rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def se3_from_odom(msg: Odometry) -> np.ndarray:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def se3_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def se3_from_xyz_rpy(xyz, rpy) -> np.ndarray:
    x, y, z = [float(v) for v in xyz]
    r, p, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return T


def se3_from_rt(R9, t3) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R9, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t3, dtype=np.float64).reshape(3)
    return T


def fill_pose(msg: Odometry, T: np.ndarray) -> None:
    msg.pose.pose.position.x = float(T[0, 3])
    msg.pose.pose.position.y = float(T[1, 3])
    msg.pose.pose.position.z = float(T[2, 3])
    x, y, z, w = rot_to_quat(T[:3, :3])
    msg.pose.pose.orientation.x = x
    msg.pose.pose.orientation.y = y
    msg.pose.pose.orientation.z = z
    msg.pose.pose.orientation.w = w


class OdinSystemOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__('odin_system_odom_bridge')
        self.declare_parameter('odin_odom_topic', '/odin1/odometry')
        self.declare_parameter('mid_odom_topic', '/Odometry_mid')
        self.declare_parameter('out_odom_topic', '/Odometry')
        self.declare_parameter('out_path_topic', '/path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('align_delay_sec', 3.0)
        self.declare_parameter('imu_to_lidar_xyz', [-0.02663, 0.03447, 0.02174])
        self.declare_parameter('imu_to_lidar_rpy', [0.0, 0.0, 0.0])
        # CAD: base_link → Odin IMU
        self.declare_parameter('base_to_odin_imu_xyz', [0.313654, -0.028974, 0.104418])
        self.declare_parameter('base_to_odin_imu_rpy', [0.0, 0.0, 0.0])
        # Existing launch static: body → base_link (x y z yaw pitch roll)
        self.declare_parameter('body_to_base_xyz', [-0.140709, -0.02329, -0.111671])
        self.declare_parameter('body_to_base_rpy', [0.0, -0.226893, 0.0])  # roll pitch yaw
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
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

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.body_frame = str(self.get_parameter('body_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.align_delay_sec = float(self.get_parameter('align_delay_sec').value)

        self.T_imu_L = se3_from_xyz_rpy(
            self.get_parameter('imu_to_lidar_xyz').value,
            self.get_parameter('imu_to_lidar_rpy').value,
        )
        self.T_base_imu = se3_from_xyz_rpy(
            self.get_parameter('base_to_odin_imu_xyz').value,
            self.get_parameter('base_to_odin_imu_rpy').value,
        )
        # body_to_base_rpy declared as roll,pitch,yaw
        bxyz = list(self.get_parameter('body_to_base_xyz').value)
        brpy = list(self.get_parameter('body_to_base_rpy').value)
        self.T_body_base = se3_from_xyz_rpy(bxyz, brpy)
        self.T_imu_L1 = se3_from_rt(
            self.get_parameter('extrinsic_R').value,
            self.get_parameter('extrinsic_T').value,
        )
        self.T_L1_L2 = se3_from_rt(
            self.get_parameter('extrinsic_R_L2_wrt_L1').value,
            self.get_parameter('extrinsic_T_L2_wrt_L1').value,
        )
        self.T_body_odin_lidar = self.T_imu_L1 @ self.T_L1_L2

        self._mid_T: Optional[np.ndarray] = None
        self._odin_T: Optional[np.ndarray] = None
        self._aligned = False
        self._T_align = np.eye(4, dtype=np.float64)
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._n_pub = 0
        self._path = Path()
        self._path.header.frame_id = self.map_frame

        reliable = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        out = str(self.get_parameter('out_odom_topic').value)
        path_topic = str(self.get_parameter('out_path_topic').value)
        self.pub_odom = self.create_publisher(Odometry, out, reliable)
        self.pub_path = self.create_publisher(Path, path_topic, 10)
        self.tf_br = TransformBroadcaster(self)

        self.create_subscription(
            Odometry, str(self.get_parameter('odin_odom_topic').value), self.on_odin, reliable
        )
        self.create_subscription(
            Odometry, str(self.get_parameter('mid_odom_topic').value), self.on_mid, reliable
        )
        self.create_timer(0.5, self._try_align)
        self.create_timer(2.0, self._status)
        self.get_logger().info(
            f'B2 system odom: odin={self.get_parameter("odin_odom_topic").value} '
            f'mid={self.get_parameter("mid_odom_topic").value} → {out} '
            f'+ TF {self.map_frame}→{self.body_frame} (after extrinsic lock)'
        )

    def _status(self) -> None:
        self.get_logger().info(
            f'system_odom pub={self._n_pub} aligned={self._aligned} '
            f'mid={"Y" if self._mid_T is not None else "N"} '
            f'odin={"Y" if self._odin_T is not None else "N"}'
        )

    def on_mid(self, msg: Odometry) -> None:
        if self._aligned:
            return
        self._mid_T = se3_from_odom(msg)

    def on_odin(self, msg: Odometry) -> None:
        T_oi = se3_from_odom(msg)
        self._odin_T = T_oi
        if not self._aligned:
            return

        T_map_imu = self._T_align @ T_oi
        T_map_base = T_map_imu @ se3_inv(self.T_base_imu)
        T_map_body = T_map_base @ se3_inv(self.T_body_base)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.body_frame
        fill_pose(out, T_map_body)
        out.twist = msg.twist
        self.pub_odom.publish(out)
        self._n_pub += 1

        if self.publish_tf:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.body_frame
            t.transform.translation.x = out.pose.pose.position.x
            t.transform.translation.y = out.pose.pose.position.y
            t.transform.translation.z = out.pose.pose.position.z
            t.transform.rotation = out.pose.pose.orientation
            self.tf_br.sendTransform(t)

        ps = PoseStamped()
        ps.header = out.header
        ps.pose = out.pose.pose
        self._path.header.stamp = out.header.stamp
        self._path.poses.append(ps)
        if len(self._path.poses) > 20000:
            self._path.poses = self._path.poses[-10000:]
        if (self._n_pub % 5) == 0:
            self.pub_path.publish(self._path)

    def _try_align(self) -> None:
        if self._aligned:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (now - self._t0) < self.align_delay_sec:
            return
        if self._mid_T is None or self._odin_T is None:
            return
        T_odom_odin_lidar = self._odin_T @ self.T_imu_L
        T_map_odin_lidar = self._mid_T @ self.T_body_odin_lidar
        self._T_align = T_map_odin_lidar @ se3_inv(T_odom_odin_lidar)
        self._aligned = True
        t = self._T_align[:3, 3]
        self.get_logger().info(
            f'Locked Odin→Mid-map align t=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}] '
            f'(system /Odometry now follows /odin1/odometry)'
        )


def main() -> None:
    rclpy.init()
    node = OdinSystemOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
