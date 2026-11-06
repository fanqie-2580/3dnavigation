#!/usr/bin/env python3
"""Project delayed Odin cloud into Mid map using Mid TF at Odin stamp.

Dock TCP often forwards /odin1/cloud_slam (already in Odin odom) but the host
publishes it as /odin1/cloud_raw with frame_id=odom. Treating those points as
lidar-local makes walls yaw-misalign / co-rotate. When frame_id is odom/map:

  p_L = inv(T_odom_imu(t) * T_imu_L) * p_odom
  p_map = T_map_body(t) * T_body_L1 * T_L1_L2 * p_L

If frame_id is lidar (true raw), skip the inv(odom) step.
"""
from __future__ import annotations

import math
import struct
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def mat_from_tf(tf: TransformStamped) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
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
    T[:3, 3] = [t.x, t.y, t.z]
    return T


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
    """Keep map translation and yaw; zero pitch/roll."""
    yaw = math.atan2(T[1, 0], T[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.eye(4, dtype=np.float64)
    out[0, 0] = c
    out[0, 1] = -s
    out[1, 0] = s
    out[1, 1] = c
    out[:3, 3] = T[:3, 3]
    return out


class OdinMidTimedProject(Node):
    def __init__(self) -> None:
        super().__init__('odin_mid_timed_project')
        self.declare_parameter('cloud_topic', '/odin1/cloud_raw')
        self.declare_parameter('out_topic', '/odin1/cloud_in_map')
        self.declare_parameter('odin_odom_topic', '/odin1/odometry')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('tf_cache_sec', 12.0)
        self.declare_parameter('max_stamp_delay_sec', 6.0)
        self.declare_parameter('odom_max_dt_sec', 0.25)
        self.declare_parameter('stride', 2)
        self.declare_parameter('blind', 0.3)
        self.declare_parameter('min_confidence', 30.0)
        # auto: unproject if cloud frame_id is odom/map (dock forwards cloud_slam)
        self.declare_parameter('cloud_in_odom', 'auto')
        self.declare_parameter('use_yaw_only_body', False)
        self.declare_parameter('use_yaw_only_odin_odom', False)
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [0.12157888, 0.00549610, 0.03169932])
        # Go2-relative: Odin≈I, Mid@Go2 Ry(+13°) → Mid←Odin Ry(-13°). CAD T kept.
        # (Old ICP Ry(-10.15°) was fit before odom-cloud unproject; left ~+3.6° pitch.)
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [
                0.97437006, 0.0, -0.22495105,
                0.0, 1.0, 0.0,
                0.22495105, 0.0, 0.97437006,
            ],
        )
        # Official Odin IMU→lidar (wiki), used when un-projecting odom-frame cloud.
        self.declare_parameter('extrinsic_T_imu_L', [-0.02663, 0.03447, 0.02174])
        self.declare_parameter(
            'extrinsic_R_imu_L', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.body_frame = str(self.get_parameter('body_frame').value)
        self.max_delay = float(self.get_parameter('max_stamp_delay_sec').value)
        self.odom_max_dt = float(self.get_parameter('odom_max_dt_sec').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.blind2 = float(self.get_parameter('blind').value) ** 2
        self.min_conf = float(self.get_parameter('min_confidence').value)
        self.use_yaw_only = bool(self.get_parameter('use_yaw_only_body').value)
        self.use_yaw_only_odin = bool(self.get_parameter('use_yaw_only_odin_odom').value)
        mode = str(self.get_parameter('cloud_in_odom').value).strip().lower()
        self.cloud_in_odom_mode = mode  # auto|true|false
        cache = float(self.get_parameter('tf_cache_sec').value)

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

        self._tf = Buffer(cache_time=Duration(seconds=cache))
        self._tf_listener = TransformListener(self._tf, self)
        self._odin_odom: Deque[Tuple[float, np.ndarray]] = deque(maxlen=2000)
        self._drop = 0
        self._ok = 0
        self._drop_odom = 0

        from rclpy.qos import qos_profile_sensor_data

        out = str(self.get_parameter('out_topic').value)
        inn = str(self.get_parameter('cloud_topic').value)
        odom_topic = str(self.get_parameter('odin_odom_topic').value)
        self.pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, inn, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(2.0, self._stats)
        self.get_logger().info(
            f'Odin→Mid-map timed project: {inn} → {out}, '
            f'TF {self.map_frame}←{self.body_frame} '
            f'yaw_only={self.use_yaw_only} cloud_in_odom={self.cloud_in_odom_mode} '
            f'cache={cache}s max_delay={self.max_delay}s'
        )

    def _stats(self) -> None:
        total = self._ok + self._drop
        if total == 0:
            return
        self.get_logger().info(
            f'timed_project ok={self._ok} drop={self._drop} '
            f'odom_miss={self._drop_odom} '
            f'({100.0 * self._ok / total:.0f}% hit)'
        )
        self._ok = 0
        self._drop = 0
        self._drop_odom = 0

    def on_odom(self, msg: Odometry) -> None:
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        self._odin_odom.append((t, mat_from_odom(msg)))

    def _lookup_odin_odom(self, stamp) -> Optional[np.ndarray]:
        if not self._odin_odom:
            return None
        t = Time.from_msg(stamp).nanoseconds * 1e-9
        best = None
        best_dt = 1e9
        for ot, T in self._odin_odom:
            dt = abs(ot - t)
            if dt < best_dt:
                best_dt = dt
                best = T
        if best is None or best_dt > self.odom_max_dt:
            return None
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

    def on_cloud(self, msg: PointCloud2) -> None:
        stamp = msg.header.stamp
        now = self.get_clock().now()
        delay = (now - Time.from_msg(stamp)).nanoseconds * 1e-9
        if delay < -0.5 or delay > self.max_delay:
            self._drop += 1
            return
        try:
            tf = self._tf.lookup_transform(
                self.map_frame,
                self.body_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            self._drop += 1
            return

        arr = self._xyz(msg)
        if arr.size == 0:
            return

        if self._cloud_is_in_odom(msg):
            T_oi = self._lookup_odin_odom(stamp)
            if T_oi is None:
                self._drop += 1
                self._drop_odom += 1
                return
            if self.use_yaw_only_odin:
                T_oi = yaw_only_pose(T_oi)
            T_oL = T_oi @ self.T_imu_L
            # p_L = inv(T_oL) * p_odom
            R = T_oL[:3, :3]
            t = T_oL[:3, 3]
            arr = (arr - t) @ R

        T_mb = mat_from_tf(tf)
        if self.use_yaw_only:
            T_mb = yaw_only_pose(T_mb)
        T = T_mb @ self.T_body_odin
        world = (arr @ T[:3, :3].T) + T[:3, 3]
        header = Header()
        header.stamp = stamp
        header.frame_id = self.map_frame
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, world.astype(np.float32)))
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
