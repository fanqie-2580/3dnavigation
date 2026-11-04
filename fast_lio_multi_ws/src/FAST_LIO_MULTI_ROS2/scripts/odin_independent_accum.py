#!/usr/bin/env python3
"""Independent Odin1 map accumulation — NO Mid360 pose association.

Preferred (mode 1 SLAM):
  /odin1/cloud_slam  already in odom  → voxel as-is  (do NOT re-multiply odom)

Fallback (mode 0 / raw):
  /odin1/cloud_raw × (/odin1/odometry as imu pose) × T_imu_lidar
  Uses nearest odom by device stamp (never "latest while cloud is delayed").

Publishes:
  /odin1/cloud_in_odom   current scan in odin_map
  /viz/accum_odin       voxel accum in odin_map

Optional one-shot RViz align: lock map→odin_map so Mid body and Odin imu
coincide at first samples (visual only; maps stay independent after that).
"""
from __future__ import annotations

import math
import struct
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


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


def se3_from_pose(pose: Pose) -> np.ndarray:
    p = pose.position
    q = pose.orientation
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


def se3_from_rt(R9, t3) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R9, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(t3, dtype=np.float64).reshape(3)
    return T


def rpy_deg_to_R(rpy_deg) -> np.ndarray:
    r, p, y = [math.radians(float(v)) for v in rpy_deg]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    return Rz @ Ry @ Rx


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
        self.declare_parameter('cloud_topic', '/odin1/cloud_slam')
        self.declare_parameter('odom_topic', '/odin1/odometry')
        self.declare_parameter('mid_odom_topic', '/Odometry')
        self.declare_parameter('map_frame', 'odin_map')
        self.declare_parameter('parent_frame', 'map')
        # true: only publish map→odin_map TF (CT-LIO owns cloud/accum topics)
        self.declare_parameter('align_only', False)
        # true: cloud already in odom (cloud_slam / CT-LIO registered). false: raw × odom.
        self.declare_parameter('cloud_already_in_odom', True)
        self.declare_parameter('stride', 2)
        self.declare_parameter('blind', 0.3)
        self.declare_parameter('min_confidence', 0)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        self.declare_parameter('accum_skip_sec', 2.0)
        self.declare_parameter('odom_buffer_sec', 3.0)
        self.declare_parameter('imu_to_lidar_xyz', [-0.004, 0.013, 0.026])
        self.declare_parameter('imu_to_lidar_rpy_deg', [0.66, -0.11, -0.29])
        # Visual-only TF lock (does not fuse poses into the cloud).
        #   extrinsic : mode0 Mid↔Odin calib (default) — T_map_odinmap at lock
        #   off / yaw_xy / icp_yaw_xy / full : alternatives
        self.declare_parameter('align_mode', 'extrinsic')
        self.declare_parameter('align_delay_sec', 4.0)
        self.declare_parameter('mid_accum_topic', '/viz/accum_mid360')
        # Same numbers as mid360_odin1.yaml mapping.extrinsic_* (mode0 calibration)
        self.declare_parameter('extrinsic_T', [-0.011, -0.02329, 0.04412])
        self.declare_parameter(
            'extrinsic_R',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [0.150, 0.0, 0.035])
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [
                0.97317887, 0.0, -0.23004974,
                0.0, 1.0, 0.0,
                0.23004974, 0.0, 0.97317887,
            ],
        )

        self.cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.mid_odom_topic = str(self.get_parameter('mid_odom_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.align_only = bool(self.get_parameter('align_only').value)
        self.cloud_already_in_odom = bool(self.get_parameter('cloud_already_in_odom').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.blind2 = float(self.get_parameter('blind').value) ** 2
        self.min_confidence = int(self.get_parameter('min_confidence').value)
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self.odom_buffer_sec = float(self.get_parameter('odom_buffer_sec').value)
        self.align_mode = str(self.get_parameter('align_mode').value).strip().lower()
        self.align_delay_sec = float(self.get_parameter('align_delay_sec').value)
        self.mid_accum_topic = str(self.get_parameter('mid_accum_topic').value)
        self.align_to_mid_once = self.align_mode not in ('off', 'none', 'identity', 'false', '0')
        self._t0 = time.time()
        self._mid_cloud: Optional[np.ndarray] = None
        self._odin_cloud: Optional[np.ndarray] = None
        self._align_timer_started = False

        xyz = list(self.get_parameter('imu_to_lidar_xyz').value)
        rpy = list(self.get_parameter('imu_to_lidar_rpy_deg').value)
        self.T_imu_lidar = np.eye(4, dtype=np.float64)
        self.T_imu_lidar[:3, :3] = rpy_deg_to_R(rpy)
        self.T_imu_lidar[:3, 3] = [float(v) for v in xyz]

        # Mid IMU→L1, L1→L2 (Odin) — mode0 extrinsic used only for map↔odin_map TF
        self.T_imu_L1 = se3_from_rt(
            self.get_parameter('extrinsic_R').value,
            self.get_parameter('extrinsic_T').value,
        )
        self.T_L1_L2 = se3_from_rt(
            self.get_parameter('extrinsic_R_L2_wrt_L1').value,
            self.get_parameter('extrinsic_T_L2_wrt_L1').value,
        )
        # Mid body(=imu) → Odin lidar
        self.T_body_odin_lidar = self.T_imu_L1 @ self.T_L1_L2

        self._accum = VoxelAccum(voxel, max_n)
        self._dirty = False
        self._odom_buf: Deque[Tuple[float, np.ndarray]] = deque()
        self._mid_T: Optional[np.ndarray] = None
        self._odin_T: Optional[np.ndarray] = None
        self._aligned = False

        from rclpy.qos import qos_profile_sensor_data

        self.pub_scan = None
        self.pub_accum = None
        if not self.align_only:
            self.pub_scan = self.create_publisher(PointCloud2, '/odin1/cloud_in_odom', 5)
            self.pub_accum = self.create_publisher(PointCloud2, '/viz/accum_odin', 1)
        self._static_br = StaticTransformBroadcaster(self)
        self._publish_identity_tf()

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        if self.align_to_mid_once:
            self.create_subscription(Odometry, self.mid_odom_topic, self.on_mid_odom, 50)
            if self.align_mode.startswith('icp') and not self.align_only:
                self.create_subscription(
                    PointCloud2, self.mid_accum_topic, self.on_mid_accum, 1
                )
        if not self.align_only:
            self.create_subscription(
                PointCloud2, self.cloud_topic, self.on_cloud, qos_profile_sensor_data
            )
            self.create_timer(max(0.2, period), self.publish_accum)
            self.create_service(Trigger, '~/clear_accum', self.on_clear)
        if self.align_to_mid_once:
            self.create_timer(0.5, self._try_align)

        self.get_logger().info(
            f'INDEPENDENT Odin: cloud={self.cloud_topic} odom={self.odom_topic} '
            f'frame={self.map_frame} already_in_odom={self.cloud_already_in_odom} '
            f'align_only={self.align_only} align_mode={self.align_mode} '
            f'(no Mid pose for points)'
        )

    def _publish_identity_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.map_frame
        t.transform.rotation.w = 1.0
        self._static_br.sendTransform(t)

    def _publish_align_tf(self, T_parent_child: np.ndarray) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.map_frame
        t.transform.translation.x = float(T_parent_child[0, 3])
        t.transform.translation.y = float(T_parent_child[1, 3])
        t.transform.translation.z = float(T_parent_child[2, 3])
        x, y, z, w = rot_to_quat(T_parent_child[:3, :3])
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self._static_br.sendTransform(t)
        self.get_logger().info(
            f'Locked visual TF {self.parent_frame}->{self.map_frame} '
            f't=[{t.transform.translation.x:.3f},{t.transform.translation.y:.3f},'
            f'{t.transform.translation.z:.3f}] (one-shot, maps stay independent)'
        )

    def on_clear(self, _req, resp):
        self._accum.clear()
        self._dirty = True
        self._t0 = time.time()
        resp.success = True
        resp.message = 'cleared independent Odin accum'
        return resp

    def on_mid_odom(self, msg: Odometry) -> None:
        if self._aligned:
            return
        self._mid_T = se3_from_pose(msg.pose.pose)

    def on_odom(self, msg: Odometry) -> None:
        t = stamp_sec(msg.header.stamp)
        T = se3_from_pose(msg.pose.pose)
        self._odom_buf.append((t, T))
        cutoff = t - self.odom_buffer_sec
        while self._odom_buf and self._odom_buf[0][0] < cutoff:
            self._odom_buf.popleft()
        if not self._aligned:
            self._odin_T = T

    def on_mid_accum(self, msg: PointCloud2) -> None:
        if self._aligned:
            return
        arr = self._xyz_stride(msg)
        if len(arr) >= 200:
            self._mid_cloud = arr

    @staticmethod
    def _horiz_yaw(T: np.ndarray) -> float:
        """Yaw of body X projected onto horizontal plane (immune to pitch)."""
        return math.atan2(float(T[1, 0]), float(T[0, 0]))

    def _icp_yaw_xy(self, mid: np.ndarray, odin: np.ndarray) -> np.ndarray:
        """Coarse 2D yaw+xy that maximizes near-neighbor hits (visual only)."""
        def downsample(a: np.ndarray, n: int = 1500) -> np.ndarray:
            if len(a) <= n:
                return a[:, :2]
            idx = np.linspace(0, len(a) - 1, n).astype(np.int32)
            return a[idx, :2]

        m = downsample(mid)
        o = downsample(odin)
        best = None
        for yaw_deg in range(-180, 180, 5):
            yaw = math.radians(yaw_deg)
            c, s = math.cos(yaw), math.sin(yaw)
            R2 = np.array([[c, -s], [s, c]], dtype=np.float64)
            o_r = o @ R2.T
            # xy from centroid delta after rotation
            t = m.mean(0) - o_r.mean(0)
            o_t = o_r + t
            # score: fraction of odin pts within 0.35m of some mid pt (grid hash)
            inv = 1.0 / 0.35
            grid = {}
            for x, y in m:
                grid[(int(math.floor(x * inv)), int(math.floor(y * inv)))] = True
            hits = 0
            for x, y in o_t:
                key = (int(math.floor(x * inv)), int(math.floor(y * inv)))
                if key in grid:
                    hits += 1
                    continue
                # 1-cell neighborhood
                found = False
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if (key[0] + dx, key[1] + dy) in grid:
                            found = True
                            break
                    if found:
                        break
                if found:
                    hits += 1
            score = hits / max(len(o_t), 1)
            if best is None or score > best[0]:
                best = (score, yaw, t)
        assert best is not None
        score, yaw, t = best
        self.get_logger().info(
            f'ICP visual align score={score:.3f} yaw_deg={math.degrees(yaw):.1f} '
            f't=[{t[0]:.2f},{t[1]:.2f}]'
        )
        c, s = math.cos(yaw), math.sin(yaw)
        T = np.eye(4, dtype=np.float64)
        T[0, 0] = c
        T[0, 1] = -s
        T[1, 0] = s
        T[1, 1] = c
        T[0, 3] = float(t[0])
        T[1, 3] = float(t[1])
        return T

    def _try_align(self) -> None:
        if self._aligned or not self.align_to_mid_once:
            return
        if (time.time() - self._t0) < self.align_delay_sec:
            return

        if self.align_mode in ('extrinsic', 'mode0', 'calib'):
            # mode0 extrinsic: place odin_map so Odin lidar matches Mid's estimate at lock.
            # T_map_odinmap * T_odom_imu * T_imu_L2 = T_map_body * T_body_odin_lidar
            if self._mid_T is None or self._odin_T is None:
                return
            T_odom_odin_lidar = self._odin_T @ self.T_imu_lidar
            T_map_odin_lidar = self._mid_T @ self.T_body_odin_lidar
            T_map_odinmap = T_map_odin_lidar @ se3_inv(T_odom_odin_lidar)
            self.get_logger().info(
                'Align via mode0 extrinsic '
                f'T_L2=[{self.T_L1_L2[0,3]:.3f},{self.T_L1_L2[1,3]:.3f},{self.T_L1_L2[2,3]:.3f}]'
            )
        elif self.align_mode.startswith('icp'):
            if self._mid_cloud is None or len(self._accum) < 300:
                return
            odin = self._accum.numpy()
            if len(odin) < 300:
                return
            T_map_odinmap = self._icp_yaw_xy(self._mid_cloud, odin)
        elif self.align_mode in ('full', 'se3'):
            if self._mid_T is None or self._odin_T is None:
                return
            T_map_odinmap = self._mid_T @ se3_inv(self._odin_T)
        else:
            if self._mid_T is None or self._odin_T is None:
                return
            yaw = self._horiz_yaw(self._mid_T) - self._horiz_yaw(self._odin_T)
            c, s = math.cos(yaw), math.sin(yaw)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
            t = self._mid_T[:3, 3] - R @ self._odin_T[:3, 3]
            T_map_odinmap = np.eye(4, dtype=np.float64)
            T_map_odinmap[:3, :3] = R
            T_map_odinmap[0, 3] = float(t[0])
            T_map_odinmap[1, 3] = float(t[1])
            T_map_odinmap[2, 3] = 0.0
        self._publish_align_tf(T_map_odinmap)
        self._aligned = True

    def _nearest_odom(self, t_cloud: float) -> Optional[np.ndarray]:
        """Nearest by stamp. Prefer nearest over 'latest' when cloud is delayed."""
        if not self._odom_buf:
            return None
        best = None
        best_dt = 1e9
        for t, T in self._odom_buf:
            dt = abs(t - t_cloud)
            if dt < best_dt:
                best_dt = dt
                best = T
        return None if best is None else best.copy()

    def _xyz_stride(self, msg: PointCloud2) -> np.ndarray:
        offs = {f.name: f.offset for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return np.zeros((0, 3), dtype=np.float64)
        has_conf = 'confidence' in offs
        pts: list[list[float]] = []
        n = msg.width * msg.height
        k = 0
        for i in range(n):
            b = i * msg.point_step
            x, y, z = struct.unpack_from('fff', msg.data, b + offs['x'])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if has_conf and self.min_confidence > 0:
                # confidence often uint8/uint16/float — try float then u8
                try:
                    conf = struct.unpack_from('f', msg.data, b + offs['confidence'])[0]
                except Exception:
                    conf = float(msg.data[b + offs['confidence']])
                if conf < self.min_confidence:
                    continue
            if (not self.cloud_already_in_odom) and (x * x + y * y + z * z) < self.blind2:
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
            T_odom_imu = self._nearest_odom(stamp_sec(msg.header.stamp))
            if T_odom_imu is None:
                return
            T_odom_lidar = T_odom_imu @ self.T_imu_lidar
            R, t = T_odom_lidar[:3, :3], T_odom_lidar[:3, 3]
            world = (arr @ R.T) + t
            stamp = msg.header.stamp

        if self.pub_scan is not None:
            header = Header()
            header.stamp = stamp
            header.frame_id = self.map_frame
            self.pub_scan.publish(make_cloud(header, world))

        if (time.time() - self._t0) >= self.accum_skip_sec:
            if self._accum.add(world):
                self._dirty = True

    def publish_accum(self) -> None:
        if self.pub_accum is None:
            return
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
