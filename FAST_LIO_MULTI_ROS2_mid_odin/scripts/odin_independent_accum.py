#!/usr/bin/env python3
"""Independent Odin1 map accumulation — NO Mid360 pose association.

Preferred (mode 1 SLAM):
  /odin1/cloud_slam  already in odom  → voxel as-is  (do NOT re-multiply odom)

Fallback (mode 0 / raw / Plan A ground):
  lidar-frame cloud × (/odin1/odometry as imu pose) × T_imu_lidar
  Uses nearest odom by device stamp (never "latest while cloud is delayed").

Plan B1 (densify with Odin odom into Mid map):
  publish_in_parent_frame=true → after one-shot extrinsic align, transform
  every scan/accum into parent_frame (usually map) and publish as map cloud.
  Mid /Odometry stays the system pose; this node only paints Odin ground.

Publishes (defaults):
  /odin1/cloud_in_odom   current scan (odin_map or parent if projected)
  /viz/accum_odin       voxel accum
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
        self.declare_parameter('out_scan_topic', '/odin1/cloud_in_odom')
        self.declare_parameter('out_accum_topic', '/viz/accum_odin')
        # true: only publish map→odin_map TF (CT-LIO owns cloud/accum topics)
        self.declare_parameter('align_only', False)
        # true: cloud already in odom (cloud_slam / CT-LIO registered). false: raw × odom.
        self.declare_parameter('cloud_already_in_odom', True)
        # true: lidar-frame cloud × Mid /Odometry at the same stamp × extrinsic.
        # One-shot yaw lock cannot follow two odometries that drift apart, so
        # later scans were voxelized away from the wall they belong to.
        self.declare_parameter('project_with_mid', False)
        # After extrinsic lock, transform scans into parent_frame (Mid map) for B1 densify.
        self.declare_parameter('publish_in_parent_frame', False)
        # Hold publishing until align lock when projecting into parent (avoid smeared start).
        self.declare_parameter('wait_align_for_parent', True)
        self.declare_parameter('stride', 2)
        self.declare_parameter('blind', 0.3)
        self.declare_parameter('min_confidence', 0)
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_accum_points', 400000)
        self.declare_parameter('accum_publish_period', 0.5)
        self.declare_parameter('accum_skip_sec', 2.0)
        self.declare_parameter('odom_buffer_sec', 3.0)
        self.declare_parameter('odom_pair_max_dt', 0.20)
        # Autoware/LIO-SAM style: warp Odin scan from its stamp pose to the
        # latest Mid /Odometry pose so live viz matches robot/Mid motion.
        # (MA-LIO / CTE-MLO use continuous-time; SE3 relative warp is the
        # practical equivalent when Odin arrives ~200ms late over TCP.)
        # Live Odin scan warp to Mid pose (viz only). Does not touch Mid IEKF.
        self.declare_parameter('motion_sync_to_mid', True)
        self.declare_parameter('motion_sync_max_dt', 0.80)
        # Skip warp under Mid noise when robot is still (else Odin shakes).
        self.declare_parameter('motion_sync_min_trans', 0.02)
        self.declare_parameter('motion_sync_min_rot_deg', 0.5)
        # Mid IEKF still steps a few mm / a few tenths of a degree while the
        # robot is still. Reprojecting every Odin scan with that pose makes the
        # cloud shake. Keep the last pose until real motion leaves this band.
        self.declare_parameter('pose_hold_trans_m', 0.025)
        self.declare_parameter('pose_hold_yaw_deg', 0.8)
        # Official Odin IMU→lidar (wiki), R≈I — matches downsample bridge.
        self.declare_parameter('imu_to_lidar_xyz', [-0.02663, 0.03447, 0.02174])
        self.declare_parameter('imu_to_lidar_rpy_deg', [0.0, 0.0, 0.0])
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
        self.declare_parameter('extrinsic_T_L2_wrt_L1', [0.12157888, 0.00549610, 0.03169932])
        self.declare_parameter(
            'extrinsic_R_L2_wrt_L1',
            [
                0.97437006, 0.0, -0.22495105,
                0.0, 1.0, 0.0,
                0.22495105, 0.0, 0.97437006,
            ],
        )

        self.cloud_topic = str(self.get_parameter('cloud_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.mid_odom_topic = str(self.get_parameter('mid_odom_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.out_scan_topic = str(self.get_parameter('out_scan_topic').value)
        self.out_accum_topic = str(self.get_parameter('out_accum_topic').value)
        self.align_only = bool(self.get_parameter('align_only').value)
        self.cloud_already_in_odom = bool(self.get_parameter('cloud_already_in_odom').value)
        self.project_with_mid = bool(self.get_parameter('project_with_mid').value)
        self.publish_in_parent_frame = bool(self.get_parameter('publish_in_parent_frame').value)
        self.wait_align_for_parent = bool(self.get_parameter('wait_align_for_parent').value)
        self.stride = max(1, int(self.get_parameter('stride').value))
        self.blind2 = float(self.get_parameter('blind').value) ** 2
        self.min_confidence = int(self.get_parameter('min_confidence').value)
        voxel = float(self.get_parameter('voxel_size').value)
        max_n = int(self.get_parameter('max_accum_points').value)
        period = float(self.get_parameter('accum_publish_period').value)
        self.accum_skip_sec = float(self.get_parameter('accum_skip_sec').value)
        self.odom_buffer_sec = float(self.get_parameter('odom_buffer_sec').value)
        self.odom_pair_max_dt = float(self.get_parameter('odom_pair_max_dt').value)
        self.motion_sync_to_mid = bool(self.get_parameter('motion_sync_to_mid').value)
        self.motion_sync_max_dt = float(self.get_parameter('motion_sync_max_dt').value)
        self.motion_sync_min_trans = float(self.get_parameter('motion_sync_min_trans').value)
        self.motion_sync_min_rot_deg = float(self.get_parameter('motion_sync_min_rot_deg').value)
        self.pose_hold_trans_m = float(self.get_parameter('pose_hold_trans_m').value)
        self.pose_hold_yaw_deg = float(self.get_parameter('pose_hold_yaw_deg').value)
        self._held_mid_T: Optional[np.ndarray] = None
        self._held_odin_T: Optional[np.ndarray] = None
        self._n_motion_hold = 0
        self.align_mode = str(self.get_parameter('align_mode').value).strip().lower()
        self.align_delay_sec = float(self.get_parameter('align_delay_sec').value)
        self.mid_accum_topic = str(self.get_parameter('mid_accum_topic').value)
        self.align_to_mid_once = self.align_mode not in ('off', 'none', 'identity', 'false', '0')
        self._t0 = time.time()
        self._mid_cloud: Optional[np.ndarray] = None
        self._odin_cloud: Optional[np.ndarray] = None
        self._align_timer_started = False
        self._n_pub = 0
        self._n_skip_odom = 0
        self._n_skip_align = 0
        self._n_motion_sync = 0
        self._last_sync_dt = 0.0

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
        self._mid_odom_buf: Deque[Tuple[float, np.ndarray]] = deque()
        self._mid_T: Optional[np.ndarray] = None
        self._mid_stamp_latest = None
        self._odin_T: Optional[np.ndarray] = None
        self._aligned = False
        self._T_parent_child = np.eye(4, dtype=np.float64)

        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        reliable = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub_scan = None
        self.pub_accum = None
        if not self.align_only:
            self.pub_scan = self.create_publisher(PointCloud2, self.out_scan_topic, 5)
            self.pub_accum = self.create_publisher(PointCloud2, self.out_accum_topic, 1)
        self._static_br = StaticTransformBroadcaster(self)
        if not self.project_with_mid:
            self._publish_identity_tf()

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        # Always buffer Mid odom for motion sync (and one-shot align).
        if self.align_to_mid_once or self.motion_sync_to_mid or self.project_with_mid:
            self.create_subscription(Odometry, self.mid_odom_topic, self.on_mid_odom, 50)
            if self.align_mode.startswith('icp') and not self.align_only:
                self.create_subscription(
                    PointCloud2, self.mid_accum_topic, self.on_mid_accum, 1
                )
        if not self.align_only:
            # Prefer RELIABLE (ground bridge); also OK for most Odin TCP republish.
            self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, reliable)
            self.create_timer(max(0.2, period), self.publish_accum)
            self.create_timer(2.0, self._status)
            self.create_service(Trigger, '~/clear_accum', self.on_clear)
        if self.align_to_mid_once:
            self.create_timer(0.5, self._try_align)

        self.get_logger().info(
            f'INDEPENDENT Odin: cloud={self.cloud_topic} odom={self.odom_topic} '
            f'already_in_odom={self.cloud_already_in_odom} project_with_mid={self.project_with_mid} '
            f'align_only={self.align_only} align_mode={self.align_mode} '
            f'publish_in_parent={self.publish_in_parent_frame} '
            f'motion_sync_to_mid={self.motion_sync_to_mid} '
            f'out_scan={self.out_scan_topic}'
        )

    def _status(self) -> None:
        self.get_logger().info(
            f'odin_odom_accum pub={self._n_pub} skip_odom={self._n_skip_odom} '
            f'skip_align={self._n_skip_align} aligned={self._aligned} '
            f'accum_n={len(self._accum)} odom_buf={len(self._odom_buf)} '
            f'motion_sync={self._n_motion_sync} hold={self._n_motion_hold} '
            f'sync_dt={self._last_sync_dt*1000:.0f}ms mid_odom_buf={len(self._mid_odom_buf)}'
        )

    def _output_frame(self) -> str:
        if self.project_with_mid:
            return self.parent_frame
        if self.publish_in_parent_frame and self._aligned:
            return self.parent_frame
        return self.map_frame

    def _to_output_frame(self, pts_odin: np.ndarray) -> Optional[np.ndarray]:
        """pts in odin odom/map_frame → optional parent (Mid map) via locked align."""
        if not self.publish_in_parent_frame:
            return pts_odin
        if not self._aligned:
            if self.wait_align_for_parent:
                return None
            return pts_odin
        R = self._T_parent_child[:3, :3]
        t = self._T_parent_child[:3, 3]
        return (pts_odin @ R.T) + t

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
        # Both streams are on the Livox scan clock now. Index by header stamp
        # so a late cloud still uses the pose from when it was captured.
        t = stamp_sec(msg.header.stamp)
        T = se3_from_pose(msg.pose.pose)
        self._mid_T = T
        self._mid_stamp_latest = msg.header.stamp
        self._mid_odom_buf.append((t, T))
        cutoff = t - self.odom_buffer_sec
        while self._mid_odom_buf and self._mid_odom_buf[0][0] < cutoff:
            self._mid_odom_buf.popleft()

    def _nearest_mid_odom(self, t_query: float, max_dt: Optional[float] = None):
        if not self._mid_odom_buf:
            return None
        best = None
        best_dt = 1e9
        for t, T in self._mid_odom_buf:
            dt = abs(t - t_query)
            if dt < best_dt:
                best_dt = dt
                best = (t, T)
        limit = self.motion_sync_max_dt if max_dt is None else max_dt
        if best is None or best_dt > limit:
            return None
        return best

    def _stable_pose(self, T: np.ndarray, held_name: str) -> np.ndarray:
        """Reuse the last pose while the change is inside odometry noise."""
        held = getattr(self, held_name)
        if held is None:
            held = T.copy()
            setattr(self, held_name, held)
            return held
        dyaw = self._horiz_yaw(T) - self._horiz_yaw(held)
        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi
        dxyz = T[:3, 3] - held[:3, 3]
        if (
            float(np.linalg.norm(dxyz)) < self.pose_hold_trans_m
            and abs(math.degrees(dyaw)) < self.pose_hold_yaw_deg
        ):
            self._n_motion_hold += 1
            return held
        held = T.copy()
        setattr(self, held_name, held)
        return held

    def _warp_to_latest_mid(
        self, pts: np.ndarray, t_cloud: float
    ) -> Tuple[np.ndarray, Optional[object], float]:
        """Warp map-frame points from pose@t_cloud to latest Mid pose (SE3).

        p_dst = T_mid(t_now) * inv(T_mid(t_cloud)) * p_src
        Same idea as Autoware concatenate motion compensation / LIO-SAM deskew
        relative transform, applied to a delayed second LiDAR.
        """
        if (not self.motion_sync_to_mid) or pts.size == 0 or not self._mid_odom_buf:
            return pts, None, 0.0
        src = self._nearest_mid_odom(t_cloud)
        if src is None:
            return pts, None, 0.0
        t_src, T_src = src
        t_dst, T_dst = self._mid_odom_buf[-1]
        dt = float(t_dst - t_src)
        if abs(dt) < 1e-4:
            return pts, None, dt
        T = T_dst @ se3_inv(T_src)
        R = T[:3, :3]
        tr = T[:3, 3]
        trans = float(np.linalg.norm(tr))
        # Rotation angle from trace(R): stable for small Mid noise.
        cos_a = 0.5 * (float(R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)
        cos_a = max(-1.0, min(1.0, cos_a))
        rot_deg = math.degrees(math.acos(cos_a))
        if trans < self.motion_sync_min_trans and rot_deg < self.motion_sync_min_rot_deg:
            # Stationary (or Mid noise only): keep Odin pose frozen.
            self._n_motion_hold += 1
            self._last_sync_dt = dt
            return pts, None, dt
        out = (pts @ R.T) + tr
        self._n_motion_sync += 1
        self._last_sync_dt = dt
        return out, self._mid_stamp_latest, dt

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
        self._T_parent_child = T_map_odinmap
        self._publish_align_tf(T_map_odinmap)
        self._aligned = True

    def _nearest_odom(self, t_cloud: float, max_dt: Optional[float] = None) -> Optional[np.ndarray]:
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
        limit = self.odom_pair_max_dt if max_dt is None else max_dt
        if best is None or best_dt > limit:
            return None
        return best.copy()

    def _xyz_stride(self, msg: PointCloud2) -> np.ndarray:
        """Vectorized XYZ extract — Python per-point unpack was a major lag source."""
        offs = {f.name: (f.offset, f.datatype) for f in msg.fields}
        if not all(k in offs for k in ('x', 'y', 'z')):
            return np.zeros((0, 3), dtype=np.float64)
        n = int(msg.width) * int(msg.height)
        if n <= 0:
            return np.zeros((0, 3), dtype=np.float64)
        step = int(msg.point_step)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < n * step:
            n = raw.size // step
        if n <= 0:
            return np.zeros((0, 3), dtype=np.float64)
        arr = raw[: n * step].reshape(n, step)

        def col_f32(name: str) -> np.ndarray:
            o = offs[name][0]
            return arr[:, o : o + 4].copy().view(np.float32).reshape(-1)

        xyz = np.column_stack([col_f32('x'), col_f32('y'), col_f32('z')]).astype(np.float64)
        keep = np.isfinite(xyz).all(axis=1)
        if 'confidence' in offs and self.min_confidence > 0:
            o, dt = offs['confidence']
            if dt == 7:  # FLOAT32
                conf = arr[:, o : o + 4].copy().view(np.float32).reshape(-1)
            elif dt == 4:  # UINT16
                conf = arr[:, o : o + 2].copy().view(np.uint16).reshape(-1).astype(np.float32)
            else:
                conf = arr[:, o].astype(np.float32)
            keep &= conf >= float(self.min_confidence)
        if not self.cloud_already_in_odom and self.blind2 > 0.0:
            keep &= (xyz[:, 0] ** 2 + xyz[:, 1] ** 2 + xyz[:, 2] ** 2) >= self.blind2
        xyz = xyz[keep]
        if self.stride > 1 and len(xyz) > 0:
            xyz = xyz[:: int(self.stride)]
        return xyz

    def on_cloud(self, msg: PointCloud2) -> None:
        arr = self._xyz_stride(msg)
        if arr.size == 0:
            return

        if self.project_with_mid:
            t_cloud = stamp_sec(msg.header.stamp)
            hit = self._nearest_mid_odom(t_cloud, max_dt=0.08)
            if hit is None:
                self._n_skip_odom += 1
                return
            T_mid = self._stable_pose(hit[1], '_held_mid_T')
            if self.cloud_already_in_odom:
                # Odom-frame cloud. Unprojecting with the live Odin pose shakes
                # the body cloud while the robot is still.
                T_odin = self._nearest_odom(t_cloud, max_dt=0.08)
                if T_odin is None:
                    self._n_skip_odom += 1
                    return
                T_odin = self._stable_pose(T_odin, '_held_odin_T')
                T_map = (
                    T_mid
                    @ self.T_body_odin_lidar
                    @ se3_inv(self.T_imu_lidar)
                    @ se3_inv(T_odin)
                )
            else:
                T_map = T_mid @ self.T_body_odin_lidar
            R, t = T_map[:3, :3], T_map[:3, 3]
            world = (arr @ R.T) + t
            stamp = msg.header.stamp
        elif self.cloud_already_in_odom:
            world = arr
            stamp = msg.header.stamp
        else:
            T_odom_imu = self._nearest_odom(stamp_sec(msg.header.stamp))
            if T_odom_imu is None:
                self._n_skip_odom += 1
                return
            T_odom_lidar = T_odom_imu @ self.T_imu_lidar
            R, t = T_odom_lidar[:3, :3], T_odom_lidar[:3, 3]
            world = (arr @ R.T) + t
            stamp = msg.header.stamp

        out = self._to_output_frame(world)
        if out is None:
            self._n_skip_align += 1
            return

        # Live scan: warp to latest Mid pose for visual/control sync.
        # Accum keeps capture-time geometry (do not smear the map).
        # Use cloud stamp when it is on Jetson clock (restamp_on_receive);
        # otherwise fall back to "now - small lag" via ROS clock.
        t_cloud = stamp_sec(stamp)
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        # If stamp looks like sensor epoch (far from now), use receive estimate.
        if abs(now_ros - t_cloud) > 2.0:
            t_cloud = now_ros - 0.05
        out_live, sync_stamp, _sync_dt = self._warp_to_latest_mid(out, t_cloud)
        # Only retouch stamp when a real warp happened; otherwise keep cloud
        # stamp so stationary frames are bit-stable in time too.
        pub_stamp = self.get_clock().now().to_msg() if sync_stamp is not None else stamp

        if self.pub_scan is not None:
            header = Header()
            header.stamp = pub_stamp
            header.frame_id = self._output_frame()
            self.pub_scan.publish(make_cloud(header, out_live))
            self._n_pub += 1

        if (time.time() - self._t0) >= self.accum_skip_sec:
            # Accum always stores output-frame points (parent map when B1).
            if self._accum.add(out):
                self._dirty = True

    def publish_accum(self) -> None:
        if self.pub_accum is None:
            return
        if not (self._dirty or len(self._accum)):
            return
        if self.publish_in_parent_frame and self.wait_align_for_parent and not self._aligned:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._output_frame()
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
