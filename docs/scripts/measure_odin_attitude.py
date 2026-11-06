#!/usr/bin/env python3
"""Measure Odin1 attitude relative to Mid360 (live).

Modes:
  gravity — yaw-free R from accelerometer directions (Mid IMU ← Odin IMU)
  icp     — residual SE3 of /odin1/cloud_in_map → /cloud_registered, then
            R_L2_new ≈ R_delta @ R_L2_old
  both    — run gravity then icp (default)

Examples:
  python3 measure_odin_attitude.py --mode gravity
  python3 measure_odin_attitude.py --mode both --l2-rpy-deg 0 -13 0
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, Optional, Tuple

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
            [2 * (x * z + y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rpy_deg(R: np.ndarray) -> np.ndarray:
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.rad2deg([roll, pitch, yaw])


def rot_angle_deg(A: np.ndarray, B: np.ndarray) -> float:
    R = A.T @ B
    return math.degrees(math.acos(float(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))


def gravity_align(g_from: np.ndarray, g_to: np.ndarray) -> np.ndarray:
    """Minimal rotation R with R @ g_from ≈ g_to (direction only)."""
    a = g_from / (np.linalg.norm(g_from) + 1e-12)
    b = g_to / (np.linalg.norm(g_to) + 1e-12)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-8:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def rpy_deg_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    r, p, y = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def read_xyz(
    msg: PointCloud2,
    max_n: int = 60000,
    stride: int = 1,
    rmin: float = 0.8,
    rmax: float = 12.0,
    zmin: float = -1.5,
    zmax: float = 3.0,
) -> np.ndarray:
    pts = []
    for i, p in enumerate(
        point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ):
        if stride > 1 and (i % stride):
            continue
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        r = math.hypot(x, y)
        if rmin <= r <= rmax and zmin <= z <= zmax:
            pts.append((x, y, z))
        if len(pts) >= max_n:
            break
    return np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 3))


def voxel_down(P: np.ndarray, v: float = 0.10) -> np.ndarray:
    if len(P) == 0:
        return P
    q = np.floor(P / v).astype(np.int64)
    key = q[:, 0] * 73856093 ^ q[:, 1] * 19349663 ^ q[:, 2] * 83492791
    _, idx = np.unique(key, return_index=True)
    return P[idx]


def nearest_idx(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    idx = np.empty(len(src), dtype=np.int64)
    for i in range(0, len(src), 2000):
        s = src[i : i + 2000]
        d2 = ((s[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        idx[i : i + 2000] = d2.argmin(1)
    return idx


def icp_se3(
    src: np.ndarray, dst: np.ndarray, iters: int = 40, trim: float = 0.55
) -> Tuple[np.ndarray, float, float]:
    """Rigid T: p_dst ≈ T @ p_src. Returns (T, median_nn, mean_nn)."""
    T = np.eye(4)
    X = src.copy()
    for _ in range(iters):
        j = nearest_idx(X, dst)
        Y = dst[j]
        d = np.linalg.norm(X - Y, axis=1)
        thr = np.percentile(d, trim * 100.0)
        m = d <= max(thr, 0.05)
        if int(m.sum()) < 50:
            break
        A, B = X[m], Y[m]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt = Vt.copy()
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = cb - R @ ca
        X = (R @ X.T).T + t
        dT = np.eye(4)
        dT[:3, :3] = R
        dT[:3, 3] = t
        T = dT @ T
        if np.linalg.norm(t) < 1e-4 and rot_angle_deg(np.eye(3), R) < 0.05:
            break
    j = nearest_idx(X, dst)
    d = np.linalg.norm(X - dst[j], axis=1)
    return T, float(np.median(d)), float(np.mean(d))


def print_R(name: str, R: np.ndarray) -> None:
    print(f"\n=== {name} ===")
    print(np.array2string(R, precision=6, suppress_small=True))
    print("rpy_deg [roll,pitch,yaw] =", np.round(rpy_deg(R), 4).tolist())
    print("|R-I|_deg =", round(rot_angle_deg(np.eye(3), R), 4))
    flat = ", ".join(f"{v:.8f}" for v in R.reshape(-1))
    print(f"yaml R flat:\n  extrinsic_R_L2_wrt_L1: [{flat}]")


class MeasureNode(Node):
    def __init__(self) -> None:
        super().__init__("measure_odin_attitude")
        self.buf: Dict[str, object] = {}
        self.acc_mid = []
        self.acc_odin = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        def take(key: str, keep_latest: bool = True):
            def cb(msg):
                if keep_latest or key not in self.buf:
                    self.buf[key] = msg

            return cb

        self.create_subscription(Imu, "/livox/imu", self._on_mid_imu, qos)
        self.create_subscription(Imu, "/odin1/imu", self._on_odin_imu, qos)
        self.create_subscription(Odometry, "/Odometry", take("mid_odom"), 10)
        self.create_subscription(Odometry, "/odin1/odometry", take("odin_odom"), qos)
        self.create_subscription(PointCloud2, "/cloud_registered", take("mid_cloud"), qos)
        self.create_subscription(
            PointCloud2, "/odin1/cloud_in_map", take("odin_map"), qos
        )

    def _on_mid_imu(self, msg: Imu) -> None:
        self.buf["mid_imu"] = msg
        a = msg.linear_acceleration
        self.acc_mid.append([a.x, a.y, a.z])

    def _on_odin_imu(self, msg: Imu) -> None:
        self.buf["odin_imu"] = msg
        a = msg.linear_acceleration
        self.acc_odin.append([a.x, a.y, a.z])


def run_gravity(node: MeasureNode, collect_sec: float = 3.0) -> np.ndarray:
    print(f"\n[gravity] collecting IMU for {collect_sec:.1f}s ...")
    node.acc_mid.clear()
    node.acc_odin.clear()
    t0 = time.time()
    while time.time() - t0 < collect_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(node.acc_mid) < 10 or len(node.acc_odin) < 10:
        raise RuntimeError(
            f"not enough IMU samples mid={len(node.acc_mid)} odin={len(node.acc_odin)}"
        )
    am = np.mean(node.acc_mid, axis=0)
    ao = np.mean(node.acc_odin, axis=0)
    print("Mid  acc mean", np.round(am, 4).tolist(), "norm", round(float(np.linalg.norm(am)), 4))
    print("Odin acc mean", np.round(ao, 4).tolist(), "norm", round(float(np.linalg.norm(ao)), 4))
    R = gravity_align(ao, am)
    print_R("Gravity R_midIMU_odinIMU (≈ R_L1_L2 if Mid extrinsic R=I)", R)

    if "mid_odom" in node.buf and "odin_odom" in node.buf:
        qm = node.buf["mid_odom"].pose.pose.orientation
        qo = node.buf["odin_odom"].pose.pose.orientation
        Rm = quat_to_R(qm.x, qm.y, qm.z, qm.w)
        Ro = quat_to_R(qo.x, qo.y, qo.z, qo.w)
        print("\nAbsolute odom rpy (yaw origins independent, compare pitch/roll only):")
        print("  Mid /Odometry      ", np.round(rpy_deg(Rm), 3).tolist())
        print("  Odin /odin1/odometry", np.round(rpy_deg(Ro), 3).tolist())
        print(
            "  Δ(roll,pitch) Odin-Mid",
            np.round(rpy_deg(Ro)[:2] - rpy_deg(Rm)[:2], 3).tolist(),
        )
    return R


def run_icp(
    node: MeasureNode,
    R_l2_old: np.ndarray,
    wait_sec: float = 6.0,
) -> Tuple[np.ndarray, np.ndarray]:
    print(f"\n[icp] waiting clouds up to {wait_sec:.1f}s ...")
    node.buf.pop("mid_cloud", None)
    node.buf.pop("odin_map", None)
    t0 = time.time()
    while time.time() - t0 < wait_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
        if "mid_cloud" in node.buf and "odin_map" in node.buf:
            break
    if "mid_cloud" not in node.buf or "odin_map" not in node.buf:
        raise RuntimeError("missing /cloud_registered or /odin1/cloud_in_map")

    Pm = voxel_down(read_xyz(node.buf["mid_cloud"], stride=2), 0.10)
    Po = voxel_down(read_xyz(node.buf["odin_map"], stride=1), 0.10)
    print(f"voxel Mid={len(Pm)} Odin_in_map={len(Po)}")
    if len(Pm) < 200 or len(Po) < 200:
        raise RuntimeError("too few points after voxel filter")

    T_delta, med, mean = icp_se3(Po, Pm, iters=40, trim=0.55)
    R_delta = T_delta[:3, :3]
    t_delta = T_delta[:3, 3]
    print("\nICP residual T_delta (Odin_in_map → Mid cloud_registered):")
    print("  t_mm", np.round(t_delta * 1000, 2).tolist())
    print("  rpy_deg", np.round(rpy_deg(R_delta), 4).tolist())
    print("  |R_delta|_deg", round(rot_angle_deg(np.eye(3), R_delta), 4))
    print("  nn median/mean m", round(med, 4), round(mean, 4))

    R_new = R_delta @ R_l2_old
    print_R("Suggested R_L2_new = R_delta @ R_L2_old", R_new)
    print(
        "\nvs old L2: ΔR_deg =",
        round(rot_angle_deg(R_l2_old, R_new), 4),
        " old rpy",
        np.round(rpy_deg(R_l2_old), 3).tolist(),
    )
    return R_delta, R_new


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--mode",
        choices=("gravity", "icp", "both"),
        default="both",
        help="measurement mode (default: both)",
    )
    ap.add_argument(
        "--l2-rpy-deg",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        default=[0.0, -13.0, 0.0],
        help="current extrinsic_R_L2_wrt_L1 as RPY ZYX degrees (default: 0 -13 0)",
    )
    ap.add_argument("--collect-sec", type=float, default=3.0)
    ap.add_argument("--icp-wait-sec", type=float, default=6.0)
    args = ap.parse_args()

    R_l2_old = rpy_deg_to_R(*args.l2_rpy_deg)
    print("Current L2 RPY deg (input):", args.l2_rpy_deg)
    print_R("R_L2_old", R_l2_old)

    rclpy.init()
    node = MeasureNode()
    try:
        R_grav: Optional[np.ndarray] = None
        if args.mode in ("gravity", "both"):
            R_grav = run_gravity(node, collect_sec=args.collect_sec)
            print(
                "\nGravity vs CAD/old L2 ΔR_deg =",
                round(rot_angle_deg(R_l2_old, R_grav), 4),
            )

        if args.mode in ("icp", "both"):
            _, R_new = run_icp(node, R_l2_old, wait_sec=args.icp_wait_sec)
            if R_grav is not None:
                print(
                    "\nICP-suggested vs gravity ΔR_deg =",
                    round(rot_angle_deg(R_grav, R_new), 4),
                )
            print(
                "\n>>> Paste into mid360_odin1.yaml / launch (attitude only; keep T):"
            )
            flat = ", ".join(f"{v:.8f}" for v in R_new.reshape(-1))
            print(f"extrinsic_R_L2_wrt_L1: [{flat}]")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
