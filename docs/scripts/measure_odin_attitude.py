#!/usr/bin/env python3
"""Measure Odin1 attitude live vs Mid360 or Go2.

--ref mid  (default): Mid IMU / map clouds → extrinsic_R_L2_wrt_L1
--ref go2: Go2 body IMU (lf/lowstate) → R_base_odin (≈ Go2 IMU / base_link)

Modes (mid):
  gravity — yaw-free R from accel (Mid ← Odin)
  icp     — residual /odin1/cloud_in_map → /cloud_registered
  both    — gravity + icp

Modes (go2):
  gravity — yaw-free R from accel (Go2 ← Odin); ICP vs Mid map 不适用 Go2 真值系

Examples:
  python3 measure_odin_attitude.py --ref mid --mode both --l2-rpy-deg 0 -10.15 0
  python3 measure_odin_attitude.py --ref go2 --mode gravity
"""
from __future__ import annotations

import argparse
import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

try:
    from unitree_go.msg import LowState
except ImportError:  # pragma: no cover
    LowState = None  # type: ignore


# URDF: Go2 base → Go2 IMU (attitude identical: R=I)
T_BASE_GO2IMU_T = np.array([-0.02557, 0.0, 0.04232], dtype=np.float64)
# Official Mid lidar in Go2 body-IMU frame
def Ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


R_GO2IMU_MIDL = Ry(13.0)


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
    print(f"R flat 9:\n  [{flat}]")


class MeasureNode(Node):
    def __init__(self, ref: str) -> None:
        super().__init__("measure_odin_attitude")
        self.ref = ref
        self.buf: Dict[str, object] = {}
        self.acc_ref: List[List[float]] = []
        self.acc_odin: List[List[float]] = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        def take(key: str):
            def cb(msg):
                self.buf[key] = msg

            return cb

        self.create_subscription(Imu, "/odin1/imu", self._on_odin_imu, qos)
        self.create_subscription(Odometry, "/odin1/odometry", take("odin_odom"), qos)

        if ref == "mid":
            self.create_subscription(Imu, "/livox/imu", self._on_ref_imu, qos)
            self.create_subscription(Odometry, "/Odometry", take("mid_odom"), 10)
            self.create_subscription(
                PointCloud2, "/cloud_registered", take("mid_cloud"), qos
            )
            self.create_subscription(
                PointCloud2, "/odin1/cloud_in_map", take("odin_map"), qos
            )
        else:
            if LowState is None:
                raise RuntimeError(
                    "unitree_go not importable. First:\n"
                    "  source /home/ll/unitree_ros2/setup.sh\n"
                    "then re-run with --ref go2"
                )
            self.create_subscription(LowState, "lf/lowstate", self._on_go2, 10)
            self.create_subscription(LowState, "lowstate", self._on_go2, 10)

    def _on_odin_imu(self, msg: Imu) -> None:
        self.buf["odin_imu"] = msg
        a = msg.linear_acceleration
        self.acc_odin.append([a.x, a.y, a.z])

    def _on_ref_imu(self, msg: Imu) -> None:
        self.buf["ref_imu"] = msg
        a = msg.linear_acceleration
        self.acc_ref.append([a.x, a.y, a.z])

    def _on_go2(self, msg) -> None:
        self.buf["go2"] = msg
        a = msg.imu_state.accelerometer
        self.acc_ref.append([float(a[0]), float(a[1]), float(a[2])])


def run_gravity_mid(node: MeasureNode, collect_sec: float) -> np.ndarray:
    print(f"\n[gravity/mid] collecting {collect_sec:.1f}s ...")
    node.acc_ref.clear()
    node.acc_odin.clear()
    t0 = time.time()
    while time.time() - t0 < collect_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(node.acc_ref) < 10 or len(node.acc_odin) < 10:
        raise RuntimeError(
            f"IMU samples mid={len(node.acc_ref)} odin={len(node.acc_odin)}"
        )
    am = np.mean(node.acc_ref, axis=0)
    ao = np.mean(node.acc_odin, axis=0)
    print("Mid  acc", np.round(am, 4).tolist(), "norm", round(float(np.linalg.norm(am)), 4))
    print("Odin acc", np.round(ao, 4).tolist(), "norm", round(float(np.linalg.norm(ao)), 4))
    R = gravity_align(ao, am)
    print_R("R_midIMU_odinIMU ≈ R_L1_L2 (Mid extrinsic R≈I)", R)
    return R


def run_gravity_go2(node: MeasureNode, collect_sec: float) -> np.ndarray:
    print(f"\n[gravity/go2] collecting {collect_sec:.1f}s (need unitree DDS on robot NIC) ...")
    node.acc_ref.clear()
    node.acc_odin.clear()
    t0 = time.time()
    while time.time() - t0 < collect_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(node.acc_ref) < 10 or len(node.acc_odin) < 10:
        raise RuntimeError(
            f"IMU samples go2={len(node.acc_ref)} odin={len(node.acc_odin)}. "
            "Check: source unitree_ros2/setup.sh and robot lf/lowstate."
        )
    ag = np.mean(node.acc_ref, axis=0)
    ao = np.mean(node.acc_odin, axis=0)
    print("Go2  acc", np.round(ag, 4).tolist(), "norm", round(float(np.linalg.norm(ag)), 4))
    print("Odin acc", np.round(ao, 4).tolist(), "norm", round(float(np.linalg.norm(ao)), 4))

    # R maps Odin → Go2 IMU (== base attitude, URDF R=I)
    R_go2_odin = gravity_align(ao, ag)
    print_R("R_go2IMU_odinIMU ≈ R_base_odin (attitude; yaw-free)", R_go2_odin)

    if "go2" in node.buf:
        rpy = node.buf["go2"].imu_state.rpy
        print(
            "Go2 lowstate.rpy deg",
            [round(math.degrees(float(v)), 3) for v in rpy],
        )
    if "odin_odom" in node.buf:
        q = node.buf["odin_odom"].pose.pose.orientation
        print("Odin odom rpy deg", np.round(rpy_deg(quat_to_R(q.x, q.y, q.z, q.w)), 3).tolist())

    # Convert to Mid←Odin using official Mid@Go2IMU Ry(+13°)
    R_mid_odin = R_GO2IMU_MIDL.T @ R_go2_odin
    print_R(
        "Derived R_midL_odin ≈ R_L1_L2  (via official Mid Ry(+13°) in Go2 IMU)",
        R_mid_odin,
    )
    print(
        "\nNote: translation still needs CAD/tape. Go2 IMU origin vs base_link:\n"
        f"  t_base_go2IMU = {T_BASE_GO2IMU_T.tolist()} m (URDF), R=I"
    )
    print("|R_go2_odin - I|_deg (should be small if Odin level on dog) =",
          round(rot_angle_deg(np.eye(3), R_go2_odin), 3))
    return R_go2_odin


def run_icp(
    node: MeasureNode, R_l2_old: np.ndarray, wait_sec: float
) -> Tuple[np.ndarray, np.ndarray]:
    print(f"\n[icp/mid] waiting clouds up to {wait_sec:.1f}s ...")
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
    R_delta, t_delta = T_delta[:3, :3], T_delta[:3, 3]
    print("\nICP residual T_delta (Odin_in_map → Mid):")
    print("  t_mm", np.round(t_delta * 1000, 2).tolist())
    print("  rpy_deg", np.round(rpy_deg(R_delta), 4).tolist())
    print("  |R_delta|_deg", round(rot_angle_deg(np.eye(3), R_delta), 4))
    print("  nn median/mean m", round(med, 4), round(mean, 4))

    R_new = R_delta @ R_l2_old
    print_R("Suggested R_L2_new = R_delta @ R_L2_old", R_new)
    return R_delta, R_new


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--ref",
        choices=("mid", "go2"),
        default="mid",
        help="attitude reference: mid (default) or go2 body IMU",
    )
    ap.add_argument(
        "--mode",
        choices=("gravity", "icp", "both"),
        default="both",
        help="mid: gravity|icp|both; go2: only gravity (icp ignored)",
    )
    ap.add_argument(
        "--l2-rpy-deg",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        default=[0.0, -13.0, 0.0],
        help="current L2 RPY deg for ICP compose (mid only)",
    )
    ap.add_argument("--collect-sec", type=float, default=3.0)
    ap.add_argument("--icp-wait-sec", type=float, default=6.0)
    args = ap.parse_args()

    if args.ref == "go2" and args.mode in ("icp", "both"):
        print("[warn] --ref go2: ICP against Mid map skipped; running gravity only.")
        args.mode = "gravity"

    R_l2_old = rpy_deg_to_R(*args.l2_rpy_deg)
    if args.ref == "mid":
        print("Current L2 RPY deg (input):", args.l2_rpy_deg)
        print_R("R_L2_old", R_l2_old)

    rclpy.init()
    node = MeasureNode(args.ref)
    try:
        if args.ref == "go2":
            R_go2 = run_gravity_go2(node, args.collect_sec)
            R_l2 = R_GO2IMU_MIDL.T @ R_go2
            print("\n>>> Optional paste for Mid L2 R (derived via official Mid@Go2):")
            flat = ", ".join(f"{v:.8f}" for v in R_l2.reshape(-1))
            print(f"extrinsic_R_L2_wrt_L1: [{flat}]")
        else:
            R_grav = None
            if args.mode in ("gravity", "both"):
                R_grav = run_gravity_mid(node, args.collect_sec)
                print(
                    "\nGravity vs old L2 ΔR_deg =",
                    round(rot_angle_deg(R_l2_old, R_grav), 4),
                )
            if args.mode in ("icp", "both"):
                _, R_new = run_icp(node, R_l2_old, args.icp_wait_sec)
                if R_grav is not None:
                    print(
                        "\nICP vs gravity ΔR_deg =",
                        round(rot_angle_deg(R_grav, R_new), 4),
                    )
                flat = ", ".join(f"{v:.8f}" for v in R_new.reshape(-1))
                print("\n>>> Paste into yaml/launch (attitude only):")
                print(f"extrinsic_R_L2_wrt_L1: [{flat}]")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
