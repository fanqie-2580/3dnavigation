#!/usr/bin/env python3
"""Offline A/B/C t_query ablation — does NOT touch the live fusion node.

A: Mid_center + (span - med)          # current online formula
B: Odin_header + bias_snapshot + span # pairing bias BEFORE EMA update
C: Odin_header + delta_phys + span    # delta_phys from IMU xcorr on SAME clip
                                      # (labeled proxy — not lidar clock truth)

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 time_query_ablation_offline.py \
      --record-sec 25 --out /tmp/time_query_ablation_out
"""
from __future__ import annotations

import argparse
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import cKDTree

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception:  # noqa: BLE001
    CustomMsg = None

# Extrinsic L2 in L1 (from mid360_odin1.yaml)
R_L2_L1 = np.array(
    [
        [0.97437006, 0.0, -0.22495105],
        [0.0, 1.0, 0.0],
        [0.22495105, 0.0, 0.97437006],
    ],
    dtype=np.float64,
)
T_L2_L1 = np.array([0.12157888, 0.00549610, 0.1587], dtype=np.float64)

# Bias EMA (same as laserMapping_adaptive)
BIAS0 = 0.065
BIAS_EMA = 0.05
BIAS_MIN = 0.0
BIAS_MAX = 0.12
TIGHT_DT = 0.055


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def quat_to_R(x, y, z, w) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w) + 1e-12
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def slerp_R(Ra, Rb, u: float) -> np.ndarray:
    # naive via quaternion slerp
    def R_to_q(R):
        tr = np.trace(R)
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
            else:
                s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
        return np.array([x, y, z, w], dtype=np.float64)

    qa, qb = R_to_q(Ra), R_to_q(Rb)
    if np.dot(qa, qb) < 0:
        qb = -qb
    dot = float(np.clip(np.dot(qa, qb), -1.0, 1.0))
    if dot > 0.9995:
        q = qa + u * (qb - qa)
        q /= np.linalg.norm(q)
    else:
        th = math.acos(dot)
        q = (math.sin((1 - u) * th) * qa + math.sin(u * th) * qb) / math.sin(th)
    return quat_to_R(q[0], q[1], q[2], q[3])


def exp_so3(w: np.ndarray, dt: float) -> np.ndarray:
    th = np.linalg.norm(w) * abs(dt)
    if th < 1e-12:
        return np.eye(3)
    k = (w * dt) / (np.linalg.norm(w) * abs(dt) + 1e-12)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], dtype=np.float64)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


@dataclass
class OdomPose:
    t: float
    R: np.ndarray
    p: np.ndarray
    wz: float


class Recorder(Node):
    """Stash raw messages lightly during record; decode PointCloud2 AFTER spin ends."""

    def __init__(self):
        super().__init__("time_query_ablation_rec")
        self.odoms: list[OdomPose] = []
        self.mid_imu: list[tuple] = []
        self.odin_imu: list[tuple] = []
        self.odin_cloud_msgs: list = []
        self.mid_cloud_msgs: list = []
        self.livox_scans: list[dict] = []
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=500)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=4000)
        self.create_subscription(Odometry, "/Odometry", self.cb_odom, 100)
        self.create_subscription(Imu, "/livox/imu", self.cb_mid_imu, qos_be)
        self.create_subscription(Imu, "/odin1/imu", self.cb_odin_imu, qos_rel)
        self.create_subscription(PointCloud2, "/odin1/cloud_raw", self.cb_odin_cloud, qos_be)
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_mid_cloud, 50)
        if CustomMsg is not None:
            self.create_subscription(CustomMsg, "/livox/lidar", self.cb_livox, 50)

    def cb_odom(self, m: Odometry):
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        self.odoms.append(
            OdomPose(
                stamp_sec(m),
                quat_to_R(q.x, q.y, q.z, q.w),
                np.array([p.x, p.y, p.z], dtype=np.float64),
                float(m.twist.twist.angular.z),
            )
        )

    def cb_mid_imu(self, m: Imu):
        self.mid_imu.append(
            (stamp_sec(m), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        )

    def cb_odin_imu(self, m: Imu):
        self.odin_imu.append(
            (
                stamp_sec(m),
                m.angular_velocity.x,
                m.angular_velocity.y,
                m.angular_velocity.z,
                m.linear_acceleration.x,
                m.linear_acceleration.y,
                m.linear_acceleration.z,
            )
        )

    def cb_odin_cloud(self, m: PointCloud2):
        # keep reference only — decode later
        self.odin_cloud_msgs.append(m)

    def cb_mid_cloud(self, m: PointCloud2):
        self.mid_cloud_msgs.append(m)

    def cb_livox(self, m):
        if not m.points:
            return
        # only header + max offset — O(n) but cheaper than full XYZ copy every time
        tmax = 0
        for p in m.points:
            if p.offset_time > tmax:
                tmax = p.offset_time
        self.livox_scans.append({"beg": stamp_sec(m), "span": float(tmax) * 1e-9 if tmax else 0.1})


def decode_odin_clouds(msgs, max_pts=3000):
    out = []
    rng = np.random.default_rng(0)
    for m in msgs:
        names = [f.name for f in m.fields]
        want = ["x", "y", "z"] + (["offset_time"] if "offset_time" in names else [])
        pts = list(pc2.read_points(m, field_names=want, skip_nans=True))
        if not pts:
            continue
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        off = (
            np.array([p[3] for p in pts], dtype=np.float64)
            if "offset_time" in names
            else np.zeros(len(pts), dtype=np.float64)
        )
        mask = np.linalg.norm(xyz, axis=1) > 0.05
        xyz, off = xyz[mask], off[mask]
        if len(xyz) < 50:
            continue
        if len(xyz) > max_pts:
            idx = rng.choice(len(xyz), max_pts, replace=False)
            xyz, off = xyz[idx], off[idx]
        out.append({"t": stamp_sec(m), "xyz": xyz, "off": off})
    return out


def decode_mid_clouds(msgs, max_pts=2500):
    out = []
    rng = np.random.default_rng(1)
    for m in msgs:
        pts = list(pc2.read_points(m, field_names=["x", "y", "z"], skip_nans=True))
        if len(pts) < 20:
            continue
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        if len(xyz) > max_pts:
            xyz = xyz[rng.choice(len(xyz), max_pts, replace=False)]
        out.append({"t": stamp_sec(m), "xyz": xyz})
    return out

def sample_pose(odoms: list[OdomPose], t: float):
    if not odoms:
        return None
    if t <= odoms[0].t:
        return odoms[0]
    if t >= odoms[-1].t:
        return odoms[-1]
    lo, hi = 0, len(odoms) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if odoms[mid].t < t:
            lo = mid
        else:
            hi = mid
    a, b = odoms[lo], odoms[hi]
    u = (t - a.t) / max(1e-6, b.t - a.t)
    R = slerp_R(a.R, b.R, u)
    p = (1 - u) * a.p + u * b.p
    wz = (1 - u) * a.wz + u * b.wz
    return OdomPose(t, R, p, wz)


def undistort_odin_native(xyz: np.ndarray, off: np.ndarray, imu: np.ndarray, beg: float) -> np.ndarray:
    """Approximate UndistortOdinNative: gyro-only (bg=0), leave at scan end.
    imu rows: t, gx,gy,gz, ax,ay,az (odin frame). Acc scale/ba omitted → gyro path.
    """
    if len(xyz) == 0 or len(imu) < 2:
        return xyz.copy()
    span = float(off.max()) if len(off) else 0.0
    if span < 1e-4:
        return xyz.copy()
    end = beg + span
    # build relative rotation chain from beg→end using gyro
    times = [beg]
    Rs = [np.eye(3)]
    R = np.eye(3)
    # imu window
    t_imu = imu[:, 0]
    i0 = max(0, np.searchsorted(t_imu, beg) - 1)
    last = beg
    for i in range(i0, len(imu) - 1):
        t0 = float(imu[i, 0])
        t1 = float(imu[i + 1, 0])
        if t1 < beg:
            continue
        w = 0.5 * (imu[i, 1:4] + imu[i + 1, 1:4])
        a = max(t0, last)
        b = min(t1, end)
        dt = b - a
        if dt > 1e-6:
            R = R @ exp_so3(w, dt)
            last = b
            times.append(last)
            Rs.append(R.copy())
        if t1 >= end:
            break
    if last < end - 1e-6:
        w = imu[-1, 1:4]
        R = R @ exp_so3(w, end - last)
        times.append(end)
        Rs.append(R.copy())
    R_end = Rs[-1]
    R_end_T = R_end.T
    # compensate each point to end: P_e = R_end^T * R(t) * P_t  (ignore lever / T for ablation)
    out = np.empty_like(xyz)
    times_a = np.asarray(times)
    for k, (p, ot) in enumerate(zip(xyz, off)):
        t = beg + float(ot)
        j = int(np.searchsorted(times_a, t) - 1)
        j = max(0, min(j, len(Rs) - 2))
        dt = t - times_a[j]
        # use gyro between j and j+1
        if j + 1 < len(imu):
            # fall back constant R_j
            Rj = Rs[j]
        else:
            Rj = Rs[j]
        # refine with local omega if possible
        ii = max(0, np.searchsorted(t_imu, t) - 1)
        ii = min(ii, len(imu) - 1)
        w = imu[ii, 1:4]
        Ri = Rj @ exp_so3(w, dt) if abs(dt) > 1e-6 else Rj
        out[k] = R_end_T @ (Ri @ p)
    return out


def imu_clock_offset(mid_imu: np.ndarray, odin_imu: np.ndarray) -> dict:
    """Return d_odin - d_mid from ω cross-corr. PROXY only — not lidar header truth."""
    if len(mid_imu) < 100 or len(odin_imu) < 100:
        return {"ok": False, "reason": "insufficient_imu"}
    # rotate odin ω into Mid frame
    w_o = (R_L2_L1 @ odin_imu[:, 1:4].T).T
    mid = mid_imu
    t0 = max(mid[0, 0], odin_imu[0, 0]) + 0.2
    t1 = min(mid[-1, 0], odin_imu[-1, 0]) - 0.2
    if t1 - t0 < 2.0:
        return {"ok": False, "reason": "short_overlap"}
    fs = 200.0
    ts = np.arange(t0, t1, 1.0 / fs)
    Mm = np.column_stack([np.interp(ts, mid[:, 0], mid[:, c]) for c in (1, 2, 3)])
    Oo = np.column_stack([np.interp(ts, odin_imu[:, 0], w_o[:, c]) for c in (0, 1, 2)])
    mn = np.linalg.norm(Mm, axis=1)
    if float(mn.max()) < 0.15:
        return {"ok": False, "reason": "no_active_turn", "max_w": float(mn.max())}

    def xcorr(a, b, max_lag_s=0.3):
        a = (a - a.mean()) / (a.std() + 1e-12)
        b = (b - b.mean()) / (b.std() + 1e-12)
        max_lag = int(max_lag_s * fs)
        lags = np.arange(-max_lag, max_lag + 1)
        c = np.correlate(a, b, mode="full")
        mid_i = len(a) - 1
        vals = np.array([c[mid_i + int(L)] / len(a) for L in lags])
        i = int(np.argmax(vals))
        return float(lags[i] / fs), float(vals[i])

    L, peak = xcorr(np.linalg.norm(Mm, axis=1), np.linalg.norm(Oo, axis=1))
    # high-motion
    mask = mn > max(0.2, np.percentile(mn, 70))
    Lh, peakh = (None, None)
    if mask.mean() > 0.05:
        w = mn.copy()
        w[~mask] = 0
        Lh, peakh = xcorr(np.linalg.norm(Mm, axis=1) * w, np.linalg.norm(Oo, axis=1) * w)
    return {
        "ok": True,
        "L_ms": L * 1000,
        "peak": peak,
        "d_odin_minus_d_mid_ms": -L * 1000,
        "L_hi_ms": None if Lh is None else Lh * 1000,
        "peak_hi": peakh,
        "d_hi_ms": None if Lh is None else -Lh * 1000,
        "note": "IMU ω proxy only; NOT calibrated lidar header clock",
    }


def build_static_map(mid_clouds: list[dict], still_wz_max: float, odoms: list[OdomPose]) -> np.ndarray:
    pts = []
    for c in mid_clouds:
        po = sample_pose(odoms, c["t"])
        if po is None:
            continue
        if abs(po.wz) > still_wz_max:
            continue
        pts.append(c["xyz"])
    if not pts:
        # fallback: all mid clouds
        pts = [c["xyz"] for c in mid_clouds]
    if not pts:
        return np.zeros((0, 3))
    X = np.vstack(pts)
    if len(X) > 40000:
        X = X[np.random.default_rng(2).choice(len(X), 40000, replace=False)]
    return X


def point_plane_residuals(query: np.ndarray, map_pts: np.ndarray, k: int = 5, max_query: int = 1500):
    """Fast NN distance + optional plane residual on a subsample."""
    if len(query) == 0 or len(map_pts) < 20:
        return np.array([])
    if len(query) > max_query:
        query = query[np.random.default_rng(3).choice(len(query), max_query, replace=False)]
    tree = cKDTree(map_pts)
    kk = min(k, len(map_pts))
    d, idx = tree.query(query, k=kk)
    if kk == 1 or idx.ndim == 1:
        return np.asarray(d, dtype=np.float64)
    # plane fit only every Nth point for speed; fill rest with NN dist
    res = np.asarray(d[:, 0], dtype=np.float64).copy()
    step = max(1, len(query) // 400)
    for i in range(0, len(query), step):
        nbrs = map_pts[idx[i]]
        c = nbrs.mean(axis=0)
        _, _, Vt = np.linalg.svd(nbrs - c, full_matrices=False)
        n = Vt[-1]
        n /= np.linalg.norm(n) + 1e-12
        res[i] = abs(np.dot(query[i] - c, n))
    return res


def ghost_thickness(query: np.ndarray, map_pts: np.ndarray, wall_normal_hint=None) -> dict:
    """Proxy: NN distance p90/p50 among points with range 2–8m (walls)."""
    if len(query) == 0 or len(map_pts) == 0:
        return {"p50": None, "p90": None, "n": 0}
    r = np.linalg.norm(query, axis=1)
    # use map-frame distance from origin roughly
    m = (r > 1.5) & (r < 12.0)
    q = query[m]
    if len(q) < 30:
        q = query
    tree = cKDTree(map_pts)
    d, _ = tree.query(q, k=1)
    return {"p50": float(np.median(d)), "p90": float(np.percentile(d, 90)), "mean": float(d.mean()), "n": int(len(d))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record-sec", type=float, default=25.0)
    ap.add_argument("--out", type=str, default="/tmp/time_query_ablation_out")
    ap.add_argument("--still-wz", type=float, default=0.08, help="|wz| below this → static map")
    ap.add_argument(
        "--bias-init",
        type=float,
        default=-1.0,
        help="initial pairing bias (s); <0 → use yaml 0.065; recommend online reconstructed ~0.115",
    )
    ap.add_argument("--turn-w", type=float, default=0.25, help="|ω| mid IMU threshold for turn subset")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Recorder()
    print(f"[rec] recording {args.record_sec:.1f}s — TURN left/right for ablation", flush=True)
    t_end = time.time() + args.record_sec
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.01)
    print(
        f"[rec] odom={len(node.odoms)} mid_imu={len(node.mid_imu)} odin_imu={len(node.odin_imu)} "
        f"odin_cloud_msgs={len(node.odin_cloud_msgs)} mid_cloud_msgs={len(node.mid_cloud_msgs)} "
        f"livox={len(node.livox_scans)}",
        flush=True,
    )
    print("[decode] pointclouds...", flush=True)
    odin_clouds = decode_odin_clouds(node.odin_cloud_msgs)
    mid_clouds = decode_mid_clouds(node.mid_cloud_msgs)
    print(f"[decode] odin_clouds={len(odin_clouds)} mid_clouds={len(mid_clouds)}", flush=True)

    odoms = node.odoms
    mid_imu = np.asarray(node.mid_imu, dtype=np.float64) if node.mid_imu else np.zeros((0, 4))
    odin_imu = np.asarray(node.odin_imu, dtype=np.float64) if node.odin_imu else np.zeros((0, 7))
    clocks = imu_clock_offset(mid_imu, odin_imu)
    print("[clock]", clocks, flush=True)

    # Physical delta for scheme C: ONLY if IMU proxy ok — still labeled low confidence
    if clocks.get("ok"):
        # Prefer high-motion estimate when available
        d_ms = clocks.get("d_hi_ms")
        if d_ms is None:
            d_ms = clocks["d_odin_minus_d_mid_ms"]
        # Map Odin stamp → Mid event time: t_mid = t_odin - (d_odin - d_mid) = t_odin - d
        # So additive bias on Odin header: b_phys = -(d_odin - d_mid)
        delta_phys = -float(d_ms) / 1000.0
        c_available = True
        c_confidence = "low_imu_proxy_not_lidar_header"
    else:
        delta_phys = None
        c_available = False
        c_confidence = f"unavailable:{clocks.get('reason')}"

    # Build Mid scan list for pairing
    mid_scans = []
    if node.livox_scans:
        for s in node.livox_scans:
            span = s["span"] if s["span"] > 1e-3 else 0.10
            mid_scans.append({"beg": s["beg"], "end": s["beg"] + span, "span": span, "center": s["beg"] + 0.5 * span})
    else:
        # approximate from odom 10Hz
        for o in odoms:
            mid_scans.append({"beg": o.t - 0.05, "end": o.t, "span": 0.10, "center": o.t - 0.05})

    static_map = build_static_map(mid_clouds, args.still_wz, odoms)
    print(f"[map] static_map pts={len(static_map)}", flush=True)

    # Offline Bundle-like pairing + bias EMA (same order as C++: pair with b_pre, then update)
    b = BIAS0 if args.bias_init < 0 else float(args.bias_init)
    bias_init_used = b
    bias_series = []
    results = []

    odin_clouds = sorted(odin_clouds, key=lambda x: x["t"])

    def mid_omega_at(t: float) -> float:
        if len(mid_imu) < 2:
            return 0.0
        # nearest
        i = int(np.argmin(np.abs(mid_imu[:, 0] - t)))
        return float(np.linalg.norm(mid_imu[i, 1:4]))

    n_eval = 0
    for fi, oc in enumerate(odin_clouds):
        H2 = oc["t"]
        off = oc["off"]
        xyz = oc["xyz"]
        span = float(off.max()) if len(off) else 0.0
        med = float(np.median(off)) if len(off) else 0.0
        if span < 1e-3:
            continue

        b_pre = b  # snapshot for this frame BEFORE EMA update
        best = None
        best_score = 1e9
        center_dt = None
        mid_ref = None
        for i, ms in enumerate(mid_scans):
            signed = (H2 + b_pre) - ms["beg"]
            if abs(signed) > TIGHT_DT:
                continue
            C2b = H2 + b_pre + med
            cdt = C2b - ms["center"]
            score = abs(cdt) + (0.004 if signed > 0 else 0.0)
            if score < best_score:
                best_score = score
                best = i
                center_dt = cdt
                mid_ref = ms
        if mid_ref is None:
            # still update nothing; skip eval
            continue

        # EMA update AFTER pairing (mirrors fill_odin_meas)
        b = b - BIAS_EMA * center_dt
        b = min(BIAS_MAX, max(BIAS_MIN, b))
        bias_series.append({"H2": H2, "b_pre": b_pre, "b_post": b, "center_dt": center_dt})

        # evaluate every 2nd paired frame for speed (bias still updated every pair)
        if n_eval % 1 == 0 and (fi % 2 == 1):
            # fall through to metrics
            pass
        elif fi % 2 == 0:
            continue
        n_eval += 1
        if n_eval % 10 == 0:
            print(f"[eval] paired_metrics={n_eval}/{len(odin_clouds)}", flush=True)

        tA = mid_ref["center"] + (span - med)
        tB = H2 + b_pre + span
        tC = (H2 + delta_phys + span) if c_available else None

        # omega from Mid IMU (Odometry.twist often zero in this stack)
        w_norm = mid_omega_at(mid_ref["end"])

        # deskew once (skip heavy path when nearly still)
        xyz_raw = xyz
        if w_norm >= 0.12 and len(odin_imu):
            m = (odin_imu[:, 0] >= H2 - 0.05) & (odin_imu[:, 0] <= H2 + span + 0.05)
            imu_s = odin_imu[m]
            xyz_d = undistort_odin_native(xyz, off, imu_s, H2) if len(imu_s) >= 2 else xyz.copy()
        else:
            xyz_d = xyz.copy()

        def to_map(t_query, cloud_body_odin):
            po = sample_pose(odoms, t_query)
            if po is None:
                return None
            # p_L1 = R_L2_L1 p_L2 + t
            p_l1 = (R_L2_L1 @ cloud_body_odin.T).T + T_L2_L1
            return (po.R @ p_l1.T).T + po.p, po

        schemes = {"A": tA, "B": tB}
        if c_available:
            schemes["C"] = tC

        entry = {
            "H2": H2,
            "span_ms": span * 1000,
            "med_ms": med * 1000,
            "center_dt_ms": center_dt * 1000,
            "b_pre_ms": b_pre * 1000,
            "b_post_ms": b * 1000,
            "tA_vs_mid_end_ms": (tA - mid_ref["end"]) * 1000,
            "tB_vs_mid_end_ms": (tB - mid_ref["end"]) * 1000,
            "tB_minus_tA_ms": (tB - tA) * 1000,
            "wz": w_norm,
            "w_norm": w_norm,
        }
        if c_available:
            entry["tC_vs_mid_end_ms"] = (tC - mid_ref["end"]) * 1000
            entry["tC_minus_tA_ms"] = (tC - tA) * 1000

        for name, tq in schemes.items():
            mapped, po = to_map(tq, xyz_d)
            if mapped is None or len(static_map) == 0:
                continue
            res = point_plane_residuals(mapped, static_map, k=5)
            gh = ghost_thickness(mapped, static_map)
            entry[f"{name}_res_med"] = float(np.median(res)) if len(res) else None
            entry[f"{name}_res_p90"] = float(np.percentile(res, 90)) if len(res) else None
            entry[f"{name}_ghost_p50"] = gh["p50"]
            entry[f"{name}_ghost_p90"] = gh["p90"]

            # deskew vs raw at same pose (geometry change) — only for B time as reference
            if name == "B":
                mapped_raw, _ = to_map(tq, xyz_raw)
                if mapped_raw is not None:
                    # median point displacement due to deskew
                    disp = np.linalg.norm(mapped - mapped_raw, axis=1)
                    entry["deskew_disp_med_m"] = float(np.median(disp))
                    entry["deskew_disp_p90_m"] = float(np.percentile(disp, 90))
                    res_raw = point_plane_residuals(mapped_raw, static_map, k=5)
                    entry["B_raw_res_med"] = float(np.median(res_raw)) if len(res_raw) else None
                    entry["B_deskew_res_med"] = entry.get("B_res_med")

        results.append(entry)

    node.destroy_node()
    rclpy.shutdown()

    turn_thr = float(args.turn_w)

    def summarize(key_prefix, frames, turn_only=False):
        rows = [r for r in frames if (r.get("w_norm", 0) >= turn_thr if turn_only else True)]
        rows = [r for r in rows if r.get(f"{key_prefix}_res_med") is not None]
        if not rows:
            return None
        return {
            "n": len(rows),
            "res_med_mean": float(np.mean([r[f"{key_prefix}_res_med"] for r in rows])),
            "res_med_median": float(np.median([r[f"{key_prefix}_res_med"] for r in rows])),
            "res_p90_median": float(np.median([r[f"{key_prefix}_res_p90"] for r in rows])),
            "ghost_p50_median": float(np.median([r[f"{key_prefix}_ghost_p50"] for r in rows])),
            "ghost_p90_median": float(np.median([r[f"{key_prefix}_ghost_p90"] for r in rows])),
            "mean_w": float(np.mean([r["w_norm"] for r in rows])),
        }

    bias_arr = np.array([x["b_pre"] for x in bias_series]) if bias_series else np.array([])
    summary = {
        "record_sec": args.record_sec,
        "n_paired_frames": len(results),
        "bias_ema": {
            "init_ms": bias_init_used * 1000,
            "yaml_default_ms": BIAS0 * 1000,
            "final_pre_ms": float(bias_arr[-1] * 1000) if len(bias_arr) else None,
            "min_ms": float(bias_arr.min() * 1000) if len(bias_arr) else None,
            "max_ms": float(bias_arr.max() * 1000) if len(bias_arr) else None,
            "median_ms": float(np.median(bias_arr) * 1000) if len(bias_arr) else None,
            "hit_ceil_frac": float(np.mean(bias_arr >= BIAS_MAX - 1e-9)) if len(bias_arr) else None,
            "note": "b_pre = bias used for pairing/B BEFORE EMA update; ROS param server stays at yaml init",
        },
        "imu_clock_proxy": clocks,
        "scheme_C": {
            "available": c_available,
            "delta_phys_ms": None if delta_phys is None else delta_phys * 1000,
            "confidence": c_confidence,
        },
        "all": {k: summarize(k, results, False) for k in ("A", "B", "C")},
        "turn_w_ge_thr": {k: summarize(k, results, True) for k in ("A", "B", "C")},
        "turn_thr": turn_thr,
        "imu_counts": {"mid": int(len(mid_imu)), "odin": int(len(odin_imu))},
    }

    # deskew effect on turn frames
    deskew_rows = [r for r in results if r.get("w_norm", 0) >= turn_thr and r.get("deskew_disp_med_m") is not None]
    if deskew_rows:
        summary["deskew_vs_raw_turn"] = {
            "n": len(deskew_rows),
            "disp_med_median_m": float(np.median([r["deskew_disp_med_m"] for r in deskew_rows])),
            "disp_p90_median_m": float(np.median([r["deskew_disp_p90_m"] for r in deskew_rows])),
            "res_raw_med_median": float(np.median([r["B_raw_res_med"] for r in deskew_rows if r.get("B_raw_res_med") is not None])),
            "res_deskew_med_median": float(np.median([r["B_deskew_res_med"] for r in deskew_rows if r.get("B_deskew_res_med") is not None])),
        }

    # winner
    def pick(block):
        cands = []
        for k in ("A", "B", "C"):
            if block.get(k):
                cands.append((block[k]["res_med_median"], k))
        if not cands:
            return None
        cands.sort()
        return cands

    summary["rank_all_by_res_med"] = pick(summary["all"])
    summary["rank_turn_by_res_med"] = pick(summary["turn_w_ge_thr"])

    with open(out / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    with open(out / "frames.json", "w") as f:
        json.dump(results[:500], f)  # cap
    np.savez(
        out / "bias_series.npz",
        b_pre=np.array([x["b_pre"] for x in bias_series]),
        center_dt=np.array([x["center_dt"] for x in bias_series]),
    )
    print(json.dumps(summary, indent=2))
    print(f"[done] wrote {out}", flush=True)


if __name__ == "__main__":
    main()
