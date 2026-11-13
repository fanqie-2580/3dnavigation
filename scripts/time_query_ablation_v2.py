#!/usr/bin/env python3
"""TIME_QUERY_ABLATION_V2 — bag-based A/B/C/D with methodology fixes.

Fixes vs v1:
1) IMU xcorr: no shared zero-mask; full-signal + segment median lag
2) Deskew: faithful port of UndistortOdinNative (gyro+acc_scale path); requires full-rate bag IMU
3) Static map: still bag + Mid IMU |ω| gate (not Odometry.twist)
4) Ghost: wall-plane signed-distance IQR / multi-peak thickness (not bare NN)
5) Pose: uses /Odometry (Stage2). Online orange uses Stage1 — documented, not claimed identical.

Does NOT modify formal fusion path.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

# Extrinsics (mid360_odin1.yaml)
R_L2_L1 = np.array(
    [[0.97437006, 0.0, -0.22495105], [0.0, 1.0, 0.0], [0.22495105, 0.0, 0.97437006]],
    dtype=np.float64,
)
T_L2_L1 = np.array([0.12157888, 0.00549610, 0.1587], dtype=np.float64)
# Odin lidar wrt Odin IMU (wiki / yaml)
R_L2_IMU = np.eye(3)
T_L2_IMU = np.array([-0.02663, 0.03447, 0.02174], dtype=np.float64)

BIAS0, BIAS_EMA, BIAS_MIN, BIAS_MAX = 0.065, 0.05, 0.0, 0.12
TIGHT_DT = 0.055
G = 9.81


def quat_to_R(x, y, z, w):
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


def slerp_R(Ra, Rb, u):
    def R_to_q(R):
        tr = np.trace(R)
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            return np.array(
                [ (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s ]
            )
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
        if i == 1:
            s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])

    qa, qb = R_to_q(Ra), R_to_q(Rb)
    if np.dot(qa, qb) < 0:
        qb = -qb
    dot = float(np.clip(np.dot(qa, qb), -1.0, 1.0))
    if dot > 0.9995:
        q = qa + u * (qb - qa)
        q /= np.linalg.norm(q) + 1e-12
    else:
        th = math.acos(dot)
        q = (math.sin((1 - u) * th) * qa + math.sin(u * th) * qb) / math.sin(th)
    return quat_to_R(q[0], q[1], q[2], q[3])


def exp_so3(w, dt):
    ang = np.linalg.norm(w) * abs(dt)
    if ang < 1e-12:
        return np.eye(3)
    k = w / (np.linalg.norm(w) + 1e-12)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], dtype=np.float64)
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def read_bag(path: str):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=path, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {k: [] for k in ("/livox/imu", "/odin1/imu", "/Odometry", "/odin1/cloud_raw", "/livox/lidar", "/cloud_registered")}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in out:
            continue
        msg = deserialize_message(data, get_message(typemap[topic]))
        out[topic].append(msg)
    return out


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def imu_array(msgs, with_acc=False):
    rows = []
    for m in msgs:
        t = stamp(m)
        g = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        if with_acc:
            a = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
            rows.append((t, *g, *a))
        else:
            rows.append((t, *g))
    return np.asarray(rows, dtype=np.float64)


def odom_poses(msgs):
    poses = []
    for m in msgs:
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        poses.append(
            {
                "t": stamp(m),
                "R": quat_to_R(q.x, q.y, q.z, q.w),
                "p": np.array([p.x, p.y, p.z], dtype=np.float64),
            }
        )
    return poses


def sample_pose(poses, t):
    if not poses:
        return None
    if t <= poses[0]["t"]:
        return poses[0]
    if t >= poses[-1]["t"]:
        return poses[-1]
    lo, hi = 0, len(poses) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if poses[mid]["t"] < t:
            lo = mid
        else:
            hi = mid
    a, b = poses[lo], poses[hi]
    u = (t - a["t"]) / max(1e-6, b["t"] - a["t"])
    return {"t": t, "R": slerp_R(a["R"], b["R"], u), "p": (1 - u) * a["p"] + u * b["p"]}


def pc2_xyz_offset(msg, max_pts=4000, rng=None):
    rng = rng or np.random.default_rng(0)
    # manual parse for speed/reliability
    field = {f.name: f for f in msg.fields}
    if "x" not in field:
        return None
    step = msg.point_step
    n = msg.width * msg.height
    data = memoryview(msg.data)
    xs, ys, zs, offs = [], [], [], []
    has_off = "offset_time" in field
    ox, oy, oz = field["x"].offset, field["y"].offset, field["z"].offset
    oo = field["offset_time"].offset if has_off else 0
    for i in range(n):
        base = i * step
        x = struct.unpack_from("<f", data, base + ox)[0]
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if x * x + y * y + z * z < 0.0025:
            continue
        xs.append(x)
        ys.append(y)
        zs.append(z)
        if has_off:
            offs.append(struct.unpack_from("<f", data, base + oo)[0])
        else:
            offs.append(0.0)
    if len(xs) < 50:
        return None
    xyz = np.column_stack([xs, ys, zs]).astype(np.float64)
    off = np.asarray(offs, dtype=np.float64)
    if len(xyz) > max_pts:
        idx = rng.choice(len(xyz), max_pts, replace=False)
        xyz, off = xyz[idx], off[idx]
    return xyz, off


def livox_scans(msgs):
    scans = []
    for m in msgs:
        if not m.points:
            continue
        tmax = max(p.offset_time for p in m.points)
        span = float(tmax) * 1e-9 if tmax else 0.1
        beg = stamp(m)
        scans.append({"beg": beg, "end": beg + span, "span": span, "center": beg + 0.5 * span})
    return scans


def imu_stats(arr, name):
    if len(arr) < 2:
        return {"name": name, "n": 0}
    dt = np.diff(arr[:, 0])
    dt = dt[dt > 0]
    rate = 1.0 / np.median(dt) if len(dt) else 0
    gaps = dt[dt > 3.0 / max(rate, 1)]
    return {
        "name": name,
        "n": int(len(arr)),
        "duration_s": float(arr[-1, 0] - arr[0, 0]),
        "rate_hz_med": float(rate),
        "dt_p99_ms": float(np.percentile(dt, 99) * 1000) if len(dt) else None,
        "gap_count_gt_3x": int(len(gaps)),
        "drop_est_frac": float(len(gaps) / max(1, len(dt))),
    }


def imu_clock_offset_fixed(mid_imu, odin_imu):
    """Fixed xcorr: full-signal + per-segment lags; NO shared zero mask."""
    if len(mid_imu) < 200 or len(odin_imu) < 200:
        return {"ok": False, "reason": "insufficient_imu"}
    w_o = (R_L2_L1 @ odin_imu[:, 1:4].T).T
    t0 = max(mid_imu[0, 0], odin_imu[0, 0]) + 0.3
    t1 = min(mid_imu[-1, 0], odin_imu[-1, 0]) - 0.3
    if t1 - t0 < 3.0:
        return {"ok": False, "reason": "short_overlap"}
    fs = 200.0
    ts = np.arange(t0, t1, 1.0 / fs)
    Mm = np.column_stack([np.interp(ts, mid_imu[:, 0], mid_imu[:, c]) for c in (1, 2, 3)])
    Oo = np.column_stack([np.interp(ts, odin_imu[:, 0], w_o[:, c]) for c in (0, 1, 2)])
    mn = np.linalg.norm(Mm, axis=1)
    on = np.linalg.norm(Oo, axis=1)
    if float(mn.max()) < 0.2:
        return {"ok": False, "reason": "no_active_turn", "max_w": float(mn.max())}

    def xcorr_lag(a, b, max_lag_s=0.35):
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        if len(a) < 20 or len(b) < 20:
            return None, None
        a = (a - a.mean()) / (a.std() + 1e-12)
        b = (b - b.mean()) / (b.std() + 1e-12)
        max_lag = int(max_lag_s * fs)
        lags = np.arange(-max_lag, max_lag + 1)
        c = np.correlate(a, b, mode="full")
        mid_i = len(a) - 1
        vals = np.array([c[mid_i + int(L)] / len(a) for L in lags])
        i = int(np.argmax(vals))
        return float(lags[i] / fs), float(vals[i])

    L_full, peak_full = xcorr_lag(mn, on)
    # Segment approach: contiguous Mid-high windows; crop BOTH (no zero fill)
    thr = max(0.25, np.percentile(mn, 75))
    high = mn >= thr
    segs = []
    i = 0
    while i < len(high):
        if not high[i]:
            i += 1
            continue
        j = i
        while j < len(high) and high[j]:
            j += 1
        if j - i >= int(0.6 * fs):  # >=0.6s — short crops bias lag→0
            segs.append((i, j))
        i = j
    seg_lags = []
    for a, b in segs:
        L, p = xcorr_lag(mn[a:b], on[a:b], max_lag_s=0.25)
        if L is not None and p is not None and p > 0.5:
            seg_lags.append(L)
    L_seg = float(np.median(seg_lags)) if seg_lags else None
    # Prefer full-signal when segment median collapses near 0 but full disagrees
    # (short high-ω crops often lack delay observability → spurious 0).
    if L_full is not None and (L_seg is None or (abs(L_seg) < 0.002 and abs(L_full) >= 0.003)):
        L_use = L_full
        L_src = "full"
    elif L_seg is not None:
        L_use = L_seg
        L_src = "segment_med"
    else:
        L_use = L_full
        L_src = "full"
    return {
        "ok": True,
        "L_full_ms": None if L_full is None else L_full * 1000,
        "peak_full": peak_full,
        "n_segments": len(segs),
        "n_seg_lags": len(seg_lags),
        "L_seg_med_ms": None if L_seg is None else L_seg * 1000,
        "L_used_source": L_src,
        "d_odin_minus_d_mid_ms": -L_use * 1000,
        "b_clock_proxy_ms": L_use * 1000,  # additive on Odin header: -(d_o-d_m)=L
        "note": "IMU proxy only; NOT lidar header truth. No shared zero-mask.",
    }


def undistort_odin_native_faithful(xyz, off, imu7, beg, mean_acc_norm=1.0):
    """Closer port of UndistortOdinNative: relative SE3 to scan end, with R_LI/T_LI.
    bg=ba=0 (no Mid bias mix in offline eval). acc_scale = G/mean_acc_norm.
    """
    if len(xyz) == 0 or len(imu7) < 2:
        return xyz.copy(), {"ok": False, "reason": "thin_imu"}
    span = float(off.max())
    if span < 1e-4:
        return xyz.copy(), {"ok": False, "reason": "no_span"}
    end = beg + span
    acc_scale = G / max(1e-3, mean_acc_norm)
    R_LI, T_LI = R_L2_IMU, T_L2_IMU
    R_IL = R_LI.T

    # trim imu
    t_imu = imu7[:, 0]
    keep = t_imu >= (beg - 0.05)
    imu = imu7[keep]
    if len(imu) < 2:
        return xyz.copy(), {"ok": False, "reason": "imu_window"}

    times = [beg]
    Rs = [np.eye(3)]
    poss = [np.zeros(3)]
    vels = [np.zeros(3)]
    gyrs = [np.zeros(3)]
    accs = [np.zeros(3)]
    R = np.eye(3)
    pos = np.zeros(3)
    vel = np.zeros(3)
    last = beg
    angvel = np.zeros(3)
    acc_s = np.zeros(3)
    for i in range(len(imu) - 1):
        t_head, t_tail = float(imu[i, 0]), float(imu[i + 1, 0])
        if t_tail < beg:
            continue
        w = 0.5 * (imu[i, 1:4] + imu[i + 1, 1:4])
        a = 0.5 * (imu[i, 4:7] + imu[i + 1, 4:7]) * acc_scale
        t0 = max(t_head, last)
        t1 = min(t_tail, end)
        dt = t1 - t0
        if dt > 1e-6:
            R = R @ exp_so3(w, dt)
            aw = R @ a
            pos = pos + vel * dt + 0.5 * aw * dt * dt
            vel = vel + aw * dt
            angvel, acc_s = w, aw
            last = t1
            times.append(last)
            Rs.append(R.copy())
            poss.append(pos.copy())
            vels.append(vel.copy())
            gyrs.append(angvel.copy())
            accs.append(acc_s.copy())
        if t_tail >= end:
            break
    if end - last > 1e-6:
        dt = min(0.2, end - last)
        R = R @ exp_so3(angvel, dt)
        pos = pos + vel * dt + 0.5 * acc_s * dt * dt
        times.append(end)
        Rs.append(R.copy())
        poss.append(pos.copy())
        vels.append(vel.copy())
        gyrs.append(angvel.copy())
        accs.append(acc_s.copy())

    R_end, pos_end = Rs[-1], poss[-1]
    R_end_T = R_end.T
    times_a = np.asarray(times)
    out = np.empty_like(xyz)
    for k, (p, ot) in enumerate(zip(xyz, off)):
        t_off = float(ot)
        j = int(np.searchsorted(times_a, beg + t_off) - 1)
        j = max(0, min(j, len(Rs) - 2))
        dt = (beg + t_off) - times_a[j]
        Ri = Rs[j] @ exp_so3(gyrs[j + 1] if j + 1 < len(gyrs) else gyrs[j], dt)
        T_ei = poss[j] + vels[j] * dt + 0.5 * accs[min(j + 1, len(accs) - 1)] * dt * dt - pos_end
        P = R_IL @ (R_end_T @ (Ri @ (R_LI @ p + T_LI) + T_ei) - T_LI)
        out[k] = P
    disp = np.linalg.norm(out - xyz, axis=1)
    return out, {
        "ok": True,
        "imu_n": int(len(imu)),
        "poses": int(len(Rs)),
        "span_ms": span * 1000,
        "disp_med_m": float(np.median(disp)),
        "disp_p90_m": float(np.percentile(disp, 90)),
    }


def build_static_map_from_still(still_mid_clouds, still_mid_imu, still_poses, w_max=0.06, max_pts=50000):
    """Use Mid IMU |ω|, not Odometry.twist."""
    pts = []
    for c in still_mid_clouds:
        t = c["t"]
        # nearest mid imu
        if len(still_mid_imu) == 0:
            continue
        i = int(np.argmin(np.abs(still_mid_imu[:, 0] - t)))
        w = float(np.linalg.norm(still_mid_imu[i, 1:4]))
        if w > w_max:
            continue
        pts.append(c["xyz"])
    if not pts:
        return np.zeros((0, 3)), {"n_frames": 0}
    X = np.vstack(pts)
    if len(X) > max_pts:
        X = X[np.random.default_rng(2).choice(len(X), max_pts, replace=False)]
    return X, {"n_frames": len(pts), "n_pts": int(len(X)), "w_max": w_max}


def fit_wall_planes(map_pts, n_planes=4, dist_thresh=0.04, min_inliers=800):
    """Simple RANSAC planes for ghost thickness."""
    remaining = map_pts.copy()
    planes = []
    rng = np.random.default_rng(4)
    for _ in range(n_planes):
        if len(remaining) < min_inliers:
            break
        best = None
        for _ in range(80):
            idx = rng.choice(len(remaining), 3, replace=False)
            p0, p1, p2 = remaining[idx]
            n = np.cross(p1 - p0, p2 - p0)
            nn = np.linalg.norm(n)
            if nn < 1e-8:
                continue
            n = n / nn
            # prefer vertical-ish walls
            if abs(n[2]) > 0.5:
                continue
            d = remaining @ n - np.dot(n, p0)
            inl = np.where(np.abs(d) < dist_thresh)[0]
            if best is None or len(inl) > best[0]:
                best = (len(inl), n, float(np.dot(n, p0)), inl)
        if best is None or best[0] < min_inliers:
            break
        _, n, c0, inl = best
        # refine
        pts = remaining[inl]
        c = pts.mean(0)
        _, _, Vt = np.linalg.svd(pts - c, full_matrices=False)
        n = Vt[-1]
        if n[2] < 0:
            n = -n
        planes.append({"n": n, "c": float(np.dot(n, c)), "inliers": int(len(inl))})
        remaining = np.delete(remaining, inl, axis=0)
    return planes


def wall_ghost_metrics(query, planes):
    """Signed-distance IQR / robust thickness on wall planes."""
    if not planes or len(query) == 0:
        return {"thickness_med": None, "iqr_med": None, "planes_used": 0}
    thicks, iqrs = [], []
    for pl in planes:
        sd = query @ pl["n"] - pl["c"]
        # near band
        m = np.abs(sd) < 0.35
        if m.sum() < 40:
            continue
        s = sd[m]
        q25, q75 = np.percentile(s, [25, 75])
        iqrs.append(float(q75 - q25))
        # thickness: 90% mass width
        thicks.append(float(np.percentile(s, 95) - np.percentile(s, 5)))
    if not thicks:
        return {"thickness_med": None, "iqr_med": None, "planes_used": 0}
    return {
        "thickness_med": float(np.median(thicks)),
        "iqr_med": float(np.median(iqrs)),
        "planes_used": len(thicks),
    }


def point_plane_res(query, map_pts, max_q=1200):
    if len(query) == 0 or len(map_pts) < 30:
        return np.array([])
    if len(query) > max_q:
        query = query[np.random.default_rng(5).choice(len(query), max_q, replace=False)]
    tree = cKDTree(map_pts)
    d, idx = tree.query(query, k=min(5, len(map_pts)))
    if idx.ndim == 1:
        return d
    res = d[:, 0].copy()
    for i in range(0, len(query), max(1, len(query) // 300)):
        nbr = map_pts[idx[i]]
        c = nbr.mean(0)
        _, _, Vt = np.linalg.svd(nbr - c, full_matrices=False)
        n = Vt[-1]
        n /= np.linalg.norm(n) + 1e-12
        res[i] = abs(np.dot(query[i] - c, n))
    return res


def frame_overlap(prev, cur):
    if prev is None or len(prev) == 0 or len(cur) == 0:
        return None
    tree = cKDTree(prev)
    d, _ = tree.query(cur, k=1)
    return float(np.median(d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--still-bag", required=True)
    ap.add_argument("--turn-bag", required=True)
    ap.add_argument("--bias-init", type=float, default=-1.0)
    ap.add_argument("--out", default="/tmp/tq_ablation_v2_out")
    ap.add_argument("--turn-w", type=float, default=0.25)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    print("[load] still bag...", flush=True)
    still = read_bag(args.still_bag)
    print("[load] turn bag...", flush=True)
    turn = read_bag(args.turn_bag)

    still_mid_imu = imu_array(still["/livox/imu"])
    still_odin_imu = imu_array(still["/odin1/imu"], with_acc=True)
    turn_mid_imu = imu_array(turn["/livox/imu"])
    turn_odin_imu = imu_array(turn["/odin1/imu"], with_acc=True)
    still_poses = odom_poses(still["/Odometry"])
    turn_poses = odom_poses(turn["/Odometry"])

    imu_qc = {
        "still_mid": imu_stats(still_mid_imu, "still_mid"),
        "still_odin": imu_stats(still_odin_imu, "still_odin"),
        "turn_mid": imu_stats(turn_mid_imu, "turn_mid"),
        "turn_odin": imu_stats(turn_odin_imu, "turn_odin"),
    }
    print("[imu_qc]", json.dumps(imu_qc, indent=2), flush=True)

    # decode clouds
    print("[decode] clouds...", flush=True)
    rng = np.random.default_rng(0)
    still_mid_clouds = []
    for m in still["/cloud_registered"]:
        r = pc2_xyz_offset(m, max_pts=3000, rng=rng)
        if r:
            still_mid_clouds.append({"t": stamp(m), "xyz": r[0]})
    turn_odin = []
    for m in turn["/odin1/cloud_raw"]:
        r = pc2_xyz_offset(m, max_pts=3500, rng=rng)
        if r:
            turn_odin.append({"t": stamp(m), "xyz": r[0], "off": r[1]})
    mid_scans = livox_scans(turn["/livox/lidar"])
    print(f"still_mid_clouds={len(still_mid_clouds)} turn_odin={len(turn_odin)} mid_scans={len(mid_scans)}", flush=True)

    static_map, map_meta = build_static_map_from_still(still_mid_clouds, still_mid_imu, still_poses)
    planes = fit_wall_planes(static_map)
    print(f"[map] pts={len(static_map)} planes={len(planes)} meta={map_meta}", flush=True)

    clocks = imu_clock_offset_fixed(turn_mid_imu, turn_odin_imu)
    print("[clock]", clocks, flush=True)
    if clocks.get("ok"):
        # additive bias on Odin header so Mid[t]~Odin[t-L] ⇒ b_clock = L = -(d_o-d_m)
        delta_c = clocks["b_clock_proxy_ms"] / 1000.0
        c_ok = True
        c_conf = "low_imu_proxy_not_lidar_header"
    else:
        delta_c = None
        c_ok = False
        c_conf = f"unavailable:{clocks.get('reason')}"
    delta_d = 0.0  # scheme D

    # reconstruct online bias from fusion log if available, else EMA from this bag
    b = BIAS0 if args.bias_init < 0 else float(args.bias_init)

    # mean acc from odin still for scale
    if len(still_odin_imu):
        mean_acc_norm = float(np.median(np.linalg.norm(still_odin_imu[:, 4:7], axis=1)))
    else:
        mean_acc_norm = 1.0

    results = []
    bias_series = []
    prev_maps = {"A": None, "B": None, "C": None, "D": None}
    n_turn = 0
    left_like = right_like = 0

    for fi, oc in enumerate(turn_odin):
        H2, xyz, off = oc["t"], oc["xyz"], oc["off"]
        span = float(off.max()) if len(off) else 0.0
        med = float(np.median(off)) if len(off) else 0.0
        if span < 1e-3:
            continue
        b_pre = b
        mid_ref = None
        center_dt = None
        best = 1e9
        for ms in mid_scans:
            signed = (H2 + b_pre) - ms["beg"]
            if abs(signed) > TIGHT_DT:
                continue
            cdt = (H2 + b_pre + med) - ms["center"]
            score = abs(cdt) + (0.004 if signed > 0 else 0.0)
            if score < best:
                best = score
                center_dt = cdt
                mid_ref = ms
        if mid_ref is None:
            continue
        b = min(BIAS_MAX, max(BIAS_MIN, b - BIAS_EMA * center_dt))
        bias_series.append(b_pre)

        # Always keep turn frames; subsample still frames for speed
        i_w = int(np.argmin(np.abs(turn_mid_imu[:, 0] - mid_ref["end"])))
        w_peek = float(np.linalg.norm(turn_mid_imu[i_w, 1:4]))
        if w_peek < args.turn_w and (fi % 3) != 0:
            continue

        # Mid IMU ω and yaw-rate sign for L/R
        i = i_w
        wvec = turn_mid_imu[i, 1:4]
        w_norm = float(np.linalg.norm(wvec))
        wz = float(wvec[2])

        tA = mid_ref["center"] + (span - med)
        tB = H2 + b_pre + span
        tC = (H2 + delta_c + span) if c_ok else None
        tD = H2 + delta_d + span

        # deskew once with full-rate bag IMU
        m = (turn_odin_imu[:, 0] >= H2 - 0.02) & (turn_odin_imu[:, 0] <= H2 + span + 0.05)
        imu_s = turn_odin_imu[m]
        xyz_d, dmeta = undistort_odin_native_faithful(xyz, off, imu_s, H2, mean_acc_norm)

        schemes = {"A": tA, "B": tB, "D": tD}
        if c_ok:
            schemes["C"] = tC

        entry = {
            "H2": H2,
            "span_ms": span * 1000,
            "med_ms": med * 1000,
            "center_dt_ms": center_dt * 1000,
            "b_pre_ms": b_pre * 1000,
            "mid_end": mid_ref["end"],
            "mid_center": mid_ref["center"],
            "tA": tA,
            "tB": tB,
            "tD": tD,
            "tC": tC,
            "w_norm": w_norm,
            "wz": wz,
            "deskew": dmeta,
            "pose_source": "Odometry_Stage2_not_Stage1",
        }

        for name, tq in schemes.items():
            po = sample_pose(turn_poses, tq)
            if po is None:
                continue
            p_l1 = (R_L2_L1 @ xyz_d.T).T + T_L2_L1
            mapped = (po["R"] @ p_l1.T).T + po["p"]
            # also raw for intra-scan fan vs whole-frame
            p_l1_r = (R_L2_L1 @ xyz.T).T + T_L2_L1
            mapped_raw = (po["R"] @ p_l1_r.T).T + po["p"]

            res = point_plane_res(mapped, static_map)
            nn = point_plane_res(mapped, static_map)  # same helper returns nn/plane mix
            # pure NN
            tree = cKDTree(static_map)
            dnn, _ = tree.query(mapped[np.random.default_rng(6).choice(len(mapped), min(800, len(mapped)), replace=False)], k=1)
            gh = wall_ghost_metrics(mapped, planes)
            ov = frame_overlap(prev_maps[name], mapped)
            prev_maps[name] = mapped

            res_raw = point_plane_res(mapped_raw, static_map)
            entry[f"{name}_res_med"] = float(np.median(res)) if len(res) else None
            entry[f"{name}_nn_p50"] = float(np.median(dnn))
            entry[f"{name}_nn_p90"] = float(np.percentile(dnn, 90))
            entry[f"{name}_wall_thick"] = gh["thickness_med"]
            entry[f"{name}_wall_iqr"] = gh["iqr_med"]
            entry[f"{name}_overlap_med"] = ov
            entry[f"{name}_raw_res_med"] = float(np.median(res_raw)) if len(res_raw) else None
            # whole-frame vs intra: deskew improvement
            if entry[f"{name}_raw_res_med"] is not None and entry[f"{name}_res_med"] is not None:
                entry[f"{name}_deskew_delta_res"] = entry[f"{name}_raw_res_med"] - entry[f"{name}_res_med"]

        results.append(entry)
        if w_norm >= args.turn_w:
            n_turn += 1
            if wz > 0.1:
                left_like += 1
            elif wz < -0.1:
                right_like += 1
        if (fi + 1) % 40 == 0:
            print(f"[eval] {fi+1}/{len(turn_odin)} turn_frames={n_turn}", flush=True)

    def summarize(key, turn_only=False):
        rows = [
            r
            for r in results
            if r.get(f"{key}_res_med") is not None
            and ((r["w_norm"] >= args.turn_w) if turn_only else True)
        ]
        if not rows:
            return None

        def med(field):
            xs = [r[field] for r in rows if r.get(field) is not None]
            return float(np.median(xs)) if xs else None

        return {
            "n": len(rows),
            "mean_w": float(np.mean([r["w_norm"] for r in rows])),
            "res_med": med(f"{key}_res_med"),
            "nn_p50": med(f"{key}_nn_p50"),
            "nn_p90": med(f"{key}_nn_p90"),
            "wall_thick": med(f"{key}_wall_thick"),
            "wall_iqr": med(f"{key}_wall_iqr"),
            "overlap_med": med(f"{key}_overlap_med"),
            "deskew_delta_res": med(f"{key}_deskew_delta_res"),
            "corr_w_res": float(
                np.corrcoef([r["w_norm"] for r in rows], [r[f"{key}_res_med"] for r in rows])[0, 1]
            )
            if len(rows) > 3
            else None,
        }

    deskew_ok = [r for r in results if r.get("deskew", {}).get("ok")]
    summary = {
        "imu_qc": imu_qc,
        "clock": clocks,
        "scheme_C": {"available": c_ok, "delta_ms": None if not c_ok else delta_c * 1000, "confidence": c_conf},
        "scheme_D": {"delta_ms": 0.0},
        "bias": {
            "init_ms": (BIAS0 if args.bias_init < 0 else args.bias_init) * 1000,
            "median_b_pre_ms": float(np.median(bias_series) * 1000) if bias_series else None,
            "max_b_pre_ms": float(np.max(bias_series) * 1000) if bias_series else None,
        },
        "static_map": map_meta,
        "n_planes": len(planes),
        "n_results": len(results),
        "n_turn": n_turn,
        "turn_wz_pos": left_like,
        "turn_wz_neg": right_like,
        "deskew_ok_frac": len(deskew_ok) / max(1, len(results)),
        "deskew_disp_med_median": float(np.median([r["deskew"]["disp_med_m"] for r in deskew_ok])) if deskew_ok else None,
        "pose_note": "Trajectory is /Odometry after Stage2. Online /cloud_odin_registered uses Stage1@t_query. A/B/C/D share same traj so relative ranking is meaningful; absolute match to online orange is not claimed.",
        "all": {k: summarize(k, False) for k in "ABCD"},
        "turn": {k: summarize(k, True) for k in "ABCD"},
    }

    def rank(block):
        c = [(block[k]["wall_thick"] if block[k] and block[k]["wall_thick"] is not None else 1e9, k) for k in "ABCD" if block.get(k)]
        # fallback to res
        if not c or all(x[0] == 1e9 for x in c):
            c = [(block[k]["res_med"], k) for k in "ABCD" if block.get(k) and block[k].get("res_med") is not None]
        return sorted(c)

    summary["rank_turn_wall_thick"] = rank(summary["turn"])
    summary["rank_turn_res"] = sorted(
        [(summary["turn"][k]["res_med"], k) for k in "ABCD" if summary["turn"].get(k) and summary["turn"][k].get("res_med") is not None]
    )

    with open(out / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    with open(out / "frames.json", "w") as f:
        json.dump(results, f)
    print(json.dumps(summary, indent=2))
    print(f"[done] {out}", flush=True)


if __name__ == "__main__":
    main()
