#!/usr/bin/env python3
"""ODIN cross-time eval V2: Mid-only frozen map + paired A/B + b_clock sweep.

Does NOT modify IEKF. Uses published /cloud_odin_xtform_{A,B} (+ Mid registered).
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

TOPICS = [
    "/cloud_odin_xtform_A",
    "/cloud_odin_xtform_B",
    "/odin_xtform_metrics",
    "/cloud_registered",
    "/Odometry",
    "/livox/imu",
]

MET_KEYS = [
    "dt_ms",
    "rel_yaw_deg",
    "rel_trans_m",
    "resA_med",
    "resB_med",
    "nnA_med",
    "nnB_med",
    "n_try",
    "passA",
    "passB",
    "w_norm",
    "t_odin",
    "t_mid",
    "b_pre_ms",
    "n_plane_A",
    "n_plane_B",
    "n_plane_both",
    "map_odin_cand",
    "map_odin_ins",
    "map_odin_blk",
    "map_block_flag",
]


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def cloud_xyz(msg):
    """Fast XYZ extract from unstructured PointCloud2 (x/y/z float32)."""
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3))
    names = [f.name for f in msg.fields]
    if names[:3] == ["x", "y", "z"] and msg.point_step >= 12:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        pts = (
            arr.reshape(n, msg.point_step)[:, :12]
            .view(np.float32)
            .reshape(n, 3)
            .astype(np.float64)
        )
        mask = np.isfinite(pts).all(axis=1)
        return pts[mask]
    from sensor_msgs_py import point_cloud2

    pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3))
    return np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)


def read_bag(bag):
    storage = StorageOptions(uri=str(bag), storage_id="sqlite3")
    converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage, converter)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        if topic not in out:
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        out[topic].append((msg, t_recv * 1e-9))
    return out


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


def odom_poses(entries):
    ts, pos, rot = [], [], []
    for msg, _ in entries:
        t = stamp_sec(msg)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ts.append(t)
        pos.append([p.x, p.y, p.z])
        rot.append(quat_to_R(q.x, q.y, q.z, q.w))
    return np.asarray(ts), np.asarray(pos), rot


def interp_pose(ts, pos, rot, t):
    if len(ts) == 0:
        return None, None
    if t <= ts[0]:
        return pos[0].copy(), rot[0].copy()
    if t >= ts[-1]:
        return pos[-1].copy(), rot[-1].copy()
    i = int(np.searchsorted(ts, t))
    u = (t - ts[i - 1]) / max(1e-9, ts[i] - ts[i - 1])
    p = (1 - u) * pos[i - 1] + u * pos[i]
    # crude slerp via matrix blend + re-ortho (adequate for small steps)
    Rm = (1 - u) * rot[i - 1] + u * rot[i]
    U, _, Vt = np.linalg.svd(Rm)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return p, R


def imu_series(entries):
    ts, wn, wz = [], [], []
    for msg, _ in entries:
        t = stamp_sec(msg)
        g = msg.angular_velocity
        ts.append(t)
        wn.append(math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z))
        wz.append(g.z)
    return np.asarray(ts), np.asarray(wn), np.asarray(wz)


def nearest(ts, vals, t):
    if len(ts) == 0:
        return 0.0
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return float(vals[0])
    if i >= len(ts):
        return float(vals[-1])
    return float(vals[i - 1] if abs(ts[i - 1] - t) < abs(ts[i] - t) else vals[i])


def odom_speed(ts, pos):
    if len(ts) < 2:
        return ts[:0], ts[:0]
    spd_t, spd = [], []
    for i in range(1, len(ts)):
        dt = ts[i] - ts[i - 1]
        if dt < 1e-3:
            continue
        d = np.linalg.norm(pos[i] - pos[i - 1])
        spd_t.append(ts[i])
        spd.append(d / dt)
    return np.asarray(spd_t), np.asarray(spd)


def classify(w, wz, v, w_turn=0.25, v_move=0.08):
    turning = abs(w) >= w_turn
    moving = v >= v_move
    if not turning and not moving and w < 0.08:
        return "still"
    if turning and not moving:
        return "turn_left" if wz > 0 else "turn_right"
    if moving and not turning:
        return "translate"
    if moving and turning:
        return "combined"
    if turning:
        return "turn_left" if wz > 0 else "turn_right"
    return "other"


def fit_planes(pts, n_planes=6, dist_thr=0.03, min_inliers=200):
    """Simple RANSAC multi-plane for signed-distance walls."""
    remain = pts.copy()
    planes = []
    rng = np.random.default_rng(0)
    for _ in range(n_planes):
        if remain.shape[0] < min_inliers:
            break
        best_inliers = None
        best_n = None
        best_d = None
        for _t in range(80):
            if remain.shape[0] < 3:
                break
            ids = rng.choice(remain.shape[0], 3, replace=False)
            p0, p1, p2 = remain[ids]
            n = np.cross(p1 - p0, p2 - p0)
            nn = np.linalg.norm(n)
            if nn < 1e-8:
                continue
            n = n / nn
            d = -float(n @ p0)
            dist = np.abs(remain @ n + d)
            inl = np.where(dist < dist_thr)[0]
            if best_inliers is None or inl.size > best_inliers.size:
                best_inliers = inl
                best_n, best_d = n, d
        if best_inliers is None or best_inliers.size < min_inliers:
            break
        planes.append((best_n, best_d, int(best_inliers.size)))
        remain = np.delete(remain, best_inliers, axis=0)
    return planes


def build_mid_only_map(mid_entries, imu_ts, imu_w, od_ts, od_spd, max_pts=40000):
    chunks = []
    meta = {"n_cand": 0, "n_keep": 0, "reject_w": 0, "reject_v": 0}
    # Stride candidates so still frames don't explode runtime
    step = max(1, len(mid_entries) // 250)
    for msg, _ in mid_entries[::step]:
        meta["n_cand"] += 1
        t = stamp_sec(msg)
        w = nearest(imu_ts, imu_w, t)
        v = nearest(od_ts, od_spd, t) if len(od_ts) else 0.0
        if w > 0.06:
            meta["reject_w"] += 1
            continue
        if v > 0.05:
            meta["reject_v"] += 1
            continue
        xyz = cloud_xyz(msg)
        if xyz.shape[0] == 0:
            continue
        if xyz.shape[0] > 8000:
            xyz = xyz[np.linspace(0, xyz.shape[0] - 1, 8000).astype(int)]
        chunks.append(xyz)
        meta["n_keep"] += 1
    if not chunks:
        return np.zeros((0, 3)), meta, []
    pts = np.vstack(chunks)
    if pts.shape[0] > max_pts:
        idx = np.random.default_rng(1).choice(pts.shape[0], max_pts, replace=False)
        pts = pts[idx]
    meta["n_pts"] = int(pts.shape[0])
    planes = fit_planes(pts, n_planes=4, min_inliers=120)
    meta["n_planes"] = len(planes)
    return pts, meta, planes


def pair_ab(a_e, b_e, met_e, recv_tol=0.01):
    """Pair A/B/metrics by bag receive time; require equal point counts for A/B."""
    a_msgs = [e[0] for e in a_e]
    a_recv = np.array([e[1] for e in a_e])
    b_msgs = [e[0] for e in b_e]
    b_recv = np.array([e[1] for e in b_e])
    m_msgs = [e[0] for e in met_e]
    m_recv = np.array([e[1] for e in met_e])
    pairs = []
    used_b, used_m = set(), set()
    for i, am in enumerate(a_msgs):
        if b_recv.size == 0:
            break
        jb = int(np.argmin(np.abs(b_recv - a_recv[i])))
        if jb in used_b or abs(b_recv[jb] - a_recv[i]) > recv_tol:
            continue
        na = am.width
        nb = b_msgs[jb].width
        if na != nb or na == 0:
            continue
        jm = -1
        if m_recv.size:
            jm = int(np.argmin(np.abs(m_recv - a_recv[i])))
            if jm in used_m or abs(m_recv[jm] - a_recv[i]) > 0.05:
                jm = -1
        used_b.add(jb)
        if jm >= 0:
            used_m.add(jm)
        met = None
        if jm >= 0:
            d = list(m_msgs[jm].data)
            met = {k: float(d[i]) for i, k in enumerate(MET_KEYS) if i < len(d)}
        pairs.append((am, b_msgs[jb], met, a_recv[i]))
    return pairs


def local_plane_residuals(pts, tree, k=5, plane_thr=0.05, max_nn=0.5, max_fit=600):
    """Per-point |ax+by+cz+d| from kNN plane; subsample for speed on Jetson."""
    n = pts.shape[0]
    res = np.full(n, np.nan)
    ok_plane = np.zeros(n, dtype=bool)
    if n == 0 or tree.n < k:
        return res, ok_plane
    if n > max_fit:
        fit_idx = np.linspace(0, n - 1, max_fit).astype(int)
    else:
        fit_idx = np.arange(n)
    q = pts[fit_idx]
    dists, idxs = tree.query(q, k=k, workers=-1)
    if k == 1:
        dists = dists.reshape(-1, 1)
        idxs = idxs.reshape(-1, 1)
    for j, i in enumerate(fit_idx):
        if not np.isfinite(dists[j, 0]) or dists[j, 0] >= max_nn:
            continue
        neigh = tree.data[idxs[j]]
        c = neigh.mean(axis=0)
        _, _, vh = np.linalg.svd(neigh - c, full_matrices=False)
        nrm = vh[-1]
        nn = np.linalg.norm(nrm)
        if nn < 1e-8:
            continue
        nrm = nrm / nn
        d0 = -float(nrm @ c)
        if np.max(np.abs(neigh @ nrm + d0)) > plane_thr:
            continue
        res[i] = abs(float(nrm @ pts[i] + d0))
        ok_plane[i] = True
    return res, ok_plane


def score_paired(xa, xb, tree, planes, max_nn=0.5, max_pts=1200):
    """Same indices; NN + local plane residual + fixed-wall signed distance."""
    n = min(xa.shape[0], xb.shape[0])
    if n == 0:
        return None
    if n > max_pts:
        idx = np.linspace(0, n - 1, max_pts).astype(int)
    else:
        idx = np.arange(n)
    qa, qb = xa[idx], xb[idx]
    da, _ = tree.query(qa, k=1, workers=-1)
    db, _ = tree.query(qb, k=1, workers=-1)
    ra, oka = local_plane_residuals(qa, tree, max_nn=max_nn, max_fit=400)
    rb, okb = local_plane_residuals(qb, tree, max_nn=max_nn, max_fit=400)
    ok_nn = np.isfinite(da) & np.isfinite(db) & (da < max_nn) & (db < max_nn)
    ok_pl = oka & okb
    ok_both = ok_nn & ok_pl

    def plane_stats(pts):
        if not planes:
            return None, None
        nrm, d0, _ = max(planes, key=lambda p: p[2])
        cands = [p for p in planes if abs(p[0][2]) < 0.5]
        if cands:
            nrm, d0, _ = max(cands, key=lambda p: p[2])
        s = pts @ nrm + d0
        return float(np.median(s)), float(np.percentile(s, 90) - np.percentile(s, 10))

    sa_med, sa_w = plane_stats(qa)
    sb_med, sb_w = plane_stats(qb)
    return {
        "n": int(idx.size),
        "n_nn_both": int(ok_nn.sum()),
        "n_plane_A": int(oka.sum()),
        "n_plane_B": int(okb.sum()),
        "n_plane_both": int(ok_pl.sum()),
        "nnA_p50": float(np.median(da[np.isfinite(da) & (da < max_nn)])) if np.any(da < max_nn) else None,
        "nnB_p50": float(np.median(db[np.isfinite(db) & (db < max_nn)])) if np.any(db < max_nn) else None,
        "nnA_pair_p50": float(np.median(da[ok_nn])) if ok_nn.any() else None,
        "nnB_pair_p50": float(np.median(db[ok_nn])) if ok_nn.any() else None,
        "d_nn_pair": float(np.median(db[ok_nn] - da[ok_nn])) if ok_nn.any() else None,
        "resA_pair_p50": float(np.median(ra[ok_pl])) if ok_pl.any() else None,
        "resB_pair_p50": float(np.median(rb[ok_pl])) if ok_pl.any() else None,
        "d_res_pair": float(np.median(rb[ok_pl] - ra[ok_pl])) if ok_pl.any() else None,
        "frac_plane_A": float(np.mean(oka)),
        "frac_plane_B": float(np.mean(okb)),
        "frac_plane_both": float(np.mean(ok_pl)),
        "wall_signedA_med": sa_med,
        "wall_signedB_med": sb_med,
        "wall_widthA": sa_w,
        "wall_widthB": sb_w,
        "frac_nn_A": float(np.mean(np.isfinite(da) & (da < max_nn))),
        "frac_nn_B": float(np.mean(np.isfinite(db) & (db < max_nn))),
        "frac_nn_both": float(np.mean(ok_nn)),
        "frac_nn_plane_both": float(np.mean(ok_both)),
    }


def body_from_A(xa, t_mid, od_ts, od_pos, od_rot):
    """Recover L1 body assuming A = R_mid*p + t_mid with identity extrinsic in world-lidar.
    Online uses T_WI*T_IL; Odometry is Mid IMU/body. Approximate with odom pose as T_WL
    is imperfect — for b_clock sweep we use relative: B(b)= T(t_odin+b)*inv(T_mid)*A.
    """
    p, R = interp_pose(od_ts, od_pos, od_rot, t_mid)
    if p is None:
        return None
    # p_body = R^T (p_w - t)
    return (R.T @ (xa - p).T).T


def project_body(pb, t_query, od_ts, od_pos, od_rot):
    p, R = interp_pose(od_ts, od_pos, od_rot, t_query)
    if p is None:
        return None
    return (R @ pb.T).T + p


def summarize(rows, key_pref=""):
    if not rows:
        return None

    def med(k):
        xs = [r[k] for r in rows if r.get(k) is not None]
        return float(np.median(xs)) if xs else None

    out = {
        "n": len(rows),
        "nnA_pair_p50": med("nnA_pair_p50"),
        "nnB_pair_p50": med("nnB_pair_p50"),
        "d_nn_pair": med("d_nn_pair"),
        "resA_pair_p50": med("resA_pair_p50"),
        "resB_pair_p50": med("resB_pair_p50"),
        "d_res_pair": med("d_res_pair"),
        "frac_plane_both": med("frac_plane_both"),
        "frac_nn_both": med("frac_nn_both"),
        "wall_signedA_med": med("wall_signedA_med"),
        "wall_signedB_med": med("wall_signedB_med"),
        "wall_absA": None,
        "wall_absB": None,
        "online_d_res": med("online_d_res"),
        "map_ins_frac": med("map_ins_frac"),
        "frac_B_better_res": None,
        "frac_A_better_res": None,
    }
    dres = [r["d_res_pair"] for r in rows if r.get("d_res_pair") is not None]
    if dres:
        out["frac_B_better_res"] = float(np.mean([x < -1e-4 for x in dres]))
        out["frac_A_better_res"] = float(np.mean([x > 1e-4 for x in dres]))
    wa = [abs(r["wall_signedA_med"]) for r in rows if r.get("wall_signedA_med") is not None]
    wb = [abs(r["wall_signedB_med"]) for r in rows if r.get("wall_signedB_med") is not None]
    if wa:
        out["wall_absA"] = float(np.median(wa))
    if wb:
        out["wall_absB"] = float(np.median(wb))
    return out


def record_bag(out_dir: Path, sec: float):
    import shutil

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ros2", "bag", "record", "-o", str(out_dir), "--compression-mode", "none", *TOPICS]
    print(f"[rec] {sec:.0f}s → {out_dir}")
    proc = subprocess.Popen(cmd)
    try:
        time.sleep(sec)
    finally:
        proc.send_signal(2)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
    return out_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=Path, default=None)
    ap.add_argument("--record-sec", type=float, default=0.0)
    ap.add_argument("--out", type=Path, default=Path("/tmp/odin_xtform_v2"))
    ap.add_argument(
        "--b-clock-ms",
        type=str,
        default="-150,-120,-100,-80,-60,-40,-20,0,20,40",
        help="Comma list of candidate b_clock (ms) added to t_odin=H2+span",
    )
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    bag = args.bag
    if args.record_sec > 0:
        bag = args.out / "bag"
        record_bag(bag, args.record_sec)
    if bag is None:
        raise SystemExit("need --bag or --record-sec")
    if not (bag / "metadata.yaml").exists():
        cands = list(bag.rglob("metadata.yaml"))
        if not cands:
            raise SystemExit("no bag metadata")
        bag = cands[0].parent

    print("[read] bag…", flush=True)
    data = read_bag(bag)
    for t in TOPICS:
        print(f"  {t}: {len(data[t])}", flush=True)

    imu_ts, imu_w, imu_wz = imu_series(data["/livox/imu"])
    od_ts, od_pos, od_rot = odom_poses(data["/Odometry"])
    spd_t, spd = odom_speed(od_ts, od_pos)

    print("[map] building Mid-only…", flush=True)
    mid_map, map_meta, planes = build_mid_only_map(
        data["/cloud_registered"], imu_ts, imu_w, spd_t, spd
    )
    if mid_map.shape[0] < 500:
        raise SystemExit(f"Mid-only map too small: {map_meta}")
    tree = cKDTree(mid_map)
    print(f"[map] {map_meta}", flush=True)

    pairs = pair_ab(data["/cloud_odin_xtform_A"], data["/cloud_odin_xtform_B"], data["/odin_xtform_metrics"])
    print(f"[pair] A/B frames={len(pairs)}", flush=True)

    rows = []
    insert_rows = []
    for fi, (am, bm, met, recv) in enumerate(pairs):
        if (fi + 1) % 50 == 0:
            print(f"[score] {fi+1}/{len(pairs)}", flush=True)
        xa, xb = cloud_xyz(am), cloud_xyz(bm)
        sc = score_paired(xa, xb, tree, planes)
        if sc is None:
            continue
        t_mid = met["t_mid"] if met and "t_mid" in met else stamp_sec(am)
        t_odin = met["t_odin"] if met and "t_odin" in met else t_mid
        w = nearest(imu_ts, imu_w, t_mid)
        wz = nearest(imu_ts, imu_wz, t_mid)
        v = nearest(spd_t, spd, t_mid) if len(spd_t) else 0.0
        seg = classify(w, wz, v)
        row = {
            **sc,
            "seg": seg,
            "t_mid": t_mid,
            "t_odin": t_odin,
            "w": w,
            "wz": wz,
            "v": v,
            "dt_ms": (t_odin - t_mid) * 1000.0,
        }
        if met:
            row["online_d_res"] = met.get("resB_med", 0) - met.get("resA_med", 0)
            cand = met.get("map_odin_cand", 0)
            ins = met.get("map_odin_ins", 0)
            row["map_ins_frac"] = (ins / cand) if cand else None
            row["map_block_flag"] = met.get("map_block_flag", None)
            row["n_plane_A_online"] = met.get("n_plane_A")
            row["n_plane_B_online"] = met.get("n_plane_B")
            row["n_plane_both_online"] = met.get("n_plane_both")
            if cand:
                insert_rows.append(row)
        rows.append(row)

    # b_clock sweep on motion frames (geometry-optimal ≠ calibrated clock)
    b_list = [float(x) for x in args.b_clock_ms.split(",") if x.strip()]
    sweep = []
    motion_pairs = []
    for am, bm, met, recv in pairs:
        if not met or "t_mid" not in met:
            continue
        t_mid, t_odin = met["t_mid"], met["t_odin"]
        w = nearest(imu_ts, imu_w, t_mid)
        wz = nearest(imu_ts, imu_wz, t_mid)
        v = nearest(spd_t, spd, t_mid) if len(spd_t) else 0.0
        seg = classify(w, wz, v)
        if seg == "still":
            continue
        xa = cloud_xyz(am)
        pb = body_from_A(xa, t_mid, od_ts, od_pos, od_rot)
        if pb is None:
            continue
        motion_pairs.append((pb, t_odin, t_mid, seg, w))

    # Cap sweep frames for Jetson runtime; stride keeps temporal coverage.
    sweep_pairs = motion_pairs[:: max(1, len(motion_pairs) // 80)]
    print(f"[sweep] motion_frames={len(motion_pairs)} used={len(sweep_pairs)} b={b_list}")
    for b_ms in b_list:
        nn_scores, res_scores = [], []
        for pb, t_odin, t_mid, seg, w in sweep_pairs:
            # b_clock is added to published t_odin (=H2+span); NOT b_pair.
            pw = project_body(pb, t_odin + b_ms * 1e-3, od_ts, od_pos, od_rot)
            if pw is None or pw.shape[0] == 0:
                continue
            if pw.shape[0] > 800:
                idx = np.linspace(0, pw.shape[0] - 1, 800).astype(int)
                pw = pw[idx]
            d, _ = tree.query(pw, k=1, workers=-1)
            d_ok = d[np.isfinite(d) & (d < 0.5)]
            if d_ok.size == 0:
                continue
            nn_scores.append(float(np.median(d_ok)))
            # Fixed reference planes (fast) as residual proxy for sweep
            if planes:
                nrm, d0, _ = max(
                    ([p for p in planes if abs(p[0][2]) < 0.5] or planes),
                    key=lambda p: p[2],
                )
                res_scores.append(float(np.median(np.abs(pw @ nrm + d0))))
        sweep.append(
            {
                "b_clock_ms": b_ms,
                "n": len(nn_scores),
                "nn_p50_med": float(np.median(nn_scores)) if nn_scores else None,
                "res_p50_med": float(np.median(res_scores)) if res_scores else None,
            }
        )

    by_seg = {}
    for seg in ("still", "turn_left", "turn_right", "translate", "combined", "other"):
        by_seg[seg] = summarize([r for r in rows if r["seg"] == seg])

    # insert audit
    insert_audit = None
    if insert_rows:
        fracs = [r["map_ins_frac"] for r in insert_rows if r.get("map_ins_frac") is not None]
        blocked = [r for r in insert_rows if r.get("map_block_flag") == 1.0]
        insert_audit = {
            "n_frames_with_met": len(insert_rows),
            "ins_frac_med": float(np.median(fracs)) if fracs else None,
            "ins_frac_mean": float(np.mean(fracs)) if fracs else None,
            "frac_frames_block_flag": float(len(blocked) / len(insert_rows)),
            "ins_frac_by_seg": {},
        }
        for seg in by_seg:
            xs = [
                r["map_ins_frac"]
                for r in insert_rows
                if r["seg"] == seg and r.get("map_ins_frac") is not None
            ]
            if xs:
                insert_audit["ins_frac_by_seg"][seg] = {
                    "n": len(xs),
                    "med": float(np.median(xs)),
                }

    # online selection bias from metrics if present
    sel_bias = None
    if any(r.get("n_plane_A_online") is not None for r in rows):
        dplane = [
            (r["n_plane_A_online"] or 0) - (r["n_plane_B_online"] or 0)
            for r in rows
            if r.get("n_plane_A_online") is not None
        ]
        both = [r.get("n_plane_both_online") for r in rows if r.get("n_plane_both_online") is not None]
        sel_bias = {
            "median_planeA_minus_B": float(np.median(dplane)) if dplane else None,
            "median_n_plane_both": float(np.median(both)) if both else None,
        }

    best_b = None
    valid_sw = [s for s in sweep if s["nn_p50_med"] is not None]
    if valid_sw:
        best_b = min(valid_sw, key=lambda s: s["nn_p50_med"])

    motion = summarize([r for r in rows if r["seg"] != "still"])
    still = by_seg.get("still")

    # verdicts
    leak_likely = bool(
        insert_audit
        and insert_audit.get("ins_frac_med") is not None
        and insert_audit["ins_frac_med"] > 0.05
    )
    mid_only_changes = None
    rank_metric = None
    if motion:
        # Prefer paired plane residual; fall back to NN
        rank_metric = motion.get("d_res_pair")
        if rank_metric is None:
            rank_metric = motion.get("d_nn_pair")
        if rank_metric is not None:
            if rank_metric < -0.002:
                mid_only_changes = "B_better_on_mid_only"
            elif rank_metric > 0.002:
                mid_only_changes = "A_better_on_mid_only"
            else:
                mid_only_changes = "similar"
    best_b_res = None
    valid_sw_res = [s for s in sweep if s.get("res_p50_med") is not None]
    if valid_sw_res:
        best_b_res = min(valid_sw_res, key=lambda s: s["res_p50_med"])
    if best_b is None and valid_sw:
        best_b = min(valid_sw, key=lambda s: s["nn_p50_med"])

    summary = {
        "map": map_meta,
        "n_pairs": len(pairs),
        "insert_audit": insert_audit,
        "selection_bias_online": sel_bias,
        "by_segment_mid_only": by_seg,
        "motion_mid_only": motion,
        "still_mid_only": still,
        "b_clock_sweep_motion": sweep,
        "b_clock_geom_best_nn": best_b,
        "b_clock_geom_best_res": best_b_res,
        "note_b_clock": "geom-optimal offset ≠ physical clock calibration",
        "leak_likely_favoring_A": leak_likely,
        "mid_only_A_vs_B": mid_only_changes,
        "rank_metric_d_res_or_nn": rank_metric,
        "answers": {
            "1_leak_could_explain_V1": leak_likely,
            "2_mid_only_changes_ranking": mid_only_changes,
            "3_b_clock_stable_opt": best_b_res or best_b,
            "4_support_xtform": mid_only_changes == "B_better_on_mid_only",
            "5_ready_for_stage2": False,  # filled below
        },
    }
    # Ready only if Mid-only clearly favors B on paired residual, still OK, and
    # we do NOT treat geom-best b_clock as a calibrated physical clock.
    dres = motion.get("d_res_pair") if motion else None
    summary["answers"]["5_ready_for_stage2"] = bool(
        mid_only_changes == "B_better_on_mid_only"
        and dres is not None
        and dres < -0.005
        and still
        and (still.get("d_res_pair") is None or abs(still["d_res_pair"]) < 0.01)
    )

    out_json = args.out / "summary.json"
    out_json.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"[wrote] {out_json}")


if __name__ == "__main__":
    main()
