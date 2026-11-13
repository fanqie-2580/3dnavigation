#!/usr/bin/env python3
"""ODIN time-calibration + cross-time eval V3.

- Re-parses insert-audit metrics (cand/gate/det/queued/ikd_total).
- Scores A/B on an independent Mid-only map (optional ICP align).
- b_clock sweep with full T_W = T_WI * T_IL (matches pointBodyToWorld).
- Analyzes Mid/Odin LiDAR↔IMU header relationships (NOT b_pair as clock).

Does NOT modify IEKF / Stage2 / gates / extrinsics.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

# Mid LiDAR ↔ Mid IMU from mid360_odin1.yaml (extrinsic_est_en: false)
DEFAULT_T_IL = np.array([-0.011, -0.02329, 0.04412], dtype=np.float64)
DEFAULT_R_IL = np.eye(3, dtype=np.float64)

FUSION_TOPICS = [
    "/cloud_odin_xtform_A",
    "/cloud_odin_xtform_B",
    "/odin_xtform_metrics",
    "/cloud_registered",
    "/Odometry",
    "/livox/imu",
]
TIME_TOPICS = [
    "/livox/imu",
    "/livox/lidar",
    "/odin1/imu",
    "/odin1/cloud_raw",
    "/odin1/cloud_raw_ds",
    "/odin1/cloud_slam",
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
    "map_odin_gate_pass",  # was mislabeled "ins" in V2
    "map_odin_blk",
    "map_block_flag",
    "map_odin_det_pass",
    "map_odin_queued",
    "map_ikd_add_total",
    "map_odin_queued_ds",
    "map_odin_queued_raw",
]


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def cloud_xyz(msg):
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
        return pts[np.isfinite(pts).all(axis=1)]
    from sensor_msgs_py import point_cloud2

    pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3))
    return np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)


def read_bag(bag, topics):
    storage = StorageOptions(uri=str(bag), storage_id="sqlite3")
    converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage, converter)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in topics if t in type_map}
    while reader.has_next():
        topic, data, t_recv = reader.read_next()
        if topic not in out:
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        out[topic].append((msg, t_recv * 1e-9))
    return out, type_map


def quat_to_R(x, y, z, w):
    return Rotation.from_quat([x, y, z, w]).as_matrix()


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
        spd_t.append(ts[i])
        spd.append(float(np.linalg.norm(pos[i] - pos[i - 1]) / dt))
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


def build_mid_only_map(mid_entries, imu_ts, imu_w, od_ts, od_spd, max_pts=40000):
    """Still frames only: |w| and |v| gates. Points must already be Mid-only cloud."""
    chunks = []
    meta = {"n_cand": 0, "n_keep": 0, "reject_w": 0, "reject_v": 0}
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
        return np.zeros((0, 3)), meta
    pts = np.vstack(chunks)
    if pts.shape[0] > max_pts:
        idx = np.random.default_rng(1).choice(pts.shape[0], max_pts, replace=False)
        pts = pts[idx]
    meta["n_pts"] = int(pts.shape[0])
    return pts, meta


def umeyama_se3(src, dst):
    """Rigid SE3: dst ≈ R @ src + t (Umeyama)."""
    assert src.shape == dst.shape and src.shape[0] >= 3
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    X = src - mu_s
    Y = dst - mu_d
    C = (Y.T @ X) / src.shape[0]
    U, S, Vt = np.linalg.svd(C)
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    t = mu_d - R @ mu_s
    err = np.linalg.norm((R @ src.T).T + t - dst, axis=1)
    return R, t, float(np.median(err)), float(np.mean(err))


def icp_se3(src, dst, n_iter=40, max_corr=0.35):
    """Point-to-point ICP: transform src → dst frame."""
    tree = cKDTree(dst)
    R = np.eye(3)
    t = np.zeros(3)
    last = None
    for _ in range(n_iter):
        cur = (R @ src.T).T + t
        d, idx = tree.query(cur, k=1, workers=-1)
        ok = np.isfinite(d) & (d < max_corr)
        if ok.sum() < 50:
            break
        Rn, tn, med, mean = umeyama_se3(cur[ok], dst[idx[ok]])
        R = Rn @ R
        t = Rn @ t + tn
        last = {"n_corr": int(ok.sum()), "med": med, "mean": mean}
        if med < 0.01:
            break
    return R, t, last


def local_plane_residuals(pts, tree, k=5, plane_thr=0.05, max_nn=0.5, max_fit=400):
    n = pts.shape[0]
    res = np.full(n, np.nan)
    ok_plane = np.zeros(n, dtype=bool)
    if n == 0 or tree.n < k:
        return res, ok_plane
    fit_idx = (
        np.linspace(0, n - 1, min(n, max_fit)).astype(int)
        if n > max_fit
        else np.arange(n)
    )
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


def score_paired(xa, xb, tree, max_nn=0.5, max_pts=1200):
    n = min(xa.shape[0], xb.shape[0])
    if n == 0:
        return None
    idx = (
        np.linspace(0, n - 1, max_pts).astype(int) if n > max_pts else np.arange(n)
    )
    qa, qb = xa[idx], xb[idx]
    da, _ = tree.query(qa, k=1, workers=-1)
    db, _ = tree.query(qb, k=1, workers=-1)
    ra, oka = local_plane_residuals(qa, tree, max_nn=max_nn)
    rb, okb = local_plane_residuals(qb, tree, max_nn=max_nn)
    ok_nn = np.isfinite(da) & np.isfinite(db) & (da < max_nn) & (db < max_nn)
    ok_pl = oka & okb
    return {
        "n": int(idx.size),
        "n_nn_both": int(ok_nn.sum()),
        "n_plane_both": int(ok_pl.sum()),
        "nnA_pair_p50": float(np.median(da[ok_nn])) if ok_nn.any() else None,
        "nnB_pair_p50": float(np.median(db[ok_nn])) if ok_nn.any() else None,
        "d_nn_pair": float(np.median(db[ok_nn] - da[ok_nn])) if ok_nn.any() else None,
        "resA_pair_p50": float(np.median(ra[ok_pl])) if ok_pl.any() else None,
        "resB_pair_p50": float(np.median(rb[ok_pl])) if ok_pl.any() else None,
        "d_res_pair": float(np.median(rb[ok_pl] - ra[ok_pl])) if ok_pl.any() else None,
        "frac_nn_both": float(np.mean(ok_nn)),
        "frac_plane_both": float(np.mean(ok_pl)),
    }


def pair_ab(a_e, b_e, met_e, recv_tol=0.01):
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
        if am.width != b_msgs[jb].width or am.width == 0:
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


def world_to_lidar(p_w, t_wi, R_wi, R_il, t_il):
    """Inverse of pointBodyToWorld: p_L = R_IL^{-1}(R_WI^{-1}(p_W - t_WI) - t_IL)."""
    p_i = (R_wi.T @ (p_w - t_wi).T).T
    return (R_il.T @ (p_i - t_il).T).T


def lidar_to_world(p_l, t_wi, R_wi, R_il, t_il):
    """p_W = R_WI (R_IL p_L + t_IL) + t_WI."""
    return (R_wi @ (R_il @ p_l.T + t_il.reshape(3, 1))).T + t_wi


def summarize(rows):
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
        "frac_nn_both": med("frac_nn_both"),
        "frac_plane_both": med("frac_plane_both"),
        "online_d_res": med("online_d_res"),
        "queued_frac": med("queued_frac"),
        "gate_frac": med("gate_frac"),
    }
    dres = [r["d_res_pair"] for r in rows if r.get("d_res_pair") is not None]
    if dres:
        out["frac_B_better_res"] = float(np.mean([x < -1e-4 for x in dres]))
        out["frac_A_better_res"] = float(np.mean([x > 1e-4 for x in dres]))
    return out


def header_series(entries):
    return np.asarray([stamp_sec(m) for m, _ in entries], dtype=np.float64)


def livox_custom_end_times(entries):
    """Livox CustomMsg: timebase(ns) + point offset; end ≈ timebase + max offset."""
    ends = []
    for msg, _ in entries:
        # livox_ros_driver2/msg/CustomMsg
        tb = getattr(msg, "timebase", None)
        if tb is None:
            ends.append(stamp_sec(msg))
            continue
        t0 = float(tb) * 1e-9
        offs = [float(p.offset_time) * 1e-9 for p in msg.points] if msg.points else [0.0]
        ends.append(t0 + max(offs))
    return np.asarray(ends, dtype=np.float64)


def pairwise_dt_stats(a, b, max_dt=0.5):
    """For each a, nearest b; report median (a-b)."""
    if len(a) == 0 or len(b) == 0:
        return None
    dts = []
    for t in a:
        j = int(np.argmin(np.abs(b - t)))
        d = t - b[j]
        if abs(d) <= max_dt:
            dts.append(d)
    if not dts:
        return None
    dts = np.asarray(dts)
    return {
        "n": int(dts.size),
        "med_ms": float(np.median(dts) * 1000),
        "p10_ms": float(np.percentile(dts, 10) * 1000),
        "p90_ms": float(np.percentile(dts, 90) * 1000),
        "mean_ms": float(np.mean(dts) * 1000),
    }


def analyze_time_axes(bag):
    data, _ = read_bag(bag, TIME_TOPICS)
    out = {"topics": {k: len(v) for k, v in data.items()}}
    mid_imu = header_series(data.get("/livox/imu", []))
    odin_imu = header_series(data.get("/odin1/imu", []))
    # Mid lidar: CustomMsg preferred
    if data.get("/livox/lidar"):
        mid_lidar_end = livox_custom_end_times(data["/livox/lidar"])
        mid_lidar_hdr = header_series(data["/livox/lidar"])
        out["mid_lidar_header_vs_imu"] = pairwise_dt_stats(mid_lidar_hdr, mid_imu)
        out["mid_lidar_end_vs_imu"] = pairwise_dt_stats(mid_lidar_end, mid_imu)
        out["mid_lidar_end_minus_header_ms"] = float(
            np.median(mid_lidar_end - mid_lidar_hdr) * 1000
        )
    for key in ("/odin1/cloud_raw", "/odin1/cloud_raw_ds", "/odin1/cloud_slam"):
        if not data.get(key):
            continue
        od_hdr = header_series(data[key])
        out[f"odin_{key.split('/')[-1]}_header_vs_odin_imu"] = pairwise_dt_stats(
            od_hdr, odin_imu
        )
        out[f"odin_{key.split('/')[-1]}_header_vs_mid_imu"] = pairwise_dt_stats(
            od_hdr, mid_imu
        )
    if len(odin_imu) and len(mid_imu):
        out["odin_imu_vs_mid_imu"] = pairwise_dt_stats(odin_imu, mid_imu, max_dt=1.0)
    out["note"] = (
        "These are header/timebase relationships, NOT physical LiDAR clock offset "
        "and NOT b_pair / dual-IMU xcorr / NN scan-phase."
    )
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fusion-bag", type=Path, required=True)
    ap.add_argument(
        "--mid-only-bag",
        type=Path,
        default=None,
        help="Bag from Mid-only IEKF (lidar2_update_en=false). If omitted, "
        "falls back to fusion /cloud_registered (fusion-pose contaminated).",
    )
    ap.add_argument("--out", type=Path, default=Path("/tmp/odin_xtform_v3"))
    ap.add_argument(
        "--b-clock-ms",
        type=str,
        default="-150,-120,-100,-80,-60,-40,-20,0,20,40,60,80,100,120",
    )
    ap.add_argument("--time-bag", type=Path, default=None)
    ap.add_argument("--no-icp", action="store_true")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    def resolve(bag: Path) -> Path:
        if (bag / "metadata.yaml").exists():
            return bag
        cands = list(bag.rglob("metadata.yaml"))
        if not cands:
            raise SystemExit(f"no bag at {bag}")
        return cands[0].parent

    fusion_bag = resolve(args.fusion_bag)
    print("[read] fusion", fusion_bag, flush=True)
    fusion, _ = read_bag(fusion_bag, FUSION_TOPICS)
    for t, v in fusion.items():
        print(f"  {t}: {len(v)}", flush=True)

    imu_ts, imu_w, imu_wz = imu_series(fusion.get("/livox/imu", []))
    od_ts, od_pos, od_rot = odom_poses(fusion.get("/Odometry", []))
    spd_t, spd = odom_speed(od_ts, od_pos)

    map_source = "fusion_cloud_registered_pose_contaminated"
    mid_map = np.zeros((0, 3))
    map_meta = {}
    align = None

    if args.mid_only_bag is not None:
        mid_bag = resolve(args.mid_only_bag)
        print("[read] mid-only", mid_bag, flush=True)
        mid_data, _ = read_bag(
            mid_bag, ["/cloud_registered", "/Odometry", "/livox/imu"]
        )
        m_imu_ts, m_imu_w, _ = imu_series(mid_data.get("/livox/imu", []))
        m_od_ts, m_od_pos, _ = odom_poses(mid_data.get("/Odometry", []))
        m_spd_t, m_spd = odom_speed(m_od_ts, m_od_pos)
        mid_map, map_meta = build_mid_only_map(
            mid_data.get("/cloud_registered", []),
            m_imu_ts,
            m_imu_w,
            m_spd_t,
            m_spd,
        )
        map_source = "independent_mid_only_iekf"
        map_meta["source_bag"] = str(mid_bag)
        # Align Mid-only map → fusion still Mid clouds (same physical scene)
        if not args.no_icp and mid_map.shape[0] >= 500:
            fus_still, fus_meta = build_mid_only_map(
                fusion.get("/cloud_registered", []), imu_ts, imu_w, spd_t, spd
            )
            if fus_still.shape[0] >= 500:
                # subsample for ICP
                rng = np.random.default_rng(2)
                src = mid_map[
                    rng.choice(mid_map.shape[0], min(8000, mid_map.shape[0]), False)
                ]
                dst = fus_still[
                    rng.choice(
                        fus_still.shape[0], min(8000, fus_still.shape[0]), False
                    )
                ]
                R_a, t_a, icp_info = icp_se3(src, dst)
                mid_map = (R_a @ mid_map.T).T + t_a
                align = {
                    "method": "icp_point_to_point",
                    "R": R_a.tolist(),
                    "t": t_a.tolist(),
                    "icp": icp_info,
                    "fusion_still_map": fus_meta,
                }
                map_source = "independent_mid_only_iekf_icp_to_fusion"
                print(f"[align] icp={icp_info}", flush=True)
    else:
        mid_map, map_meta = build_mid_only_map(
            fusion.get("/cloud_registered", []), imu_ts, imu_w, spd_t, spd
        )
        map_meta["warning"] = (
            "Poses of /cloud_registered come from fused IEKF when "
            "lidar2_update_en=true; Mid points only, but trajectory may be "
            "Odin-Stage2 influenced."
        )

    if mid_map.shape[0] < 500:
        raise SystemExit(f"Mid map too small: {map_meta}")
    tree = cKDTree(mid_map)
    print(f"[map] source={map_source} {map_meta}", flush=True)

    pairs = pair_ab(
        fusion.get("/cloud_odin_xtform_A", []),
        fusion.get("/cloud_odin_xtform_B", []),
        fusion.get("/odin_xtform_metrics", []),
    )
    print(f"[pair] {len(pairs)}", flush=True)

    rows = []
    insert_rows = []
    for fi, (am, bm, met, recv) in enumerate(pairs):
        if (fi + 1) % 80 == 0:
            print(f"[score] {fi+1}/{len(pairs)}", flush=True)
        sc = score_paired(cloud_xyz(am), cloud_xyz(bm), tree)
        if sc is None:
            continue
        t_mid = met["t_mid"] if met and "t_mid" in met else stamp_sec(am)
        t_odin = met["t_odin"] if met and "t_odin" in met else t_mid
        w = nearest(imu_ts, imu_w, t_mid)
        wz = nearest(imu_ts, imu_wz, t_mid)
        v = nearest(spd_t, spd, t_mid) if len(spd_t) else 0.0
        row = {**sc, "seg": classify(w, wz, v), "t_mid": t_mid, "t_odin": t_odin, "w": w, "v": v}
        if met:
            row["online_d_res"] = met.get("resB_med", 0) - met.get("resA_med", 0)
            cand = met.get("map_odin_cand", 0) or 0
            gate = met.get("map_odin_gate_pass", 0) or 0
            queued = met.get("map_odin_queued", 0) or 0
            det = met.get("map_odin_det_pass", 0) or 0
            row["gate_frac"] = (gate / cand) if cand else None
            row["det_frac"] = (det / cand) if cand else None
            row["queued_frac"] = (queued / cand) if cand else None
            row["ikd_add_total"] = met.get("map_ikd_add_total")
            row["map_block_flag"] = met.get("map_block_flag")
            row["n_plane_A_online"] = met.get("n_plane_A")
            row["n_plane_B_online"] = met.get("n_plane_B")
            row["n_plane_both_online"] = met.get("n_plane_both")
            insert_rows.append(row)
        rows.append(row)

    # b_clock sweep with full T_WI * T_IL
    R_il, t_il = DEFAULT_R_IL, DEFAULT_T_IL
    b_list = [float(x) for x in args.b_clock_ms.split(",") if x.strip()]
    motion_pairs = []
    for am, bm, met, recv in pairs:
        if not met or "t_mid" not in met:
            continue
        t_mid, t_odin = met["t_mid"], met["t_odin"]
        seg = classify(
            nearest(imu_ts, imu_w, t_mid),
            nearest(imu_ts, imu_wz, t_mid),
            nearest(spd_t, spd, t_mid) if len(spd_t) else 0.0,
        )
        if seg == "still":
            continue
        xa = cloud_xyz(am)
        p, R = interp_pose(od_ts, od_pos, od_rot, t_mid)
        if p is None:
            continue
        pb = world_to_lidar(xa, p, R, R_il, t_il)
        motion_pairs.append((pb, t_odin, t_mid, seg))

    sweep_pairs = motion_pairs[:: max(1, len(motion_pairs) // 80)]
    # Also compare naive (ignore extrinsic) at a few offsets
    sweep, sweep_naive = [], []
    print(f"[sweep] n={len(sweep_pairs)} b={b_list}", flush=True)
    for b_ms in b_list:
        nn_full, res_full, nn_naive = [], [], []
        for pb, t_odin, t_mid, seg in sweep_pairs:
            p, R = interp_pose(od_ts, od_pos, od_rot, t_odin + b_ms * 1e-3)
            if p is None:
                continue
            pw = lidar_to_world(pb, p, R, R_il, t_il)
            if pw.shape[0] > 800:
                pw = pw[np.linspace(0, pw.shape[0] - 1, 800).astype(int)]
            d, _ = tree.query(pw, k=1, workers=-1)
            d_ok = d[np.isfinite(d) & (d < 0.5)]
            if d_ok.size == 0:
                continue
            nn_full.append(float(np.median(d_ok)))
            rloc, rok = local_plane_residuals(pw, tree, max_fit=200)
            if rok.any():
                res_full.append(float(np.median(rloc[rok])))
            # naive: treat odom as T_WL
            pw_n = (R @ pb.T).T + p
            if pw_n.shape[0] > 800:
                pw_n = pw_n[np.linspace(0, pw_n.shape[0] - 1, 800).astype(int)]
            dn, _ = tree.query(pw_n, k=1, workers=-1)
            dn_ok = dn[np.isfinite(dn) & (dn < 0.5)]
            if dn_ok.size:
                nn_naive.append(float(np.median(dn_ok)))
        sweep.append(
            {
                "b_clock_ms": b_ms,
                "n": len(nn_full),
                "nn_p50_med": float(np.median(nn_full)) if nn_full else None,
                "res_p50_med": float(np.median(res_full)) if res_full else None,
            }
        )
        sweep_naive.append(
            {
                "b_clock_ms": b_ms,
                "n": len(nn_naive),
                "nn_p50_med": float(np.median(nn_naive)) if nn_naive else None,
            }
        )

    by_seg = {
        s: summarize([r for r in rows if r["seg"] == s])
        for s in ("still", "turn_left", "turn_right", "translate", "combined", "other")
    }
    motion = summarize([r for r in rows if r["seg"] != "still"])
    still = by_seg.get("still")

    insert_audit = None
    if insert_rows:
        def medf(k):
            xs = [r[k] for r in insert_rows if r.get(k) is not None]
            return float(np.median(xs)) if xs else None

        insert_audit = {
            "n": len(insert_rows),
            "gate_frac_med": medf("gate_frac"),
            "det_frac_med": medf("det_frac"),
            "queued_frac_med": medf("queued_frac"),
            "queued_frac_mean": float(
                np.mean(
                    [r["queued_frac"] for r in insert_rows if r.get("queued_frac") is not None]
                )
            )
            if any(r.get("queued_frac") is not None for r in insert_rows)
            else None,
            "frac_frames_block": float(
                np.mean([1.0 if r.get("map_block_flag") == 1.0 else 0.0 for r in insert_rows])
            ),
            "ikd_add_total_med": medf("ikd_add_total"),
            "note": (
                "V2 'ins_frac' was gate_pass/cand, NOT true ikdtree inserts. "
                "queued/cand is points entering PointToAdd/PointNoNeedDownsample. "
                "ikd_add_total is Add_Points return for Mid+Odin; downsample=false "
                "path does not increment the return counter."
            ),
            "by_seg_queued": {},
        }
        for s in by_seg:
            xs = [
                r["queued_frac"]
                for r in insert_rows
                if r["seg"] == s and r.get("queued_frac") is not None
            ]
            if xs:
                insert_audit["by_seg_queued"][s] = {
                    "n": len(xs),
                    "med": float(np.median(xs)),
                }

    best_full = min(
        (s for s in sweep if s["nn_p50_med"] is not None),
        key=lambda s: s["nn_p50_med"],
        default=None,
    )
    best_naive = min(
        (s for s in sweep_naive if s["nn_p50_med"] is not None),
        key=lambda s: s["nn_p50_med"],
        default=None,
    )
    best_res = min(
        (s for s in sweep if s.get("res_p50_med") is not None),
        key=lambda s: s["res_p50_med"],
        default=None,
    )

    dres = motion.get("d_res_pair") if motion else None
    dnn = motion.get("d_nn_pair") if motion else None
    if dres is not None:
        if dres < -0.002:
            mid_rank = "B_better"
        elif dres > 0.002:
            mid_rank = "A_better"
        else:
            mid_rank = "similar"
    else:
        mid_rank = None

    time_axis = None
    tb = args.time_bag or args.fusion_bag
    try:
        time_axis = analyze_time_axes(resolve(tb))
    except Exception as e:
        time_axis = {"error": str(e)}

    independent = map_source.startswith("independent")
    ready = bool(
        independent
        and mid_rank == "B_better"
        and dres is not None
        and dres < -0.005
        and still
        and (still.get("d_res_pair") is None or abs(still["d_res_pair"]) < 0.01)
    )

    summary = {
        "map_source": map_source,
        "map_meta": map_meta,
        "align": align,
        "n_pairs": len(pairs),
        "insert_audit_v3": insert_audit,
        "by_segment": by_seg,
        "motion": motion,
        "still": still,
        "b_clock_sweep_full_TWI_TIL": sweep,
        "b_clock_sweep_naive_ignore_extrinsic": sweep_naive,
        "b_clock_geom_best_nn_full": best_full,
        "b_clock_geom_best_nn_naive": best_naive,
        "b_clock_geom_best_res_full": best_res,
        "note_b_clock": "geom-optimal ≠ physical clock; full uses T_WI*T_IL",
        "extrinsic_T_IL": DEFAULT_T_IL.tolist(),
        "time_axes": time_axis,
        "answers": {
            "1_geom_b_clock_near_0": bool(
                best_full and abs(best_full["b_clock_ms"]) <= 20
            ),
            "2_sensitive_to_map_pose_model": bool(
                (best_full and best_naive)
                and abs(best_full["b_clock_ms"] - best_naive["b_clock_ms"]) > 20
            )
            or (not independent),
            "3_B_stable_on_independent": mid_rank == "B_better" and independent,
            "4_ready_stage2_trial": ready,
            "mid_only_rank": mid_rank,
            "d_res_pair_motion": dres,
            "d_nn_pair_motion": dnn,
            "independent_map": independent,
        },
    }
    out_json = args.out / "summary.json"
    out_json.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"[wrote] {out_json}")


if __name__ == "__main__":
    main()
