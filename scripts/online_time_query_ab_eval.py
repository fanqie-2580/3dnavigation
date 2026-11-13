#!/usr/bin/env python3
"""Evaluate online /cloud_odin_registered_{A,D} vs Mid map (Stage1 native deskew).

Pairing (anti-misalign):
  1) Prefer explicit /cloud_odin_tq_ab_id if present (shared frame id).
  2) Else pair by bag receive-time proximity + equal point count fingerprint.
  Index-only pairing is reported as a cross-check, not the primary metric.

Does NOT change fusion.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
import subprocess
import time
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


TOPICS = [
    "/cloud_odin_registered_A",
    "/cloud_odin_registered_D",
    "/cloud_odin_registered",
    "/cloud_registered",
    "/Laser_map",
    "/Odometry",
    "/livox/imu",
    "/cloud_odin_tq_ab_id",
]


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def cloud_xyz(msg) -> np.ndarray:
    from sensor_msgs_py import point_cloud2

    pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3), dtype=np.float64)
    return np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)


def cloud_fingerprint(msg) -> tuple:
    """Geometry-invariant fingerprint of the published cloud (count + xyz moments).

    A and D share the same Undistort body but different map poses, so raw XYZ
    differ. Fingerprint uses n_points only for A↔D; for official↔mode match we
    use full xyz hash when poses should match.
    """
    xyz = cloud_xyz(msg)
    n = int(xyz.shape[0])
    if n == 0:
        return (0, 0.0, 0.0, 0.0)
    # Stable moments (pose-dependent — only compare when same t_query expected)
    c = xyz.mean(axis=0)
    return (n, float(c[0]), float(c[1]), float(c[2]))


def cloud_n(msg) -> int:
    return int(msg.width) if hasattr(msg, "width") else cloud_xyz(msg).shape[0]


def read_bag(bag_dir: Path):
    storage = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
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
        # Some bags may lack optional topics
        try:
            msg = deserialize_message(data, get_message(type_map[topic]))
        except Exception:
            continue
        out[topic].append((msg, t_recv * 1e-9))  # ns → s
    return out, type_map


def unwrap(entries):
    """Support both (msg, t_recv) and bare msg."""
    if not entries:
        return [], []
    if isinstance(entries[0], tuple):
        msgs = [e[0] for e in entries]
        recv = [e[1] for e in entries]
        return msgs, recv
    return list(entries), [stamp_sec(m) for m in entries]


def imu_w_series(imu_msgs):
    ts, w = [], []
    for m in imu_msgs:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        g = m.angular_velocity
        ts.append(t)
        w.append(math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z))
    return np.asarray(ts), np.asarray(w)


def nearest_w(ts, w, t):
    if len(ts) == 0:
        return 0.0
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return float(w[0])
    if i >= len(ts):
        return float(w[-1])
    return float(w[i - 1] if abs(ts[i - 1] - t) < abs(ts[i] - t) else w[i])


def build_still_map(mid_clouds, imu_ts, imu_w, w_max=0.06, max_pts=50000):
    chunks = []
    for msg in mid_clouds:
        t = stamp_sec(msg)
        if nearest_w(imu_ts, imu_w, t) > w_max:
            continue
        xyz = cloud_xyz(msg)
        if xyz.shape[0] == 0:
            continue
        chunks.append(xyz)
    if not chunks:
        return np.zeros((0, 3)), {"n_frames": 0}
    pts = np.vstack(chunks)
    if pts.shape[0] > max_pts:
        idx = np.random.default_rng(0).choice(pts.shape[0], max_pts, replace=False)
        pts = pts[idx]
    return pts, {"n_frames": len(chunks), "n_pts": int(pts.shape[0])}


def score_cloud(query: np.ndarray, tree: cKDTree, max_nn=0.5):
    if query.shape[0] == 0:
        return None
    if query.shape[0] > 8000:
        idx = np.random.default_rng(1).choice(query.shape[0], 8000, replace=False)
        query = query[idx]
    d, _ = tree.query(query, k=1, workers=-1)
    d = d[np.isfinite(d) & (d < max_nn)]
    if d.size == 0:
        return None
    return {
        "nn_p50": float(np.median(d)),
        "nn_p90": float(np.percentile(d, 90)),
        "nn_mean": float(np.mean(d)),
        "n": int(d.size),
    }


def cloud_delta_p50(a_msg, b_msg, max_pts=5000):
    """Median point-to-point distance after associating by index subsample (same n)."""
    xa, xb = cloud_xyz(a_msg), cloud_xyz(b_msg)
    if xa.shape[0] == 0 or xb.shape[0] == 0:
        return None
    n = min(xa.shape[0], xb.shape[0], max_pts)
    rng = np.random.default_rng(3)
    ia = rng.choice(xa.shape[0], n, replace=False)
    # NN from A sample into B
    tree = cKDTree(xb)
    d, _ = tree.query(xa[ia], k=1, workers=-1)
    d = d[np.isfinite(d)]
    return float(np.median(d)) if d.size else None


def pair_by_recv_and_count(a_entries, d_entries, recv_tol=0.005):
    """Primary pairing: bag receive-time + equal point count (anti-shift)."""
    a_msgs, a_recv = unwrap(a_entries)
    d_msgs, d_recv = unwrap(d_entries)
    d_recv_arr = np.asarray(d_recv, dtype=np.float64)
    used = set()
    pairs = []
    n_reject_count = 0
    n_reject_recv = 0
    for i, am in enumerate(a_msgs):
        if d_recv_arr.size == 0:
            break
        j = int(np.argmin(np.abs(d_recv_arr - a_recv[i])))
        if j in used:
            n_reject_recv += 1
            continue
        if abs(d_recv_arr[j] - a_recv[i]) > recv_tol:
            n_reject_recv += 1
            continue
        if cloud_n(am) != cloud_n(d_msgs[j]):
            n_reject_count += 1
            continue
        used.add(j)
        pairs.append((am, d_msgs[j], stamp_sec(am), stamp_sec(d_msgs[j]), a_recv[i], d_recv[j]))
    meta = {
        "method": "recv_time+n_points",
        "recv_tol_s": recv_tol,
        "n_pairs": len(pairs),
        "n_A": len(a_msgs),
        "n_D": len(d_msgs),
        "reject_recv": n_reject_recv,
        "reject_count": n_reject_count,
        "index_would_pair": min(len(a_msgs), len(d_msgs)),
    }
    return pairs, meta


def pair_by_ab_id(a_entries, d_entries, id_entries):
    """Pair via explicit /cloud_odin_tq_ab_id (UInt64) published once per Bundle."""
    if not id_entries:
        return None, {"method": "ab_id", "available": False}
    a_msgs, a_recv = unwrap(a_entries)
    d_msgs, d_recv = unwrap(d_entries)
    id_msgs, id_recv = unwrap(id_entries)
    # Each id should land between A and D publishes (same callback).
    a_recv_arr = np.asarray(a_recv)
    d_recv_arr = np.asarray(d_recv)
    used_a, used_d = set(), set()
    pairs = []
    for k, im in enumerate(id_msgs):
        fid = int(im.data) if hasattr(im, "data") else int(im)
        ia = int(np.argmin(np.abs(a_recv_arr - id_recv[k]))) if a_recv_arr.size else -1
        id_ = int(np.argmin(np.abs(d_recv_arr - id_recv[k]))) if d_recv_arr.size else -1
        if ia < 0 or id_ < 0:
            continue
        if ia in used_a or id_ in used_d:
            continue
        if abs(a_recv_arr[ia] - id_recv[k]) > 0.02 or abs(d_recv_arr[id_] - id_recv[k]) > 0.02:
            continue
        if cloud_n(a_msgs[ia]) != cloud_n(d_msgs[id_]):
            continue
        used_a.add(ia)
        used_d.add(id_)
        pairs.append(
            (
                a_msgs[ia],
                d_msgs[id_],
                stamp_sec(a_msgs[ia]),
                stamp_sec(d_msgs[id_]),
                a_recv[ia],
                d_recv[id_],
                fid,
            )
        )
    return pairs, {
        "method": "ab_id",
        "available": True,
        "n_ids": len(id_msgs),
        "n_pairs": len(pairs),
    }


def record_bag(out_dir: Path, sec: float):
    import shutil

    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(out_dir),
        "--compression-mode",
        "none",
        *TOPICS,
    ]
    print(f"[rec] {sec:.0f}s → {out_dir}  (turn L/R if possible)")
    proc = subprocess.Popen(cmd)
    try:
        time.sleep(sec)
    finally:
        proc.send_signal(2)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
    return out_dir


def summarize(sel):
    if not sel:
        return None

    def med(k):
        return float(np.median([r[k] for r in sel]))

    return {
        "n": len(sel),
        "w_mean": float(np.mean([r["w_norm"] for r in sel])),
        "A_nn_p50": med("A_nn_p50"),
        "D_nn_p50": med("D_nn_p50"),
        "A_nn_p90": med("A_nn_p90"),
        "D_nn_p90": med("D_nn_p90"),
        "D_minus_A_p50_med": med("D_minus_A_p50"),
        "dt_AD_ms_med": med("dt_AD_ms"),
        "recv_dt_ms_med": med("recv_dt_ms"),
        "frac_D_better": float(np.mean([r["D_minus_A_p50"] < -0.005 for r in sel])),
        "frac_A_better": float(np.mean([r["D_minus_A_p50"] > 0.005 for r in sel])),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=Path, default=None)
    ap.add_argument("--record-sec", type=float, default=0.0)
    ap.add_argument("--out", type=Path, default=Path("/tmp/online_tq_ab_out"))
    ap.add_argument("--w-turn", type=float, default=0.25)
    ap.add_argument("--official-vs", choices=["A", "D", "auto"], default="auto")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    bag = args.bag
    if args.record_sec > 0:
        bag = args.out / "bag"
        record_bag(bag, args.record_sec)
    if bag is None:
        raise SystemExit("Provide --bag or --record-sec")

    meta = bag / "metadata.yaml"
    if not meta.exists():
        cands = list(bag.rglob("metadata.yaml"))
        if not cands:
            raise SystemExit(f"No bag metadata under {bag}")
        bag = cands[0].parent

    print(f"[load] {bag}")
    data, type_map = read_bag(bag)
    for t in TOPICS:
        print(f"  {t}: {len(data.get(t, []))}")

    a_e, d_e = data["/cloud_odin_registered_A"], data["/cloud_odin_registered_D"]
    id_e = data.get("/cloud_odin_tq_ab_id", [])

    pairs_id, meta_id = pair_by_ab_id(a_e, d_e, id_e)
    pairs_recv, meta_recv = pair_by_recv_and_count(a_e, d_e)

    if pairs_id and meta_id.get("n_pairs", 0) >= 10:
        pairs, pair_meta = pairs_id, meta_id
        # normalize tuple length
        pairs = [
            (p[0], p[1], p[2], p[3], p[4], p[5])
            for p in pairs
        ]
    else:
        pairs, pair_meta = pairs_recv, meta_recv

    # Cross-check: how often index pairing would disagree with recv pairing
    a_msgs, _ = unwrap(a_e)
    d_msgs, _ = unwrap(d_e)
    n_idx = min(len(a_msgs), len(d_msgs))
    n_idx_agree = 0
    recv_set = {(id(p[0]), id(p[1])) for p in pairs}
    for i in range(n_idx):
        if (id(a_msgs[i]), id(d_msgs[i])) in recv_set:
            n_idx_agree += 1
    pair_meta["index_agree_with_primary"] = n_idx_agree
    pair_meta["index_agree_frac"] = float(n_idx_agree / max(1, len(pairs)))
    print(f"[pair] primary={pair_meta}")

    imu_msgs, _ = unwrap(data["/livox/imu"])
    imu_ts, imu_w = imu_w_series(imu_msgs)
    mid_msgs, _ = unwrap(data["/cloud_registered"])
    mid_map, map_meta = build_still_map(mid_msgs, imu_ts, imu_w)
    laser_msgs, _ = unwrap(data["/Laser_map"])
    if mid_map.shape[0] < 1000 and laser_msgs:
        lm = cloud_xyz(laser_msgs[-1])
        if lm.shape[0] > mid_map.shape[0]:
            if lm.shape[0] > 80000:
                idx = np.random.default_rng(2).choice(lm.shape[0], 80000, replace=False)
                lm = lm[idx]
            mid_map = lm
            map_meta = {"source": "Laser_map", "n_pts": int(lm.shape[0])}
    if mid_map.shape[0] < 100:
        raise SystemExit("Static map too small")
    tree = cKDTree(mid_map)

    rows = []
    for am, dm, ta, td, ra, rd in pairs:
        # Score motion at Mid-ish time ≈ A stamp (near Mid_end)
        w = nearest_w(imu_ts, imu_w, ta)
        sa = score_cloud(cloud_xyz(am), tree)
        sd = score_cloud(cloud_xyz(dm), tree)
        if sa is None or sd is None:
            continue
        rows.append(
            {
                "tA": ta,
                "tD": td,
                "dt_AD_ms": (td - ta) * 1000.0,
                "recv_dt_ms": (rd - ra) * 1000.0,
                "w_norm": w,
                "A_nn_p50": sa["nn_p50"],
                "A_nn_p90": sa["nn_p90"],
                "D_nn_p50": sd["nn_p50"],
                "D_nn_p90": sd["nn_p90"],
                "D_minus_A_p50": sd["nn_p50"] - sa["nn_p50"],
                "n_pts": cloud_n(am),
            }
        )

    # Official vs A/D (pose-match): receive-time pair + xyz delta
    off_e = data["/cloud_odin_registered"]
    off_msgs, off_recv = unwrap(off_e)
    off_vs = {"A": [], "D": []}
    for label, entries in (("A", a_e), ("D", d_e)):
        msgs, recv = unwrap(entries)
        recv_arr = np.asarray(recv)
        for i, om in enumerate(off_msgs):
            if recv_arr.size == 0:
                break
            j = int(np.argmin(np.abs(recv_arr - off_recv[i])))
            if abs(recv_arr[j] - off_recv[i]) > 0.01:
                continue
            if cloud_n(om) != cloud_n(msgs[j]):
                continue
            dlt = cloud_delta_p50(om, msgs[j])
            if dlt is None:
                continue
            off_vs[label].append(dlt)

    def off_sum(xs):
        if not xs:
            return {"n": 0, "delta_p50_med": None}
        return {"n": len(xs), "delta_p50_med": float(np.median(xs))}

    all_s = summarize(rows)
    turn_s = summarize([r for r in rows if r["w_norm"] >= args.w_turn])
    still_s = summarize([r for r in rows if r["w_norm"] < 0.08])

    summary = {
        "map": map_meta,
        "pairing": pair_meta,
        "pairing_recv_fallback": meta_recv,
        "pairing_ab_id": meta_id,
        "all": all_s,
        "turn": turn_s,
        "still": still_s,
        "official_vs_A": off_sum(off_vs["A"]),
        "official_vs_D": off_sum(off_vs["D"]),
        "verdict": None,
    }
    if turn_s:
        dlt = turn_s["D_minus_A_p50_med"]
        if dlt < -0.01 and turn_s["frac_D_better"] >= 0.55:
            summary["verdict"] = "D_wins_online_turn"
        elif dlt > 0.01 and turn_s["frac_A_better"] >= 0.55:
            summary["verdict"] = "A_wins_online_turn"
        else:
            summary["verdict"] = "inconclusive"

    out_json = args.out / "summary.json"
    out_json.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"[wrote] {out_json}")


if __name__ == "__main__":
    main()
