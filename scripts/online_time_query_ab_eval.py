#!/usr/bin/env python3
"""Evaluate online /cloud_odin_registered_{A,D} vs Mid map (Stage1 native deskew).

Records (or loads) a bag with:
  A/D debug clouds, official orange, Mid registered, Laser_map, Odometry, Mid IMU.

Does NOT change fusion. Scores only frames where both A and D exist near the same stamp.
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
    "/cloud_odin_registered_A",
    "/cloud_odin_registered_D",
    "/cloud_odin_registered",
    "/cloud_registered",
    "/Laser_map",
    "/Odometry",
    "/livox/imu",
]


def stamp_sec(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def cloud_xyz(msg) -> np.ndarray:
    # sensor_msgs/PointCloud2 → Nx3 float64
    from sensor_msgs_py import point_cloud2

    pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not pts:
        return np.zeros((0, 3), dtype=np.float64)
    arr = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
    return arr


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
        topic, data, _t = reader.read_next()
        if topic not in out:
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        out[topic].append(msg)
    return out


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
        # fallback: last Laser_map handled by caller
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


def pair_by_index(a_msgs, d_msgs):
    """Pair A/D by publish order (same Bundle call). Stamps differ by design (~tA vs tD)."""
    n = min(len(a_msgs), len(d_msgs))
    pairs = []
    for i in range(n):
        am, dm = a_msgs[i], d_msgs[i]
        ta, td = stamp_sec(am), stamp_sec(dm)
        pairs.append((am, dm, ta, td))
    return pairs


def record_bag(out_dir: Path, sec: float):
    import shutil

    # ros2 bag record creates out_dir; it must not already exist.
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
        proc.send_signal(2)  # SIGINT
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
    return out_dir


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=Path, default=None, help="Existing bag dir")
    ap.add_argument("--record-sec", type=float, default=0.0)
    ap.add_argument("--out", type=Path, default=Path("/tmp/online_tq_ab_out"))
    ap.add_argument("--w-turn", type=float, default=0.25)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    bag = args.bag
    if args.record_sec > 0:
        bag = args.out / "bag"
        record_bag(bag, args.record_sec)
    if bag is None:
        raise SystemExit("Provide --bag or --record-sec")

    # ros2 bag may create bag/bag if out existed; find metadata
    meta = bag / "metadata.yaml"
    if not meta.exists():
        cands = list(bag.rglob("metadata.yaml"))
        if not cands:
            raise SystemExit(f"No bag metadata under {bag}")
        bag = cands[0].parent

    print(f"[load] {bag}")
    data = read_bag(bag)
    for t in TOPICS:
        print(f"  {t}: {len(data[t])}")

    imu_ts, imu_w = imu_w_series(data["/livox/imu"])
    mid_map, map_meta = build_still_map(data["/cloud_registered"], imu_ts, imu_w)
    if mid_map.shape[0] < 1000 and data["/Laser_map"]:
        # prefer denser still mid; else last Laser_map snapshot
        lm = cloud_xyz(data["/Laser_map"][-1])
        if lm.shape[0] > mid_map.shape[0]:
            if lm.shape[0] > 80000:
                idx = np.random.default_rng(2).choice(lm.shape[0], 80000, replace=False)
                lm = lm[idx]
            mid_map = lm
            map_meta = {"source": "Laser_map", "n_pts": int(lm.shape[0])}
    if mid_map.shape[0] < 100:
        raise SystemExit("Static map too small")
    tree = cKDTree(mid_map)

    pairs = pair_by_index(data["/cloud_odin_registered_A"], data["/cloud_odin_registered_D"])
    print(f"[pair] A/D pairs={len(pairs)} (index-aligned; stamp Δ is tA vs tD)")

    rows = []
    for am, dm, ta, td in pairs:
        w = nearest_w(imu_ts, imu_w, 0.5 * (ta + td))
        sa = score_cloud(cloud_xyz(am), tree)
        sd = score_cloud(cloud_xyz(dm), tree)
        if sa is None or sd is None:
            continue
        rows.append(
            {
                "tA": ta,
                "tD": td,
                "dt_AD_ms": (td - ta) * 1000.0,
                "w_norm": w,
                "A_nn_p50": sa["nn_p50"],
                "A_nn_p90": sa["nn_p90"],
                "D_nn_p50": sd["nn_p50"],
                "D_nn_p90": sd["nn_p90"],
                "D_minus_A_p50": sd["nn_p50"] - sa["nn_p50"],
            }
        )

    # also score official orange vs A on same-ish stamps
    off_rows = []
    off = data["/cloud_odin_registered"]
    a_ts = np.array([stamp_sec(m) for m in data["/cloud_odin_registered_A"]])
    for om in off:
        to = stamp_sec(om)
        if a_ts.size == 0:
            break
        i = int(np.argmin(np.abs(a_ts - to)))
        if abs(a_ts[i] - to) > 0.02:
            continue
        so = score_cloud(cloud_xyz(om), tree)
        sa = score_cloud(cloud_xyz(data["/cloud_odin_registered_A"][i]), tree)
        if so and sa:
            off_rows.append(
                {
                    "off_nn_p50": so["nn_p50"],
                    "A_nn_p50": sa["nn_p50"],
                    "off_minus_A": so["nn_p50"] - sa["nn_p50"],
                }
            )

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
            "frac_D_better": float(np.mean([r["D_minus_A_p50"] < -0.005 for r in sel])),
            "frac_A_better": float(np.mean([r["D_minus_A_p50"] > 0.005 for r in sel])),
        }

    all_s = summarize(rows)
    turn_s = summarize([r for r in rows if r["w_norm"] >= args.w_turn])
    still_s = summarize([r for r in rows if r["w_norm"] < 0.08])

    summary = {
        "map": map_meta,
        "all": all_s,
        "turn": turn_s,
        "still": still_s,
        "official_vs_A": {
            "n": len(off_rows),
            "off_minus_A_med": float(np.median([r["off_minus_A"] for r in off_rows]))
            if off_rows
            else None,
        },
        "verdict": None,
    }
    if turn_s:
        dlt = turn_s["D_minus_A_p50_med"]
        if dlt < -0.01 and turn_s["frac_D_better"] >= 0.55:
            summary["verdict"] = "D_wins_online_turn — candidate to replace A (keep A code until confirmed)"
        elif dlt > 0.01 and turn_s["frac_A_better"] >= 0.55:
            summary["verdict"] = "A_wins_online_turn — keep A; do not switch"
        else:
            summary["verdict"] = "inconclusive — keep A; need more turn samples or metric"

    out_json = args.out / "summary.json"
    out_json.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"[wrote] {out_json}")


if __name__ == "__main__":
    main()
