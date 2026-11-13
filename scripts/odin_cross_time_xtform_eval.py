#!/usr/bin/env python3
"""Evaluate /odin_xtform_metrics shadow A vs B by motion segment.

A = Odin_end body × Mid_end pose (current Stage2-style)
B = SE3 re-express to Mid_end body, then × same Mid_end pose

Does not modify fusion. Requires mapping.odin_cross_time_shadow_en:=true.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

import numpy as np

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

TOPICS = [
    "/odin_xtform_metrics",
    "/livox/imu",
    "/Odometry",
    "/cloud_odin_xtform_A",
    "/cloud_odin_xtform_B",
]

# metrics layout from laserMapping_adaptive shadow_odin_cross_time_eval
KEYS = [
    "dt_ms",
    "rel_yaw_deg",
    "rel_trans_m",
    "resA_med",
    "resB_med",
    "nnA_med",
    "nnB_med",
    "n_scored",
    "passA_frac",
    "passB_frac",
    "w_norm",
    "t_odin",
    "t_mid",
    "b_pre_ms",
]


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
        msg = deserialize_message(data, get_message(type_map[topic]))
        out[topic].append((msg, t_recv * 1e-9))
    return out


def parse_metrics(entries):
    rows = []
    for msg, _recv in entries:
        d = list(msg.data)
        if len(d) < 11:
            continue
        row = {k: float(d[i]) for i, k in enumerate(KEYS) if i < len(d)}
        if row.get("resA_med", -1) < 0 or row.get("resB_med", -1) < 0:
            continue
        row["d_res"] = row["resB_med"] - row["resA_med"]
        row["d_nn"] = row["nnB_med"] - row["nnA_med"]
        rows.append(row)
    return rows


def imu_series(entries):
    ts, wz, wn = [], [], []
    for msg, _ in entries:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g = msg.angular_velocity
        ts.append(t)
        wz.append(g.z)
        wn.append(math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z))
    return np.asarray(ts), np.asarray(wz), np.asarray(wn)


def nearest(ts, vals, t):
    if len(ts) == 0:
        return 0.0
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return float(vals[0])
    if i >= len(ts):
        return float(vals[-1])
    return float(vals[i - 1] if abs(ts[i - 1] - t) < abs(ts[i] - t) else vals[i])


def odom_speed(entries):
    """Finite-diff speed from /Odometry pose (twist often zero)."""
    ts, spd = [], []
    prev = None
    for msg, _ in entries:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        if prev is not None:
            dt = t - prev[0]
            if dt > 1e-3:
                dx = p.x - prev[1]
                dy = p.y - prev[2]
                dz = p.z - prev[3]
                ts.append(t)
                spd.append(math.sqrt(dx * dx + dy * dy + dz * dz) / dt)
        prev = (t, p.x, p.y, p.z)
    return np.asarray(ts), np.asarray(spd)


def classify(w_norm, wz, v, w_turn=0.25, v_move=0.08):
    turning = abs(w_norm) >= w_turn
    moving = v >= v_move
    if not turning and not moving and w_norm < 0.08:
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


def summarize(rows):
    if not rows:
        return None

    def med(k):
        return float(np.median([r[k] for r in rows]))

    def mean(k):
        return float(np.mean([r[k] for r in rows]))

    return {
        "n": len(rows),
        "dt_ms_med": med("dt_ms"),
        "rel_yaw_deg_med": med("rel_yaw_deg"),
        "rel_trans_m_med": med("rel_trans_m"),
        "resA_med": med("resA_med"),
        "resB_med": med("resB_med"),
        "d_res_med": med("d_res"),
        "nnA_med": med("nnA_med"),
        "nnB_med": med("nnB_med"),
        "d_nn_med": med("d_nn"),
        "passA_med": med("passA_frac"),
        "passB_med": med("passB_frac"),
        "w_mean": mean("w_norm"),
        "frac_B_better_res": float(np.mean([r["d_res"] < -0.002 for r in rows])),
        "frac_A_better_res": float(np.mean([r["d_res"] > 0.002 for r in rows])),
        "b_pre_ms_med": med("b_pre_ms"),
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
    print(f"[rec] {sec:.0f}s → {out_dir}")
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=Path, default=None)
    ap.add_argument("--record-sec", type=float, default=0.0)
    ap.add_argument("--out", type=Path, default=Path("/tmp/odin_xtform_out"))
    ap.add_argument("--label", type=str, default="mixed")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    bag = args.bag
    if args.record_sec > 0:
        bag = args.out / "bag"
        record_bag(bag, args.record_sec)
    if bag is None:
        raise SystemExit("Provide --bag or --record-sec")
    if not (bag / "metadata.yaml").exists():
        cands = list(bag.rglob("metadata.yaml"))
        if not cands:
            raise SystemExit(f"No bag under {bag}")
        bag = cands[0].parent

    data = read_bag(bag)
    rows = parse_metrics(data["/odin_xtform_metrics"])
    print(f"[load] metrics frames={len(rows)} imu={len(data['/livox/imu'])}")

    imu_ts, imu_wz, imu_wn = imu_series(data["/livox/imu"])
    od_ts, od_spd = odom_speed(data["/Odometry"])

    for r in rows:
        t = 0.5 * (r["t_odin"] + r["t_mid"])
        # metrics already has w_norm from gate; refine with IMU wz / odom v
        r["wz"] = nearest(imu_ts, imu_wz, t)
        r["w_imu"] = nearest(imu_ts, imu_wn, t)
        r["v"] = nearest(od_ts, od_spd, t) if len(od_ts) else 0.0
        r["seg"] = classify(r["w_imu"], r["wz"], r["v"])

    by_seg = {}
    for seg in ("still", "turn_left", "turn_right", "translate", "combined", "other"):
        by_seg[seg] = summarize([r for r in rows if r["seg"] == seg])

    # continuous motion chunks: merge adjacent non-still
    chunks = []
    cur = []
    for r in sorted(rows, key=lambda x: x["t_mid"]):
        if r["seg"] == "still":
            if cur:
                chunks.append(cur)
                cur = []
        else:
            cur.append(r)
    if cur:
        chunks.append(cur)
    chunk_stats = []
    for i, ch in enumerate(chunks):
        if len(ch) < 3:
            continue
        s = summarize(ch)
        s["chunk_id"] = i
        s["seg_modes"] = sorted({r["seg"] for r in ch})
        s["duration_s"] = float(ch[-1]["t_mid"] - ch[0]["t_mid"])
        chunk_stats.append(s)

    all_s = summarize(rows)
    motion_s = summarize([r for r in rows if r["seg"] != "still"])
    still_s = by_seg.get("still")

    # hist coverage proxy: all rows here already passed C++ hist gate
    verdict = "inconclusive"
    if motion_s and still_s:
        if (
            motion_s["d_res_med"] < -0.005
            and motion_s["frac_B_better_res"] >= 0.55
            and still_s["d_res_med"] > -0.01
        ):
            verdict = "B_helps_motion_still_ok — candidate for Stage2-only formal switch"
        elif motion_s["d_res_med"] > 0.005 and motion_s["frac_A_better_res"] >= 0.55:
            verdict = "A_better_or_B_hurts — do not switch Stage2"
        elif abs(motion_s["d_res_med"]) <= 0.005:
            verdict = "no_clear_gain — keep formal path; investigate clock/extrinsic"
    elif motion_s:
        if motion_s["d_res_med"] < -0.005 and motion_s["frac_B_better_res"] >= 0.55:
            verdict = "B_helps_motion (no still baseline)"
        else:
            verdict = "inconclusive_motion"

    summary = {
        "label": args.label,
        "bag": str(bag),
        "all": all_s,
        "motion": motion_s,
        "by_segment": by_seg,
        "chunks": chunk_stats,
        "verdict": verdict,
        "notes": {
            "A": "Odin_end L1 × Mid_end pose (Stage2-style)",
            "B": "inv(T_mid)*T_odin*p then × Mid_end (cross-time SE3)",
            "t_odin_end": "H2+span (not b_pair)",
            "negative_d_res": "B better (lower plane residual)",
        },
    }
    out_json = args.out / "summary.json"
    out_json.write_text(json.dumps(summary, indent=2))
    # also dump per-frame csv
    csv_path = args.out / "frames.csv"
    with csv_path.open("w") as f:
        f.write(
            "t_mid,seg,dt_ms,rel_yaw_deg,rel_trans_m,resA,resB,d_res,nnA,nnB,d_nn,passA,passB,w,v,wz\n"
        )
        for r in rows:
            f.write(
                f"{r['t_mid']},{r['seg']},{r['dt_ms']},{r['rel_yaw_deg']},{r['rel_trans_m']},"
                f"{r['resA_med']},{r['resB_med']},{r['d_res']},{r['nnA_med']},{r['nnB_med']},{r['d_nn']},"
                f"{r['passA_frac']},{r['passB_frac']},{r['w_imu']},{r['v']},{r['wz']}\n"
            )
    print(json.dumps(summary, indent=2))
    print(f"[wrote] {out_json}")


if __name__ == "__main__":
    main()
