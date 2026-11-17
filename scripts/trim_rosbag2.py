#!/usr/bin/env python3
"""Trim start/end of a ROS 2 bag (sqlite3 / mcap).

Example (drop first 5s and last 5s):
  python3 /home/ll/3dnavigation/scripts/trim_rosbag2.py \\
    /home/ll/pv_lio_dual_ws/bags/corridor_YYYYMMDD_HHMMSS \\
    --head-sec 5 --tail-sec 5
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _open_reader(uri: str):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    reader = SequentialReader()
    for storage_id in ("sqlite3", "mcap"):
        try:
            reader.open(
                StorageOptions(uri=uri, storage_id=storage_id),
                ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
            return reader, storage_id
        except Exception:
            continue
    raise RuntimeError(f"Cannot open bag: {uri}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Trim head/tail seconds from a ros2 bag")
    ap.add_argument("bag", type=str, help="Input bag directory")
    ap.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        help="Output bag dir (default: <bag>_trim_hHtT)",
    )
    ap.add_argument("--head-sec", type=float, default=5.0, help="Seconds to drop at start")
    ap.add_argument("--tail-sec", type=float, default=5.0, help="Seconds to drop at end")
    args = ap.parse_args()

    bag = str(Path(args.bag).resolve())
    head_ns = int(max(0.0, args.head_sec) * 1e9)
    tail_ns = int(max(0.0, args.tail_sec) * 1e9)

    reader, storage_id = _open_reader(bag)
    meta = reader.get_metadata()
    t0 = int(meta.starting_time.nanoseconds)
    duration_ns = int(meta.duration.nanoseconds)
    t1 = t0 + duration_ns
    keep0 = t0 + head_ns
    keep1 = t1 - tail_ns
    if keep1 <= keep0:
        print(
            f"ERROR: keep window empty "
            f"(duration={duration_ns/1e9:.2f}s, head={args.head_sec}, tail={args.tail_sec})",
            file=sys.stderr,
        )
        return 2

    out = args.output or f"{bag}_trim_h{args.head_sec:g}t{args.tail_sec:g}"
    out_path = Path(out)
    if out_path.exists():
        print(f"ERROR: output exists: {out}", file=sys.stderr)
        return 2

    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(out_path), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    for topic in reader.get_all_topics_and_types():
        writer.create_topic(
            TopicMetadata(
                name=topic.name,
                type=topic.type,
                serialization_format=topic.serialization_format,
            )
        )

    kept = 0
    skipped = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if ts < keep0 or ts > keep1:
            skipped += 1
            continue
        writer.write(topic, data, ts)
        kept += 1

    print(
        f"OK in={bag}\n"
        f"   out={out_path}\n"
        f"   duration_in={duration_ns/1e9:.2f}s  keep=[{(keep0-t0)/1e9:.2f}, {(keep1-t0)/1e9:.2f}]s\n"
        f"   msgs kept={kept} skipped={skipped} storage={storage_id}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
