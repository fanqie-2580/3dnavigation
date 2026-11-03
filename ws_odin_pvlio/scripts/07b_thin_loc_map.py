#!/usr/bin/env python3
"""Thin a localization PCD: keep near-ground/walls dense, sparsify or drop ceiling.

Useful when an Odin1 map is dense with ceiling points that hurt Mid360 matching
and inflate open3d_loc memory.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--voxel", type=float, default=0.05, help="global pre-downsample")
    ap.add_argument("--keep-below", type=float, default=1.8, help="dense band above p5(z)")
    ap.add_argument("--ceiling-voxel", type=float, default=0.40)
    ap.add_argument("--drop-above", type=float, default=2.4)
    args = ap.parse_args()

    cloud = o3d.io.read_point_cloud(args.input)
    n0 = len(cloud.points)
    if n0 == 0:
        raise SystemExit(f"empty: {args.input}")
    if args.voxel > 0:
        cloud = cloud.voxel_down_sample(args.voxel)

    pts = np.asarray(cloud.points)
    z0 = float(np.percentile(pts[:, 2], 5))
    z = pts[:, 2]
    low = z <= (z0 + args.keep_below)
    mid = (~low) & (z <= (z0 + args.drop_above))

    parts = []
    if np.any(low):
        parts.append(cloud.select_by_index(np.where(low)[0].tolist()))
    if np.any(mid):
        high = cloud.select_by_index(np.where(mid)[0].tolist())
        if args.ceiling_voxel > 0:
            high = high.voxel_down_sample(args.ceiling_voxel)
        parts.append(high)

    out = parts[0]
    for p in parts[1:]:
        out += p

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(args.output, out, write_ascii=False)
    pts2 = np.asarray(out.points)
    print(
        f"[thin] {n0} -> {len(pts2)}  z0(p5)={z0:.3f} "
        f"keep<{z0+args.keep_below:.2f} thin<{z0+args.drop_above:.2f} drop above"
    )
    print(f"[thin] wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
