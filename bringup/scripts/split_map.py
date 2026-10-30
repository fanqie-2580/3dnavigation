#!/usr/bin/env python3
"""Split a FAST-LIO PCD into ground / obstacle layers for DDDMR global planner."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input_pcd", type=Path, help="Input map PCD/PLY")
    p.add_argument("--ground-out", type=Path, required=True)
    p.add_argument("--obstacle-out", type=Path, required=True)
    p.add_argument("--loc-out", type=Path, default=None, help="Optional copy for open3d_loc")
    p.add_argument("--voxel", type=float, default=0.1, help="Voxel size before split")
    p.add_argument(
        "--ground-z-max",
        type=float,
        default=0.35,
        help="Points with z <= percentile_low + this offset are ground",
    )
    p.add_argument(
        "--ground-percentile",
        type=float,
        default=5.0,
        help="Low-z percentile used as approximate ground reference",
    )
    p.add_argument(
        "--obstacle-z-min",
        type=float,
        default=0.15,
        help="Obstacle points must be at least this above ground reference",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not args.input_pcd.is_file():
        raise SystemExit(f"input map not found: {args.input_pcd}")

    cloud = o3d.io.read_point_cloud(str(args.input_pcd))
    if cloud.is_empty():
        raise SystemExit(f"empty point cloud: {args.input_pcd}")

    if args.voxel > 0:
        cloud = cloud.voxel_down_sample(args.voxel)

    pts = np.asarray(cloud.points)
    z0 = float(np.percentile(pts[:, 2], args.ground_percentile))
    ground_mask = pts[:, 2] <= (z0 + args.ground_z_max)
    obstacle_mask = pts[:, 2] >= (z0 + args.obstacle_z_min)

    ground = o3d.geometry.PointCloud()
    ground.points = o3d.utility.Vector3dVector(pts[ground_mask])
    obstacle = o3d.geometry.PointCloud()
    obstacle.points = o3d.utility.Vector3dVector(pts[obstacle_mask])

    if ground.is_empty() or obstacle.is_empty():
        raise SystemExit(
            f"split failed (ground={len(ground.points)}, obstacle={len(obstacle.points)}). "
            "Adjust --ground-z-max / --obstacle-z-min."
        )

    args.ground_out.parent.mkdir(parents=True, exist_ok=True)
    args.obstacle_out.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(args.ground_out), ground, write_ascii=False)
    o3d.io.write_point_cloud(str(args.obstacle_out), obstacle, write_ascii=False)

    if args.loc_out is not None:
        args.loc_out.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_point_cloud(str(args.loc_out), cloud, write_ascii=False)

    print(f"input points: {len(pts)}")
    print(f"ground ref z≈{z0:.3f}")
    print(f"ground: {len(ground.points)} -> {args.ground_out}")
    print(f"obstacle: {len(obstacle.points)} -> {args.obstacle_out}")
    if args.loc_out is not None:
        print(f"loc map: {len(cloud.points)} -> {args.loc_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
