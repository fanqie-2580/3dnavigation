#!/usr/bin/env python3
"""Offline ground/obstacle segmentation for Odin (or other) global PCD maps.

Produces dddmr-compatible XYZ PCDs:
  segmentation_ground_map.pcd
  segmentation_obstacle_map.pcd
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d


def save_xyz_pcd(path: Path, pts: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
    with open(path, "w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(pts)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(pts)}\n")
        f.write("DATA binary\n")
    with open(path, "ab") as f:
        f.write(pts.astype("<f4").tobytes())


def plane_angle_deg(normal: np.ndarray) -> float:
    n = normal / (np.linalg.norm(normal) + 1e-12)
    # angle between normal and +Z
    c = abs(float(n[2]))
    c = min(1.0, max(0.0, c))
    return float(np.degrees(np.arccos(c)))


def segment(
    cloud: o3d.geometry.PointCloud,
    voxel: float,
    distance: float,
    max_angle_deg: float,
    max_height: float,
    min_obstacle_height: float,
    ransac_n: int,
    num_iterations: int,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud]:
    if voxel > 0:
        cloud = cloud.voxel_down_sample(voxel)

    pts = np.asarray(cloud.points)
    if pts.size == 0:
        raise RuntimeError("empty point cloud after downsample")

    # Fit ground only on the lower band. A loose max_height (e.g. p5+3m) still
    # keeps the ceiling, and RANSAC then prefers the larger ceiling plane.
    z0 = float(np.percentile(pts[:, 2], 5))
    z_med = float(np.median(pts[:, 2]))
    ground_band = max_height if max_height > 0 else 1.2
    # Never let the fit band reach above the cloud median when floor/ceiling coexist.
    z_fit_max = min(z0 + ground_band, z_med - 0.05) if z_med > z0 + 0.3 else (z0 + ground_band)
    fit_idx = np.where(pts[:, 2] <= z_fit_max)[0]
    if len(fit_idx) < max(100, int(0.02 * len(pts))):
        # Fallback: lowest 35% by z
        fit_idx = np.argsort(pts[:, 2])[: max(100, int(0.35 * len(pts)))]
    fit_cloud = cloud.select_by_index(fit_idx.tolist())
    print(
        f"[seg] ground-fit band z<= {z_fit_max:.2f} "
        f"(p5={z0:.2f}, median={z_med:.2f}, n_fit={len(fit_idx)}/{len(pts)})"
    )

    plane_model, fit_inliers = fit_cloud.segment_plane(
        distance_threshold=distance,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )
    a, b, c, d = plane_model
    angle = plane_angle_deg(np.array([a, b, c], dtype=np.float64))
    if angle > max_angle_deg:
        raise RuntimeError(
            f"dominant plane tilt {angle:.1f} deg > max_angle {max_angle_deg:.1f} deg; "
            "map may be badly tilted or indoor walls dominate — try larger distance or check map leveling"
        )
    if len(fit_inliers) < max(100, int(0.05 * len(fit_idx))):
        raise RuntimeError(
            f"too few ground inliers ({len(fit_inliers)}/{len(fit_idx)}); increase distance_threshold"
        )

    # Classify the full cloud by distance to the fitted (low) plane.
    denom = np.sqrt(a * a + b * b + c * c) + 1e-12
    signed = (a * pts[:, 0] + b * pts[:, 1] + c * pts[:, 2] + d) / denom
    ground_idx = np.where(np.abs(signed) <= distance)[0]
    # Guard: reject a "ground" that sits in the upper half of the map.
    ground_z_mean = float(np.mean(pts[ground_idx, 2])) if len(ground_idx) else 1e9
    if ground_z_mean > z_med:
        raise RuntimeError(
            f"fitted plane mean z={ground_z_mean:.2f} is above map median {z_med:.2f} "
            "(looks like ceiling); tighten --max-height / check map leveling"
        )
    print(f"[seg] plane angle={angle:.1f}deg mean_ground_z={ground_z_mean:.2f}")

    ground = cloud.select_by_index(ground_idx.tolist())
    obstacle = cloud.select_by_index(ground_idx.tolist(), invert=True)

    # Distance of each obstacle point to the fitted plane (signed)
    if len(obstacle.points) > 0 and min_obstacle_height > 0:
        op = np.asarray(obstacle.points)
        denom = np.sqrt(a * a + b * b + c * c) + 1e-12
        dist = (a * op[:, 0] + b * op[:, 1] + c * op[:, 2] + d) / denom
        # Keep points clearly above (or below) the ground plane
        mask = np.abs(dist) >= min_obstacle_height
        obstacle = obstacle.select_by_index(np.where(mask)[0].tolist())

    return ground, obstacle


def main() -> int:
    ap = argparse.ArgumentParser(description="Segment Odin global PCD into ground/obstacle for dddmr")
    ap.add_argument(
        "--input",
        default="/home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd",
        help="input global map PCD",
    )
    ap.add_argument(
        "--out-dir",
        default="/home/ll/ws_odin_pvlio/maps",
        help="output directory for segmentation_*.pcd",
    )
    ap.add_argument("--voxel", type=float, default=0.05, help="voxel downsample size (m), 0=off")
    ap.add_argument("--distance", type=float, default=0.08, help="RANSAC plane distance threshold (m)")
    ap.add_argument("--max-angle", type=float, default=25.0, help="max ground plane tilt vs Z (deg)")
    ap.add_argument(
        "--max-height",
        type=float,
        default=3.0,
        help="drop points higher than p5(z)+max_height before fit (m); 0=off",
    )
    ap.add_argument(
        "--min-obstacle-height",
        type=float,
        default=0.12,
        help="obstacle points closer than this to plane are dropped (m)",
    )
    ap.add_argument("--ransac-n", type=int, default=3)
    ap.add_argument("--iterations", type=int, default=2000)
    args = ap.parse_args()

    inp = Path(args.input)
    if not inp.is_file():
        print(f"[error] input not found: {inp}", file=sys.stderr)
        return 1

    cloud = o3d.io.read_point_cloud(str(inp))
    n0 = len(cloud.points)
    if n0 == 0:
        print(f"[error] empty cloud: {inp}", file=sys.stderr)
        return 1
    print(f"[seg] loaded {n0} points from {inp}")

    ground, obstacle = segment(
        cloud,
        voxel=args.voxel,
        distance=args.distance,
        max_angle_deg=args.max_angle,
        max_height=args.max_height,
        min_obstacle_height=args.min_obstacle_height,
        ransac_n=args.ransac_n,
        num_iterations=args.iterations,
    )

    out_dir = Path(args.out_dir)
    g_path = out_dir / "segmentation_ground_map.pcd"
    o_path = out_dir / "segmentation_obstacle_map.pcd"
    save_xyz_pcd(g_path, np.asarray(ground.points))
    save_xyz_pcd(o_path, np.asarray(obstacle.points))
    print(f"[seg] ground   {len(ground.points)} -> {g_path}")
    print(f"[seg] obstacle {len(obstacle.points)} -> {o_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as e:
        print(f"[error] {e}", file=sys.stderr)
        raise SystemExit(1)
