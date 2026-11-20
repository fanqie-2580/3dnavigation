# Minimal FAST-LIO-MULTI Bundle on official PV-LIO

Package rename: `pv_lio` → `pv_lio_multi` (coexists with dual’s `pv_lio`).

## Intent

Smallest port so Mid360 + Odin1 behave like **official FAST-LIO-MULTI Bundle**:

1. Subscribe two lidars + one primary IMU (Mid `/livox/imu`)
2. Bundle sync: wait for L1, soft-wait L2, transform L2→L1, merge
3. `UndistortPclMultiLiDAR` with Mid IMU
4. Single IEKF update on merged cloud
5. Merged feats enter **PV-LIO voxel map** (unchanged mapping backend)

## Intentional diffs vs official PV-LIO

| Item | Change |
|------|--------|
| Package name | `pv_lio_multi` |
| `MeasureGroup` | `lidar2`, `lidar_beg_time2`, `lidar_end_time2` |
| `IMU_Processing` | `UndistortPclMultiLiDAR` + `Process(..., multi_lidar)` |
| Preprocess | `ODIN1=6`, `odin1_handler`, `min_confidence` |
| `voxelMapping` | L2 buffer/callback, Bundle `sync_packages`, L2→L1 extrinsic, soft-wait |
| Config/launch | `mid360_odin1_multi.yaml`, `mapping_mid360_odin1_multi.launch.py` |

**Not changed:** PV-LIO voxel octree / observation model / single IEKF iteration path (beyond feeding merged undistorted feats).

## Intentional diffs vs official FAST-LIO-MULTI Bundle

| Item | Official MULTI | This port |
|------|----------------|-----------|
| Mapping backend | ikd-Tree FAST-LIO | PV-LIO voxel map |
| Soft-wait for late L2 | No (blocks until both buffers non-empty) | Yes `~0.18s` then Mid-only |
| ODIN1 PointCloud2 | Not present | Required `odin1_handler` |
| Dual defenses | N/A | **Not** ported (no 2-stage IEKF, residual_z, odin_z_factor, quality gates, dual IMU deskew, speed clamp) |
| Empty L2 undistort | Official returns early on empty L2 | Soft Mid-only: undistort L1 and skip empty L2 |

## Known Mid+Odin risks (pure Bundle)

- Mid and Odin scan phases / stamp offsets differ; Bundle merges without dual’s time-window pairing / bias EMA → unpaired or phase-mismatched Odin points can pull Z.
- `UndistortPclMultiLiDAR` uses `min(beg1,beg2)` for IMU pose offsets but keeps each cloud’s own curvature origin (official MULTI behavior) → L2 deskew can be wrong when stamps diverge → **Z drift**.
- Both lidars share Mid IMU deskew only; Odin motion relative to Mid IMU is ignored beyond fixed extrinsic.
- Soft-wait Mid-only frames leave map Mid-biased until Odin catches up.

## Topics / types

- `/livox/lidar` — Livox `CustomMsg` (`lidar_type=1`)
- `/livox/imu` — primary IMU
- `/odin1/cloud_raw` — `sensor_msgs/PointCloud2` (`lidar_type2=6`)

## Build / smoke

```bash
cd /home/ll/pv_lio_multi_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pv_lio_multi

source install/setup.bash
ros2 launch pv_lio_multi mapping_mid360_odin1_multi.launch.py rviz:=false use_sim_time:=true
ros2 bag play /home/ll/pv_lio_dual_ws/bags/corridor_20261118_outbound_1014_1017 --clock --rate 1.0 --duration 30
```
