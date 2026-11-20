# fastlio_multi_pvlio

**Independent** of `pv_lio_dual_ws` / `FAST_LIO_MULTI_ROS2_mid_odin`.  
Goal: **official FAST-LIO-MULTI Bundle tight couple** for Mid360 + Odin1, with **PV-LIO voxel mapping** instead of FAST-LIO ikd-Tree.

## Layout

```text
fastlio_multi_pvlio_ws/
  src/fastlio_multi_pvlio/     # runnable ROS 2 package
  reference/FAST_LIO_MULTI_ROS2_upstream/   # official MULTI source (read-only)
  docs/reference_fast_lio_multi_mid360_odin1.yaml  # mid_odin MULTI config snapshot
  maps/                        # /save_map output
```

## Coupling vs mapping

| Layer | Source |
|-------|--------|
| Dual-lidar Bundle sync + merge + one IEKF | FAST-LIO-MULTI Bundle semantics |
| L2→L1 extrinsic, Mid IMU deskew of both clouds | MULTI-style (`UndistortPclMultiLiDAR`) |
| Map / plane residual / Global densify | **PV-LIO voxel map** |
| Mid360 + Odin1 message types | Adapted (`lidar_type2=6`) |

**Not** using dual’s S1/S2 gates, residual_z bands, or Odin pose-replace factors.

## Build

```bash
cd /home/ll/fastlio_multi_pvlio_ws
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/local_setup.bash
colcon build --symlink-install --packages-select fastlio_multi_pvlio
```

## Run (multifloor bag)

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/local_setup.bash
source /home/ll/fastlio_multi_pvlio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DISPLAY=:1

ros2 launch fastlio_multi_pvlio mapping_mid360_odin1_multi.launch.py \
  rviz:=true use_sim_time:=true

# other terminal
ros2 bag play /home/ll/pv_lio_dual_ws/bags/multifloor_20261120_1031_1037 --clock -r 1.0 \
  --remap /tf:=/tf_bag_unused \
  --topics /livox/lidar /livox/imu /odin1/cloud_raw /odin1/imu
```

Save map:

```bash
ros2 service call /save_map std_srvs/srv/Trigger {}
```

## Relation to other trees

| Tree | Role |
|------|------|
| `pv_lio_dual_ws` | Heterogeneous dual (S1/S2, residual_z, densify gates) — **unchanged** |
| `FAST_LIO_MULTI_ROS2_mid_odin` | Official MULTI + Mid/Odin on **ikd-Tree** — reference only |
| `pv_lio_multi_ws` | Earlier minimal Bundle-on-PV port; this ws is the dedicated rename/isolation |
| **this ws** | Bundle tight couple + PV voxel, Mid+Odin |
