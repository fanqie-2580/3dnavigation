# Mid360 + Odin1 多雷达融合（一路位姿/地图）

工作区：`/home/ll/fast_lio_multi_ws`（包名 `fast_lio_multi`，基于 [FAST_LIO_MULTI_ROS2](https://github.com/Draxran/FAST_LIO_MULTI_ROS2)）

把 **Livox Mid360** 与 **Odin1 dToF 激光雷达**紧耦合成一路里程计与点云，再 remap 到现有导航话题。

## 话题约定

| 角色 | 输入 | 说明 |
|------|------|------|
| 主雷达 | `/livox/lidar` | Livox CustomMsg |
| 主 IMU | `/livox/imu` | Mid360 IMU |
| 副雷达 | `/odin1/cloud_raw` → `/odin1/cloud_raw_ds` | Odin 原始雷达；桥接体素降采样 |

| 融合输出 | 导航别名（nav bridge） |
|----------|------------------------|
| `/Odometry` | `/baselink2map` |
| `/cloud_registered` | `/cloud_registered_1` |
| `/cloud_registered_body` | `/cloud_registered_body_1` |

**禁止**把 `/odin1/cloud_slam` 喂进 MULTI（已在 Odin 内置位姿下，会双重里程计）。

## 1. Odin 必须 mode 0

```bash
bash /home/ll/ws_odin_pvlio/scripts/12_set_odin_mode0_for_fusion.sh
# 坞上 control_command.yaml 也要 custom_map_mode: 0，然后重启 Odin 驱动/桥
```

确认：

```bash
grep custom_map_mode /home/ll/ws_odin_pvlio/src/odin_ros_driver/config/control_command.yaml
# -> custom_map_mode: 0
```

## 2. 编译（已做过一次可跳过）

```bash
cd /home/ll/fast_lio_multi_ws
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
colcon build --symlink-install --packages-select fast_lio_multi
```

## 3. 启动顺序

每个终端先：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
```

1. Livox Mid360（若未开）  
2. Odin 桥/驱动（mode 0，有 `/odin1/cloud_raw`）  
3. 融合：

```bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/fast_lio_multi_ws/install/setup.bash

ros2 launch fast_lio_multi mid360_odin1_multi.launch.py \
  update_method:=adaptive \
  config:=mid360_odin1.yaml \
  odin_voxel:=0.08
```

后台：

```bash
nohup ros2 launch fast_lio_multi mid360_odin1_multi.launch.py \
  update_method:=adaptive \
  > /tmp/mid360_odin1_multi.log 2>&1 &
```

配置文件：[`config/mid360_odin1.yaml`](/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/config/mid360_odin1.yaml)

- `lidar_type: 1`（Mid360 CustomMsg）  
- `lidar_type2: 4`（Odin 通用 XYZ PointCloud2，已加 `XYZ_ONLY` 预处理）  
- `extrinsic_T_L2_wrt_L1`：**粗值单位阵**，上机后请按安装位标定再改  

## 4. 检查

```bash
ros2 node list | rg 'laserMapping_multi|odin_raw|multi_lio_nav'
ros2 topic hz /Odometry
ros2 topic hz /baselink2map
ros2 topic hz /cloud_registered_1
ros2 topic echo /Odometry --once
```

正常日志应含：`Multi LiDAR on!`、`lidar_type[0]: 1, lidar_type[1]: 4`。

无传感器时节点仍可起，但不会出稳定 odom；有两路点云+IMU 后才验收位姿。

## 5. 接到 DDDMR / EGO

导航侧仍订：

- odom：`/baselink2map`  
- cloud：`/cloud_registered_1`  

与 [`go2_localization_to_navigation.md`](./go2_localization_to_navigation.md) 一致。融合地图可存 PCD 后跑：

```bash
bash /home/ll/ws_odin_pvlio/scripts/07_segment_map.sh <fused_map.pcd>
```

## 6. 关闭

```bash
pkill -f 'mid360_odin1_multi.launch.py' || true
pkill -9 -f 'laserMapping_adaptive|laserMapping_async|laserMapping_bundle|odin_raw_downsample|multi_lio_nav_bridge' || true
```

## 7. 风险与待标定

- Odin 极密：默认 `odin_voxel:=0.08` + `point_filter_num2: 8`  
- 外参 `extrinsic_T_L2_wrt_L1` / `R` 需实测，否则两路点云错位  
- 无硬时间同步时避免急转首测  
- 与旧 `open3d_loc` 单雷达链路并存；联调时不要同时发两套 `/baselink2map`
