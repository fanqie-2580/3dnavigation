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
- **`common.multi_lidar` 默认 `true`（Mid360 + Odin1）**：外参不准时 CloudMap 可能双墙/重影；可临时改 `false` 只跑 Mid360  
- `extrinsic_T_L2_wrt_L1`：已写入一次 ICP 粗估（约 T=`[0.16,0,0.04]`、俯仰 ~14°）；仍有残差时请实测标定再改 yaml

重新打开双雷达：

```bash
# 编辑 mid360_odin1.yaml → common.multi_lidar: true
# 确认 extrinsic_T/R_L2_wrt_L1 为实测值后重启 launch
```

## 4. 检查

```bash
ros2 node list | rg 'laserMapping_multi|odin_raw|multi_lio_nav'
ros2 param get /laserMapping_multi common.multi_lidar
ros2 topic hz /Odometry
ros2 topic hz /baselink2map
ros2 topic hz /cloud_registered_1
ros2 topic echo /Odometry --once
```

- `multi_lidar=false`：只跑 Mid360，CloudMap 应无双墙  
- `multi_lidar=true`：日志应含 `Multi LiDAR on!`、`lidar_type[0]: 1, lidar_type[1]: 4`

无传感器时节点仍可起，但不会出稳定 odom；有主雷达+IMU 后才验收位姿。

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

- **一动就杂乱（根因）**：外参不准的 Odin 若进入 IEKF/ikdtree，运动会把位姿和地图拉歪；旧版 `dual_lidar_color_viz` 还用 raw 点云 + 最新 `/Odometry` 做 Decay，未去畸变，走动必糊。  
- **当前策略**：`mapping.lidar2_update_en: false`（默认）— Mid360 独占位姿与地图；Odin 仍打 `intensity=200` 进 `/cloud_registered` 供分色，但不参与更新。外参标定好后再改 `true`。  
- **Mid360 / Odin 同时水平**：
  1. Odin `/odin1/cloud_raw` 已接近重力水平（~1°）；Mid360 IMU 相对重力 ~9°。  
  2. `IMU_init` 对 Mid360 做 `FromTwoVectors` 重力对齐（`map` 相对 Grid 摆正）。  
  3. IMU 初始化完成后，把 `LiDAR2_wrt_LiDAR1.R` 左乘 `R_wi^T`，抵消 Odin 上同一套重力旋转。yaml 墙面 ICP 外参仍需再标定。  
- **点云跟着机器人转**：RViz 对 `/cloud_registered` 开长 Decay 时，异步 Odin 帧会整帧顶掉世界系扫描，看起来像地图跟着 yaw 转。现 `lidar2_update_en=false` 时不再发布纯 Odin 帧；RViz 主看 **CloudMap**（`/Laser_map`，世界系固定），当前扫描 Decay=0。  
- **双色融合累积**：`lidar2_update_en:false` 时强制 bundle，Mid+Odin 同一 IMU 去畸变/位姿；青←`/cloud_registered`，橙←`/cloud_odin_registered`。
- **累积地图**：`/Laser_map` 为 ikdtree 世界系地图，机器人转时地图应钉在 Grid 上，只有 Odom 箭头转。  
- 推荐标定：卷尺/CAD 安装量测，或静止下 Mid360↔Odin 多视角手眼/ICP（约束俯仰/滚转），再写回 `extrinsic_*_L2_wrt_L1`  
- Odin 极密：默认 `odin_voxel:=0.08` + `point_filter_num2: 8`  
- 无硬时间同步时避免急转首测  
- 与旧 `open3d_loc` 单雷达链路并存；联调时不要同时发两套 `/baselink2map`
