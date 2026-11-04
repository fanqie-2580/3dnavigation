# Mid360 + Odin1 多雷达融合（一路位姿/地图）

工作区：`/home/ll/fast_lio_multi_ws`（包名 `fast_lio_multi`，基于 [FAST_LIO_MULTI_ROS2](https://github.com/Draxran/FAST_LIO_MULTI_ROS2)）

把 **Livox Mid360** 与 **Odin1 dToF 激光雷达**紧耦合成一路里程计与点云，再 remap 到现有导航话题。

## 时间戳不一致：开源参考 & 本仓库做法

Odin TCP 戳是设备 uptime，Mid/LIO 是 ROS 时间——**硬 restamp 再乘 Mid odom 会在转弯时糊成扇形**，这不是外参 yaw 问题，是跨时钟融合问题。

| 项目 | 链接 | 做法 |
|------|------|------|
| FAST-LIO-MULTI | https://github.com/engcang/FAST_LIO_MULTI | bundle / **async** / adaptive；副雷达按自己的到达更新 |
| MA-LIO | https://github.com/minwoo0611/MA-LIO | ApproximateTime + **连续时间 B 样条**，点级时间不确定度 |
| M-LOAM | https://github.com/gogojjh/M-LOAM | 多雷达在线外参 + 建图 |
| SLICT | https://github.com/brytsknguyen/slict | 连续时间、多输入 LIO |
| LI-Init | https://github.com/hku-mars/LiDAR_IMU_Init | 标定 **time offset** + 外参 |
| MILIOM (论文) | https://ar5iv.labs.arxiv.org/html/2104.11888 | 主雷达定拍，副雷达靠 **IMU 传播 deskew** |

本仓库橙色累积默认 `odin_mode:=imu_prop`：

1. **忽略** Odin header 时间戳  
2. 用 **接收时刻**（Mid/ROS 钟）  
3. 取最近 Mid `/Odometry`，用 `/livox/imu` 陀螺积分补到接收时刻  
4. 再乘 L2 外参做世界系 voxel 累积  

备选：`odin_own`（跟 `odin_odom_mapper` 一样用 `/odin1/odometry` 自己累，再锁一次到 Mid `map`）。

## 话题约定

| 角色 | 输入 | 说明 |
|------|------|------|
| 主雷达 | `/livox/lidar` | Livox CustomMsg |
| 主 IMU | `/livox/imu` | Mid360 IMU |
| 副雷达 | `/odin1/cloud_raw` → `/odin1/cloud_raw_ds` | Odin 原始雷达；桥接体素降采样 |

| 融合输出 | 导航别名（nav bridge） |
|----------|------------------------|
| `/Odometry` | `/baselink2map` |
| `/cloud_fused`（Mid+Odin 当前帧合在一起） | `/cloud_registered_1` |
| `/viz/accum_fused` | Mid+Odin 累积融合图（RViz 主看这个） |
| `/cloud_registered` / `/cloud_odin_registered` | 分色单雷达当前帧 |
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
- `extrinsic_T/R_L2_wrt_L1`：ICP 俯仰粗估（~14°）；map 占用对齐易被 FOV 误导，以 body 系残差为准  
- `/viz/accum_odin` 默认吃 `/cloud_odin_registered`（与 Mid 同为 LIO map 云；禁止 raw×10Hz odom，否则 30Hz 同戳扇形重影）  
- `/odin1/cloud_raw_ds`：无 odom 时先用 now() 启动；有 odom 后 `one_cloud_per_odom` 每戳一帧

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
  1. Odin `/odin1/cloud_raw` 已接近重力水平；Mid 本体在重力对齐后的 `map` 里仍有 ~9° 俯仰（安装角）。  
  2. Mid 点云用完整 `R_wi`（含 pitch）投到 `map`，所以墙是正的。  
  3. Odin 必须走 `extrinsic_L2_gravity_frame: true`：只用 Mid 的 **yaw + 平移**；**禁止**把 Mid 安装俯仰编进 Odin。Odin 相对墙面若仍微下倾，只加很小的 `Ry` tip-up（当前 ~1.5°），不要再用 ±8°。  
  4. RViz 坐标系：`dual_lidar_frames` 发布 `map→mid360_lidar`、`map→odin1_lidar`（TF 开 Show Names）。  
- **Odin 累积跟着转圈转（已修）**：实测 `/cloud_odin_registered` stamp 比 `/Odometry` 落后约 **5s**，按 stamp 查位姿 = 旧 yaw × 新 body 墙 → 橙色扇形；Mid 用当前扫描 deskew 所以仍正。现改为 Odin **只缓冲最新 body 云**，在 Mid IEKF 更新后用**同一帧** `state_point` 做 yaw-only 投影再发布。  
- **点云跟着机器人转（RViz）**：对当前扫描开长 Decay 也会像地图在转；主看 `/viz/accum_*` / `/Laser_map`，Decay=0。  
- **双色融合累积**：青 `/viz/accum_mid360`←`/cloud_registered`，橙 `/viz/accum_odin`←`/cloud_odin_registered`。Odin 用与 Mid 相同的 LIO `state_point` 进 `map`（不单独用 raw+/Odometry），避免走动后橙图旋转漂移。
- **累积地图**：`/Laser_map` 为 ikdtree 世界系地图，机器人转时地图应钉在 Grid 上，只有 Odom 箭头转。  
- 推荐标定：卷尺/CAD 安装量测，或静止下 Mid360↔Odin 多视角手眼/ICP（约束俯仰/滚转），再写回 `extrinsic_*_L2_wrt_L1`  
- **当前 Go2 安装（目测/卷尺）**：Odin 在前上方、Mid 在后下方；`T_L2_wrt_L1 ≈ [0.150, 0, +0.035]` m（前 150 mm、高 35 mm）。旧值 `z=+0.208` 与实物不符。  
- Odin 极密：默认 `odin_voxel:=0.08` + `point_filter_num2: 8`  
- 无硬时间同步时避免急转首测  
- 与旧 `open3d_loc` 单雷达链路并存；联调时不要同时发两套 `/baselink2map`
