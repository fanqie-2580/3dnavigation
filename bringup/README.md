# Go2 建图导航 Bringup

用本仓库三套栈联调：

| 阶段 | 组件 | 运行位置 |
|------|------|----------|
| 建图 | FAST-LIO (`mapping.launch.py`) | 宿主机 |
| 定位 | FAST-LIO + open3d_loc (`localization_3d_go2`) | 宿主机 |
| 全局规划 | DDDMR (`dddmr_global_ego_docker`) | Docker `dddmr_dev` |
| 局部规划 | EGO (`ct_lio_fastlio_reference_tracking`) | 宿主机 |

默认机器人：**Unitree Go2 + Livox MID360**。

## 数据流

```text
Livox (/livox/lidar, /livox/imu)
        │
        ▼
   FAST-LIO ──► /cloud_registered_1, /Odometry
        │
        ▼
  open3d_loc ──► /baselink2map  (map ← base_link)
        │
        ├──────────────► DDDMR global ──► /global_path
        │                      ▲
        │                      │ mapcloud / mapground (离线 PCD)
        │
        └──────────────► EGO local ──► /cmd_vel
                           ▲
                           └── /global_path + /cloud_registered_1
```

关键点：

- `map` / `odom` / `camera_init` / `body` / `base_link`
- `/baselink2map`、`/cloud_registered_1`、`/livox/lidar_pointcloud2`、`/global_path`、`/cmd_vel`

## 准备

1. 编译（宿主机）

```bash
cd /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble
source /opt/ros/humble/setup.bash && colcon build --symlink-install

cd /home/ll/ego-planner-swarm-ros2_version
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

2. 编译 DDDMR（Docker）

```bash
# 若没有容器，先按 AGENTS.md 启动 dddmr_dev
docker exec -it dddmr_dev bash -lc '
  cd /root/dddmr_ws && source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install
'
```

3. 统一 RMW（宿主机与 Docker 一致）

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

4. 启动 Livox 驱动（另开终端，按你现场的 launch）

## 建图流程

**终端 A** — 建图：

```bash
/home/ll/3dnavigation/bringup/scripts/01_mapping.sh
```

遥控机器人扫完场地后，**终端 B** — 存图并切分：

```bash
/home/ll/3dnavigation/bringup/scripts/02_save_map.sh
```

会生成：

- `bringup/maps/fastlio_map.pcd` / `loc_map.pcd`
- `bringup/maps/ground_map.pcd` / `obstacle_map.pcd`
- 并同步到 FAST_LIO `data/` 与 DDDMR `global_planner/data/`

地面/障碍切分是按高度粗分，复杂场景请用 CloudCompare 再精修后覆盖上述 PCD。

## 导航流程（建图完成后）

开 **4 个终端**（Livox 驱动另算）：

```bash
# 1) 定位（宿主机）
/home/ll/3dnavigation/bringup/scripts/03_localization.sh
# RViz 里用 2D Pose Estimate 给初值

# 2) 全局规划（Docker）
/home/ll/3dnavigation/bringup/scripts/04_global_planner.sh
# RViz 里 Publish Point / Goal Pose → 出 /global_path

# 3) 局部规划（宿主机）
/home/ll/3dnavigation/bringup/scripts/05_local_planner.sh
# 跟踪参考路径并发布 /cmd_vel
```

自检：

```bash
/home/ll/3dnavigation/bringup/scripts/00_check.sh
```

## 常用覆盖参数

```bash
export MAP_LOC_PCD=/path/to/your_map.pcd
export USE_SIM_TIME=true          # 播 bag 时
export DDDMR_CONTAINER=dddmr_dev
export CMD_VEL_TOPIC=/cmd_vel
```

改完后重新 `source bringup/env.sh` 或直接带环境变量跑脚本。

## 已有示例地图

若暂未自建图，可用仓库自带：

```bash
export MAP_LOC_PCD=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd
# DDDMR 默认可直接用 global_planner/data 下的 segmentation_*_map.pcd
```

## 故障排查

| 现象 | 检查 |
|------|------|
| Docker 与宿主机互相看不到 topic | `RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID` 是否一致；容器是否 `--network host` |
| EGO 不动 | `/baselink2map`、`/global_path`、`/cloud_registered_1` 是否在更新 |
| 全局规划失败 | obstacle/ground PCD 是否非空；初值定位是否成功 |
| `/map_save` 失败 | 建图节点是否在跑，且 `pcd_save.pcd_save_en: true` |
