# Go2 + MID360：定位 → 导航命令（已验证）

> **推送到 GitHub**：改完代码/文档后按  
> [`push_to_github.md`](./push_to_github.md)  
>（上传目录 `/home/ll/3dnavigation_github_upload` → HTTPS + `/tmp/gh_token.txt` PAT → `fanqie-2580/3dnavigation` `main`）。

适用：**Unitree Go2 + Livox MID360**，ROS 2 Humble，`RMW=rmw_cyclonedds_cpp`。  
验证时间：2026-11-02，对照当时宿主机 / `dddmr_dev` 实际在跑的 launch 命令行。

启动顺序：

```text
Livox → 定位(open3d_loc) → DDDMR 全局(Docker) → EGO 局部 → go2_bridge
```

| 步骤 | 组件 | 位置 |
|------|------|------|
| 1 | `livox_ros_driver2` | 宿主机 |
| 2 | FAST-LIO + open3d_loc | 宿主机 |
| 3 | `dddmr_global_ego_docker` | Docker `dddmr_dev` |
| 4 | EGO `dddmr_reference_tracking` | 宿主机 |
| 5 | `go2_bridge_node` | 宿主机 |

路径：

```text
/home/ll/ws_livox
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble
/home/ll/dddmr_navigation          # Docker: /root/dddmr_ws
/home/ll/ego-planner-swarm-ros2_version
/home/ll/ros_ws                    # go2_cmd_vel_handler
/home/ll/unitree_ros2
```

当前验证用地图：

```text
定位地图: /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/odin1_odom_map.pcd
全局障碍: /home/ll/dddmr_navigation/src/dddmr_global_planner/data/segmentation_obstacle_map.pcd  # 已稀疏天花板
全局地面: /home/ll/dddmr_navigation/src/dddmr_global_planner/data/segmentation_ground_map.pcd
```

> launch 默认定位图是 `ct_lio_map.pcd`；当前联调实际用的是上面的 `odin1_odom_map.pcd`，请按本文件写的参数启动。  
> Odin1 建图 + Mid360 定位易对不齐、以及天花板稀疏化说明见  
> [`odin1_mid360_map_and_fusion.md`](./odin1_mid360_map_and_fusion.md)。  
> 可选瘦身定位图：`/home/ll/ws_odin_pvlio/maps/odin1_odom_map_thin_ceiling.pcd`。

---

## 0. 每个宿主机终端先执行

```bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
xhost +local:root >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
```

Docker 容器（若未起）：

```bash
docker start dddmr_dev 2>/dev/null || docker run -it \
  --name dddmr_dev \
  --network host --runtime nvidia --privileged \
  -v /dev:/dev \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /home/ll/dddmr_navigation:/root/dddmr_ws \
  -v /home/ll/dddmr_bags:/root/dddmr_bags \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  dddmr:l4t_r36 /bin/bash
```

---

## 1. Livox MID360

```bash
# 第 0 节环境已 export
source /home/ll/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

后台：

```bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox_mid360.log 2>&1 &
```

应有节点：`livox_lidar_publisher`，话题 `/livox/lidar`。

---

## 2. 定位（FAST-LIO + open3d_loc）

```bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash

MAP=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/odin1_odom_map.pcd

ros2 launch open3d_loc localization_3d_go2.launch.py \
  map_file:=$MAP \
  fastlio_config_file:=mid360.yaml \
  body_to_base_qy:=-0.1132032138 \
  body_to_base_qw:=0.9935718557 \
  localization_rviz:=true \
  publish_livox_pointcloud2:=true
```

后台：

```bash
nohup ros2 launch open3d_loc localization_3d_go2.launch.py \
  map_file:=$MAP \
  fastlio_config_file:=mid360.yaml \
  body_to_base_qy:=-0.1132032138 \
  body_to_base_qw:=0.9935718557 \
  localization_rviz:=true \
  publish_livox_pointcloud2:=true \
  > /tmp/localization_3d_go2.log 2>&1 &
```

在定位 RViz 用 **2D Pose Estimate**（`/initialpose`）给初值。  
确认有：`/baselink2map`、`/cloud_registered_1`、`/livox/lidar_pointcloud2`。

---

## 3. DDDMR 全局规划（Docker `dddmr_dev`）

交互进入：

```bash
docker exec -it dddmr_dev bash
```

容器内：

```bash
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /root/dddmr_ws/install/setup.bash
cd /root/dddmr_ws

ros2 launch p2p_move_base dddmr_global_ego_docker.launch.py \
  config_file:=/root/dddmr_ws/src/dddmr_p2p_move_base/config/p2p_move_base_localization_static_map.yaml \
  map_dir:=/root/dddmr_ws/src/dddmr_global_planner/data/segmentation_obstacle_map.pcd \
  ground_dir:=/root/dddmr_ws/src/dddmr_global_planner/data/segmentation_ground_map.pcd \
  odom_topic:=/baselink2map \
  lidar_topic:=/livox/lidar_pointcloud2 \
  use_rviz:=false \
  publish_sensor_tf:=true \
  publish_base_footprint_tf:=true
```

宿主机一键后台（含 DDDMR RViz）：

```bash
docker exec -d dddmr_dev bash -lc '
  export DISPLAY=:1 QT_X11_NO_MITSHM=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  export ROS_LOCALHOST_ONLY=0
  source /opt/ros/humble/setup.bash
  source /root/dddmr_ws/install/setup.bash
  cd /root/dddmr_ws
  nohup ros2 launch p2p_move_base dddmr_global_ego_docker.launch.py \
    config_file:=/root/dddmr_ws/src/dddmr_p2p_move_base/config/p2p_move_base_localization_static_map.yaml \
    map_dir:=/root/dddmr_ws/src/dddmr_global_planner/data/segmentation_obstacle_map.pcd \
    ground_dir:=/root/dddmr_ws/src/dddmr_global_planner/data/segmentation_ground_map.pcd \
    odom_topic:=/baselink2map \
    lidar_topic:=/livox/lidar_pointcloud2 \
    use_rviz:=false \
    publish_sensor_tf:=true \
    publish_base_footprint_tf:=true \
    >/tmp/dddmr_global_ego.log 2>&1 &
  sleep 8
  nohup rviz2 -d /root/dddmr_ws/src/dddmr_p2p_move_base/rviz/p2p_move_base_localization.rviz \
    >/tmp/dddmr_nav_rviz.log 2>&1 &
'
```

> 必须显式传 `config_file:=..._static_map.yaml`。launch 默认是 `p2p_move_base_localization.yaml`，与当前联调不一致。

下目标：在 DDDMR RViz 用 **Publish Point**（`/clicked_point`）。  
输出：`/global_path` → `/ego_repaired_global_path`。

---

## 4. EGO 局部规划（宿主机）

```bash
source /home/ll/ego-planner-swarm-ros2_version/install/setup.bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash

ros2 launch ego_planner dddmr_reference_tracking.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path \
  reference_path_z_offset:=0.4
```

后台：

```bash
nohup ros2 launch ego_planner dddmr_reference_tracking.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path \
  reference_path_z_offset:=0.4 \
  > /tmp/ego_local_planner.log 2>&1 &
```

`reference_path_z_offset:=0.4` 对齐 SCAN `body_height`，避免 Mid360 地面点把路径当障碍截断。  
正常日志：`WAIT_TARGET`、`Trajectory controller ready`、订阅 `/ego_repaired_global_path`。

---

## 5. 运动桥 go2_bridge

```bash
source /home/ll/unitree_ros2/setup.sh
source /home/ll/ros_ws/install/setup.bash
ros2 run go2_cmd_vel_handler go2_bridge_node
```

后台：

```bash
nohup ros2 run go2_cmd_vel_handler go2_bridge_node > /tmp/go2_bridge.log 2>&1 &
```

应看到 SportClient / `RecoveryStand`。`/cmd_vel`：traj_server 发、bridge 收。

---

## 6. 联调检查

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash

# 关键进程是否在跑（比 topic list 更稳，DDS 忙时 list/hz 可能卡住）
pgrep -af 'livox_ros_driver2_node|localization_3d_go2|dddmr_global_ego_docker|dddmr_reference_tracking|go2_bridge_node' | head

ros2 topic hz /baselink2map
ros2 topic hz /cloud_registered_1
ros2 topic hz /livox/lidar_pointcloud2
ros2 topic info /ego_repaired_global_path -v
ros2 topic info /cmd_vel -v
ros2 topic info /clicked_point -v
```

日志：

```text
/tmp/livox_mid360.log
/tmp/localization_3d_go2.log
/tmp/dddmr_global_ego.log      # 容器内 /tmp
/tmp/ego_local_planner.log
/tmp/go2_bridge.log
```

---

## 7. 关闭（按模块）

```bash
# EGO
pkill -f 'dddmr_reference_tracking.launch.py' || true
pkill -9 -f 'ego_planner_node|traj_server|dddmr_reference_tracking' || true

# bridge
pkill -9 -f 'go2_bridge_node' || true

# 定位（保留 Livox）
pkill -f 'localization_3d_go2.launch.py' || true
pkill -9 -f 'global_localization_node|fastlio_mapping|localization_3d_go2' || true

# DDDMR
docker exec dddmr_dev bash -lc '
  pkill -f dddmr_global_ego_docker || true
  pkill -9 -f "pcl_publisher|mcl_feature|global_planner_node|dddmr_global" || true
  pkill -f p2p_move_base_localization.rviz || true
' 2>/dev/null || true

# Livox（需要时）
pkill -9 -f 'livox_ros_driver2' || true
```

---

## 8. 验证记录（2026-11-02）

当时在跑的命令行与本文一致：

1. `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
2. `localization_3d_go2.launch.py map_file:=.../odin1_odom_map.pcd fastlio_config_file:=mid360.yaml body_to_base_qy:=-0.1132032138 body_to_base_qw:=0.9935718557 localization_rviz:=true publish_livox_pointcloud2:=true`
3. `dddmr_global_ego_docker.launch.py config_file:=.../p2p_move_base_localization_static_map.yaml map_dir:=.../segmentation_obstacle_map.pcd ground_dir:=.../segmentation_ground_map.pcd odom_topic:=/baselink2map lidar_topic:=/livox/lidar_pointcloud2 use_rviz:=false publish_sensor_tf:=true publish_base_footprint_tf:=true`
4. `dddmr_reference_tracking.launch.py ... reference_path_z_offset:=0.4`
5. `ros2 run go2_cmd_vel_handler go2_bridge_node`

已核对：launch 文件存在、地图 PCD 存在、`dddmr_dev` 在跑、上述进程均已拉起；EGO `reference_path_z_offset` 默认已为 `0.4`。

易错点：

- 宿主机与 Docker 的 `RMW_IMPLEMENTATION` 必须都是 Cyclone。
- 下目标用 **Publish Point**，不是 2D Nav Goal。
- 定位图与 DDDMR `map_dir`/`ground_dir` 要同一场景；换图后三处一起换。
- 不要省略 `config_file:=..._static_map.yaml` 和 `reference_path_z_offset:=0.4`。
