# Odin1 单独 / Mid360 单独：建图·分割·定位·导航命令

> 本文只写**互不融合**的两条链路命令行，以及源码是否还在、如何避免和 Mid+Odin 融合版冲突。  
> 融合版见 [`mid360_odin1_multi_lio.md`](./mid360_odin1_multi_lio.md)（工作区 `/home/ll/fast_lio_multi_ws`）。  
> 推送 GitHub：[`push_to_github.md`](./push_to_github.md)。

验证对照：2026-11-09 实机按本文「A. Odin1 单独导航」逐步拉起成功。

---

## 0. 三套链路源码还在吗？（结论：都在，路径已分开）

| 链路 | 建图 / 分割 | 定位 launch | 全局 launch | 地图目录 |
|------|-------------|-------------|-------------|----------|
| **Odin1 单独** | `/home/ll/ws_odin_pvlio/scripts/02…11_*.sh`、`07_segment_map.*` | `open3d_loc/launch/localization_3d_odin1.launch.py` | `dddmr_global_ego_odin1.launch.py` | `ws_odin_pvlio/odin1_nav/maps/` + `dddmr…/data/odin1/` |
| **Mid360 单独** | `fast_lio/mapping.launch.py` + Mid 点云存图 | `localization_3d_go2.launch.py` | `dddmr_global_ego_docker.launch.py` | `FAST_LIO…/data/*.pcd` + `dddmr…/data/segmentation_*.pcd`（**无** `odin1/` 子目录） |
| **Mid+Odin 融合** | 融合 IEKF，不走上面两套定位 launch | `fast_lio_multi` → `mid360_odin1_multi.launch.py` | 可挂 docker 或 odin1 全局，但 **odom 来自融合** | 融合自己的 config：`mid360_odin1.yaml` |

### 已核对仍存在的关键文件

```text
# Odin1
/home/ll/ws_odin_pvlio/scripts/02_run_mapping.sh
/home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
/home/ll/ws_odin_pvlio/scripts/06_save_map.sh
/home/ll/ws_odin_pvlio/scripts/07_segment_map.sh
/home/ll/ws_odin_pvlio/scripts/07_segment_map.py
/home/ll/ws_odin_pvlio/scripts/08_start_dock_mapping.sh
/home/ll/ws_odin_pvlio/scripts/09_pull_dock_map.sh
/home/ll/ws_odin_pvlio/scripts/10_stop_dock_mapping.sh
/home/ll/ws_odin_pvlio/scripts/11_start_odin1_nav.sh
/home/ll/ws_odin_pvlio/odin1_nav/README.md
/home/ll/ws_odin_pvlio/odin1_nav/scripts/odin1_loc_topic_adapter.py
/home/ll/ws_odin_pvlio/odin1_nav/maps/odin1_odom_map.pcd
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/open3d_loc/launch/localization_3d_odin1.launch.py
/home/ll/dddmr_navigation/src/dddmr_p2p_move_base/launch/dddmr_global_ego_odin1.launch.py

# Mid360
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/FAST_LIO/launch/mapping.launch.py
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/open3d_loc/launch/localization_3d_go2.launch.py
/home/ll/dddmr_navigation/src/dddmr_p2p_move_base/launch/dddmr_global_ego_docker.launch.py
/home/ll/3dnavigation/docs/go2_mapping_nav_commands.md
/home/ll/3dnavigation/docs/go2_localization_to_navigation.md

# 融合（独立工作区，勿和上面两套同时发 /baselink2map）
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/launch/mid360_odin1_multi.launch.py
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/config/mid360_odin1.yaml
```

### 防冲突规则（必读）

1. **同一时刻只跑一套定位**：`localization_3d_odin1` / `localization_3d_go2` / `mid360_odin1_multi` **三者不要并存**（都会争 `/baselink2map`）。  
2. **地图目录不要互相覆盖**：  
   - Mid360 全局默认：`…/data/segmentation_{obstacle,ground}_map.pcd`  
   - Odin1 全局默认：`…/data/odin1/segmentation_{obstacle,ground}_map.pcd`  
   - Odin1 定位图副本：`ws_odin_pvlio/odin1_nav/maps/`（不要用融合现场临时对齐后的图覆盖这份「真 Odin 图」除非有意更新）  
3. **Odin TCP 话题模式不同**：  
   - Odin1 单独定位：坞上转发 **`/odin1/cloud_slam`**（已在 odom 系）  
   - Mid+Odin 融合：通常转发 **`/odin1/cloud_raw`**  
   换链路时务必重启坞上 `odin_ros1_tcp_server.py` 并改 Jetson client 的 `cloud_topic`。  
4. **融合代码**只在 `fast_lio_multi_ws`；不要改 `localization_3d_go2` / `localization_3d_odin1` 去「塞融合」。

公共环境（宿主机每个终端）：

```bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_LOCALHOST_ONLY=0
xhost +local:root >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
```

---

## A. Odin1 单独：建图 → 分割 → 定位 → 导航

一键导航（内部含坞桥 + 定位 + dddmr_odin1 + ego）：

```bash
bash /home/ll/ws_odin_pvlio/scripts/11_start_odin1_nav.sh
```

下面是**拆开逐步执行**的命令（与 2026-11-09 实机一致）。

### A0. 停掉其它定位栈（防冲突）

```bash
pkill -f 'mid360_odin1_multi.launch.py' 2>/dev/null || true
pkill -f 'laserMapping_adaptive' 2>/dev/null || true
pkill -f 'localization_3d_go2|localization_3d_odin1' 2>/dev/null || true
pkill -f 'fastlio_mapping|global_localization_node' 2>/dev/null || true
pkill -f 'dddmr_global_ego_docker|dddmr_global_ego_odin1' 2>/dev/null || true
pkill -f 'dddmr_reference_tracking|ego_planner_node|traj_server' 2>/dev/null || true
pkill -f 'odin_ros2_tcp_client.py|odin1_loc_topic_adapter.py' 2>/dev/null || true
pkill -f 'msg_MID360_launch.py|livox_ros_driver2_node' 2>/dev/null || true
docker exec dddmr_dev bash -lc 'pkill -9 -f "dddmr_global_ego|pcl_publisher|mcl_feature|global_planner_node|rviz2" 2>/dev/null || true'
```

### A1. 建图（Odin 自带里程计）

推荐坞上建图（稠密 `cloud_slam` 不易 TCP 炸）：

```bash
bash /home/ll/ws_odin_pvlio/scripts/08_start_dock_mapping.sh
# 走完场景后存图并拉回 Jetson：
bash /home/ll/ws_odin_pvlio/scripts/09_pull_dock_map.sh
bash /home/ll/ws_odin_pvlio/scripts/10_stop_dock_mapping.sh
```

或 Jetson 侧桥接建图：

```bash
bash /home/ll/ws_odin_pvlio/scripts/02_run_mapping.sh
# 走完：
bash /home/ll/ws_odin_pvlio/scripts/06_save_map.sh
bash /home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
```

默认定位图输出：

```text
/home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd
```

拷到导航用目录（保持 Odin1 链路自洽）：

```bash
cp -a /home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd \
  /home/ll/ws_odin_pvlio/odin1_nav/maps/odin1_odom_map.pcd
# 可选：同步一份到 open3d 默认 data（注意别和 Mid360 定位图混用语义）
# cp -a ... /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/odin1_odom_map.pcd
```

### A2. 分割（地面 / 障碍 → dddmr）

```bash
bash /home/ll/ws_odin_pvlio/scripts/07_segment_map.sh \
  /home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd

# 拷到 Odin1 专用全局图目录（不要覆盖 Mid360 的 data/segmentation_*.pcd）
cp -a /home/ll/ws_odin_pvlio/maps/segmentation_ground_map.pcd \
      /home/ll/ws_odin_pvlio/maps/segmentation_obstacle_map.pcd \
      /home/ll/ws_odin_pvlio/odin1_nav/maps/
cp -a /home/ll/ws_odin_pvlio/maps/segmentation_ground_map.pcd \
      /home/ll/ws_odin_pvlio/maps/segmentation_obstacle_map.pcd \
      /home/ll/dddmr_navigation/src/dddmr_global_planner/data/odin1/
```

### A3. 坞上驱动 + TCP（导航用 `cloud_slam`）

```bash
WS=/home/ll/ws_odin_pvlio
sshpass -p '123' scp "$WS/bridge/odin_ros1_tcp_server.py" \
  unitree@192.168.123.18:/home/unitree/odin_bridge/odin_ros1_tcp_server.py

sshpass -p '123' ssh -o StrictHostKeyChecking=no unitree@192.168.123.18 'bash -s' <<'EOF'
pkill -f odin_ros1_tcp_server.py 2>/dev/null || true
if ! pgrep -f host_sdk_sample >/dev/null; then
  bash /home/unitree/odin_bridge/start_dock_mapping.sh \
    >/home/unitree/odin_bridge/logs/start_for_nav.log 2>&1 &
  sleep 12
fi
source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.123.18
setsid python3 /home/unitree/odin_bridge/odin_ros1_tcp_server.py \
  _cloud_topic:=/odin1/cloud_slam \
  > /home/unitree/odin_bridge/logs/tcp_server_nav.log 2>&1 < /dev/null &
sleep 3
ss -ltn | grep 5601
timeout 3 rostopic hz /odin1/cloud_slam
EOF
```

### A4. Jetson：TCP client + topic adapter

```bash
source /opt/ros/humble/setup.bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash

nohup python3 /home/ll/ws_odin_pvlio/bridge/odin_ros2_tcp_client.py \
  --ros-args \
  -p server_host:=192.168.123.18 \
  -p server_port:=5601 \
  -p cloud_topic:=/odin1/cloud_slam \
  > /tmp/odin1_nav_tcp_client.log 2>&1 &

nohup python3 /home/ll/ws_odin_pvlio/odin1_nav/scripts/odin1_loc_topic_adapter.py \
  > /tmp/odin1_loc_adapter.log 2>&1 &

# 期望：/odin1/cloud_slam、/odin1/odometry、/cloud_registered_1、/cloud_registered_body_1
ros2 topic hz /odin1/cloud_slam
```

### A5. 定位（open3d_loc，加载 PCD，无 Mid360）

```bash
MAP=/home/ll/ws_odin_pvlio/odin1_nav/maps/odin1_odom_map.pcd

nohup ros2 launch open3d_loc localization_3d_odin1.launch.py \
  map_file:="$MAP" \
  localization_rviz:=true \
  > /tmp/odin1_localization.log 2>&1 &

# 期望：Loaded map points ≈ 65万；/baselink2map 有输出
# RViz：2D Pose Estimate → /initialpose
ros2 topic hz /baselink2map
```

### A6. 全局规划（Docker，`dddmr_global_ego_odin1`）

```bash
docker start dddmr_dev 2>/dev/null || true

docker exec -d dddmr_dev bash -lc '
  export DISPLAY=:1 QT_X11_NO_MITSHM=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  export ROS_LOCALHOST_ONLY=0
  source /opt/ros/humble/setup.bash
  source /root/dddmr_ws/install/setup.bash
  DEST=$(ros2 pkg prefix p2p_move_base)/share/p2p_move_base/launch
  cp -f /root/dddmr_ws/src/dddmr_p2p_move_base/launch/dddmr_global_ego_odin1.launch.py "$DEST/"
  nohup ros2 launch p2p_move_base dddmr_global_ego_odin1.launch.py \
    odom_topic:=/baselink2map \
    lidar_topic:=/cloud_registered_body_1 \
    use_rviz:=true \
    publish_sensor_tf:=true \
    sensor_frame:=body \
    >/tmp/dddmr_odin1_nav.log 2>&1 &
'

# 默认地图：
#   map_dir  = .../data/odin1/segmentation_obstacle_map.pcd
#   ground_dir = .../data/odin1/segmentation_ground_map.pcd
# RViz：Publish Point → /clicked_point
```

### A7. 局部规划 + 运动桥

```bash
source /home/ll/ego-planner-swarm-ros2_version/install/setup.bash
nohup ros2 launch ego_planner dddmr_reference_tracking.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path \
  > /tmp/ego_odin1_local.log 2>&1 &

source /home/ll/unitree_ros2/setup.sh
source /home/ll/ros_ws/install/setup.bash
nohup ros2 run go2_cmd_vel_handler go2_bridge_node \
  > /tmp/go2_bridge.log 2>&1 &
```

### A8. 停止 Odin1 导航

```bash
pkill -f 'localization_3d_odin1|global_localization_node' 2>/dev/null || true
pkill -f 'odin_ros2_tcp_client.py|odin1_loc_topic_adapter.py' 2>/dev/null || true
pkill -f 'dddmr_reference_tracking|ego_planner_node|traj_server' 2>/dev/null || true
pkill -f 'go2_bridge_node' 2>/dev/null || true
docker exec dddmr_dev bash -lc 'pkill -f dddmr_global_ego_odin1 2>/dev/null || true'
sshpass -p '123' ssh unitree@192.168.123.18 'pkill -f odin_ros1_tcp_server.py' 2>/dev/null || true
```

---

## B. Mid360 单独：建图 → 分割 → 定位 → 导航

完整手册仍以这两份为准（命令已验证过）：

- 建图+导航总册：[`go2_mapping_nav_commands.md`](./go2_mapping_nav_commands.md)  
- 定位→导航最短清单：[`go2_localization_to_navigation.md`](./go2_localization_to_navigation.md)

下面是浓缩命令行。

### B0. 停掉 Odin1 / 融合（防冲突）

```bash
pkill -f 'mid360_odin1_multi.launch.py|laserMapping_adaptive' 2>/dev/null || true
pkill -f 'localization_3d_odin1|odin1_loc_topic_adapter|odin_ros2_tcp_client' 2>/dev/null || true
pkill -f 'dddmr_global_ego_odin1' 2>/dev/null || true
```

### B1. 建图（FAST-LIO Mid360）

```bash
source /home/ll/ws_livox/install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox_mid360.log 2>&1 &

source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:=true

# 正常结束建图节点后，把生成的 scans.pcd 拷到定位目录，例如：
# cp -a "$HOME/scans.pcd" \
#   /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd
```

### B2. 分割（给 dddmr 全局层）

Mid360 场景常用同一套 `07_segment_map.sh`（输入换成 Mid 全局 PCD），输出放到 **无 `odin1/` 前缀** 的默认路径：

```bash
# 例：INPUT 为 Mid360 存下来的全局图
bash /home/ll/ws_odin_pvlio/scripts/07_segment_map.sh /path/to/mid360_global.pcd

cp -a /home/ll/ws_odin_pvlio/maps/segmentation_ground_map.pcd \
      /home/ll/ws_odin_pvlio/maps/segmentation_obstacle_map.pcd \
      /home/ll/dddmr_navigation/src/dddmr_global_planner/data/
# ↑ 注意：这是 Mid360 默认全局图路径，不要写进 data/odin1/
```

### B3. 定位（FAST-LIO + open3d_loc Go2）

```bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash

# 使用与场景匹配的定位 PCD（示例）
MAP=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd
# 或历史联调：odin1_odom_map.pcd（跨传感器时可能漂，见 odin1_mid360_map_and_fusion.md）

nohup ros2 launch open3d_loc localization_3d_go2.launch.py \
  map_file:="$MAP" \
  fastlio_config_file:=mid360.yaml \
  body_to_base_qy:=-0.1132032138 \
  body_to_base_qw:=0.9935718557 \
  localization_rviz:=true \
  publish_livox_pointcloud2:=true \
  > /tmp/localization_3d_go2.log 2>&1 &
```

### B4. 全局（`dddmr_global_ego_docker`）+ EGO + bridge

```bash
docker exec -d dddmr_dev bash -lc '
  export DISPLAY=:1 QT_X11_NO_MITSHM=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  source /opt/ros/humble/setup.bash
  source /root/dddmr_ws/install/setup.bash
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

source /home/ll/ego-planner-swarm-ros2_version/install/setup.bash
nohup ros2 launch ego_planner dddmr_reference_tracking.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path \
  reference_path_z_offset:=0.4 \
  > /tmp/ego_local_planner.log 2>&1 &

source /home/ll/unitree_ros2/setup.sh
source /home/ll/ros_ws/install/setup.bash
nohup ros2 run go2_cmd_vel_handler go2_bridge_node > /tmp/go2_bridge.log 2>&1 &
```

---

## C. 和融合版怎么并存开发、互不踩脚

| 项目 | Odin1 单独 | Mid360 单独 | Mid+Odin 融合 |
|------|------------|-------------|---------------|
| 工作区 | `ws_odin_pvlio` + open3d_loc odin1 launch | `ws_livox` + FAST_LIO + open3d go2 | **`fast_lio_multi_ws` 独立** |
| 定位进程 | `localization_3d_odin1` | `localization_3d_go2` | `mid360_odin1_multi` |
| 全局 launch | `dddmr_global_ego_odin1` | `dddmr_global_ego_docker` | 可复用 docker/odin1，但只接融合 odom |
| 全局 PCD | `data/odin1/*` | `data/segmentation_*` | 应用融合系对齐后的图；勿回写污染单独链路真图 |
| Odin TCP | `cloud_slam` | 不用 | `cloud_raw` |

**开发建议**：改融合只动 `fast_lio_multi_ws`；改 Odin1 单独只动 `ws_odin_pvlio` + `localization_3d_odin1` + `dddmr_global_ego_odin1`；改 Mid360 单独只动 `localization_3d_go2` + `dddmr_global_ego_docker` + Mid 默认 `data/segmentation_*`。

---

## D. 各部分主要 `.cpp` 源文件（仍在）

> 建图/定位桥接有一部分是 **Python**（`odin_ros2_tcp_client.py`、`odin_odom_mapper.py`、`odin1_loc_topic_adapter.py`、融合侧 `multi_lio_nav_bridge.py` 等），下面只列核心 C++。

### D1. Mid360 驱动

```text
/home/ll/ws_livox/src/livox_ros_driver2/src/livox_ros_driver2.cpp
/home/ll/ws_livox/src/livox_ros_driver2/src/driver_node.cpp
/home/ll/ws_livox/src/livox_ros_driver2/src/lds_lidar.cpp
/home/ll/ws_livox/src/livox_ros_driver2/src/lddc.cpp
```

### D2. Mid360 单独：建图 / 前端里程计（FAST-LIO）

```text
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/FAST_LIO/src/laserMapping.cpp      # 建图+里程计主节点
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/FAST_LIO/src/preprocess.cpp
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp
```

### D3. 定位匹配（open3d_loc，Odin1 单独 & Mid360 单独共用库，不同 launch）

```text
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/open3d_loc/src/global_localization.cpp
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/open3d_loc/src/open3d_registration/open3d_registration.cpp
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/open3d_loc/src/open3d_conversions/open3d_conversions.cpp
```

- Mid360 单独入口 launch：`…/launch/localization_3d_go2.launch.py`（内部还会起 `FAST_LIO/src/laserMapping.cpp`）  
- Odin1 单独入口 launch：`…/launch/localization_3d_odin1.launch.py`（**不起** Mid FAST-LIO，只跑 `global_localization.cpp`）

### D4. Odin 驱动 / 可选 PV-LIO（Odin 侧 C++；日常 Odin 建图主路径是 Python odom mapper）

```text
/home/ll/ws_odin_pvlio/src/odin_ros_driver/src/host_sdk_sample.cpp
/home/ll/ws_odin_pvlio/src/PV-LIO_ROS2/src/voxelMapping.cpp          # 可选旧建图，非默认
/home/ll/ws_odin_pvlio/src/PV-LIO_ROS2/src/preprocess.cpp
```

### D5. Mid+Odin 融合（独立树，勿与 D2 的 `laserMapping.cpp` 混改）

```text
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/src/laserMapping_adaptive.cpp  # 当前实机用
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/src/laserMapping_async.cpp
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/src/laserMapping_bundle.cpp
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/src/preprocess.cpp
/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/include/ikd-Tree/ikd_Tree.cpp
```

### D6. 全局规划（DDDMR，两条单独链路共用同一套 C++，靠不同 launch/地图目录区分）

```text
/home/ll/dddmr_navigation/src/dddmr_mcl_3dl/src/pcl_publisher.cpp
/home/ll/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/src/mcl_feature_node.cpp
/home/ll/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp
/home/ll/dddmr_navigation/src/dddmr_lego_loam/lego_loam_bor/src/featureAssociation.cpp
/home/ll/dddmr_navigation/src/dddmr_global_planner/src/global_planner_node.cpp
/home/ll/dddmr_navigation/src/dddmr_global_planner/src/global_planner.cpp
/home/ll/dddmr_navigation/src/dddmr_global_planner/src/dynamic_window_aware_global_planner.cpp
/home/ll/dddmr_navigation/src/dddmr_global_planner/src/ego_reference_path_provider.cpp
/home/ll/dddmr_navigation/src/dddmr_global_planner/src/a_star_on_pc.cpp
/home/ll/dddmr_navigation/src/dddmr_perception_3d/src/perception_3d_ros.cpp
/home/ll/dddmr_navigation/src/dddmr_perception_3d/plugins/static_layer.cpp
```

### D7. 局部规划（EGO，三条链路共用）

路径含中文目录名 `ego-planner（改）`：

```text
…/planner/plan_manage/src/ego_planner_node.cpp
…/planner/plan_manage/src/ego_replan_fsm.cpp
…/planner/plan_manage/src/planner_manager.cpp
…/planner/plan_manage/src/traj_server.cpp
…/planner/plan_env/src/grid_map.cpp
…/planner/bspline_opt/src/bspline_optimizer.cpp
…/planner/bspline_opt/src/uniform_bspline.cpp
…/planner/path_searching/src/dyn_a_star.cpp
```

根目录：`/home/ll/ego-planner-swarm-ros2_version/src/ego-planner（改）/planner/`

### D8. 和「谁改哪份 cpp」对照

| 你要改的能力 | 改哪份 cpp | 不要动 |
|--------------|------------|--------|
| Mid360 单独 LIO/建图 | `FAST_LIO/.../laserMapping.cpp` | `fast_lio_multi_ws/.../laserMapping_*.cpp` |
| 融合双雷达 LIO | `fast_lio_multi_ws/.../laserMapping_adaptive.cpp` | `FAST_LIO/.../laserMapping.cpp` |
| 地图匹配定位 | `open3d_loc/.../global_localization.cpp` | — |
| 全局路径/参考路径 | `dddmr_global_planner/src/*.cpp` | — |
| 局部避障/跟踪 | EGO `planner_manager.cpp` / `grid_map.cpp` / `bspline_optimizer.cpp` / `traj_server.cpp` | — |

---

## E. 相关文档索引

| 文档 | 内容 |
|------|------|
| [`odin1_odom_mapping.md`](../../ws_odin_pvlio/docs/odin1_odom_mapping.md)（`/home/ll/ws_odin_pvlio/docs/`） | Odin 建图细节 |
| [`odin1_nav/README.md`](../../ws_odin_pvlio/odin1_nav/README.md) | Odin1 导航一键入口 |
| [`go2_mapping_nav_commands.md`](./go2_mapping_nav_commands.md) | Mid360 建图导航总册 |
| [`go2_localization_to_navigation.md`](./go2_localization_to_navigation.md) | Mid360 定位→导航最短清单 |
| [`odin1_mid360_map_and_fusion.md`](./odin1_mid360_map_and_fusion.md) | 为何 Odin 图 + Mid 定位易漂 |
| [`mid360_odin1_multi_lio.md`](./mid360_odin1_multi_lio.md) | 融合版 |
