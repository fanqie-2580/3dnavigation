# Go2 + MID360：建图 → 导航命令手册

适用机型：**Unitree Go2 + Livox MID360**。  
ROS：**Humble**。当前联调默认 **RMW = CycloneDDS**（宿主机与 `dddmr_dev` 一致）。

| 阶段 | 组件 | 运行位置 |
|------|------|----------|
| 雷达驱动 | `livox_ros_driver2` | 宿主机 |
| 建图 / 定位 | FAST-LIO + open3d_loc | 宿主机 |
| 全局规划 | DDDMR `dddmr_global_ego_docker` | Docker `dddmr_dev` |
| 局部规划 | EGO `dddmr_reference_tracking` | 宿主机 |
| 运动桥 | `go2_bridge_node` | 宿主机（`unitree_ros2/setup.sh`） |

路径速查：

```text
/home/ll/ws_livox
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble
/home/ll/dddmr_navigation          # Docker 内: /root/dddmr_ws
/home/ll/ego-planner-swarm-ros2_version
/home/ll/ros_ws                    # go2_cmd_vel_handler
/home/ll/unitree_ros2
```

当前联调定位地图（已验证）：

```text
/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/odin1_odom_map.pcd
```

> **定位 → 导航最短命令清单**见同目录  
> [`go2_localization_to_navigation.md`](./go2_localization_to_navigation.md)（已按 2026-11-02 在跑进程核对）。

Go2 MID360 `body→base` 外参（当前联调）：

```text
body_to_base_qy:=-0.1132032138
body_to_base_qw:=0.9935718557
```

---

## 0. 公共环境（每个宿主机终端先执行）

```bash
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE
xhost +local:root >/dev/null 2>&1 || true

source /opt/ros/humble/setup.bash
```

运动桥终端额外：

```bash
source /home/ll/unitree_ros2/setup.sh   # 会设 Cyclone + 网卡 enP8p1s0，供 Sport API
source /home/ll/ros_ws/install/setup.bash
```

---

## 1. Docker：准备 / 进入 dddmr

### 1.1 首次或容器不存在时启动（推荐固定名，不要 `--rm`）

```bash
docker run -it \
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

### 1.2 容器已有时

```bash
docker start dddmr_dev
docker exec -it dddmr_dev bash
```

### 1.3 容器内 ROS 环境（跑全局规划前）

```bash
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
source /root/dddmr_ws/install/setup.bash
cd /root/dddmr_ws
```

若容器内尚无 Cyclone RMW：

```bash
apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp
```

---

## 2. 建图流程

建图时一般**不跑**定位 launch、DDDMR、EGO、运动桥。

### 2.1 启动 Livox

```bash
# 见第 0 节公共环境
source /home/ll/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

后台示例：

```bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox_mid360.log 2>&1 &
```

### 2.2 启动 FAST-LIO 建图

```bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rviz:=true
```

`mid360.yaml` 中 `pcd_save.pcd_save_en: true`、`interval: -1` 时，**正常结束建图节点**会写出整图 PCD（通常为工作目录下的 `scans.pcd`，以终端提示为准）。

### 2.3 保存 / 部署定位用地图

```bash
# 建图结束后，把生成的 PCD 拷到定位默认路径（按实际输出名改 SRC）
SRC="$HOME/scans.pcd"   # 或建图终端打印的路径
cp -a "$SRC" /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd
```

> DDDMR 全局层默认还用  
> `global_planner/data/segmentation_obstacle_map.pcd` 与 `segmentation_ground_map.pcd`。  
> 换场景后需同步更新这两份（或 launch 时改 `map_dir` / `ground_dir`），否则定位图与全局图不一致。

### 2.4 关闭建图相关

```bash
pkill -f 'mapping.launch.py' || true
pkill -f 'fastlio_mapping' || true
# 若本次只建图、不要动雷达，可不动 livox；要全关见第 7 节
```

---

## 3. 导航流程（推荐顺序）

顺序：**Livox → 定位 → DDDMR 全局 → EGO 局部 → 运动桥**。  
下列命令默认第 0 节环境变量已 export。

### 3.1 Livox（若未在跑）

```bash
source /home/ll/ws_livox/install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox_mid360.log 2>&1 &
```

### 3.2 定位（FAST-LIO + open3d_loc）

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
确认：`ros2 topic hz /baselink2map` 约 10 Hz。

### 3.3 DDDMR 全局规划（Docker）

```bash
docker exec -it dddmr_dev bash
# 容器内：
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

宿主机一键后台（容器内写日志）：

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

输出路径：`/global_path` → 修复后 `/ego_repaired_global_path`。

**下目标：在 DDDMR RViz 用 Publish Point**（`/clicked_point`）。  
不要用 2D Nav Goal（`/goal_pose` 当前无订阅者）。

### 3.4 EGO 局部规划（宿主机）

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

正常应看到：`WAIT_TARGET`、`Trajectory controller ready`、订阅 `/ego_repaired_global_path`。

### 3.5 运动桥 go2_bridge（宿主机）

```bash
source /home/ll/unitree_ros2/setup.sh
source /home/ll/ros_ws/install/setup.bash

ros2 run go2_cmd_vel_handler go2_bridge_node
```

后台：

```bash
nohup ros2 run go2_cmd_vel_handler go2_bridge_node > /tmp/go2_bridge.log 2>&1 &
```

应看到 SportClient 初始化与 `RecoveryStand`。`/cmd_vel` 需 1 发（traj_server）1 收（bridge）。

---

## 4. 常用检查命令

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash

ros2 topic hz /baselink2map
ros2 topic hz /livox/lidar_pointcloud2
ros2 topic hz /cloud_registered_1
ros2 topic info /cmd_vel -v
ros2 topic info /clicked_point -v
ros2 topic info /ego_repaired_global_path -v
ros2 topic echo /ego_repaired_global_path --once

# Docker 内
docker exec dddmr_dev bash -lc '
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/humble/setup.bash
  source /root/dddmr_ws/install/setup.bash
  ros2 node list | head
'
```

日志：

```text
/tmp/livox_mid360.log
/tmp/localization_3d_go2.log
/tmp/dddmr_global_ego.log          # 在容器内 /tmp，宿主机同路径若挂载则可见
/tmp/ego_local_planner.log
/tmp/go2_bridge.log
```

---

## 5. 按模块关闭（关残留）

### 5.1 只关局部规划 EGO

```bash
pkill -f 'dddmr_reference_tracking.launch.py' || true
pkill -f 'ego_planner_node' || true
pkill -f 'traj_server' || true
pkill -f 'go2_navigation.rviz' || true
sleep 1
pkill -9 -f 'ego_planner_node|traj_server|dddmr_reference_tracking' || true
```

### 5.2 只关运动桥

```bash
pkill -f 'go2_bridge_node' || true
pkill -9 -f 'go2_bridge_node' || true
```

### 5.3 只关定位（保留 Livox）

```bash
pkill -f 'localization_3d_go2.launch.py' || true
pkill -f 'global_localization_node' || true
pkill -f 'fastlio_mapping' || true
pkill -f 'fastlio.rviz' || true
pkill -f 'rviz_map_cur' || true
# 残留 static TF（可选）
pkill -f 'odom_to_fastlio_world_broadcaster' || true
pkill -f 'fastlio_body_to_base_broadcaster' || true
sleep 1
pkill -9 -f 'global_localization_node|fastlio_mapping|localization_3d_go2' || true
```

### 5.4 只关 DDDMR 全局（Docker）

```bash
docker exec dddmr_dev bash -lc '
  pkill -f dddmr_global_ego_docker || true
  pkill -f pcl_publisher || true
  pkill -f mcl_feature || true
  pkill -f global_planner_node || true
  pkill -f p2p_move_base_localization.rviz || true
  pkill -f sensor2baselink || true
  pkill -f baselink2footprint || true
  sleep 1
  pkill -9 -f "pcl_publisher|mcl_feature|global_planner_node|dddmr_global" || true
'
```

### 5.5 只关 Livox

```bash
pkill -f 'livox_ros_driver2' || true
pkill -9 -f 'livox_ros_driver2' || true
```

---

## 6. 一键清导航栈（保留或不保留 Livox）

### 6.1 关导航、保留雷达

```bash
# 宿主机：ego / bridge / loc / 相关 rviz
pkill -f 'dddmr_reference_tracking.launch.py' || true
pkill -f 'ego_planner_node' || true
pkill -f 'traj_server' || true
pkill -f 'go2_bridge_node' || true
pkill -f 'localization_3d_go2.launch.py' || true
pkill -f 'global_localization_node' || true
pkill -f 'fastlio_mapping' || true
pkill -f 'go2_navigation.rviz' || true
pkill -f 'fastlio.rviz' || true
pkill -f 'rviz_map_cur' || true
pkill -f 'odom_to_fastlio_world_broadcaster' || true
pkill -f 'fastlio_body_to_base_broadcaster' || true

docker exec dddmr_dev bash -lc '
  pkill -f dddmr_global_ego_docker || true
  pkill -f pcl_publisher || true
  pkill -f mcl_feature || true
  pkill -f global_planner_node || true
  pkill -f rviz2 || true
  pkill -f sensor2baselink || true
  pkill -f baselink2footprint || true
' 2>/dev/null || true

sleep 2
pkill -9 -f 'ego_planner_node|traj_server|go2_bridge_node|global_localization_node|fastlio_mapping|localization_3d_go2|dddmr_reference' || true
```

### 6.2 全部清掉（含 Livox + 宿主机残余 rviz）

在 6.1 之后追加：

```bash
pkill -f 'livox_ros_driver2' || true
pkill -f '/opt/ros/humble/lib/rviz2/rviz2' || true
pkill -9 -f 'livox_ros_driver2|rviz2' || true
```

> 注意：`pkill rviz2` 会关掉所有 RViz。Docker 内僵尸（`Z`）进程一般无害，重启容器可清：  
> `docker restart dddmr_dev`（会断掉容器内节点）。

---

## 7. 编译（改代码后）

### 7.1 EGO / FAST_LIO（宿主机）

```bash
cd /home/ll/ego-planner-swarm-ros2_version
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ego_planner

cd /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select open3d_loc fast_lio
```

### 7.2 go2 桥（宿主机）

```bash
cd /home/ll/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_cmd_vel_handler
```

### 7.3 DDDMR（Docker 内）

```bash
docker exec -it dddmr_dev bash -lc '
  cd /root/dddmr_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-select p2p_move_base global_planner
'
```

---

## 8. 联调注意（易踩坑）

1. 宿主机与 Docker 的 `RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID` 必须一致；当前用 Cyclone + host 网络。  
2. 运动桥用 `unitree_ros2/setup.sh`（Sport 走 `enP8p1s0`），不要和纯 FastDDS 混用同一桥进程。  
3. 导航下目标用 **Publish Point**，不是 2D Nav Goal。  
4. 目标点到绿区空旷处；局部栅格黑洞 / 品红膨胀区过窄时会出现 truncate、`|a|>1.5`、规划失败。  
5. EGO 日志 `wait for goal or trigger` 且没有 `/ego_repaired_global_path`：先查全局是否收到 click、是否发出 path。  
6. 不要在 `/home/ll` 全局乱 `pkill`；按上面模块命令关，避免误杀无关项目。

---

## 9. 最短导航启动清单（地图已有）

```bash
# 终端 A：Livox
# 终端 B：定位 localization_3d_go2
# 终端 C：Docker 内 dddmr_global_ego_docker + RViz
# 终端 D：EGO dddmr_reference_tracking
# 终端 E：go2_bridge_node（unitree setup.sh）
# 然后在 DDDMR RViz → Publish Point
```

详细命令见第 3 节与 [`go2_localization_to_navigation.md`](./go2_localization_to_navigation.md)；关闭见第 5–6 节。
