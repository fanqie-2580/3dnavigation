# Mid360 + Odin1 双雷达 PV-LIO 建图命令

适用：**Livox Mid360 + 留形 Odin1**，双雷达融合建图。  
ROS：**Humble**。联调默认 **`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`**，显示 **`DISPLAY=:1`**。

| 组件 | 路径 / 包 | 作用 |
|------|-----------|------|
| Mid360 驱动 | `/home/ll/ws_livox` → `livox_ros_driver2` | `/livox/lidar`、`/livox/imu` |
| Odin1 驱动 | `/home/ll/ws_odin_pvlio` → `odin_ros_driver` | `/odin1/cloud_raw`、`/odin1/imu` |
| 双雷达建图 | `/home/ll/pv_lio_dual_ws` → `pv_lio` | `mapping_dual_mid_odin.launch.py` |
| 参数 | `pv_lio/config/dual_mid_odin.yaml` | sync / 外参 / Global leaf |
| RViz | `pv_lio/config/rviz_cfg/voxel_mapping.rviz` | body + Global |

> 单雷达 Odin 建图见 [`odin1_pv_lio_mapping.md`](./odin1_pv_lio_mapping.md)（`ws_odin_pvlio`）。  
> 本手册专指 **双雷达** 工作空间 `pv_lio_dual_ws`。

日志目录建议：

```bash
mkdir -p /tmp/pv_lio_dual_run
```

---

## 一键开关（推荐）

```bash
# 开启（会先 stop 再起 Mid + Odin + mapping + RViz）
bash /home/ll/3dnavigation/scripts/start_dual_mid_odin_pv_lio.sh

# 关闭
bash /home/ll/3dnavigation/scripts/stop_dual_mid_odin_pv_lio.sh
```

日志：`/tmp/pv_lio_dual_run/{livox,odin,mapping,rviz}.log`

说明：启动脚本**不要**在终端里先 `set -u` 再手动 `source /opt/ros/humble/setup.bash`；脚本内部已避开 nounset（否则会报 `AMENT_TRACE_SETUP_FILES: unbound variable`）。

---

## 0. 环境约定

```bash
export DISPLAY=:1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
```

建图节点建议用 **干净环境** 启动（避免 Autoware 等 underlay 污染）：

```bash
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/local/bin" \
  DISPLAY=:1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  bash -lc '...'
```

---

## 1. Mid360 驱动（宿主机）

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

后台：

```bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/pv_lio_dual_run/livox.log 2>&1 &
```

检查：

```bash
timeout 5 ros2 topic hz /livox/lidar
timeout 5 ros2 topic hz /livox/imu
# 期望约 10 Hz
```

---

## 2. Odin1 驱动（宿主机 USB）

先确认设备：

```bash
lsusb | grep 2207
```

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_odin_pvlio/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CFG=/home/ll/ws_odin_pvlio/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml

ros2 run odin_ros_driver host_sdk_sample --ros-args \
  -r __node:=host_sdk_sample \
  -p config_file:=$CFG
```

后台：

```bash
nohup ros2 run odin_ros_driver host_sdk_sample --ros-args \
  -r __node:=host_sdk_sample \
  -p config_file:=$CFG \
  > /tmp/pv_lio_dual_run/odin.log 2>&1 &
```

检查：

```bash
timeout 5 ros2 topic hz /odin1/cloud_raw
timeout 5 ros2 topic hz /odin1/imu
# cloud_raw 约 5–6 Hz
```

> 双雷达融合时驱动需 **Odometry / 原始点云模式**（不要让 Odin 自带 SLAM 抢数据）。

---

## 3. 双雷达 PV-LIO 建图

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/pv_lio_dual_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:1

# 推荐：干净环境
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/local/bin" \
  DISPLAY=:1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/ws_livox/install/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=false'
```

后台：

```bash
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/local/bin" \
  DISPLAY=:1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/ws_livox/install/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=false' \
  > /tmp/pv_lio_dual_run/mapping.log 2>&1 &
```

参数文件（symlink-install 改源码即生效）：

```text
/home/ll/pv_lio_dual_ws/src/PV-LIO_ROS2/config/dual_mid_odin.yaml
```

关键 topic：

| Topic | 含义 |
|-------|------|
| `/cloud_registered_body` | 实时 body 点云（Odin intensity≈200） |
| `/cloud_registered` | 世界系实时点云 |
| `/Global_cloud_map` | 累积地图 |
| `/Odometry` | 位姿 |

日志里正常配对示例：

```text
[pv-dual] ... odin_used=20xxx ... skip=ok
[pv-dual] ... odin_used=20xxx ... skip=reuse   # Mid 空档复用上一帧 Odin 并 deskew 到本 Mid-end
```

紧耦合策略（当前）：

1. **新鲜配对的 Odin 进 IEKF + Global**（与 Mid 紧耦合）。
2. **复用 Odin（补 5–6Hz 空档）只进 body/world 显示**，不进 IEKF（同一帧多次更新会漂）。
3. Odin deskew 回到 **aa15460**：原生加速度、**不加重力项**（重力修正在紧耦合下会拉歪 Mid↔Odin）。

`skip=unpaired` 且 `odin_in=0` 持续出现时，多半是建图算不过来、Mid 积压；当前代码有 `drop_stale_mid`，仍异常则重启建图节点。

---

## 4. RViz（可选单独开）

```bash
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin" \
  DISPLAY=:1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec rviz2 -d /home/ll/pv_lio_dual_ws/install/pv_lio/share/pv_lio/config/rviz_cfg/voxel_mapping.rviz' \
  > /tmp/pv_lio_dual_run/rviz.log 2>&1 &
```

或 launch 自带：

```bash
ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=true
```

---

## 5. 一键顺序（复制用）

四个终端或按序后台：

```bash
mkdir -p /tmp/pv_lio_dual_run
export DISPLAY=:1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0

# --- 1) Mid360 ---
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/pv_lio_dual_run/livox.log 2>&1 &
sleep 4

# --- 2) Odin1 ---
source /home/ll/ws_odin_pvlio/install/setup.bash
CFG=/home/ll/ws_odin_pvlio/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml
nohup ros2 run odin_ros_driver host_sdk_sample --ros-args \
  -r __node:=host_sdk_sample -p config_file:=$CFG \
  > /tmp/pv_lio_dual_run/odin.log 2>&1 &
sleep 6

# --- 3) PV-LIO dual ---
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/local/bin" \
  DISPLAY=:1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/ws_livox/install/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=false' \
  > /tmp/pv_lio_dual_run/mapping.log 2>&1 &
sleep 2

# --- 4) RViz ---
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin" \
  DISPLAY=:1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec rviz2 -d /home/ll/pv_lio_dual_ws/install/pv_lio/share/pv_lio/config/rviz_cfg/voxel_mapping.rviz' \
  > /tmp/pv_lio_dual_run/rviz.log 2>&1 &
```

---

## 6. 停止

```bash
pkill -f 'pv_lio_node|mapping_dual_mid_odin|rviz2' || true
pkill -f 'host_sdk_sample|odin_ros_driver' || true
pkill -f 'livox_ros_driver2|msg_MID360_launch' || true
```

强杀（必要时）：

```bash
pkill -9 -f 'pv_lio_node|rviz2|host_sdk_sample|livox_ros_driver2_node' || true
```

---

## 7. 改代码后编译

```bash
cd /home/ll/pv_lio_dual_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pv_lio --allow-overriding pv_lio
```

只改 `dual_mid_odin.yaml` 时一般 **不用重编**（symlink），但要 **重启** `pv_lio_node` 才加载新外参。

---

## 8. 当前外参备忘（以 yaml 为准）

文件：

```text
/home/ll/pv_lio_dual_ws/src/PV-LIO_ROS2/config/dual_mid_odin.yaml
```

常见项：

- Mid：`/livox/lidar` + `/livox/imu`
- Odin：`/odin1/cloud_raw` + `/odin1/imu`
- `extrinsic_T_L2_wrt_L1` / `extrinsic_R_L2_wrt_L1`：Odin 相对 Mid
- `global_map_leaf_size`：累积地图体素
- `lidar2_max_sync_dt` / `lidar2_soft_wait_sec` / `lidar2_reuse_max_age`：双雷达时间配对与 Odin 复用

方向约定（本机视觉标定）：

- 翻滚 **向右** → `Rx(+)`
- 翻滚 **往左** → `Rx(-)`
- 俯仰 **向上抬** → `Ry` 更负（如 -13° → -14°）
