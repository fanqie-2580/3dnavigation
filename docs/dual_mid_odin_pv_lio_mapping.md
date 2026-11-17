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
[pv-dual] ... skip=ok_s2 / ok_nodeskew_s2   # 二段软 Odin 进 IEKF
[pv-dual] ... skip=ok_map / ok_nodeskew_map # 仅 Global（end_dt 大）
[pv-dual] ... skip=ok_map_gyr               # 运动 |ω|≥0.12：禁 S2，Odin 仍进 Global
[pv-dual] ... skip=hold_*                   # 仅静止 display-hold（≤1s，不进 IEKF）
[pv-iekf] accept_s2 ... clip=N acc=A rej=0  # N=残差裁剪点数；rej>0 表示 S2 回滚
```


紧耦合 + 不拖影 + Odin 不消失 + 防漂：

1. Odin deskew：**静止且 `|end_dt|≤30ms` 可不去畸变；否则 gyro-only chase Mid-end**（禁 accel）。
2. **真紧耦合**：配对成功 → Odin 进 IEKF 残差 + **Global**（intensity≈200）。
3. **两段式紧耦合（MA-LIO + FAST_LIO_MULTI mid_odin）**：① Mid-only 锁位姿；② `|end_dt|≤50ms` 且 `|ω|<0.12` 时软 Odin（≤400 点，`cov×100`×范围²，`|残差|>5cm` 置零）；二段 `|Δp|>1.5cm` 回滚。运动 → `ok_map_gyr`；大 skew → `ok_map*`（Odin 仍全量进 Global）。**Mid 失败 → 回滚并禁 S2**。
4. **IEKF voxel 匹配面只写 Mid**；**Global = Mid + 全量 Odin**。**Mid `reject_s1` / 无有效点时禁止写 voxel+Global**；连续失败 ≥2 帧冻结到上一帧好位姿（防 IMU 空推漂）。
5. **免编译调参**：`dual_mid_odin_tune.py`。默认 **Global=静止 Mid 降采样**（[Point-LIO](https://github.com/hku-mars/Point-LIO) / [FAST_LIO_MULTI](https://github.com/engcang/FAST_LIO_MULTI) densify-when-still）；**Odin Global 关**（clip 后期~0.85）；**Body 不叠 Odin**（避免未对齐当漂）。`clip_max=0.40`；改 `.py`/yaml 后重启。
6. **防消失**：`drop_stale_mid` + Global 只体素化 pending；Mid/Odin `point_filter=2`，Odin `max_range2=20`。
7. **防拖影**：运动禁止 hold 进 IEKF/地图；仅静止 display-hold≤1s。
8. **存图异步**：`/save_map` 快照后后台写 PCD。
9. **远处地面**：单次经过更稀；`global_map_leaf_size=0.1`（过细 + 位姿飞了会 PCL overflow）。
10. 外参：**Ry≈-13.8° / T≈[0.132,-0.010,0.078]**，yaml RowMajor。

`skip=unpaired` 仍长期出现时看是否在 `drop_stale_mid`；异常则重启建图。

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
- 俯仰 **向上抬** → `Ry` 更负（如 -14° → -15°）
