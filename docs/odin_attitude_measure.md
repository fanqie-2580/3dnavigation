# Odin1 相对 Mid360 姿态实机测量（步骤 + 代码）

更新：2026-11-06  

目的：不依赖 CAD 旋转，用实机 **IMU 重力** + **点云 ICP** 估计  
`extrinsic_R_L2_wrt_L1`（Mid 雷达系 → Odin 雷达系的旋转），用于 `odin_mid_timed_project` / `mid360_odin1.yaml`。

平移 `extrinsic_T_L2_wrt_L1` 可继续用 CAD/卷尺；本文只测 **姿态 R**。

---

## 0. 坐标系约定

| 符号 | 含义 |
|------|------|
| Mid IMU / `body` | FAST-LIO 的 body（≈ Mid360 板载 IMU） |
| Mid L1 | Mid360 雷达系（与 Mid IMU 外参近似 R=I） |
| Odin L2 | `/odin1/cloud_raw` 点云系（官方 L，重力近似水平） |
| `map` | Mid LIO 地图系 |

投影链（timed project）：

```text
p_map = T_map_body * T_body_L1 * T_L1_L2 * p_odin
```

其中 `T_L1_L2` 的旋转即要测的 `extrinsic_R_L2_wrt_L1`。

官方 Mid 相对 Go2 机身 IMU 约有 **Ry(+13°)** 安装角；Odin 近水平时，L2 旋转大致接近 **Ry(-13°)**。实机再用重力 / ICP 精修。

---

## 1. 前置条件

1. Mid 驱动在跑：`/livox/imu`、`/livox/lidar`  
2. Odin 桥在跑：`/odin1/imu`、`/odin1/cloud_raw`、`/odin1/odometry`  
3. 融合在跑：`/Odometry`、`/cloud_registered`、`/odin1/cloud_in_map`  
4. 机器人尽量 **静止** 站立（或趴稳），周围有墙/地面，便于 ICP  
5. 环境：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /home/ll/fast_lio_multi_ws/install/setup.bash
```

检查话题：

```bash
ros2 topic hz /livox/imu
ros2 topic hz /odin1/imu
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered
ros2 topic hz /odin1/cloud_in_map
```

---

## 2. 步骤 A：重力对齐测姿态（俯仰/滚转，无 yaw）

### 原理

两路加速度计在静止时都测重力方向。求最小旋转 `R`，使：

```text
R * g_odin ≈ g_mid
```

得到 **无绕竖直轴自由度的** `R_mid←odin`（yaw-free）。  
因 Mid 板载 `T_body_L1` 的 R≈I，可近似当作 `R_L1_L2` 的俯仰/滚转初值。

### 操作

```bash
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --mode gravity
```

### 判读

- 输出 `rpy_deg`：相对 Mid IMU 的 roll / pitch / yaw（yaw 应接近 0，因算法消掉了绕重力轴）  
- 典型结果：pitch ≈ **-13°**（补偿 Mid 官方安装俯仰）  
- Mid IMU 加速度有时是 **g 为单位**（范数≈1），Odin 多为 **m/s²**（范数≈9.8）；脚本用方向，不受量纲影响  

---

## 3. 步骤 B：点云 ICP 测残差姿态（含小 yaw）

### 原理

当前系统已用某套 `T_L1_L2` 把 Odin 投到 `map`（`/odin1/cloud_in_map`）。  
在 **同一 `map` 系** 下，把 Odin 点云刚性配准到 Mid 的 `/cloud_registered`，得到残差 `T_δ`：

```text
p_mid ≈ T_δ * p_odin_in_map
```

若投影模型为 `p_map = T_map_body * T_body_L1 * T_L1_L2 * p_odin`，且测量瞬间 body 姿态可视为已知，则新的 L2 旋转近似：

```text
R_L2_new ≈ R_δ * R_L2_old
```

（平移残差可另文处理；改姿态时通常先只更新 R。）

### 操作

需融合已在跑，且 `cloud_in_map` 使用 **当前**（待修正）外参：

```bash
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --mode icp \
  --l2-rpy-deg 0 -13 0
```

`--l2-rpy-deg` 填 **当前 yaml 里 L2 的 roll pitch yaw（度）**；脚本会打印 `R_δ @ R_L2_old`。

或直接：

```bash
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --mode both
```

会先做重力，再做 ICP，并给出建议写入的 `extrinsic_R_L2_wrt_L1` 扁平 9 个数。

### 判读

- `|R_δ|` 很小（例如 &lt; 1°）：当前姿态已够好  
- pitch 残差几度：把 `R_L2_new` 写回 yaml/launch  
- 中位最近邻距离：越小越好（静止、有结构时通常 &lt; 0.15 m）  

---

## 4. 写回系统

编辑：

- `FAST_LIO_MULTI_ROS2_mid_odin/config/mid360_odin1.yaml`  
  - `mapping.extrinsic_R_L2_wrt_L1`  
- `launch/mid360_odin1_multi.launch.py`  
  - `odin_mid_timed_project`（及可选 `odin_independent_accum`）里同名参数  

建议：`use_yaw_only_body: false`（L2 已含相对 Mid 的俯仰时）。  

同步到 `fast_lio_multi_ws` 的 install/share 后重启 launch。

本次（2026-11-06）实机结果示例：

| 方法 | rpy (deg) |
|------|-----------|
| 重力 | ≈ [-1.28, -13.01, -0.09] |
| CAD Ry(-13°) | [0, -13, 0] |
| 重力+ICP 合成 | ≈ **[-0.28, -10.15, -0.13]** ← 已写入 |

---

## 5. 完整测量脚本

路径：[`docs/scripts/measure_odin_attitude.py`](./scripts/measure_odin_attitude.py)

```bash
# 仅重力
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --mode gravity

# 仅 ICP（默认认为当前 L2 = Ry(-13°)）
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --mode icp

# 重力 + ICP，并打印可粘贴的 yaml 行
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --mode both \
  --l2-rpy-deg 0 -13 0
```

脚本源码见同目录 `measure_odin_attitude.py`（可直接运行，无额外 pip 依赖，需已 source ROS 2）。

---

## 6. 注意

1. **Yaw**：双 IMU 重力法测不准相对航向；需要墙面结构时用 ICP。  
2. **Odin `/odin1/imu` 的 orientation** 可能恒为单位四元数，姿态请用 **加速度** 或 **`/odin1/odometry`**。  
3. Mid `/livox/imu` 与 Odin 加速度 **单位可能不同**，只比方向。  
4. ICP 在空旷/动态场景会漂；静止 + 墙地结构更稳。  
5. 改 R 后必须重启 `odin_mid_timed_project`（参数在节点启动时固化）。  
6. 本文测的是 **Mid↔Odin**，不是 Go2 官方 IMU↔Odin（那是另一条 CAD 链）。
