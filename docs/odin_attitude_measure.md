# Odin1 姿态实机测量（相对 Mid360 / 相对 Go2）

更新：2026-11-06  

目的：不依赖 CAD 旋转，用实机测 Odin 姿态。

| 参考系 `--ref` | 测什么 | 典型用途 |
|----------------|--------|----------|
| **`mid`**（默认） | Mid IMU / Mid 雷达 ← Odin | 直接写 `extrinsic_R_L2_wrt_L1` |
| **`go2`** | Go2 机身 IMU / `base_link` ← Odin | 对照 CAD `T_base_odin` 的 **R**；可选再换算成 L2 |

平移仍建议 CAD/卷尺；**静止 IMU 测不全平移，重力法也测不准相对 yaw**。

---

## 0. 坐标系约定

| 符号 | 含义 |
|------|------|
| Go2 `base_link` / 机身 IMU | URDF：`t=[-0.02557,0,0.04232]`，**R=I**（姿态相同） |
| Mid IMU / `body` | FAST-LIO body（≈ Mid360 板载 IMU） |
| Mid L1 | Mid360 雷达系（与 Mid IMU 外参近似 R=I） |
| Odin L2 / Odin IMU | `/odin1/cloud_raw`；官方 `T_imu_L` 的 R=I |
| `map` | Mid LIO 地图系 |

投影链（timed project）：

```text
p_map = T_map_body * T_body_L1 * T_L1_L2 * p_odin
```

其中 `T_L1_L2` 的旋转即要测的 `extrinsic_R_L2_wrt_L1`。

官方 Mid 相对 Go2 机身 IMU 约有 **Ry(+13°)** 安装角；Odin 近水平时，L2 旋转大致接近 **Ry(-13°)**。实机再用重力 / ICP 精修。

---

## 1. 前置条件

**共用**

1. Odin 桥：`/odin1/imu`（加速度）、`/odin1/odometry`  
2. 机器人尽量 **静止**  
3. ROS 2 Humble + Cyclone  

**`--ref mid` 额外**

- Mid：`/livox/imu`  
- 融合：`/Odometry`、`/cloud_registered`、`/odin1/cloud_in_map`（ICP 需要）

**`--ref go2` 额外**

- 能收到宇树 `lf/lowstate`（需 `source /home/ll/unitree_ros2/setup.sh`，网卡指向机器人网段）

```bash
# mid 参考
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source /home/ll/fast_lio_multi_ws/install/setup.bash

# go2 参考（另开环境；会设置 CYCLONEDDS_URI 到 enP8p1s0）
source /home/ll/unitree_ros2/setup.sh
# 仍要能订阅 /odin1/imu：可与 mid 同一 ROS_DOMAIN，或先保证 Odin client 已在跑
```

---

## 2. 相对 Go2 测姿态（`--ref go2`）

### 原理

Go2 `lf/lowstate.imu_state.accelerometer` 与 Odin `/odin1/imu` 加速度，求：

```text
R_go2_odin :  R * g_odin ≈ g_go2
```

因 URDF 里 Go2 IMU 相对 `base_link` **R=I**，该 R 即 **Odin IMU（≈L）相对 base 的姿态**（yaw-free）。

再用宇树官方 Mid 在 Go2 IMU 下的 **Ry(+13°)** 换算到 Mid←Odin：

```text
R_L1_L2 ≈ Ry(-13°) · R_go2_odin     # 即 R_go2_mid^T @ R_go2_odin
```

### 操作

```bash
source /home/ll/unitree_ros2/setup.sh
# Odin /odin1/imu 需在同一 ROS 域可见
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref go2 --mode gravity
```

### 判读

- `|R_go2_odin - I|` 很小 → Odin 相对机身接近水平同向（与前装 CAD R=I 一致）  
- 脚本会额外打印建议的 `extrinsic_R_L2_wrt_L1`（经官方 Mid 安装换算）  
- **测不到** 相对 base 的完整平移；平移继续用 CAD/卷尺  

---

## 3. 相对 Mid 测姿态（`--ref mid`，默认）

### 步骤 A：重力对齐（俯仰/滚转，无 yaw）

两路加速度计：

```text
R * g_odin ≈ g_mid
```

得到 yaw-free 的 `R_mid←odin`。Mid 板载外参 R≈I 时，可作 `R_L1_L2` 初值。

```bash
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref mid --mode gravity
```

- 典型：pitch ≈ **-13°**（抵消 Mid 官方安装俯仰）  
- Mid 加速度可能是 **g**，Odin 是 **m/s²**；只比方向即可  

---

### 步骤 B：点云 ICP 残差（含小 yaw，仅 `--ref mid`）

当前系统已用某套 `T_L1_L2` 把 Odin 投到 `map`。在同一 `map` 下 ICP：

```text
p_mid ≈ T_δ * p_odin_in_map
R_L2_new ≈ R_δ * R_L2_old
```

```bash
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref mid --mode icp \
  --l2-rpy-deg 0 -10.15 0

python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref mid --mode both
```

- `|R_δ|` 很小：当前姿态已够好  
- 中位最近邻距离：静止有结构时通常 &lt; 0.15 m  

> Go2 真值系下没有与 Mid `map` 直接对齐的点云链，故 **`--ref go2` 不做 ICP**。

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
# 相对 Go2（机身 IMU / base 姿态）
source /home/ll/unitree_ros2/setup.sh
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref go2 --mode gravity

# 相对 Mid：重力 / ICP / 两者
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref mid --mode gravity
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref mid --mode icp
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref mid --mode both \
  --l2-rpy-deg 0 -10.15 0
```

脚本：[`docs/scripts/measure_odin_attitude.py`](./scripts/measure_odin_attitude.py)

---

## 6. 注意

1. **Yaw**：重力法测不准相对航向；`--ref mid` 时用 ICP 补小 yaw。  
2. **Odin `/odin1/imu` orientation** 可能恒为单位四元数，用 **加速度** 或 **`/odin1/odometry`**。  
3. Mid / Go2 / Odin 加速度 **单位可能不同**，只比方向。  
4. ICP 需静止 + 墙地结构；空旷会漂。  
5. 改 R 后必须重启 `odin_mid_timed_project`。  
6. `--ref go2` 给的是相对 **机身** 的姿态；融合 yaml 里 L2 仍是 **Mid←Odin**（脚本可按官方 Mid Ry+13° 自动换算一版）。  
7. **平移** 相对 Go2 仍靠 CAD/卷尺，静止双 IMU 求不出来。
