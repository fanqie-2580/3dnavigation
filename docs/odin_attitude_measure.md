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

投影链（timed project，实机重力+ICP 姿态版）：

```text
p_map = T_map_body * T_body_L1 * T_L1_L2 * p_odin
```

`use_yaw_only_body=false`；`T_L1_L2` 平移用 CAD，旋转用实机重力+ICP：  
`rpy≈[-0.28°, -10.15°, -0.13°]`（原先 CAD 补偿约 `Ry(-13°)`）。

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

## 2. 相对 Go2 机身测姿态——实现说明（文字详解）

这一节说明：**不拆机器、不用 Mid、不用 CAD 旋转**，怎样用实机数据得到  
「Odin 坐标系相对 Go2 机身（`base_link`）的姿态」。

### 2.1 要测的是什么

刚体安装下，Odin 相对机身有固定外参 \(T^{\text{base}}_{\text{odin}}=\{R,t\}\)。

- **平移 \(t\)**：Odin 原点在 `base_link` 下的位置（前/左/上各多少米）。静止时两个 IMU **测不出来**，仍靠 CAD 或卷尺。
- **旋转 \(R\)**：Odin 三轴相对 `base_link` 三轴怎么转。静止时可以用 **两边加速度计看到的重力方向** 估出其中 **2 个自由度**（滚转、俯仰），**绕竖直轴的航向（yaw）估不出来**。

脚本 `--ref go2` 做的就是估这个 \(R\)（更准确说：与重力对齐的那部分）。

机身参考轴取 **Go2 机身 IMU**。Unitree URDF 里：

```text
base → imu : xyz = [-0.02557, 0, 0.04232], rpy = [0,0,0]
```

即 IMU 相对 `base_link` **只有平移、没有旋转**。因此「相对机身 IMU 的姿态」=「相对 `base_link` 的姿态」。  
Odin 侧官方 `T_{\text{imu}\leftarrow L}` 的旋转也是单位阵，点云系 L 与 Odin IMU 姿态一致，测到的 \(R\) 可同时当作相对 Odin 雷达系。

### 2.2 物理依据：静止时加速度计 = 重力在本体下的投影

机器人静置时，每个 IMU 的加速度计主要测的是重力（外加一点偏置/噪声）。  
设世界系里重力竖直向下，则：

- 在 Go2 机身 IMU 坐标系里，测得向量 \(\mathbf{a}_{\text{go2}}\)；
- 在 Odin IMU 坐标系里，测得向量 \(\mathbf{a}_{\text{odin}}\)。

二者是 **同一条物理重力**，只是表达在两个固联坐标系里。  
若 \(R\) 表示「把 Odin 系下的向量变到 Go2 系」（即 \(p_{\text{go2}} = R\, p_{\text{odin}}\)），则应有：

\[
R\,\mathbf{a}_{\text{odin}} \;\approx\; \mathbf{a}_{\text{go2}}
\]

（两边都先单位化成方向，与加速度是「g」还是「m/s²」无关。）

绕重力轴再转任意角度，重力方向不变，所以 **仅靠重力无法约束相对 yaw**。算法求的是满足上式的 **最小旋转**（转角最小的那一个），等价于：只对齐两个重力向量，不引入额外绕竖直轴的转动。

### 2.3 数据从哪来

| 侧 | 话题 / 字段 | 用法 |
|----|-------------|------|
| Go2 | `lf/lowstate`（或 `lowstate`）→ `imu_state.accelerometer` | 机身 IMU 三轴加速度，若干秒取平均 |
| Odin | `/odin1/imu` → `linear_acceleration` | Odin IMU 加速度，同样平均 |

注意：

1. `/odin1/imu` 的 **orientation 四元数经常是假的（单位阵）**，不能用来比姿态；只用加速度（或 `/odin1/odometry` 的姿态作对照）。
2. 订阅 Go2 状态需要宇树 ROS2/DDS 环境（`source unitree_ros2/setup.sh`，网卡指向 `192.168.123.x`）。
3. 同一进程里还要能收到 `/odin1/imu`（Odin TCP 桥已在跑，且 ROS_DOMAIN 一致）。

实现上脚本会连续采集约 2～3 秒样本，对 Go2、Odin 加速度分别做 **时间平均**，压低抖动后再算 \(R\)。

### 2.4 算法步骤（脚本里实际做的事）

1. **采集**：在静止条件下缓冲 `acc_go2[k]`、`acc_odin[k]`。  
2. **平均**：\(\bar{\mathbf{a}}_{\text{go2}}=\mathrm{mean}(\cdot)\)，\(\bar{\mathbf{a}}_{\text{odin}}=\mathrm{mean}(\cdot)\)。  
3. **单位化**：\(\hat{g}_{\text{go2}}=\bar{a}_{\text{go2}}/\|\cdot\|\)，\(\hat{g}_{\text{odin}}\) 同理。  
4. **最小旋转（Rodrigues / 叉积公式）**：求 \(R\) 使 \(R\hat{g}_{\text{odin}}=\hat{g}_{\text{go2}}\)，且转角最小：
   - \(\mathbf{v}=\hat{g}_{\text{odin}}\times\hat{g}_{\text{go2}}\)，\(c=\hat{g}_{\text{odin}}\cdot\hat{g}_{\text{go2}}\)；
   - 用反对称矩阵 \([\mathbf{v}]_\times\) 构造  
     \(R=I+[\mathbf{v}]_\times+[\mathbf{v}]_\times^2\dfrac{1}{1+c}\)（共线时单独处理）。  
5. **输出**：打印 \(R\)、对应 RPY、以及 \(\|R-I\|\) 的转角。  
   - 若接近单位阵：Odin 与机身轴近似平行（前装、Z 朝上时的预期）。  
   - 若有明显 pitch/roll：说明安装相对机身有俯仰/横滚。

同时可读 `lowstate.imu_state.rpy` 与 `/odin1/odometry` 的 RPY，作 **各自相对重力的绝对姿态** 对照；两边 yaw 零点独立，**不要直接相减当外参 yaw**。

### 2.5 和 CAD、和融合外参的关系

- **对照 CAD**：CAD 量的 `T_base_odin` 里旋转若为 \(I\)，则实机 \(\|R-I\|\) 也应很小；若 CAD 写成绕 Y 转 180° 而实机接近 \(I\)，说明 CAD 轴建反了（此前已出现过）。  
- **写入 FAST-LIO 的 L2**：融合链是 Mid 雷达 ← Odin，不是 base ← Odin。已知官方「Mid 在 Go2 机身 IMU 下 Ry(+13°)」时，可换算：

\[
R^{\text{Mid}}_{\text{Odin}}
= \bigl(R^{\text{Go2}}_{\text{Mid}}\bigr)^{\mathsf T} R^{\text{Go2}}_{\text{Odin}}
= R_y(-13^\circ)\, R^{\text{Go2}}_{\text{Odin}}
\]

脚本在 `--ref go2` 时会多打印这一版，便于粘到 `extrinsic_R_L2_wrt_L1`；是否采用取决于你是否信任官方 Mid 安装角。更稳的 L2 仍建议用 `--ref mid` 的重力 + 点云 ICP。

### 2.6 做不到什么（避免误解）

1. **求不出 \(t\)**（前后左右上下偏移）。  
2. **求不出相对 yaw**（机头偏左/偏右安装）。要 yaw 需视觉/点云相对机身标志，或相对 Mid 做 ICP。  
3. **狗在动时** 加速度含线加速度，重力方向被污染，结果会偏；务必静止。  
4. 趴下与站立时若机身相对水平姿态不同，测的是「当前机身姿态下的相对安装」，刚体外参 \(R\) 本身不变，但两边绝对 RPY 会变——所以用 **相对 \(R\)**，不要只看某一侧的绝对 pitch。

### 2.7 操作命令

```bash
source /home/ll/unitree_ros2/setup.sh
# 同时保证 /odin1/imu 可订阅
python3 /home/ll/3dnavigation/docs/scripts/measure_odin_attitude.py --ref go2 --mode gravity
```

### 2.8 判读要点

- `|R_go2_odin - I|` 很小 → 相对机身近似同向（与前装 CAD `R=I` 一致）  
- 脚本会附带打印经官方 Mid Ry(+13°) 换算的 L2 建议 R  
- **平移** 仍用 CAD/卷尺，本方法不管平移  

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
