# Mid360 + Odin1 融合：交给本机另一套 AI 的交接包

> 日期：2026-11-12  
> 用途：让**另一套本地 AI**在不通外网/不共享 Cursor 对话的情况下，仍能定位当前问题。  
> 详细历史见同目录 [`lidar_fusion_diagnosis.md`](./lidar_fusion_diagnosis.md)（§10f–10k）。

---

## 0. 你怎么把本文档喂给「本地 AI」

本地 AI 只能看到**它工作区里的文件**，看不到 Cursor 聊天记录。做法任选其一：

### 推荐 A：直接打开同一台机器上的目录（最省事）

在本地 AI（Codex / Continue / Aider / 另一 Cursor 窗口等）里把工作区设为下面之一：

```text
/home/ll/FAST_LIO_MULTI_ROS2_mid_odin
```

或更大一点（含诊断文档）：

```text
/home/ll/3dnavigation
```

然后把**本文件路径**贴进提示词，例如：

```text
请先完整阅读：
/home/ll/3dnavigation/docs/mid_odin_fusion_handoff_for_local_ai.md

再按文中「开放问题」做根因分析；先分析、先读代码，未确认前不要大改。
重点源码：
- src/laserMapping_adaptive.cpp
- src/IMU_Processing.hpp
- src/odin_raw_downsample_bridge.cpp
- src/odin_mid_densify*.cpp（或等价 densify）
- config/mid360_odin1.yaml
- launch/mid360_odin1_multi.launch.py
```

### 推荐 B：拷贝一份离线包到 U 盘 / 另一台电脑

```bash
mkdir -p /tmp/mid_odin_ai_pack
rsync -a --copy-links \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude '*.bag' --exclude '*.db3' --exclude '*.mcap' \
  --exclude '*.pcd' --exclude '*.ply' --exclude '*.onnx' \
  /home/ll/FAST_LIO_MULTI_ROS2_mid_odin/ \
  /tmp/mid_odin_ai_pack/FAST_LIO_MULTI_ROS2_mid_odin/

cp /home/ll/3dnavigation/docs/mid_odin_fusion_handoff_for_local_ai.md \
   /tmp/mid_odin_ai_pack/
cp /home/ll/3dnavigation/docs/lidar_fusion_diagnosis.md \
   /tmp/mid_odin_ai_pack/

# 再把 /tmp/mid_odin_ai_pack 拷到本地 AI 能读的盘
```

本地 AI 打开 `/tmp/mid_odin_ai_pack`（或拷贝后的路径），先读 handoff 文档。

### 不要只贴一段聊天摘要

只贴摘要会丢：topic 名、当前参数、已验证失败的方案、双路径（`/Laser_map` vs `/viz/accum_*`）。**至少带上本 handoff + yaml + 上述 4–5 个源文件。**

---

## 1. 系统一句话

Go2 上 **Livox Mid360**（主雷达 + IMU）与 **Odin1 dToF** 做多雷达紧耦合建图/定位。

| 角色 | 路径 / 环境 |
|------|-------------|
| 源码 | `/home/ll/FAST_LIO_MULTI_ROS2_mid_odin` |
| colcon ws | `/home/ll/fast_lio_multi_mid_odin_ws`（`src/FAST_LIO_MULTI_ROS2` → 上述源码软链） |
| 入口 | `ros2 launch fast_lio_multi mid360_odin1_multi.launch.py update_method:=adaptive ...` |
| 主节点 | `laserMapping_adaptive`（Bundle 紧耦合） |
| Odin 预处理 | `odin_raw_downsample_bridge`：`/odin1/cloud_raw` → `/odin1/cloud_l2_body` |
| 可视化累积 | `odin_mid_densify`：`/odin1/cloud_in_map`、`/viz/accum_odin`（**与 ikdtree 真相图独立**） |

**真相图**：`/Laser_map`（FAST-LIO ikdtree）  
**装饰/辅助累积**：`/viz/accum_odin`、`/viz/accum_fused`（keep-first voxel，错点难覆盖）

---

## 2. 当前运行参数（2026-11-12 晚，已上机）

启动日志典型一行：

```text
point_update=true pose_factor=false
max_sync_dt=0.055s soft_wait=25ms stamp_bias=65ms
odin_map_insert=true insert_max_dt=18ms still_dt=55ms@|w|<0.12
insert_gyr_gate=0.35rad/s cooldown=0s
odin_voxel≈0.06
```

含义：

- **紧耦合**：`lidar2_point_update_en=true`（Odin 点残差进 IEKF Stage2），`pose_factor=false`
- Mid **每帧都跑**；Odin 在 `| (L2+bias)−Mid | ≤ 55ms` 时挂上；否则 Mid-only + hold 重投影 viz
- Bridge 晚到：最多 **soft_wait 25ms**（不是等齐卡 0.5s）
- 写 `/Laser_map`：移动中 `|dt|≤18ms`；近静止 `|ω|<0.12` 放宽到 55ms；硬转同帧不写
- densify 累积 yaw 门限约 8°/s

实测（本轮）：

| 量 | 值 |
|----|-----|
| Bundle couple rate | ~98–99%（soft-wait 后） |
| `/odin1/cloud_l2_body` | 实际约 **6–8 Hz**，偶发空洞（不是稳 10Hz） |
| Mid `/cloud_registered` | ~10 Hz |
| 静止 Mid↔Odin NN p50 | ~12 cm 量级（仍偏大，外参/时间可能仍有残差） |

---

## 3. 用户仍感知的「主要问题」（开放）

按优先级：

### P0 — 转动时 Odin「先错后跳变 / 错位累积」

**现象**：转弯时 Odin 云或累积图先偏到错误位置，停下或下一帧又弹回；有时错误几何被写进累积图。

**已排除 / 已试过且不够的方案**（不要再原样重复）：

| 方案 | 结果 |
|------|------|
| 事后整帧 `Exp(-ω·bias)` 硬转 | 不够 / 引入别的问题 |
| 清零 Odin curvature 关 deskew | 转动拖影更重 → 仍跳变 |
| deskew **忽略** Bundle 残差、只中心对齐 | 仍有问题 |
| 事后质量门 / 长 cooldown 禁 Odin | 用户要求不以门控为主；且会让图更稀 |
| **等齐再融**（Mid 等到 L2） | Mid/Odin 卡住 **0.5–0.6s** → 更大跳变 |
| 仅加宽 couple 窗 + 全部 `map_insert` | couple 密了，但 **错点锁进 ikdtree** |

**当前缓解**（治标未完全治本）：

- Mid 不停 + soft-wait + 因果偏好配对
- deskew：`t = (L2 + stamp_bias − Mid) + offset`，IMU 去畸变
- 写图与 IEKF 解耦：IEKF 可到 55ms；写图移动中更严

**仍可疑的根因（请本地 AI 深挖）**：

1. **Odin 帧内 offset ~0–94ms** + USB/host stamp 与 Mid 不同源；`stamp_bias=65ms` 是否随负载漂移？
2. Bridge `align_l2_stamp_to_mid` 当前 launch 为 **false**；若改 true 又与 `stamp_bias` **双重校正**风险（历史曾踩坑）
3. 配对残差 ε 锯齿（0↔50ms）在转动时 `ω·Δε` 仍可能造成几何跳变
4. 双路径：`/Laser_map` insert vs `/viz/accum_odin` densify — 用户看到的「错」可能是橙色 accum，不一定是 ikdtree
5. 外参 `extrinsic_*_L2_wrt_L1` 静止 NN~12cm，说明几何标定可能仍有偏差，转动时被放大

### P1 — 累积图密度 vs 正确性的权衡

- 写图过严 → 用户报「太稀疏」
- 写图过松 → 用户报「错位累积」
- 当前折中：静止放宽、运动收紧；densify 8°/s + voxel 0.06  
- **开放**：是否应用更好的「可撤销累积」（非 keep-first），或仅在 still 且 `|dt|` 小且 NN 好时写入

### P2 — Odin 源频率不稳

- 硬件/驱动侧约 6–8Hz + 空洞 → 软件无法对「没出的云」做真紧耦合
- soft-wait 只能捞「晚到」，捞不到「没发」

---

## 4. 关键代码入口（读这里）

| 文件 | 看什么 |
|------|--------|
| `src/laserMapping_adaptive.cpp` | `sync_packages_bundle`（配对/soft-wait/Mid-only）、`map_incremental`（Odin 写图门）、Stage1/2 IEKF、`publish_odin_from_undistorted`（hold viz） |
| `src/IMU_Processing.hpp` | `UndistortPclMultiLiDAR` Odin deskew、`lidar2_stamp_bias_` |
| `src/odin_raw_downsample_bridge.cpp` | L2 stamp、`densify_mid_lag_comp_sec`、`align_l2_stamp_to_mid` |
| densify / `odin_mid_densify` | `/viz/accum_odin` keep-first、still 门限 |
| `config/mid360_odin1.yaml` | 上述全部参数真源 |
| `launch/mid360_odin1_multi.launch.py` | bridge/densify/fuse 是否启用、voxel |

**关键 topic**

```text
/livox/lidar, /livox/imu
/odin1/cloud_raw → /odin1/cloud_l2_body
/Odometry, /cloud_registered, /cloud_odin_registered
/Laser_map
/odin1/cloud_in_map, /viz/accum_odin, /viz/accum_fused
```

**关键 frame**：`map`、Mid lidar / IMU、Odin L2 相对 L1 外参（yaml）

---

## 5. 给本地 AI 的任务书（可复制）

```text
背景：ROS2 Humble，FAST_LIO_MULTI Mid360+Odin1 紧耦合。主诉是转动时 Odin 几何「先错后跳变」，以及累积图稀疏/错位的权衡。

请：
1) 只读分析 laserMapping_adaptive Bundle 同步、IMU deskew、map_insert、bridge stamp 四条链是否时间语义一致。
2) 明确区分 /Laser_map 与 /viz/accum_odin 两条累积路径，判断用户「错位累积」更可能来自哪条。
3) 提出不超过 2 个可验证假说；每个假说给出：改哪些文件、如何用 ros2 topic/日志验证、预期数值。
4) 禁止：等齐卡住 Mid、长 cooldown 作为主方案、同时打开 align_l2_stamp_to_mid 与非零 stamp_bias 而不论证。
5) 先报告结论再改代码；改动保持最小。

交接文档：docs/mid_odin_fusion_handoff_for_local_ai.md
长诊断：docs/lidar_fusion_diagnosis.md §10f–10k
```

---

## 6. 验证命令（本机）

```bash
# 工作空间
source /opt/ros/humble/setup.bash
source /home/ll/fast_lio_multi_mid_odin_ws/install/setup.bash

# 启动（示例；RViz 可另开）
ros2 launch fast_lio_multi mid360_odin1_multi.launch.py \
  update_method:=adaptive \
  enable_nav_bridge:=false \
  enable_color_viz:=false \
  odin_voxel:=0.06 \
  rviz:=false

# 看紧耦合与写图参数是否加载
# 日志应含：point_update=true soft_wait=… insert_max_dt=… still_dt=…

# 频率
ros2 topic hz /cloud_registered
ros2 topic hz /odin1/cloud_l2_body
ros2 topic hz /cloud_odin_registered

# 清空 densify 累积（若服务在）
ros2 service call /odin_independent_accum/clear_accum std_srvs/srv/Empty {}
ros2 service call /mid_odin_frame_fuse/clear_accum std_srvs/srv/Empty {}
```

编译：

```bash
cd /home/ll/fast_lio_multi_mid_odin_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select fast_lio_multi
```

---

## 7. Git / 备份

- 私有备份仓：https://github.com/fanqie-2580/3dnavigation  
- 上传目录：`/home/ll/3dnavigation_github_upload/FAST_LIO_MULTI_ROS2_mid_odin`  
- 近期相关提交主题：wait-align → Mid-always+hold → soft-wait → insert_max_dt → still 放宽写图  

---

## 8. 给本地 AI 的硬约束（与本工作区一致）

- 默认 ROS 2 Humble；不要改无关工程（autoware、unitree 大仓等）
- 不要改 `build/`、`install/`、bag、地图大文件
- 不要随意改 topic/frame 名
- Mid360 与 Odin1 导航树是物理分叉的；本问题只在 `FAST_LIO_MULTI_ROS2_mid_odin`
- 先分析后改；小步修改并说明如何验证
