# Odin1 + Mid-360 融合点云滞后诊断报告

- 日期：2026-11-11（初稿） / **2026-11-12 上电复测**
- 约束：只读诊断，未改工程代码/配置/系统时钟（复测仅启动驱动+融合进程采数）
- 主机：Jetson Orin NX，Ubuntu 22.04.5 LTS，ROS 2 Humble
- 本机 shell/`bashrc`：`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（初稿曾写 fastrtps，以复测为准）
- `use_sim_time`：`False`；NTP 已同步（chrony stratum 3）；未见 Mid–Odin 硬件 PTP 共时

---

## 0. 2026-11-12 上电复测结论（优先读）

### 0.1 一句话

**融合后 Odin 拖影/滞后的主因是 Python `odin_raw_downsample_bridge` 单核打满（~100% CPU），同 stamp 从 `/odin1/cloud_raw` 到 `/odin1/cloud_l2_body` 墙钟管道延迟中位约 2.03 s；Mid 主链正常（`/cloud_registered` delay ~45 ms）。**

### 0.2 驱动-only 时戳（融合未开 / C++ 与 `ros2 topic delay`）

| 话题 | 方法 | delay / lag | 频率 |
|------|------|-------------|------|
| `/livox/imu` | `ros2 topic delay` | **~1 ms** | ~200 Hz |
| `/livox/lidar` | 临时 C++ 订阅（RELIABLE） | **med 135 ms**（p95~141） | 10 Hz；点 `offset_time` 跨度 ~100 ms |
| `/odin1/imu` | `ros2 topic delay` | **~11 ms** | ~300–400 Hz |
| `/odin1/cloud_raw` | `ros2 topic delay` | **~146 ms** | ~10 Hz |
| `/odin1/cloud_slam` | 同上 | **~140 ms** | ~10 Hz |
| `/odin1/odometry` | 同上 | **~121 ms** | ~10 Hz |

跨传感器（墙钟近邻配对）：

| 对比 | stamp 差 med |
|------|----------------|
| Mid IMU − Odin IMU | **~+7.5 ms**（mode 2 下轴对齐良好） |
| Mid IMU − Odin raw | ~+142 ms（与 Odin 云 header age 同量级） |

**注意**：用 Python 订 `/livox/lidar` CustomMsg 会严重积压，假性 lag 可达数秒；**不可信**。IMU/小消息与 C++/`topic delay` 为准。

### 0.3 `use_host_ros_time: 2` 语义更正

源码 `host_sdk_sample.h`：

| 值 | 含义 |
|----|------|
| 0 | Odin 内部传感器时间（开机轴） |
| 1 | **主机 now**（接收瞬间打戳） |
| 2 | **传感器时间 − 平滑 PTP offset** → 映到主机时间轴（**不是**“接收时刻 now”） |

yaml 注释“align to host so Mid stamps match”方向对，但初稿写“host 接收时刻”不准确；mode 1 才是 receive-now。

### 0.4 Odin `offset_time`（`/odin1/cloud_raw`）

- 字段存在，dtype=FLOAT32，单位 **秒**
- 单帧约 49152 点：nonzero **~95.8%**，min≈0，max≈**0.094 s**
- **点序不按时间**：`first=0`、`last=0`，`argmax` 在中后部 → 未排序时用 `points.back().curvature` 会得到 0 扫尾；Adaptive 里对 L2 **有 sort**，此项为次要风险

### 0.5 融合开启后（`mid360_odin1_multi.launch.py` adaptive，`enable_nav_bridge:=false`）

| 话题 | `ros2 topic delay` | hz |
|------|--------------------|----|
| `/odin1/cloud_raw` | ~0.14 s | ~10 |
| `/odin1/cloud_l2_body` | **~2.19 s** | ~6.7 |
| `/odin1/cloud_in_map` | **~2.4 s** | ~6.7 |
| `/odin1/l2_mid_pose` | **~2.27 s**（与 L2 同 stamp） | ~6.7 |
| `/cloud_registered` | **~0.045 s** | ~10 |
| `/Odometry` | **~0.04–0.05 s** | ~10 |

同 stamp 管道（raw 到达墙钟 → 同 stamp 的 L2 到达）：

- **med 2028 ms**，p95 ~2135 ms（n=51）
- `odin_raw_downsample_bridge`：**~101% CPU**
- `odin_independent_accum`：~53% CPU
- `laserMapping_adaptive`：~20% CPU

`l2_stamp − raw_stamp`（按到达墙钟近邻）med ≈ **−2047 ms**：L2 **保留**了约 2 s 前的几何戳，不是“又 restamp 错成 Mid”，而是 **bridge 处理队列落后 2 s**。

densify 日志：`l2_pose_hit` 有命中；`l2_match=stamp:〜ms`；`project_with_mid=True`。Mid 位姿投影叠在 **已落后 2 s 的 Odin 几何** 上 → 运动时必拖影。

### 0.6 二进制 / 脚本时效

| 产物 | mtime |
|------|-------|
| 运行中 `laserMapping_adaptive`（build） | **2026-11-11 19:37**（install 软链；文件源更早可追到 11-05） |
| 源码 `laserMapping_adaptive.cpp` | **2026-11-11 20:49** → **仍旧于源码** |
| bridge/accum 脚本 | install 软链 → 源码树（与 11-11 晚间脚本一致） |

### 0.7 对拖影问题的修订优先级

1. **P0（新确认）**：降低 / 改写 `odin_raw_downsample_bridge` 热路径（C++、减点、latest-only 丢旧帧、降 voxel 负担），先把 raw→L2 墙钟延迟压到 ≪100 ms  
2. **P0（仍成立）**：RViz 看的是 `/odin1/cloud_in_map` densify，不是 raw  
3. **P1**：外参与 densify 时延标定（在 bridge 不再 2 s 落后后再做，否则被淹没）  
4. **P1**：重建 `laserMapping_adaptive` 使 C++ 改动生效  
5. **P2**：Odin 点云未按时间排序时的扫尾；mode 0/1/2 细调（当前 mode 2 与 Mid IMU 已 ~10 ms 级对齐，**不是**本轮 2 s 滞后主因）

### 0.8 2026-11-12 修复后复测（已改代码）

改动：`odin_raw_downsample_bridge` 预抽稀 + 加速解包/体素、cloud QoS depth=1、launch `max_points=8000`/`pre_decimate_max=16000`/`voxel=0.10`；`odin_independent_accum` cloud depth=1；重建 `laserMapping_adaptive`。

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| bridge `proc_ms` | ~2000（隐含） | **~45–52** |
| raw→L2 同 stamp 墙钟 | med **2028 ms** | med **~60–80 ms**（量级） |
| `/odin1/cloud_l2_body` delay | **~2.19 s** | **~0.22 s** |
| `/odin1/cloud_in_map` delay | **~2.4 s** | **~0.22 s** |
| `/cloud_registered` delay | ~0.045 s | ~0.05 s（不变） |

---

## 1. 实际运行架构

| 项 | 实测值 |
|----|--------|
| 源码目录 | `/home/ll/FAST_LIO_MULTI_ROS2_mid_odin` |
| colcon ws | `/home/ll/fast_lio_multi_mid_odin_ws`（`src/FAST_LIO_MULTI_ROS2` → 上述源码软链） |
| 融合可执行文件 | `.../install/.../laserMapping_adaptive` → build 产物 |
| 融合模式 | **Adaptive**（launch `update_method:=adaptive`） |
| Mid 驱动 | `livox_ros_driver2`（`msg_MID360_launch.py`，`xfer_format=1` CustomMsg） |
| Odin 驱动 | `odin_ros_driver` `host_sdk_sample`（USB） |
| Odin 时间模式 | `use_host_ros_time: 2`（`control_command.yaml`） |
| Odin 地图模式 | `custom_map_mode: 1`（SLAM） |
| IEKF IMU | **仅** `/livox/imu`（Mid），不用 Odin IMU 做状态传播 |
| Mid 输入 | `/livox/lidar`（CustomMsg）、`/livox/imu` |
| Odin→融合 L2 | `/odin1/cloud_raw` → `odin_raw_downsample_bridge` → `/odin1/cloud_l2_body` |
| RViz 橙色 Odin | `/odin1/cloud_in_map`（`odin_independent_accum`，`project_with_mid=true`） |
| RViz 青色 Mid | `/cloud_registered`（`laserMapping_multi`） |

### 源码 vs 正在跑的二进制（重要）

| 文件 | mtime |
|------|-------|
| 运行中 `laserMapping_adaptive`（build） | **2026-11-11 19:37** |
| 源码 `laserMapping_adaptive.cpp` | **2026-11-11 20:49** |

→ **C++ 映射器落后于源码约 1 小时**；Python densify/bridge 为 symlink，与源码一致。  
存在“改了源码、跑的仍是旧 binary”的风险（至少 TF EMA 等 C++ 改动未进当前进程）。

### 旁路节点堆积（实测）

`pgrep -c -f multi_lio_nav_bridge` → **8**。launch 默认 `enable_nav_bridge:=true`，多次冷启未清干净，可能加重调度延迟（**尚未量化**对帧率的影响）。

---

## 2. 已确认的事实

1. **单独驱动时点云正常、一开融合才“慢/拖”**：融合后 RViz 看的 Odin 主显示是 **`/odin1/cloud_in_map`（Mid 位姿重投影 densify）**，不是裸 `/odin1/cloud_raw`。
2. **Adaptive 使用两套独立缓冲**：`lidar_buffer`/`time_buffer`（Mid）与 `lidar_buffer2`/`time_buffer2`（Odin），**不是**原版“单队列混入两雷达”那种完全共享队列。
3. **当前 IEKF 对 Odin 点残差关闭**：`lidar2_point_update_en: false`，`lidar2_pose_factor_en: false`；Async 路径刻意 **不把 Odin-only 包推进 IEKF**（会丢弃 L2-only，避免用 L2 时间推 Mid）。
4. **状态传播只用 Mid IMU**：`imu_topic: /livox/imu`。
5. **时间戳语义**：
   - Mid：`header.stamp` ← Livox 包内设备时间（`base_time` / IMU stamp）。
   - Odin USB mode 2：传感器时间经 PTP offset 映到主机轴（**非** receive-now；mode 1 才是）。
   - 2026-11-12：Mid↔Odin IMU stamp 差 med ~7.5 ms → **轴已对齐**；驱动-only 云 age Mid~135 ms / Odin~146 ms。
6. **本会话早先实测（11-11；部分被 Python CustomMsg 积压放大）**：
   - 曾报 Mid lidar age ~0.9–1.2 s（**不可靠**）；复测 C++ 为 ~0.14 s。
   - 曾用 `cloud_slam` 反投影做 L2 时，`r_p50≈4.2 m`，而 raw `r_p50≈1.1–1.3 m` → **反投影几何错误**（已切 raw）。
7. **densify 链路额外耗时（11-12 定量）**：bridge **~2.0 s 同 stamp 墙钟管道延迟**，进程 ~100% CPU；accum ~53% CPU。
8. **11-11 诊断窗口末尾**：bridge `cloud=` 曾卡死（驱动停更）；**11-12 复测驱动与 bridge 计数连续递增**，卡死未复现。
9. **本机 Livox `/livox/lidar` QoS 为 RELIABLE**。用 BEST_EFFORT 订阅会收不到数。

---

## 3. 发现的问题（按严重度）

### P0 — RViz“Odin 滞后”主要发生在 densify 投影链路，而非 Mid IEKF 吃 Odin 点

- **依据**：`mid360_odin1_multi.rviz` 中 `Odin1_Realtime` → `/odin1/cloud_in_map`；节点 `odin_independent_accum`，`project_with_mid=true`。
- **触发**：融合 launch 打开 densify；用户对比 `/cloud_registered` vs `/odin1/cloud_in_map`。
- **影响**：Odin 点被 `T_map = T_mid × T_body_odin_lidar` 画进 Mid 地图。若 `T_mid` 对应时刻 ≠ Odin 几何时刻 → **运动时空间拖影**，观感像“慢半拍”。即使原始 `/odin1/cloud_raw` 实时，融合视图仍可拖。

### P0 — Mid / Odin `header.stamp` 不可直接当同一物理时刻做同步

- **配置**：`ws_odin_pvlio/.../control_command.yaml:9` `use_host_ros_time: 2`。
- **Mid**：`livox_ros_driver2` `lddc.cpp` 用包内 `base_time`。
- **影响**：`|t_mid−t_odin|<50ms` 一类 IEKF/`mid_restamp` 逻辑在跨传感器上 **系统性不可靠**；转弯时用错 Mid yaw → 扇形拖影。

### P1 — 曾用 `cloud_slam` 反投影生成 L2，几何错误放大拖影/重影

- **文件**：`odin_raw_downsample_bridge.py` `_unproject_to_lidar`；旧 launch `input_topic: /odin1/cloud_slam`。
- **实测**：slam/raw `r_p50~1.1–1.6`，错误 L2 `r_p50~4.2`，`/odin1/cloud_in_map` 相对 Mid NN p50~0.3–0.5 m。
- **影响**：橙色云整体错位，运动时更像拖影。当前 launch 已改为 `/odin1/cloud_raw`（**改动在源码/launch，属既有现场状态；本诊断未再改**）。

### P1 — Adaptive 中 Odin 扫描结束时间仍依赖 `points.back().curvature/1000`

- **文件**：`laserMapping_adaptive.cpp`  
  - 排序：`standard_pcl_cbk2` **559–562**  
  - 使用：`sync_packages_async` / bundle 中 `lidar_end_time2 = beg + back.curvature/1000`（约 **776–787** 行同类逻辑）
- **关键代码（节选）**：
```559:565:/home/ll/FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp
    // Odin points are not guaranteed time-ordered; sync uses points.back().curvature.
    if (!ptr->empty())
        std::sort(ptr->points.begin(), ptr->points.end(),
                  [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
```
```705:713:/home/ll/FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
```
- **preprocess**：Odin `offset_time`(s)×1000 → curvature(ms)（`preprocess.cpp` **346–350, 382**）；Livox `offset_time/1e6` → ms（**132**）。
- **触发**：L2 经 bridge 重打包后，若 `offset_time` 全 0 / 非单调 / 跨度异常 → `lidar_mean_scantime2` 回退估计。
- **影响**：错误的 `lidar_end_time2` → IMU 等待窗口错；当前 Async **丢弃 L2-only**，对 Mid 主链影响被部分隔离，但对任何仍走 L2 sync 的路径有害。

### P1 — 运行二进制与源码不同步

- **依据**：build `laserMapping_adaptive` 19:37 vs 源码 20:49。
- **影响**：已改 C++ 行为（如 TF EMA）**未进入当前进程**；排障易误判。

### P2 — Async 对 L2 的策略：丢弃旧 L2 / 丢弃 L2-only，可能造成“Odin 进 IEKF 侧永远落后或看不见”

```661:682:/home/ll/FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp
        while (!time_buffer2.empty() &&
               time_buffer2.front() + 1e-3 < time_buffer.front())
        {
            lidar_buffer2.pop_front();
            ...
        }
        ...
            // Discard Odin-only async packages (would IMU-prop Mid to L2 time).
            lidar_buffer2.pop_front();
```
- **影响**：设计上 Mid 优先；若用户期望 IEKF 内 Odin 紧跟，会表现为 Odin 侧“不更新/滞后”。**RViz densify 不依赖这条 IEKF 路径**。

### P2 — 多份 `multi_lio_nav_bridge` 残留（8 个）

- **触发**：反复 launch 且默认 `enable_nav_bridge:=true`。
- **影响**：CPU/DDS 负载 ↑，可能加剧处理延迟（**定量未测**）。

### P2 — densify 在高 yaw-rate 跳过 voxel accum

- **文件**：`odin_independent_accum.py`（`accum_max_yaw_rate_deg`）
- **影响**：减累积扇形，但 live `/odin1/cloud_in_map` 仍可能因 pose 错配拖影。

---

## 4. 时间戳诊断表

### 4.1 时间基准说明

| 源 | stamp 含义 |
|----|------------|
| Mid lidar/imu | 设备包时间（Livox `base_time` / IMU stamp）；与主机 NTP 轴接近时 age≈传输+组帧 |
| Mid `/Odometry` | FAST_LIO `publish_lidar_time`（雷达结束时刻） |
| Odin USB（mode 2） | **传感器时间 − PTP 平滑 offset**（主机轴）；**不是** mode 1 的 receive-now |
| `/odin1/cloud_l2_body` | 保留 raw 几何 stamp；若 bridge 积压，则 **stamp 旧但刚发布** → `topic delay` 变大 |
| `/odin1/cloud_in_map` | 与 L2 同几何戳（densify） |
| `/cloud_registered` | Mid 扫描结束时间 |

**禁止**在基准未统一时把 `now−stamp` 直接当成“线缆延迟”；也禁止用慢速 Python 订 CustomMsg 的假性 age。

### 4.2 本会话有效实测摘要（早先窗口；QoS/订阅正确时）

| 话题 | age p50（约） | 周期 | offset 备注 |
|------|----------------|------|-----------|
| `/livox/lidar` | 0.9–1.2 s | ~10 Hz 量级 | CustomMsg `offset_time` 为 ns 量级跨度 |
| `/livox/imu` | 15–250 ms（波动大） | 高频率 | |
| `/odin1/cloud_raw` | 0.2–1.2 s | ~10–14 Hz | `offset_time` 字段存在（s） |
| `/odin1/odometry` | ~0.1–1.1 s | ~与云同步 | |
| `/odin1/cloud_l2_body` | 曾出现 stamp 比 raw 旧 ~2 s（slam 路径） | ~bridge 限 20 Hz | raw 路径后 `last_full==last_l2` |
| `/odin1/cloud_in_map` | 随 densify | ~数 Hz～10 Hz | 空间 NN 相对 Mid 曾 p50~0.3–0.5 m |

### 4.3 本轮末尾（诊断进行时）

| 现象 | 状态 |
|------|------|
| bridge `cloud=` | **卡在 1326**（数秒不变） |
| densify `pub=` | **卡在 1189** |
| `ros2 topic echo` 短超时 | 无数据（与停更/DDS 一致） |

→ 此时“滞后”可能混有 **传感器/驱动停更**，不能单独归因算法。标注：**部分实时表为尚未稳定复测**。

### 4.4 offset_time / curvature（源码结论）

| 雷达 | 原始单位 | 写入 curvature | 排序 |
|------|----------|----------------|------|
| Mid Livox | ns（CustomMsg） | `/1e6` → ms | 驱动顺序 |
| Odin PointCloud2 | s（wiki） | `×1000` → ms | `standard_pcl_cbk2` 显式按 curvature 排序 |

Bridge 重发布 L2 时保留 `offset_time`（s）；若字段丢失则填 0 → IEKF 内扫描跨度估计退化。

---

## 5. 根因判断

### 已确认

1. **用户融合后看到的 Odin“慢/拖”主路径 = densify（`cloud_in_map`）用 Mid 位姿重投影**，不是“Odin 驱动单独变慢”这一条。
2. **Mid 与 Odin stamp 语义不同（采样 vs host 接收）**，硬同步必然失败。
3. **当前 Adaptive 配置下 Odin 点不进 IEKF 残差**；经典“双雷达 IEKF 队列拖死 Mid”被大量改掉，但 densify 仍可拖影。
4. **slam 反投影 L2 几何错误**曾是重影主因之一（有半径/NN 数据）。
5. **运行中 C++ 二进制旧于源码**。

### 高概率

1. **运动拖影**：densify 使用的 Mid 位姿与 Odin 几何时刻不一致（lookback/EMA/墙钟配对误差；或 L2 mid pose 绑定后仍有固定时延未标定）。
2. **外参 `T_L1_L2`（约 −13° pitch）误差** → 静止也有分米级重影，运动时更明显（NN~0.3 m+）。
3. **Python densify/bridge 吞吐 + 8× nav_bridge** → 处理侧延迟（缺内部 queue耗时日志，**未完全证实为瓶颈**）。

### 尚未验证

1. Odin 面阵 `offset_time` 是否具有真实逐点曝光意义，还是近似常数。
2. IMU 覆盖不足是否在当前 Mid-only Async 路径上仍触发长时间 `return false`。
3. Bundle 模式是否更差（当前未切 Bundle 做 A/B）。
4. 停更是 USB/host_sdk 故障还是 DDS。
5. 假设 A–D 中 **C（共享单队列错序）对本 fork Adaptive 主路径不成立**；对未改的 bundle/其他二进制需另验。

### 对四个怀疑项的结论

| 项 | 结论 |
|----|------|
| A offset 单位/排序 | preprocess 对 Odin s→ms、Livox ns→ms **源码正确**；L2 重打包后可能丢/置零 offset → **部分风险** |
| B back.curvature 扫尾 | 仍存在；Odin 已 sort；跨度异常时靠 mean_scantime → **高概率次要** |
| C 共享队列积压 | Adaptive **分缓冲 + 丢 L2-only** → **本配置下非主因** |
| D 过密点云算力 | 有降采样，但仍有 Python 路径；**可疑，缺耗时探针** |

---

## 6. 建议修改方案（最小改动，本轮未执行）

1. **统一“看见的 Odin”定义**  
   - 对比延迟时同时看 `/odin1/cloud_raw` 与 `/odin1/cloud_in_map`。  
   - 若 raw 不慢、in_map 慢 → 只改 densify 同步，不改 IEKF。

2. **densify 时延标定（优先）**  
   - 用墙钟或固定回转台标定 `mid_pose_lookback` / 固定 lag；避免再靠跨传感器 stamp。  
   - 保持 `/odin1/l2_mid_pose` 与 L2 同几何戳绑定（已有方向）。

3. **禁止再对 USB `cloud_slam` 做反投影进 densify**（除非重验 `r_p50≈raw`）。

4. **IEKF 路径**  
   - 维持 L2 与 Mid **分缓冲**；L2 `lidar_end_time2` 改为 `max(curvature)` 显式计算，或面阵用固定小扫描窗（如 0–5 ms），勿共享 Mid `lidar_mean_scantime`。  
   - `lidar_mean_scantime` / `lidar_mean_scantime2` 已分离 — 保持。

5. **工程卫生**  
   - 改 C++ 后 `colcon build --packages-select fast_lio_multi` 再启动。  
   - 启动前清理多余 `multi_lio_nav_bridge`；或 `enable_nav_bridge:=false`。  
   - Livox 订阅 QoS 与驱动一致（本机 RELIABLE）。

6. **不要**用“把 header 改成 now”掩盖延迟。

7. **外参**：静止 NN>0.2 m 时应重标 `extrinsic_*_L2_wrt_L1`（CAD/−13° 为粗值）。

---

## 7. 验证方案

1. **双话题对照**：静止/匀速转动时同时录 `/odin1/cloud_raw`、`/odin1/cloud_in_map`、`/cloud_registered`、`/Odometry`。  
2. **时间**：wall 到达差、densify `l2_pose_hit`/`sync_dt`、禁止混用未说明基准的 age。  
3. **几何**：静止墙面 NN p50；转动时扇形宽度（度）。  
4. **队列**：临时日志（仅实验分支）打印 `lidar_buffer(2).size()`、等待 IMU 时长、bridge `skip_age`。  
5. **进程**：`pgrep` 仅 1 个 mapping、1 个 bridge、1 个 densify、1 个 host_sdk。  
6. **二进制**：确认 running exe mtime ≥ 源码。

---

## 8. 最终结论

1. **Odin“滞后”主要落在哪？**  
   - **2026-11-12 复测确认：首先是 `odin_raw_downsample_bridge` Python 热路径 ~2 s 处理积压（100% CPU）**，然后才是 densify `cloud_in_map` 用 Mid 位姿去投这批已旧的几何。  
   - Mid `/cloud_registered` / `/Odometry` 仅 ~45 ms，**不是** Mid IEKF 被 Odin 拖死。  
   - IEKF 内 Odin 点残差仍关闭；驱动-only 时 Mid/Odin 云 age 同约 0.13–0.15 s，IMU 对齐 ~10 ms。

2. **设备共时 vs 融合内部？**  
   - mode 2 下 **时间轴已基本可对齐**（Mid↔Odin IMU ~7.5 ms）；**本轮 2 s 不是共时失败**。  
   - 融合内部 Python densify 链路才是数量级问题。

3. **要改哪里？**  
   - **先**打通/加速 bridge（latest-only 丢旧帧、C++、降载），使 L2 delay≈raw；**再** densify 时延标定与外参；重建过期 `laserMapping_adaptive`。

4. **要不要重标外参？**  
   - bridge 仍 2 s 落后时外参/时延标定无意义；压延迟后，静止 NN 仍分米级再重标。

5. **能否继续用 FAST_LIO_MULTI？**  
   - **可以**；架构（Mid 主 LIO + Odin densify）合理，瓶颈在 densify 前处理实现，不在“换融合框架”。

---

## 附录：复测已完成 / 仍可选

已完成（2026-11-12）：驱动启动、RELIABLE 时戳表、mode 2 语义核对、offset_time 分布、融合后 delay/管道 lag/CPU。  

仍可选（改代码前）：  
1. bridge 实验分支加 per-frame 耗时与 `queue lag` 日志；  
2. 临时 `latest-only`（深度 1 + 丢旧）验证拖影是否立刻消失（**属改代码，需你点头再动**）。


## 9. 2026-11-12 补充：Odin live「稍微动就晃」根因

实测（host ROS time）：

| topic | stamp_lag med |
|-------|----------------|
| `/odin1/cloud_raw` | ~150 ms |
| `/odin1/cloud_l2_body` | ~180–220 ms |
| `/cloud_registered` | ~50 ms |
| Mid stamp − L2 stamp | ~200 ms |

**根因**：Odin 几何时间戳比 Mid 当前位姿旧约 0.2 s。若用**最新 Mid**去投旧几何，转动时  
`δθ ≈ ω·Δt`（例：5°/s × 0.28 s ≈ 1.4° → 5 m 处约 12 cm）→ 肉眼「很晃」。

错误做法：`T_latest · T_ext · p_odin`  
正确做法：`T_stamp · T_ext · p_odin`，live 再 `T_latest · inv(T_stamp)` SE3 warp；且 **warp 的 T_src 必须等于投影用的 T_stamp**（不能再 nearest 查另一帧 Mid）。

相关提交：`0cc1703`、随后 warp-T_src 对齐修复。
### 补充：live vs AccumFused 不一致

`motion_sync_to_mid=True` 时 `/odin1/cloud_in_map` 被 warp 到最新 Mid，而 `/viz/accum_odin`（进 AccumFused）保持 stamp 投影。  
机器人稍动 → 橙色 live 相对融合图墙体滑动，表现为「融合图上 Odin 在晃」。  
修复：关闭 motion_sync，live 与 accum 共用同一 deadband 后的 stamp Mid 位姿。

### 补充：AccumFused 双墙（2026-11-12）

静止外参网格（pitch/yaw/xy）只能把 live NN 从 ~10.9 cm 降到 ~10.1 cm → **CAD Ry(−13°) 不是主因**。  
同场实测：

| 云 | vs Mid NN p50 |
|----|----------------|
| `/odin1/cloud_in_map` (live) | ~11 cm |
| `/viz/accum_odin` | **~26 cm** |
| `/viz/accum_fused` | **~22 cm** |

**根因**：densify `hold_trans=0.03`/`hold_rot=1.2°`/`α=0.06` 让 Odin 投影 Mid 落后于 `/cloud_registered`；keep-first voxel 把这段错位永久写进 accum → RViz 双墙。  
**修复**：hold=0、α=1（与 Mid 1:1）；`accum_max_yaw_rate_deg` 25→12；清空 `/odin_independent_accum/clear_accum` 与 fuse `~/clear_accum`。

### 补充：微转仍严重晃 + 写进图（源码根因，未上电复核）

RViz 橙色 live = `/odin1/cloud_in_map`（`odin_independent_accum`，`project_with_mid`）。  
累积图 = `/viz/accum_odin` → `mid_odin_frame_fuse` → `/viz/accum_fused`（keep-first voxel）。

**主因（位姿时间）**

1. Bridge `_match_mid_for_l2`（`odin_raw_downsample_bridge.py`）在因果 Mid 之外仍做  
   `before/after` **插值**（半窗 `interp_dt≤0.08s`），显式用到 **未来 Mid**。  
   转弯时未来 Mid yaw 已超前 → 投影 Odin「转过头」；下一帧只剩 causal → **回弹**。  
   Launch 注释写 “never future”，与实现不一致。
2. 几何上 Odin stamp_lag 仍常比 Mid 大约 0.15–0.22s。任何偏新的 Mid（插值/窗口）在微转时  
   `δθ≈ω·Δt` 被距离放大（5°/s×0.2s≈1° → 5m 处约 9cm），表现为「稍微转就很晃」。

**放大（同一 `_project_odin_cloud`）**

3. `deskew_odin=True` 且 `|yaw_rate|≥still_yaw_rate_deg(3°)` 时，用 **当前墙钟 Mid yaw rate** 去 deskew 本帧 Odin；微转刚好跨过门限，rate 噪声会整扇拧动 wedge。
4. 当前 launch `densify_hold_*=0`、`α=1`：Mid IEKF 每帧位姿变化 **100%** 进 densify（关 deadband 是为消双墙，但把微抖重新暴露给远点）。

**为何晃动点进图**

5. `VoxelAccum.add` **keep-first**：错位点一旦写入永不覆盖。  
6. `accum_max_yaw_rate_deg=12` 只挡大转；**3–12°/s 微转仍 `_accum_locked(out)`**，live 与 accum 共用同一错误投影。  
7. 另路：`mapping.lidar2_map_insert_en: true` 在 bundle + `last_odin_iekf_ok` 时还可把 Odin 点写入 C++ `ikdtree`（与 Python AccumFused 独立）。

**结论**：晃 = densify Mid 相对 Odin 几何偏新（尤以 bridge 未来插值为源码级确定缺陷）+ deskew/无 deadband 放大；进图 = keep-first accum 门限过宽（及可选 ikdtree insert）。

### 10. 按官方 FAST_LIO_MULTI 的改动（2026-11-12）

原则：**紧耦合真值 = C++ stamp 同步 + IEKF**；Python densify 只做软可视化。

| 改动 | 文件 | 效果 |
|------|------|------|
| `_match_mid_for_l2` 仅 causal Mid（禁 before/after 插值） | `odin_raw_downsample_bridge.py` | 消除转过头再回弹 |
| `deskew_odin=false`；accum 仅 still（≤3°/s 且 ≤still_trans_vel） | launch + `odin_independent_accum.py` | 微转不拧 wedge、不写 keep-first |
| fuse `accum_gyr_gate=0.05` | launch | Mid 体素也不在微转时堆 |
| `lidar2_map_insert_en=false`；point/IEKF 仍开 | `mid360_odin1.yaml` | 紧耦合保留，坏 L2 不进 ikdtree |



---

*报告路径：`/home/ll/3dnavigation/docs/lidar_fusion_diagnosis.md`*
