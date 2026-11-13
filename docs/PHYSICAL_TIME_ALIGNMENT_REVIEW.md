# PHYSICAL_TIME_ALIGNMENT_REVIEW

- **基准提交**：`d2db023`（含其上的运行栈 / `cc2ce81` `t_query`）
- **日期**：2026-11-13
- **约束**：只读；未改代码 / 参数 / 系统时间
- **目的**：检验假设——`|center_dt|≈24.7ms` 主要是双雷达 **异步扫描相位差**，而非设备时钟偏移；当前 `t_query = Mid_center+(odin_span−odin_med)` 可能把相位差误当成“中心已对齐”

证据分层标记：**[推导]** / **[源码]** / **[实测]**

---

## 0. 一句话结论

**[实测]+[推导]** IMU 角速度互相关给出的跨设备时钟差约 **−9ms（全窗）/ ≈0–2ms（高运动加权）**，峰值相关 **0.95–1.00**；而 Bundle `|center_dt|` 中位 **24.5ms**，与双 **10Hz** 异步相位模型的 **T/4=25ms** 几乎重合。  
→ **`center_dt` 主要是扫描相位，不是时钟**。  
→ 当前 `t_query` **在数学上等价于假定 `center_dt=0`**，会把 Odin 真端时刻错误吸附到 Mid 扫描中心时间轴，误差 ≈ **`−center_dt`（常 ±25ms，转角时更大）**。

---

## 1. `center_dt`、`lidar2_stamp_bias_sec`、`t_query` 关系（严格推导）

### 1.1 符号 **[推导]+[源码]**

源码：`laserMapping_adaptive.cpp` 配对段 ≈L1012–1096，发布段 ≈L2610–2624。

| 符号 | 定义 |
|------|------|
| `H₂` | Odin 云 `header.stamp`（`time_buffer2`） |
| `b` | `lidar2_stamp_bias_sec`（yaml 初值 0.065，配对 EMA 更新） |
| `m₂` | `odin_med` = 点 `curvature/1000` 的中位数（相对 `H₂`） |
| `S₂` | `odin_span` = max curvature/1000 |
| `H₁, E₁` | Mid beg / end；`S₁=E₁−H₁`；`C₁=H₁+S₁/2` |
| `C₂ᵇ` | 映射到“加 bias 后”的 Odin 中心：`H₂ + b + m₂` |
| `center_dt` | `C₂ᵇ − C₁` |
| `signed_dt` | `(H₂+b) − H₁`（header 窗，非中心） |

配对得分：`score = |center_dt| + 4ms·[signed_dt>0]`，在 `|signed_dt|≤55ms` 内取最小。

Bias EMA（配对成功时）：

```text
b ← clamp( b − α·center_dt , [b_min,b_max] )   # α=0.05, [0,0.12]
```

意图：把 **E[center_dt] → 0**，即把 **平均中心差** 吃进 `b`。

### 1.2 三类时间量必须拆开 **[推导]**

| 量 | 物理含义 | 典型尺度（本机） |
|----|----------|------------------|
| **扫描相位 φ** | 两台独立 10Hz 触发不对齐；最近邻中心残差 | 周期 T≈100ms → \|φ\| 期望 **T/4=25ms** |
| **设备时钟差 δ** | 同一刚体事件在两套 stamp 上的固定/缓变差 | **IMU 互相关 ≈ −9ms / 高运动 ≈0–2ms** |
| **传输/处理延迟** | USB/DDS/bridge 墙钟晚到；不一定改 header | raw age ~150–200ms；bridge `causal` match Mid ~50–100ms |

`center_dt` **观测方程**（近似）：

```text
center_dt ≈ φ + δ_cloud + ε_med
```

其中 `δ_cloud` 含 header 语义误差；`φ` 每帧在 (−50,50]ms 内跳变；`δ` 缓变。  
**EMA 的 `b` 只能吸收慢变均值**，**消不掉逐帧 φ**。因此稳态仍有：

```text
E[center_dt] ≈ 小（本日志 mean≈−5ms）
median(|center_dt|) ≈ 25ms   ← 相位主导
```

### 1.3 `t_query` 代数 **[推导]+[源码]**

```text
t_query = C₁ + (S₂ − m₂)          # 现行 cc2ce81
```

若配对使 `C₂ᵇ = C₁ + center_dt`，则 **真·Odin 扫描结束**（在 bias 映射轴上）为：

```text
E₂ᵇ := H₂ + b + S₂
     = (H₂ + b + m₂) + (S₂ − m₂)
     = C₂ᵇ + (S₂ − m₂)
     = C₁ + center_dt + (S₂ − m₂)
     = t_query + center_dt
```

因此：

```text
t_query − E₂ᵇ = − center_dt
```

**现行公式当且仅当 `center_dt=0` 时等于物理 Odin-end（bias 轴）**。  
日志里 `vs_mid_end ≈ expect ≈ −1…−4ms` 只验证了相对 Mid_end 的代数恒等式，**不验证** `C₁≈C₂ᵇ`。

对照：`vs_mid_end = t_query − E₁ = S₂−m₂−S₁/2`，与 `center_dt` 无关 → 与实测 `corr(|vs|,|center|)≈0` 一致，**不能**当作物理正确证明。

### 1.4 假设判定 **[推导]**

用户假设成立条件：

1. `|center_dt|` 分布贴近异步相位；且  
2. IMU 估得的 `|δ| ≪ 25ms`。

两者本轮均满足（§2、§1.2 实测）。  
→ **现行 `t_query` 把相位差误当成“已同步中心”**，橙色位姿时间误差 ≈ **`−center_dt`**。

---

## 2. IMU 互相关估计跨设备时钟差 **[实测]**

### 2.1 方法

- 话题：`/livox/imu`（~200Hz）、`/odin1/imu`（~400Hz，Reliable）
- 外参：把 Odin ω 旋到 Mid lidar/IMU 轴  
  `R = extrinsic_R_L2_wrt_L1`（Ry(−13°)，yaml）；两 IMU 相对各自雷达近似 `R=I`
- 重采样 200Hz；`correlate(Mid, Odin)`；约定  
  `Mid[t] ~ Odin[t−L]` ⇒ `L = d_mid − d_odin` ⇒ **`d_odin − d_mid = −L`**
- 需主动转动（静止峰相关 <0.4，不可信）

### 2.2 结果（2026-11-13，主动左右转）

| 条件 | 通道 | L (ms) | peak | `d_odin−d_mid` (ms) |
|------|------|--------|------|---------------------|
| 15s 含转，全窗 | ‖ω‖ | 9.12 | **0.996** | **−9.12** |
| 全窗 | wz | 8.56 | 0.997 | −8.56 |
| 全窗 | wy | 10.51 | 0.947 | −10.51 |
| 全窗 | wx | 9.58 | 0.974 | −9.58 |
| 高运动加权 | ‖ω‖ | 1.78 | 0.998 | **−1.78** |
| 高运动加权 | wz | 0.62 | 0.998 | **−0.62** |

静止 30s 对照：peak 仅 0.23–0.53，L≈10ms 但 **SNR 不足**，仅作参考。

**结论**：刚体角速度对齐下，**时钟/戳偏置量级 0–10ms，不是 25ms**。  
`|center_dt|≈24.5ms` **不能**解释为 IMU 时钟差。

原始数据：`/tmp/imu_xcorr_turn.npz`

---

## 3. Odin `header.stamp` 与 IMU / 点时间 **[源码]+[实测]**

### 3.1 驱动语义 **[源码]**

`odin_ros_driver`：`use_host_ros_time: 2`（`control_command.yaml`）。

- **yaml 注释**写：“reception time on host”  
- **实际代码** `make_aligned_stamp` / `aligned_stamp_ns`（`host_sdk_sample.h` ≈L203–241）：

```text
mode 0: 设备原始 sensor_timestamp
mode 1: host now()（真正接收墙钟）
mode 2: sensor_timestamp − smoothed_PTP_offset   // 设备时对齐到主机轴
```

云与 IMU **同一** `make_aligned_stamp`（云用 `cloud.timestamp`，IMU 用 `stream->stamp`）。

**注意**：yaml 注释与 mode2 实现不一致；以代码为准。USB 场景下 PTP offset 质量未在本轮单独标定。

### 3.2 点内时间 **[源码]+[实测]**

发布云时（`host_sdk_sample.h` ≈L711–716）：

```text
offset_time = group / dtof_subframe_odr   # 秒；按行组递增
header.stamp = make_aligned_stamp(cloud.timestamp)
```

实测 `/odin1/cloud_raw`（6 帧）：

| 量 | 值 |
|----|-----|
| `offset_time` min / med / max | **0 / 0.0457 / 0.09437 s** |
| 与 deskew 日志 `span=94.3683ms` | **一致** |
| header age | ~0.19–0.20 s（墙钟−stamp） |

FAST-LIO `xyz_handler`：`curvature_ms = offset_time_s * 1000`（`preprocess.cpp` ≈L346–350）。

**判断**：

- `header.stamp` ≈ **该帧时间原点（首组/帧起点）**，不是扫描中点，也不是“主机接收瞬间”（mode2）。  
- 点时刻 ≈ `header + offset_time`；与 `/odin1/imu` **共用对齐策略**，故 **Odin 云–Odin IMU 相对同轴**（native deskew 前提成立）。  
- 与 Mid 的绝对对齐仍依赖 PTP/`b`，不能从 header 字面理解为“真采集 UTC”。

---

## 4. `t_query` 是否把相位差当成同步误差？

### 4.1 相位模型 vs 实测 **[推导]+[实测]**

双独立 10Hz、相位均匀、最近中心配对：

```text
median(|center_dt|) = T/4 = 25 ms
```

本日志（>10k Bundle tight）：

```text
median(|center_dt|) = 24.53 ms
mean(|center_dt|)   = 24.82 ms
signed mean         = -5.01 ms   # bias EMA 后接近 0
```

四分位占比大致约 27/33/22/16)%，接近均匀略偏负——**与“相位为主 + 小时钟偏置”吻合**。

### 4.2 与 IMU δ 对比 **[实测]**

| 来源 | 量级 |
|------|------|
| \|center_dt\| | **~25ms** |
| IMU `d_odin−d_mid` | **~0–10ms** |

差约 **15–25ms** → 只能归到 **φ（扫描相位）**，不能归到时钟。

### 4.3 对橙色位姿的后果 **[推导]**

```text
Δt_pose_error ≈ −center_dt
扇形角上界 ≈ |ω| · |center_dt|
```

转角帧（`gyr_scan≥0.25`，日志）：中位 **0.66°**，p90 **1.49°**，max **3.44°**。  
若误差仅 1.2ms（`vs_mid_end`）：中位 **0.03°**（可忽略）。

**结论：是的——现行 `t_query` 把异步相位残差当成了“中心已对齐”。**

---

## 5. `UndistortOdinNative`：Mid `bg/ba`、acc 尺度、无重力

### 5.1 源码行为 **[源码]**

`IMU_Processing.hpp` ≈L355–413，调用 ≈L796：`UndistortOdinNative(..., st.bg, st.ba)`。

```text
angvel -= bg_mid
acc    = acc_odin * (G / ‖mean_acc_mid‖) − ba_mid
相对系积分，注释 grav≈0（不显式加 g）
```

### 5.2 影响机制 **[推导]**

| 混用项 | 帧内畸变机制 | 静止 | 纯转动 |
|--------|--------------|------|--------|
| Mid `bg` on Odin gyro | 假角速度 → 假旋转 deskew | 小 | **∝‖ω‖·S₂**，可扇形 |
| Mid `ba` + Mid `mean_acc` 尺度 | 假线加速度 | 重力方向偏置 | 次于 gyro；猛加减速时更大 |
| 忽略 g | 相对短窗（~95ms）部分被 `ba` 吸收 | 若 `ba` 错则漂移 | 纯转影响小于 bg |

### 5.3 验证建议（未改代码，本轮未跑点云残差） **[方法]**

1. **静止**：同一墙面/柱，比较 deskew 开/关（或前后帧）点到平面残差；期望 bg/ba 混用影响小。  
2. **主动 yaw**：固定竖边特征，量橙色扇形角；与 `|ω|·|center_dt|` 及 `|ω|·|δ_imu|` 对比。  
   - 若扇形 ≈ `|ω|·|center_dt|` → 主因是 **t_query/相位**（§4）。  
   - 若修正 `t_query` 后仍剩明显扇形且随 ‖ω‖ 线性 → 再查 **Odin 独立 bg**。

本轮 **不能**用残差数值证伪 deskew 偏置；仅能将其标为次级嫌疑。

---

## 6. 转弯期 Stage1/2、Mid-only、地图插入

### 6.1 Stage1 vs Stage2 位姿差 **[无法实测]**

源码（≈L2569–2692）：Stage1 后存 `state_stage1`；Stage2 用 `cov_scale=100`；`/Odometry` 与 Mid 云用 Stage2 后状态；橙色用 Stage1@`t_query`。  
**无 ΔR/Δt 日志，`n_odin_iekf_updates` 不落盘** → 本轮 **无法给出转角期实测修正量**。

### 6.2 Mid-only 旧帧重发 **[实测]**

```text
tight ≈ 10163, Mid-only ≈ 312 → Mid-only 率 ≈ 2.98%
```

Mid-only 时 `publish_odin_from_undistorted` 重发 **上一帧 map 快照**（不重新用当前 Mid 拧旧 body）——源码注释 ≈L1535–1545。

### 6.3 Odin 地图插入比例 **[实测]+[源码]**

写图条件（≈L1382–1395）需：`map_insert_en`、`odin_pose_update_allowed`（ω 门=0）、`last_odin_iekf_ok`、`|center_dt|≤25ms` 等。

| 集合 | `|center|≤25ms` 通过率 |
|------|------------------------|
| 全部 tight | **51.1%** |
| `gyr≥0.25` | **43.4%**（424 帧） |

`last_odin_iekf_ok` **未打点** → 真实插入 ≤ 上表。转角时近半帧因 **相位门**（不是 ω 门）禁止 Odin 进 ikdtree。

---

## 7. 分类

### 已较强支持

1. `|center_dt|≈25ms` = **异步 10Hz 相位**（分布 + T/4 理论）。  
2. IMU 互相关：`|δ|~0–10ms`，**不是 25ms 时钟**。  
3. `t_query` 相对真 `E₂ᵇ` 误差 **`−center_dt`**（代数必然）。  
4. Odin 云 `header`+`offset_time` 与 IMU 同对齐策略；`span≈94.4ms` 实测一致。

### 仍存在的问题

1. 橙色位姿时间仍按 Mid 中心吸附 → 转角扇形上界 **~0.7–1.5°（中位/ p90）**。  
2. `map_insert_max_center_dt=25ms` 与相位尺度同量级 → **~一半帧** Odin 无法进图。  
3. `b` 用点云中心 EMA 估时钟，**被相位噪声污染**；应用 IMU δ 更干净。  
4. Native deskew 混用 Mid bg/ba（次级，待 `t_query` 修正后再验）。

### 无法确认

1. Stage1↔Stage2 转角 Δpose 数值。  
2. `last_odin_iekf_ok` 真实比例。  
3. mode2 PTP offset 绝对精度；yaml“接收时刻”注释误导。  
4. deskew 偏置对扇形的定量贡献。

---

## 8. 唯一最小、可证伪修改方案（先勿盲目改 bias / 勿加 ω 门）

### 方案

**仅改橙色（及任何依赖“Odin 物理结束时刻”的）`t_query`：**

```text
# 现行（错误吸附 Mid 中心）
t_query = mid_center + (odin_span - odin_med)

# 建议（只用时钟 bias，保留扫描相位）
t_query = H₂ + b + odin_span
        = mid_center + center_dt + (odin_span - odin_med)   # 等价
```

即：**在 Mid 时钟轴上取 bias 映射后的 Odin 扫描结束**，不再假设 `C₁=C₂ᵇ`。

- **不要**把 `b` 调去“消灭”25ms 相位。  
- **不要**用 `|ω|` 门掩盖。  
- `b` 长期应用 IMU 互相关 δ 标定（可选第二步）；本最小改动可先沿用现有 `b`。

### 证伪实验（一次左右转即可）

1. 固定峰值 `|ω|≥0.5` 左右转，录 `/cloud_odin_registered` + 融合日志 `center=`。  
2. A/B（或临时日志打印两种 `t_query` 下的位姿差）：  
   - **A** 现行公式  
   - **B** `H₂+b+S₂`  
3. **通过准则**：B 的扇形角从 ≈`|ω|·|center_dt|` 降到 ≈`|ω|·|δ_imu|`（≲0.3° @ 1rad/s·5ms）；若 B **不更好甚至更差** → 本假设证伪，转查 deskew bg / 外参。

---

## 附录：关键路径

| 项 | 路径 |
|----|------|
| 配对 / bias / t_query | `FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp` |
| Native deskew | `.../src/IMU_Processing.hpp` `UndistortOdinNative` |
| Odin stamp / offset_time | `ws_odin_pvlio/.../host_sdk_sample.h` |
| 运行日志 | `/tmp/mid_odin_fusion.log` |
| IMU 互相关 | `/tmp/imu_xcorr_turn.npz` |
| 相关提交 | `cc2ce81`（t_query）、`d2db023`（runtime verification） |
