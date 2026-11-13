# TIME_QUERY_ABLATION

- **日期**：2026-11-13
- **基准**：运行栈含 `cc2ce81` / 文档至 `ad4d964`；本实验**不改**正式融合路径、门控、外参、滤波器
- **脚本**：`scripts/time_query_ablation_offline.py`（独立订阅录制 + 离线重投影）
- **产物**：`/tmp/time_query_ablation_out/summary.json`

---

## 0. 一句话

配对用的 `lidar2_stamp_bias_sec` **不是**物理跨设备时钟：在线内部值从 65ms **爬到 ~115–120ms（常顶满上限）**，而同段 IMU 角速度互相关给出的代理偏移仅 **~0–10ms**。  
离线 A/B/C 在主动转角（n=4，\|ω\|≈0.75）上：**C（小物理代理偏移）残差/重影最好，A 次之，B（header+配对 bias+span）最差**。  
因此 **不要**把 `t_query` 直接改成 `H2+b_pair+span`。

---

## 1. 配对 bias 实际范围 ≠ 物理时钟

### 1.1 更新时机（源码）

`laserMapping_adaptive.cpp` ≈L1017–1095：

1. 用**当前** `b` 算 `signed_dt` / `center_dt` 并完成配对  
2. `fill_odin_meas` 末尾：`b ← clamp(b − α·center_dt)`（α=0.05，\[0,0.12\]）  
3. 因此方案 B 必须用 **`b_pre`（本帧配对前快照）**，不是更新后的 `b_post`

`set_parameter` **未**回写 → `ros2 param get mapping.lidar2_stamp_bias_sec` **恒为 yaml 初值 0.065**，与内部真实 `b` 脱节。

### 1.2 在线重建（全日志）

由 `/tmp/mid_odin_fusion.log` 的每帧 `center=` 按相同 EMA 重放（n≈17091）：

| 量 | 值 |
|----|-----|
| `b_pre` 起始 | 65.0 ms |
| `b_pre` 中位 / 末值 | **~114.8 / ~112.7 ms** |
| `b_pre` 最大 | **120.0 ms（上限）** |
| 顶满上限占比 | **~26%** |
| \|center_dt\| 中位 | **24.8 ms**（相位） |

本 clip 离线：`bias_init=114.96ms` → 中位 `b_pre≈117.3ms`，顶满率 ~29%。

### 1.3 与“物理时钟”对照

| 来源 | 量级 | 能否当激光 header 真值 |
|------|------|------------------------|
| 配对 EMA `b` | **~115–120 ms** | **否**（吃进扫描相位 + 顶满） |
| 本 clip IMU ω 互相关 | `d_odin−d_mid`≈**−10ms**（全窗）/ **−5ms**（高运动）peak≈0.99 | **否**（刚体 IMU，非 DTOF header） |
| 先前独立转角 clip | ≈**0–9 ms** | 同上，代理 |

**结论**：配对 bias 与跨设备物理时钟是两件事；`b` 当前主要是 **相位驱动的积分器**，不能当 `t_query` 的时钟真值。

---

## 2. 三方案定义（同云、同 deskew、同外参、同 Mid 轨迹）

| 方案 | `t_query` | bias 来源 |
|------|-----------|-----------|
| **A** | `Mid_center + (span − med)` | 现行在线公式 |
| **B** | `H2 + b_pre + span` | 该帧配对前 bias 快照 |
| **C** | `H2 + δ_phys + span` | 本 clip IMU 高运动互相关导出的 **代理** `δ_phys=+5ms` |

代数：`t_B − t_A = center_dt`（本 clip \|Δt\| 中位 ≈ \|center\|）。

**C 置信度：低。** 明确标注 `low_imu_proxy_not_lidar_header`——**未**完成激光 header 物理标定，IMU 结果**不得**冒充激光时钟真值；C 仅作“小偏移相对 115ms 配对 bias”的对照臂。

Deskew：Python 近似 `UndistortOdinNative`（陀螺主导）。本录制 Mid/Odin IMU 回调仍偏少（~5Hz 量级，被点云解码挤占），转角帧 deskew 位移统计为 0 → **帧内畸变对比本轮无效**，见 §6。

参考地图：低角速度 Mid `/cloud_registered` 累积（静止阈值）。

---

## 3. 量化结果

### 3.1 全时段（n=52，含静止）

| 方案 | res_med 中位 (m) | ghost_p50 (m) | ghost_p90 (m) |
|------|------------------|---------------|---------------|
| A | 0.0666 | 0.0599 | 0.141 |
| B | 0.0667 | 0.0598 | 0.141 |
| C | 0.0667 | 0.0599 | 0.141 |

静止时三者接近（\|ω\| 小，时间误差不放大）——**预期行为**。

### 3.2 主动转角 \|ω\|≥0.25（n=4，mean \|ω\|≈0.75）

| 方案 | res_med 中位 (m) | res_p90 中位 | ghost_p50 | ghost_p90 |
|------|------------------|--------------|-----------|-----------|
| **C** | **0.074** | 0.382 | **0.068** | **0.166** |
| A | 0.098 | 0.404 | 0.094 | 0.270 |
| B | **0.112** | **0.425** | **0.104** | **0.329** |

排序（转角 res_med）：**C ≪ A < B**。

相对 A：

- C：res_med **约 −25%**，ghost_p90 **约 −38%**  
- B：res_med **约 +14%**，ghost_p90 **约 +22%**（更差）

残差–角速度相关（全帧）：A/B/C 的 `corr(w, res_med)` 均随 ω 上升；转角子集上 B 对 ω 最敏感（与 `b≈115ms` 放大 `|ω|·Δt` 一致）。

### 3.3 置信度

| 结论 | 置信度 | 理由 |
|------|--------|------|
| `b_pair` ≠ 物理时钟 | **高** | EMA 轨迹 + 顶满 + IMU 代理差两个数量级 |
| 转角上 B 差于 A | **中** | n=4 但方向一致且与 `\|ω\|·\|b−δ\|` 定性符合 |
| 转角上 C 优于 A | **低–中** | n=4；δ 来自 IMU 代理，非激光标定 |
| 可把在线 `t_query` 改成 B | **否** | 实测 B 更差 |
| deskew 前后几何 | **无法确认** | 本录制 IMU 不足，位移=0 |

---

## 4. Bias EMA 与方案 B 快照

- 离线严格模拟：配对用 `b_pre` → 算 `center_dt` → 再 `b_post = clamp(b_pre − α·center_dt)`  
- 指标里的 `b_pre_ms` 即 B 所用 bias  
- 本 clip `b_pre` ∈ **[112.5, 120] ms**，与在线重建一致  

---

## 5. 对“直接改成 H2+b+span”的否决

`PHYSICAL_TIME_ALIGNMENT_REVIEW` 曾建议 `t_query = H2+b+span`，前提是 **`b` 近似物理时钟**。  
本对照证明该前提不成立：`b≈115ms` 时 B 在转角上**系统性变差**。  
**在修复 bias 估计之前，禁止把正式路径改成 B。**

---

## 6. Deskew 前后

| 项 | 结果 |
|----|------|
| 转角 deskew 位移 med/p90 | **0**（无效） |
| 原因 | 录制端 IMU 样本过稀（odin_imu≈162 / 28s），近似 deskew 未真正积分 |

→ **不能**区分整帧位姿错与帧内畸变；需单独短时只订 IMU+云的 bag 再验。

---

## 7. 唯一最小后续改动建议（仍先验证，再合入）

**不要改 `t_query` 公式 alone。先拆开“配对 bias”与“时钟 bias”：**

1. **冻结/旁路** 用 `center_dt` 驱动的 EMA 去更新“时钟”（或把 EMA 仅作诊断）；配对窗可用固定初值或 IMU 代理，但 **写清不是激光真值**。  
2. 橙色 `t_query` 改为：  
   `t_query = H2 + b_clock + span`  
   其中 `b_clock` 先取 **接近 0～10ms 的可配置常量**（或经人工标定的激光延迟），**不是**当前 115ms `b_pair`。  
3. 验证步骤（与本脚本相同 A/B/C，增加臂 **D**：`H2+0+span`）：  
   - 主动左右转 \|ω\|≥0.5，n≥20 转角帧  
   - 若 D/C 类（小 `b_clock`）转角 ghost_p90、res_med **稳定优于 A**，且 B（大 `b_pair`）仍最差 → 合入 `t_query` 修改 + 停止相位污染时钟。  
4. 仍禁止：加 \|ω\| 门、调外参、动 IEKF 主状态来掩盖。

可选并行：用 `ros2 bag` 高可靠录 IMU，重做 native deskew 前后残差。

---

## 8. 复现

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 从融合日志重放得到当前内部 b（勿信 ros2 param get）
python3 scripts/time_query_ablation_offline.py \
  --record-sec 30 --bias-init 0.115 \
  --out /tmp/time_query_ablation_out
# 录制期间请主动 yaw 转动
```

---

## 附录：本轮原始摘要

见 `/tmp/time_query_ablation_out/summary.json`（转角排序 C → A → B）。
