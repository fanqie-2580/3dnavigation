# ODIN_TIME_QUERY_AND_FUSION_REVIEW

- **日期**：2026-11-13
- **基线提交**：`2fe8958`（在线 A/D 调试）→ 本轮在其上增加可回退正式 `odin_pub_time_mode`
- **运行二进制**：`fast_lio_multi_mid_odin_ws/build/fast_lio_multi/laserMapping_adaptive`  
  - 编译时间：2026-11-13 16:14（`colcon build --symlink-install --packages-select fast_lio_multi`）
- **约束**：本轮未重构 IEKF；正式路径仅改 Odin **可视化**位姿查询；融合跨时刻问题以审查结论给出

---

## 结论总览

### A. Odin 可视化时间查询是否可以安全切换为 D

**可以切换，但默认仍保留 A 作为回退。**

| 证据 | 结果 |
|------|------|
| 指纹/接收时刻重配对旧 bag | 转角 n=53：D nn_p50 **0.111** vs A **0.146**（Δ≈−3.5cm），**90.6%** 帧 D 更好；与 index 配对结论同向 |
| `odin_pub_time_mode:=D` 实机 | 正式 `/cloud_odin_registered` vs 调试 D：`delta_p50_med=0` → **official ≡ D** |
| D 模式左右转 bag | `/tmp/online_tq_d_turn`：转角 n=23，D nn_p50 **0.110** vs A **0.151**（Δ≈−3.6cm），**95.7%** D 更好；official≡D |
| 查询覆盖 | 日志 `kind=interp`，`fb=0`（全程无 clamp 当真、无 Mid_end 回退） |
| Mid 健康 | `/Odometry`≈10.1Hz，`/cloud_registered`≈10.0Hz，`/cloud_odin_registered`≈9.7Hz；无新异常洪水 |

**配置**：`mapping.odin_pub_time_mode: "A"|"D"`（默认 **A**）。无效 hist → **明确回退 Stage1@Mid_end**（禁止 clamp 当真）。

### B. 双雷达融合建图是否仍存在 Odin-end 与 Mid-end 不同步

**是。Native deskew 后缺少 Odin_end→Mid_end 的相对刚体/刚体运动重表达；Stage2 与 map_incremental 均在 Mid_end 状态上直接使用 Odin 点。门控只能减轻症状。**

---

## 1. 任务一：配对核验 + 可回退正式 D

### 1.1 Index 配对是否错帧

对 `/tmp/online_tq_ab_out/bag`（2fe8958 时代）用 **bag 接收时刻 ≤5ms + 等点数** 重配对：

| | index-only | recv+n_points |
|--|------------|---------------|
| 配对数 | 140 | **102** |
| 与 primary 一致 | — | 102/102（frac=1） |
| reject_recv / reject_count | — | 26 / 12 |

说明：index 会把「一侧 skip、另一侧仍发」后的序号错位混进评分；收紧后转角结论不变（D 仍明显更好）。

新代码增加 `/cloud_odin_tq_ab_id`（UInt64，同 Bundle 共享 fid）。D 模式回归 bag 用 ab_id 主键配对（`index_agree_frac=1`）。

### 1.2 正式模式实现（viz-only）

文件：`FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp`，`config/mid360_odin1.yaml`

- `odin_pub_time_mode`：`A` = `Mid_center+(span−med)`；`D` = `H2+odin_span`
- `sample_odin_pub_pose`：仅 `interp`/`extrap` 有效；`clamp_*` → 回退 `state_stage1_for_odin_`（Mid_end），计数 `n_odin_pub_query_fallback`
- **未改**：IEKF、地图插入、门控、bias EMA、UndistortOdinNative

### 1.3 实机回归证据

| 项 | 数据 |
|----|------|
| 编译 | 2026-11-13 16:14，`laserMapping_adaptive` |
| 启动参数 | `odin_pub_time_mode=D`，`publish_odin_time_query_ab_en=true`（测试后默认写回 A / false） |
| still bag | `/tmp/online_tq_d_regress`：official≡D；A≈D（静止预期） |
| turn bag | `/tmp/online_tq_d_turn/summary.json`：ab_id 配对；D 胜；official≡D |
| 指纹重评旧 bag | `/tmp/online_tq_ab_repair/summary.json` |
| 日志 | `mode=D … kind=interp … fb=0`，`ok` 递增至 480+ |

**建议操作**：需要橙云转角对齐时设 `odin_pub_time_mode: "D"`；出问题立刻改回 `"A"`。

---

## 2. 任务二：跨时刻点云混用审查（不改 IEKF）

### 2.1 调用链

```
sync_packages (Bundle)
  → ImuProcess::UndistortPclMultiLiDAR
       → Mid: deskew_cloud → Mid_end (Mid IMU)
       → Odin native: UndistortOdinNative → Odin_end (Odin IMU)
                      then static T_L2_wrt_L1  (geometry only)
  → Stage1 IEKF (Mid residuals) @ Mid_end state
  → Stage2 IEKF (Odin residuals) @ same Mid_end state
  → map_incremental(): pointBodyToWorld(state_point@Mid_end)
```

### 2.2 UndistortOdinNative（事实）

源码明确：

> Odin points stay in Odin lidar frame… compensate every point to **this scan's end**.  
> Do **NOT** map each Odin point onto Mid IMU time.

输出：点在 **Odin_end** 的 Odin lidar 系 → 乘静态外参到 Mid lidar 系。  
**没有** \(T_{\mathrm{mid\_end}\leftarrow\mathrm{odin\_end}}\) 运动补偿。

### 2.3 缺少的变换（数学）

记：

- \(p^{L2}(t)\)：Odin 点在时刻 \(t\) 的 Odin lidar 坐标  
- Native deskew 后得到 \(p^{L2}_{\mathrm{end2}}\)（已到 Odin_end）  
- 静态外参 \(T_{L1\leftarrow L2}\)：  
  \(p^{L1}_{\mathrm{end2}} = R_{L1\leftarrow L2}\,p^{L2}_{\mathrm{end2}} + t_{L1\leftarrow L2}\)

若要在 **Mid_end** 用 Mid 状态做点面残差 / 入图，需要点在 Mid_end 的 Mid body（或 lidar）坐标：

\[
p^{L1}_{\mathrm{end1}}
= T_{L1}(t_{\mathrm{end1}})^{-1}\,
  T_{L1}(t_{\mathrm{end2}})\,
  p^{L1}_{\mathrm{end2}}
\]

其中 \(T_{L1}(t)\) 为 Mid 位姿（可用 Stage1 hist / Mid IMU 在同时钟上插值）。  
**当前代码缺这一步**；等价于默认 \(T(t_{\mathrm{end1}})=T(t_{\mathrm{end2}})\)，即 \(\Delta t=0\)。

量级：\(\|\Delta p\| \sim \|\omega\|\,|\Delta t|\,r\)。  
例：\(\|\omega\|=0.5\,\mathrm{rad/s}\)，\(|\Delta t|=0.1\,\mathrm{s}\)，\(r=5\,\mathrm{m}\) → \(\sim 0.25\,\mathrm{m}\) 切向误差（与橙云 A/D 差同量级）。

注：\(\Delta t\) 在 USB 戳上常为 \(t_{\mathrm{end2}}-t_{\mathrm{end1}}\approx -b_{\mathrm{pre}}\)（约 −70…−120ms），与配对 bias **不是**同一用途，但数值相关。

### 2.4 对 Stage2 / 地图的影响

| 环节 | 行为 | 影响 |
|------|------|------|
| Stage2 | `h_share_model` 用 **当前 Mid_end** 状态把 Odin body 点投到 map | 观测模型时间戳错位 → 转弯残差偏大；弱 cov / `lidar2_max_residual` / timeσ 仅降权，**不纠正几何** |
| Stage2 时间权重 | `curvature` 仍是 **Odin 帧内** offset，却按 `mid_span` 解释 | Native 路径下权重语义不一致 |
| `map_incremental` | 同一 Mid_end `pointBodyToWorld`；可按 sync/`center_dt`/`last_odin_iekf_ok` **丢弃** Odin 点 | **减轻症状**，不修复表达；仍可能在门控放行时写入错位点 |
| Mid 状态传播 | 官方 KF 只积分到 Mid_end | **正确**；问题在 Odin 观测/入图表达，不在 Mid 传播 |

### 2.5 正确重表达方向（不改 Mid 传播）

在 **不改** Mid IEKF 预测锚点的前提下，建议仅对 Odin 点做一次后处理（进入 Stage2 / map 之前）：

1. 用 Mid Stage1 hist（或 Mid IMU）在 \(t_{\mathrm{end2}}\) 与 \(t_{\mathrm{end1}}\) 取位姿 \(T_{\mathrm{end2}}, T_{\mathrm{end1}}\)（需统一时钟：D 语义的 \(t_{\mathrm{end2}}=H2+\mathrm{span}\)，或经标定的物理对齐）。  
2. \(p^{L1}_{\mathrm{end1}} = T_{\mathrm{end1}}^{-1} T_{\mathrm{end2}}\, p^{L1}_{\mathrm{end2}}\)（含 offset_R/T 时在 IMU 系做等价 SE3）。  
3. 可视化橙云可继续用 \(T_{\mathrm{end2}}\)（本轮 D）；融合用 \(p^{L1}_{\mathrm{end1}}\)。

**本轮不做该重构**（按任务要求仅审查）。

---

## 3. 两个独立结论（重申）

**A. 可视化 t_query**  
D 已通过指纹重配对与 `official≡D` 实机验证，可作为可回退正式选项；默认保持 A。切换不修复融合跨时刻问题，只改善橙云与物理 Odin_end 的对齐。

**B. 融合建图**  
Native 路径下 **仍存在** Odin_end 点云在 Mid_end 观测/入图中的不同步；缺 \(T_{\mathrm{end1}\leftarrow\mathrm{end2}}\)；Stage2 受影响；地图门控只能减损。

---

## 4. 复现 / 切换

```bash
# 正式切 D（可回退 A）
# mid360_odin1.yaml → mapping.odin_pub_time_mode: "D"

# 调试对照
# mapping.publish_odin_time_query_ab_en: true

python3 /home/ll/3dnavigation/scripts/online_time_query_ab_eval.py \
  --bag /tmp/online_tq_d_regress/bag --out /tmp/online_tq_d_regress
```

相关产出：

- `/tmp/online_tq_ab_repair/summary.json` — 指纹重配对  
- `/tmp/online_tq_d_regress/summary.json` — mode=D，official≡D  
- 脚本：`scripts/online_time_query_ab_eval.py`
