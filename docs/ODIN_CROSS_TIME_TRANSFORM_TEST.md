# ODIN_CROSS_TIME_TRANSFORM_TEST

- **日期**：2026-11-13  
- **基线**：`d94e20b` / `docs/ODIN_TIME_QUERY_AND_FUSION_REVIEW.md`  
- **运行二进制**：`fast_lio_multi_mid_odin_ws/build/fast_lio_multi/laserMapping_adaptive`（编译 2026-11-13 ~16:47）  
- **开关**：`mapping.odin_cross_time_shadow_en`（默认 **false**）  
- **影子话题**：`/cloud_odin_xtform_{A,B}`，`/odin_xtform_metrics`  
- **脚本**：`scripts/odin_cross_time_xtform_eval.py`  
- **数据**：`/tmp/odin_xtform_mixed/`（70s 混合运动，无压缩 bag + `summary.json`）  
- **约束**：未改正式 Stage2 / IEKF / ikdtree / 门控 / 正式地图与橙云  

---

## 给后续 AI 的四句结论

1. **跨时刻 SE3（Odin_end→Mid_end，用 `t=H2+span`）并未稳定改善对 Mid 地图的点面观测残差。**  
2. **运动段中位改善 ≈ 0**（`d_res_med = resB−resA ≈ +0.4 mm`，B 略差）；左转段 B 更差约 **+4.4 mm**。  
3. **新发现**：`dt = (H2+span)−Mid_end ≈ −126 ms ≈ −b_pair`，说明用 USB `H2+span` 查 Mid Stage1 历史会把**配对偏置误当物理时钟**；这与橙云 D 可视化改善（静态 still-map）不是同一问题。  
4. **尚不具备修改正式 Stage2 的条件**；需先独立标定 `b_clock`，再以 `t_odin = H2 + b_clock + span` 重跑影子实验。  

---

## 1. 源码位置与坐标系约定

| 项 | 位置 |
|----|------|
| Native deskew → Odin_end | `IMU_Processing.hpp::UndistortOdinNative` |
| 静态外参 L2→L1 | 同文件 ~802：`Lidar2_R/T_wrt_Lidar1_` |
| Mid 投影 | `laserMapping_adaptive.cpp::pointBodyToWorld` |
| 影子 A/B | `shadow_odin_cross_time_eval()`（同文件） |
| Stage2 残差 | `h_share_model()` residual_select==2 |
| Stage1 hist | `push_mid_pose_hist` / `sample_mid_pose_at` |

### 1.1 实际代码中的 `T_world_L1`

与 `pointBodyToWorld` 一致：

\[
p_W = R_{WI}\,(R_{IL}\,p_L + t_{IL}) + t_{WI}
\]

- `state.rot/pos` = \(T_{WI}\)（Mid IMU → world）  
- `offset_R_L_I / offset_T_L_I` = \(R_{IL}, t_{IL}\)（Mid LiDAR → Mid IMU）  
- **不是**把 `state` 直接当成 `T_world_lidar` 再乘一次外参。

逆变换：

\[
p_L = R_{IL}^{-1}\big(R_{WI}^{-1}(p_W - t_{WI}) - t_{IL}\big)
\]

### 1.2 跨时刻重表达（影子 B）

Odin 点已在 **Mid L1 @ Odin_end**（native + 静态外参）。候选：

\[
p_L(\mathrm{Mid\_end})
= T_{W\leftarrow L1}(\mathrm{Mid\_end})^{-1}\,
  T_{W\leftarrow L1}(\mathrm{Odin\_end})\,
  p_L(\mathrm{Odin\_end})
\]

实现：`reexpress_L1_odin_end_to_mid_end(s_mid, s_odin, p)`  
再用**同一** `s_mid`（Stage1@Mid_end）投到 world 做点面残差。

等价地，B 的 world 点 = `lidar_to_world(s_odin, p)`（橙云 D 的投影方式）。

### 1.3 时间戳

- `t_mid_end = lidar_end_time`  
- `t_odin_end = H2 + span`（**D 语义，不用 `b_pair`**）  
- hist 仅接受 `interp/extrap`；clamp → **整帧跳过**（本轮 `skip=0`）

---

## 2. 影子路径定义（唯一差别）

同一帧、同一 `UndistortOdinNative` 副本、同一外参、同一 Stage1 hist：

| | A（现行 Stage2 风格） | B（跨时刻 SE3） |
|--|----------------------|-----------------|
| body | \(p_L(\mathrm{Odin\_end})\) | \(T_{mid}^{-1} T_{odin}\,p_L(\mathrm{Odin\_end})\) |
| 投影姿态 | Stage1 Mid_end | **同一** Stage1 Mid_end |
| 地图 | 当前 ikdtree（Mid 为主） | 同左 |

正式 Stage2 / 入图 **未**接入 B。

---

## 3. 实机定量结果（70s mixed）

参考：对 **live ikdtree** 的平面残差中位数 / NN（非 RViz 观感）。

| 片段 | n | dt_ms | rel_yaw° | resA | resB | Δ(B−A) | B更好占比 |
|------|---|-------|----------|------|------|--------|-----------|
| still | 13 | −129 | ≈0 | 0.028 | 0.027 | ≈0 | 15% |
| turn_left | 10 | −149 | **−11.7** | **0.019** | **0.029** | **+4.4 mm** | 20% |
| turn_right | 10 | −117 | +1.4 | 0.020 | 0.021 | ≈0 | 40% |
| translate | 53 | −114 | ≈0 | 0.024 | 0.024 | ≈0 | 32% |
| combined | 69 | −129 | −0.2 | 0.020 | 0.022 | +0.7 mm | 14% |
| **motion all** | **148** | **−126** | — | **0.021** | **0.023** | **+0.4 mm** | **24%** |

连续运动 chunk（≥3 帧）：多数 `d_res_med ≥ 0`（B 不更好）；个别短 translate chunk 有 −2.5 mm，不稳定。

### 失败样本（B 明显更差）

| seg | d_res | rel_yaw | dt_ms | \|ω\| |
|-----|-------|---------|-------|------|
| turn/combined | **+33 mm** | **−24.5°** | −137 | 2.2 |
| combined | +9 mm | −0.5° | −125 | 低 |
| turn_left 中位 | +4.4 mm | −11.7° | −149 | 0.84 |

模式：相对 yaw / \|ω\| 大时，用错误时钟查到的 \(T(\mathrm{Odin\_end})\) 会把点扭离 Mid 地图。

### 成功/中性

- 静止：Δ≈0（相对运动≈0，SE3≈I）。  
- hist 覆盖：本轮日志 `ok` 递增、`skip hist=0`。  
- Mid 频率未因影子路径异常（metrics ~6 Hz，受 ikdtree 近邻评分限制）。

---

## 4. 为何与「橙云 D 更好」不矛盾

| 实验 | 评分地图 | 投影 | 结论 |
|------|----------|------|------|
| 橙云 A vs D | 静止 Mid 累积图 | A用近Mid_end；D用 \(T(H2+span)\) | D 更好（转角墙对齐） |
| 本影子 A vs B | **在线 Mid ikdtree**（Mid_end 锚） | A=Mid_end；B≡D 投影 | B **未**更好 |

解释：

1. 橙云 D 改善的是「用更早 Mid 姿态贴静态墙」；  
2. Stage2 残差要对齐的是 **当前 Mid 状态/地图时间轴**；  
3. 若 `H2+span` ≠ 物理 Odin_end（缺 `b_clock`），B 用错 \(T_{odin}\) → 转弯更差；  
4. `dt≈−b_pair` 强烈暗示当前 USB 戳差被配对 EMA 吸收，**不能**把 `b_pair` 当 `b_clock` 塞进 SE3。

---

## 5. Stage2 时间权重审查（只分析，未改代码）

`h_share_model`（residual_select==2）：

```text
t_rel = curvature/1000
dt_end = t_rel - mid_span
sig_pos = |ω| * lever * (|dt_end| + σ_t)
```

Native deskew 后：

- 点几何已在 **Odin_end**；`curvature` 仍是 **Odin 帧内** offset。  
- `mid_span` 是 Mid 扫描时长。  
- 因此 `dt_end` **不再**表示「相对 Mid_end 的观测时间」。

含义现状：

- 被误当成「点相对 Mid 扫描终点的时间」；  
- 帧内时间差与跨雷达相位 **纠缠**；  
- 再叠 SE3 跨时刻后，若 body 已到 Mid_end，剩余不确定性应主要是 **deskew/外参/b_clock**，而不是 `curvature−mid_span`。

建议公式（**尚未实现**）：

- 完成正确 \(T_{mid\leftarrow odin}\) 后：对 Stage2 用近似常数 \(\sigma_t \approx \sigma_{\mathrm{clock}}\oplus\sigma_{\mathrm{deskew}}\)，或  
- `dt = 0`（已表达在 Mid_end）+ 单独的 clock σ；  
- **不要**在未校正 `b_clock` 前同时改几何与权重。

---

## 6. 是否进入正式算法修改

| 步骤 | 条件 | 本轮 |
|------|------|------|
| Stage2 仅换 Odin body（影子 B） | 运动残差稳定下降且静止不退化 | **未满足** |
| 再改 map_incremental | Stage2 开关验证通过 | **不做** |

最小后续路径：

1. 独立标定 `b_clock`（IMU 互相关 / 静止外参法，**禁止**直接用 `b_pair`）。  
2. 影子 B 改用 `t_odin = H2 + b_clock + span`（仍默认关）。  
3. 若运动 `d_res_med ≤ −5 mm` 且静止 Δ≈0 → 再提 `odin_stage2_xtform_en` 正式开关（可回退）。  
4. 地图插入另开 `odin_map_xtform_en`，分步验证。

---

## 7. 复现

```bash
# yaml 临时：mapping.odin_cross_time_shadow_en: true 后重启 launch
python3 /home/ll/3dnavigation/scripts/odin_cross_time_xtform_eval.py \
  --record-sec 70 --out /tmp/odin_xtform_mixed --label mixed_70s
```

---

## 8. 裁决

**跨时刻变换假设在「H2+span + Mid hist」实现下，未通过真实融合观测残差验证。**  
可视化 D 可继续用作橙云选项；**融合路径保持 A/现行 Stage2**，优先解决物理时钟再谈 SE3。
