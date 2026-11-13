# TIME_QUERY_ABLATION_V2

- **日期**：2026-11-13
- **相对**：`3fbc15a` / `docs/TIME_QUERY_ABLATION.md`（V1）
- **约束**：未改正式融合路径、滤波器、门控、外参
- **脚本**：`scripts/time_query_ablation_v2.py`
- **数据**：`/tmp/tq_ablation_v2_bags/{still,turn}`（无压缩 rosbag2）
- **产出**：`/tmp/tq_ablation_v2_out/summary.json`

---

## 1. 对复核意见的审计（是否属实）

| # | 质疑 | 审计结论 | 依据 |
|---|------|----------|------|
| 1 | 高运动互相关把 Mid 掩码零值同时乘到两路 IMU，偏向零延迟 | **属实** | V1 `imu_clock_offset()`：`w[~mask]=0` 后对 `Mm*w` 与 `Oo*w` 做相关；相同零区强迫对齐 |
| 2 | Odin IMU 不足，deskew 位移为 0 | **属实（V1）** | V1 实时订条约 5Hz；本 V2 bag：Odin≈399Hz，deskew `disp_med≈4.6cm`，`ok_frac=1.0` |
| 3 | `build_static_map` 用 `Odometry.twist.angular.z`（常为 0） | **属实** | V1 用 `po.wz`；本栈 twist 常 0 → 运动帧可混入“静止图” |
| 4 | `ghost_p90` 只是 NN，不是墙面重影厚度 | **属实** | V1 `ghost_thickness` = NN p50/p90 |
| 5 | 离线用 `/Odometry`（Stage2），在线橙云用 Stage1 | **属实（设计差）** | 源码 Stage1 存 hist、橙云用 `state_point_odin_pub_`；`/Odometry` 在 Stage2 后发布 |

### V2 对应修正

1. **互相关**：全窗相关为主；高运动段为**裁剪子序列**（无共享零填充）；短段若落在 ≈0 而全窗 ≥3ms，则采用全窗（本轮 `L_used_source=full`，`b_clock_proxy=10ms`）。  
2. **录制**：rosbag2 无压缩；IMU 频率/连续性见 §2。  
3. **静止图**：独立 still bag + **Mid IMU \|ω\|≤0.06** 选帧（40 帧 / 5e4 点）。  
4. **墙面度量**：RANSAC 平面 + 有符号距离 **IQR / 5–95% 宽度**；同时保留 NN 与点面残差。  
5. **位姿**：全方案共用 `/Odometry`（Stage2）轨迹，保证 A/B/C/D **相对可比**；**不声称**等于在线 Stage1 橙云。  
6. **Deskew**：忠实移植 `UndistortOdinNative` 相对 SE3（`bg=ba=0`，自估 `acc_scale`）。**仍非**在线进程内存导出；未改 `laserMapping` 故无 Stage1 dump。

---

## 2. 数据包质量（可信度：高）

| 流 | still | turn |
|----|-------|------|
| Mid IMU | 203 Hz，gap≈0 | 202 Hz，gap≈0.05% |
| Odin IMU | 399 Hz，gap≈0.05% | 399 Hz，gap≈0.02% |
| 时长 | 10.7 s | 50.7 s |
| 转角帧 \|ω\|≥0.25 | — | **n=277**（wz+ 131 / wz− 137） |

V1「IMU 饿死 / deskew=0」在 V2 **已排除**。

---

## 3. 方案定义（除 `t_query` 外全同）

| 方案 | `t_query` |
|------|-----------|
| A | `Mid_center+(span−med)`（现行） |
| B | `H2 + b_pre + span`（配对前 bias≈111–120ms） |
| C | `H2 + 10ms + span`（**IMU 代理**，低置信） |
| D | `H2 + 0 + span` |

同 deskew 点云、同外参、同静止参考图、同 Stage2 轨迹。

---

## 4. 转角主结果（n=277，mean \|ω\|≈0.58）

| 方案 | res_med (m) | nn_p50 (m) | wall_thick (m) | wall_iqr (m) | corr(w,res) |
|------|-------------|------------|----------------|--------------|-------------|
| A | 0.099 | 0.118 | 0.600 | 0.300 | **+0.29** |
| B | 0.100 | 0.117 | 0.600 | 0.300 | **+0.31** |
| C | **0.071** | **0.085** | 0.599 | 0.300 | −0.16 |
| D | **0.071** | **0.084** | 0.599 | 0.300 | −0.15 |

排序（转角 res / nn）：**C ≈ D ≪ A ≈ B**。

Deskew：全帧成功；位移中位 **4.6 cm**；deskew 使点面残差略降（~0.5–0.6 cm），属**帧内**效应，远小于 A→D 的整帧改善（~2.8 cm res_med）。

墙面 thick/IQR 四方案几乎贴合 → **本度量对 0–100ms 查询差不敏感**（平面带宽/地图噪声占主导）；**判别力来自 res_med 与 nn_p50**，以及 **残差随 ω 的相关号**（A/B 正相关，C/D 无）。

---

## 5. V1 结论：哪些仍成立 / 哪些撤回

### 仍成立

1. **配对 `b` ≠ 物理时钟**：V2 仍见 `b_pre` 中位 ~112ms、可顶满 120ms；IMU 代理仅 ~10ms。  
2. **不要实施 B（`H2+b_pair+span`）**：转角上 B **不优于** A（res 略差）。  
3. **`|center_dt|~25ms` 为扫描相位主导**（与 PHYSICAL 报告一致）；用中心吸附的 A 在转角残差随 ω 变差。  
4. **小 header 偏移（C/D）优于 Mid 中心吸附（A）**——方向与 V1 一致，且样本从 n=4 → **n=277**，左右均衡。

### 撤回或降级

1. **撤回 V1 把 `ghost_p90`(NN) 当作墙面重影厚度的定量解读。**  
2. **撤回 V1「高运动加权互相关 ≈0–2ms」作为更准时钟**——该方法有零掩码伪影；修正后全窗约 **10ms**，短段仍易假 0。  
3. **降级 V1「C 明显优于 A 的幅度/置信」**——当时 n=4、IMU 不足、deskew 无效；幅度不可引用。  
4. **V1 静止图可能污染** → 相关绝对残差水平不可与 V2 横比。

---

## 6. 物理时间偏移是否已确认？

**否。**

- C 的 10ms 仅是 **IMU 角速度互相关代理**，明确 **不是** DTOF `header` 标定。  
- 转角上 **C≈D**（0 vs 10ms 分不出）→ 在当前噪声与 Stage2 轨迹下，**不足以**锁定激光物理偏移数值。  
- 能确认的是：**把 Odin 结束时刻吸附到 Mid 扫描中心（A）有害**；**用被相位污染的配对 bias（B）无益**。

---

## 7. 唯一值得实施的最小修改

**改橙色（及同类）`t_query`：停止 `Mid_center+(span−med)`，改为**

```text
t_query = H2 + b_clock + odin_span
```

其中 **`b_clock` 为可配置小常量，默认 0（即方案 D）**，允许之后用标定替换；**禁止**使用当前配对 EMA `b_pair`（~115ms）。

**理由（由本 V2 支撑）：**

- D/C 转角点面残差与 nn_p50 **相对 A 改善约 28%**（0.099→0.071 / 0.118→0.084），且残差不再随 ω 恶化。  
- B 验证失败 → 「物理报告里的 H2+b+span」在 **`b=b_pair` 时不可直接落地**。  
- D 比 C 更简，且本数据 C≈D；在物理偏移未标定前，**默认 0 是最小、可逆、可证伪**的一步。

**验证（合入后）：** 再录左右转 bag，对比橙云转角 nn_p50 / res_med；期望接近本离线 D，且明显好于改前 A。仍不要加 ω 门。

**仍未做、需另开任务：** 从在线进程导出真实 Stage1 位姿 + `UndistortOdinNative` 点云（需临时调试接口，本轮按约束未改融合）。

---

## 8. 复现

```bash
# bags already at /tmp/tq_ablation_v2_bags/{still,turn}
python3 scripts/time_query_ablation_v2.py \
  --still-bag /tmp/tq_ablation_v2_bags/still \
  --turn-bag /tmp/tq_ablation_v2_bags/turn \
  --bias-init 0.111 \
  --out /tmp/tq_ablation_v2_out
```
