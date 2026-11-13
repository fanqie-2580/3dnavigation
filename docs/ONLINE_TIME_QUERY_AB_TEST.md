# ONLINE_TIME_QUERY_AB_TEST

- **日期**：2026-11-13
- **相对**：`docs/TIME_QUERY_ABLATION_V2.md`（离线 A/B/C/D）
- **约束**：正式 IEKF / 地图 / `/cloud_odin_registered` 未改逻辑；仅增加默认可关调试分支
- **代码**：`FAST_LIO_MULTI_ROS2_mid_odin` → `laserMapping_adaptive`
- **参数**：`mapping.publish_odin_time_query_ab_en`（默认 `false`）
- **脚本**：`scripts/online_time_query_ab_eval.py`
- **数据**：`/tmp/online_tq_ab_out/bag`（36s，无压缩）
- **产出**：`/tmp/online_tq_ab_out/summary.json`

---

## 1. 在线对照定义

同一帧 **UndistortOdinNative** 点云 + **Stage1** `mid_pose_hist_`，仅 `t_query` 不同：

| 方案 | `t_query` | Topic |
|------|-----------|-------|
| A | `Mid_center + (span − med)`（现行橙云） | `/cloud_odin_registered_A` |
| D | `H2 + 0 + span`（原始 Odin 结束戳） | `/cloud_odin_registered_D` |
| 正式橙云 | 与 A 同公式 | `/cloud_odin_registered`（不变） |

规则：

- 仅真实 Bundle Odin 帧发布 A/D；Mid-only **不** hold 重发
- 位姿查询覆盖：`interp` / 短 `extrap` 才发布；`clamp_*` 跳过并打日志
- 日志字段：`H2`、`span`、`med`、`center_dt`、`b_pre`、`tA`/`tD` vs Mid_end、coverage

启用（验证后请关回）：

```bash
# yaml 或 launch 覆盖
mapping.publish_odin_time_query_ab_en: true
```

---

## 2. 实现副作用检查

| 项 | 结果 |
|----|------|
| 默认关闭 | `publish_odin_time_query_ab_en=false` |
| 正式橙云 | `official_vs_A.off_minus_A_med = 0`（140 帧） |
| IEKF / Stage2 / 地图插入 | 未改 |
| 配对 bias EMA | 仅多记 `bundle_bias_pre_sec_` 快照 |

---

## 3. 在线主结果（index 对齐，n=140）

参考图：静止 Mid `/cloud_registered`（\|ω\|≤0.06，5e4 点）。度量：对参考图的 NN p50/p90。

| 子集 | n | mean \|ω\| | A nn_p50 | D nn_p50 | Δ(D−A) | D 更好占比 |
|------|---|------------|----------|----------|--------|------------|
| 全部 | 140 | 0.34 | 0.136 | **0.112** | **−1.7 cm** | 69% |
| 转角 \|ω\|≥0.25 | **80** | 0.52 | 0.148 | **0.113** | **−3.8 cm** | **95%** |
| 近似静止 \|ω\|\<0.08 | 22 | 0.04 | 0.109 | 0.108 | ≈0 | — |

`dt_AD = stamp(D)−stamp(A)` 中位 ≈ **−124 ms**（与日志 `tD_vs_end − tA_vs_end`、`b_pre≈110–120 ms` 一致）：D 在 Mid 时钟上更早。

日志抽样：`tA` 多为 `interp`（偶发短 `extrap`）；`tD` 均为 `interp`（本段无 clamp 丢帧）。

---

## 4. 与离线 V2 对照

| | 离线 V2（Stage2 轨迹 + 自 deskew） | 在线（Stage1 + 进程内 Native） |
|--|-----------------------------------|--------------------------------|
| 转角排序 | C≈D ≪ A≈B | **D ≪ A** |
| 转角改善量级 | res_med ~2.8 cm | nn_p50 ~3.8 cm |
| 静止 | A≈D | A≈D |
| B | 已否决 | 未再测 |

结论一致：**物理向的 D（H2+span）在转角优于现行 A**；配对 `b_pre` 不是把橙云查到正确物理时刻的量。

---

## 5. 裁决（本轮）

**在线转角：D 胜 → 可作为替换 A 的候选。**

**暂不删除 / 不切换正式 A：**

1. 仅一轮 36s / 80 转角帧；需复现左右转与更长路径  
2. D 依赖 Odin/Mid 戳同域；`b` 顶满 120ms 时 `tD` 可到 Mid_end−170ms，需确认 hist 覆盖长期稳定  
3. 正式切换应改 `odin_pose_at_scan_end` 查询公式，并保留 A 调试 topic 一段时间做回归  

建议下一步（未做）：将正式 `t_query` 切到 D，保留 `/cloud_odin_registered_A` 对照；再跑同脚本确认 `official ≡ D` 且转角不回退。

---

## 6. 复现命令

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/fast_lio_multi_mid_odin_ws/install/setup.bash

# 临时打开调试后重启 launch，录制+评估：
python3 /home/ll/3dnavigation/scripts/online_time_query_ab_eval.py \
  --record-sec 36 --out /tmp/online_tq_ab_out

# 或对已有 bag：
python3 /home/ll/3dnavigation/scripts/online_time_query_ab_eval.py \
  --bag /tmp/online_tq_ab_out/bag --out /tmp/online_tq_ab_out
```
