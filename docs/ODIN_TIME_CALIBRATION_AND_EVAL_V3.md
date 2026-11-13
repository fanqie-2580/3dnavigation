# ODIN_TIME_CALIBRATION_AND_EVAL_V3

- **日期**：2026-11-13  
- **基线**：`6eae715` / `docs/ODIN_CROSS_TIME_EVAL_V2.md`  
- **约束**：**未改**正式 Stage2 / IEKF / 地图插入策略 / 外参 / 门控；仅修正**审计计数**与离线评价脚本  
- **数据**：`/tmp/odin_xtform_v3/`  
  - `mid_only_bag/`：`lidar2_update_en=false`，无 Odin Stage2 / 无 Odin 入图  
  - `fusion_bag/`：融合 + 影子 A/B + 原始时间轴话题  
  - `summary.json`：独立 Mid 图 + ICP 对齐后的主结果  
  - `fusion_pose_map/summary.json`：同 A/B 在融合位姿 Mid 云上的对照  
- **脚本**：`scripts/odin_time_calibration_eval_v3.py`  
- **源码**：`FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp`（`map_incremental` / `shadow_odin_cross_time_eval`）

---

## 给后续 AI 的五句结论

1. **V2「ins/cand≈100%」夸大了当前帧入图泄漏**：该字段实为 **门控通过**；真正进入 `PointToAdd/PointNoNeedDownsample` 的 Odin 点仅约 **queued/cand ≈ 0.8%–1.3%**（门控开启帧）。  
2. **V2 参考图存在融合位姿依赖**：`/cloud_registered` 点是 Mid-only，但投影用的是 Stage2 后的融合 `/Odometry`。本轮用独立 Mid-only IEKF 建图 + ICP 对齐到融合静止图。  
3. **独立基准下 B 不再呈现稳定改善**（运动 n=18：Δres≈+0.9 mm，ΔNN≈−0.7 mm → similar）；样本运动不足与 ICP 残差 ~7.5 cm 是明确失败/局限。  
4. **补全 \(T_{WI}T_{IL}\) 后，`b_clock` 几何谷仍在相对 `H2+span` 的约 0–40 ms**；忽略外参时谷在 0 ms——对位姿模型敏感，但仍靠近 0，且 **≠ 物理时钟标定**。  
5. **不具备进入可回退 Stage2 试运行的证据。**

---

## 1. 地图插入统计修正

### 1.1 问题

V2 在门控通过后立刻 `last_map_odin_ins_++`，**早于**距离筛选、体素 `need_add` 与 `ikdtree.Add_Points`。

### 1.2 现字段（metrics layout v3）

| 字段 | 含义 |
|------|------|
| 17 `cand` | Odin 候选（intensity 标记） |
| 18 `gate_pass` | 门控通过（旧 V2「ins」） |
| 19 `blk` | 门控拦截 |
| 20 `block_flag` | 本帧是否整帧拦截 |
| 21 `det_pass` | 通过 `DET_RANGE` |
| 22 `queued` | 进入 `PointToAdd` ∪ `PointNoNeedDownsample` |
| 23 `ikd_add_total` | `Add_Points` 返回值之和（**整帧 Mid+Odin**） |
| 24/25 | queued 的 downsample / raw 分流 |

关键函数：`map_incremental()`；发布：`shadow_odin_cross_time_eval()`。

### 1.3 ikdtree API 限制（必须保留）

`KD_TREE::Add_Points`（`include/ikd-Tree/ikd_Tree.cpp`）：

- 返回值只在 `downsample_on=true` 时累加体素接受数；  
- `downsample_on=false`（`PointNoNeedDownsample`）**仍插入但不增加返回计数**；  
- 本树**无法按传感器**区分 Mid/Odin，除非拆分两次 `Add_Points`（会改变插入顺序 → **本轮不做**）。  

因此 **不能**把 `ikd_add_total` 当成「Odin 实际新增点数」。

### 1.4 本轮 fusion_bag 实测（n=363 metrics）

| 量 | 值 |
|----|----|
| `block_flag=1` 占比 | **45.5%** |
| 开启帧 `gate/cand` 中位 | **1.0** |
| 开启帧 `det/cand` 中位 | **1.0** |
| 开启帧 `queued/cand` 中位 | **≈1.3%**（约 21/1641） |
| 全局 `queued/cand` 中位 | **≈0.81%** |
| 开启帧 `ikd_add_total` 中位 | 15（Mid+Odin 混合，且 raw 路径不计返回值） |

### 1.5 对 V2「泄漏比例」的重评

| V2 说法 | V3 修正 |
|---------|--------|
| 约半数帧「100% Odin 插入」 | 约半数帧 **门控放行 100% 候选**；真正入队仅 ~1% |
| 当前帧 A 自插入严重污染影子评分 | **当前帧直接泄漏很弱**；顺序上仍在影子前入图，但体素数极少 |
| 历史 A 入图偏向 | **仍可能存在**（长期累加的 queued Odin），但幅度不能用 V2 的 100% 推断 |

**结论**：泄漏机制（顺序）仍在；**比例被 V2 高估约两个数量级**。V2「B 在 live ikdtree 上不改善」不能主要归因于「当前帧 100% 自插入」。

---

## 2. 参考基准与融合位姿依赖

### 2.1 依赖关系（明确）

```text
/cloud_registered  = Mid 点（publish_mid_from_undistorted）
投影姿态           = state_point（Stage1+可选 Stage2 Odin 更新之后）
/Odometry          = 同上融合轨迹
```

因此 V2「Mid-only 点云图」= **Mid 几何 + 融合位姿**。Odin Stage2 可通过轨迹间接污染参考图。

### 2.2 本轮独立基准

1. **Mid-only IEKF**：临时 `lidar2_update_en=false`、`lidar2_map_insert_en=false`（实验后已恢复正式配置）→ `mid_only_bag`  
2. 静止帧建图（|ω|≤0.06 且 |v|≤0.05）  
3. **ICP** 将独立图对齐到 fusion 静止 Mid 云（同物理场景）  
4. 对 fusion 的 `/cloud_odin_xtform_{A,B}` **同索引**评 NN + 局部点面残差  
5. **不再**使用已失败的固定墙面距离作证据  

ICP：`n_corr≈6038`，中位残差 **≈7.5 cm**（偏大 → 对齐质量有限，见失败案例）。

### 2.3 A/B 结果（独立对齐图）

| 集合 | n | ΔNN (B−A) | Δres (B−A) | 解读 |
|------|---|-----------|------------|------|
| still | 338 | ≈0 | ≈0 | 持平 |
| motion | **18** | −0.7 mm | **+0.9 mm** | similar；样本不足 |
| combined | 9 | −4.2 mm | −3.9 mm | 方向偏 B，n 小 |
| turn_right | 5 | +5.2 mm | +5.3 mm | 偏 A，n 小 |
| turn_left | 0 | — | — | **缺失** |

同 A/B 在 **融合位姿 Mid 图**上：motion Δres≈+1.1 mm，ΔNN≈−2.1 mm → 同样 **similar**，未再现 V2 的「运动稳定偏 B ~1.7 mm」。

**结论**：在独立（或同会话融合位姿）基准下，**B 无稳定可复现改善**；V2 的 −1.7 mm 对数据/地图敏感。

---

## 3. `b_clock` 可信度（完整 \(T_{WI}T_{IL}\)）

### 3.1 修复

V2 扫频把 `/Odometry` 当成 \(T_{WL}\)。正式：

\[
p_W = R_{WI}(R_{IL}p_L + t_{IL}) + t_{WI}
\]

V3 使用 yaml 外参 `extrinsic_T=[-0.011,-0.02329,0.04412]`，`R=I`（`extrinsic_est_en:false`），与 `pointBodyToWorld` / `lidar_to_world` 一致。

### 3.2 扫频（相对已发布 `t_odin=H2+span`；运动 n=18）

| b_clock / ms | NN（full） | res（full） | NN（naive） |
|-------------|------------|-------------|-------------|
| −80 | 0.116 | 0.079 | 0.130 |
| −40 | 0.104 | 0.077 | 0.117 |
| −20 | 0.099 | 0.075 | 0.111 |
| **0** | **0.095** | **0.071（谷）** | **0.105（谷）** |
| +20 | 0.096 | 0.071 | 0.107 |
| **+40** | **0.095（NN 谷）** | 0.075 | 0.109 |
| +80 | 0.101 | 0.076 | 0.110 |

- **几何最优仍靠近 0**：full 的点面谷在 **0 ms**，NN 谷在 **0–40 ms**；naive NN 谷在 **0 ms**。  
- **对模型敏感**：full vs naive 的 NN 谷可差约 40 ms；绝对值也不同。  
- **对参考图敏感**：独立图 vs 融合位姿图的 best NN 分别为 40 ms / 20 ms。  
- **仍不是物理时钟**：禁止把该谷、`b_pair`、双 IMU 互相关或扫描相位当成 LiDAR 真时钟。

### 3.3 时间轴关系（header / timebase，非时钟标定）

| 关系 | 中位 |
|------|------|
| Mid `CustomMsg.header` − Mid IMU | ≈ **−0.7 ms** |
| Mid end − Mid header | ≈ **+100 ms**（扫描时长） |
| Odin `cloud_raw_ds.header` − Odin IMU | ≈ **0 ms** |
| Odin IMU − Mid IMU | ≈ **0 ms**（p10/p90 ≈ ±2 ms） |

解释：两路 IMU 与各自 LiDAR header 在宿主机时间上近似对齐；影子里 `dt=(H2+span)−Mid_end≈−80…−160 ms` 来自 **Odin USB 戳/span 语义相对 Mid_end**，与「IMU 互相关 = LiDAR 时钟」不是同一对象。`b_pair≈110 ms` 是配对偏置，**不得**当作 `b_clock`。

---

## 4. 四问裁决

| # | 问题 | 裁决 |
|---|------|------|
| 1 | 几何最优 `b_clock` 是否仍接近 0 ms？ | **是（约 0–40 ms，相对 H2+span）**；点面谷在 0。 |
| 2 | 是否对参考地图与位姿模型敏感？ | **是**（独立/融合图、full/naive 外参均移动谷点与残差水平）。 |
| 3 | B 在独立基准下是否稳定改善？ | **否**（本轮 similar；运动样本不足 + ICP 弱）。 |
| 4 | Stage2 是否有足够证据进入可回退试运行？ | **否**。 |

---

## 5. 失败案例与局限

1. **运动段过少**（fusion 录制偏静止：motion=18，无 turn_left）→ 分段结论不可外推。  
2. **ICP 中位 7.5 cm** → 独立图对齐质量不足，可能淹没 mm 级 A/B 差。  
3. **`ikd_add_total` 不可按雷达归因**；raw 插入不计返回值。  
4. **V2 泄漏比例误读** → 历史报告中「100% 插入」应改为「100% 门控通过」。  
5. 固定墙面距离指标 **继续作废**（本轮未用作证据）。

---

## 6. 修改文件

| 文件 | 变更 |
|------|------|
| `FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp` | 入图分阶段审计；metrics v3；日志 `cand/gate/det/q` |
| `scripts/odin_time_calibration_eval_v3.py` | 独立 Mid 图、ICP、完整 \(T_{WI}T_{IL}\) 扫频、时间轴分析 |
| `docs/ODIN_TIME_CALIBRATION_AND_EVAL_V3.md` | 本报告 |
| `docs/ODIN_TIME_CALIBRATION_AND_EVAL_V3_summary.json` | 主结果副本 |
| `docs/ODIN_TIME_CALIBRATION_AND_EVAL_V3_fusion_pose_summary.json` | 对照 |

正式 yaml 已恢复：`odin_cross_time_shadow_en: false`，`lidar2_update_en: true`，`lidar2_map_insert_en: true`。

---

## 7. 复现

```bash
# Mid-only 建图（临时）：lidar2_update_en=false, lidar2_map_insert_en=false
# 融合影子（临时）：odin_cross_time_shadow_en=true 后录 fusion_bag

python3 /home/ll/3dnavigation/scripts/odin_time_calibration_eval_v3.py \
  --fusion-bag /tmp/odin_xtform_v3/fusion_bag \
  --mid-only-bag /tmp/odin_xtform_v3/mid_only_bag \
  --time-bag /tmp/odin_xtform_v3/fusion_bag \
  --out /tmp/odin_xtform_v3 \
  --b-clock-ms='-150,-120,-100,-80,-60,-40,-20,0,20,40,60,80,100,120'
```

---

## 8. 运动重录补测（2026-11-13 18:21，120s）

- **数据**：`/tmp/odin_xtform_v3/fusion_bag_motion/`（A/B=264，metrics=264）  
- **评价输出**：`motion_reeval/summary.json`（独立 Mid+ICP）、`motion_reval_fusion_pose/summary.json`（融合位姿对照）  
- **运动覆盖**：still 41 / turn_left 22 / turn_right 35 / translate 20 / combined **92** / other 38 → **motion n=207**（已补足 V3 初版 n=18 的缺口）

### 8.1 独立 Mid 图 + ICP（ICP med≈13.2 cm，偏弱）

| 片段 | n | ΔNN / mm | Δres / mm | B更好占比 |
|------|---|----------|-----------|-----------|
| still | 41 | −0.05 | +0.23 | 44% |
| turn_left | 22 | −1.4 | −0.27 | 50% |
| turn_right | 35 | +2.8 | +1.1 | 34% |
| translate | 20 | −1.8 | −0.37 | 55% |
| combined | 92 | +0.9 | +0.81 | 45% |
| **motion** | **207** | **+0.74** | **+0.53** | **43%** |

→ 独立基准下整体 **similar / 略偏 A**，无稳定 B 改善。

### 8.2 同 A/B、融合位姿 Mid 图（对照）

| 集合 | n | ΔNN / mm | Δres / mm | B更好占比 |
|------|---|----------|-----------|-----------|
| motion | 207 | **−2.5** | **−1.5** | **60%** |
| combined | 92 | — | −2.9 | — |

→ 与 V2「融合位姿图上轻度偏 B」一致；**一换独立图，优势消失**。说明 V2/对照的 B 改善对「融合轨迹污染的参考图」敏感。

### 8.3 `b_clock`（完整 \(T_{WI}T_{IL}\)，运动扫频 n=104）

| 模型 | NN 谷 | 备注 |
|------|-------|------|
| full \(T_{WI}T_{IL}\) | **0 ms** | 曲线平坦，绝对值因 ICP 偏大（NN≈0.30 m） |
| naive（忽略外参） | **−40 ms** | 再次证明对位姿模型敏感 |
| 点面 full | 120 ms | 与 NN 谷不一致 → 不作物理时钟 |

### 8.4 补测后四问（更新）

| # | 裁决 |
|---|------|
| 1 几何最优仍近 0？ | **是**（full NN 谷 = 0 ms） |
| 2 对地图/模型敏感？ | **是**（独立 vs 融合位姿：B 优劣翻转；full vs naive：谷点 0 vs −40） |
| 3 独立基准 B 稳定改善？ | **否**（motion Δres≈+0.5 mm） |
| 4 Stage2 试运行？ | **否** |

局限：独立图来自更早 `mid_only_bag`，与本次运动场景 ICP 仅 ~13 cm；理想情况下应同场同地重采 Mid-only 图。但不改变「独立基准未稳定支持 B」的方向。

---

## 9. 后续（仍不改 Stage2）

1. 同场同地重采 Mid-only 图（降低 ICP 残差）后再评一次。  
2. 物理 `b_clock` 用独立方法标定；扫频谷仅作几何参考。  
3. 若需量化历史入图偏向，对 ikdtree 做 Mid/Odin 分色统计或影子前禁用 Odin queued（仅评价开关，正式策略另议）。
