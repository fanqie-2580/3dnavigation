# ODIN_CROSS_TIME_EVAL_V2

- **日期**：2026-11-13  
- **基线**：`5113e2b` / `docs/ODIN_CROSS_TIME_TRANSFORM_TEST.md`  
- **本轮目标**：审查影子评价泄漏 → 用冻结 Mid-only 图复评 A/B → `b_clock` 敏感性（≠`b_pair`）→ 五问裁决  
- **约束**：**未改**正式 Stage2 / IEKF / 地图插入门控逻辑 / 外参；仅加审计计数、配对平面统计、影子话题（默认关）  
- **运行二进制**：`fast_lio_multi_mid_odin_ws` 的 `laserMapping_adaptive`（编译 2026-11-13 ~17:10）  
- **脚本**：`scripts/odin_cross_time_eval_v2.py`  
- **数据**：`/tmp/odin_xtform_v2/`（~90s bag + `summary.json`）；旧 V1 bag `/tmp/odin_xtform_mixed/` **无** `/cloud_registered`，不足以建独立 Mid 图  

---

## 给后续 AI 的五句结论

1. **V1「B 无改善」确实可能被评价泄漏/历史偏向夸大**：`map_incremental()` 在 `shadow_odin_cross_time_eval()` **之前**执行；本轮约半数帧把当前 Odin 候选 **100% 插入**正式 ikdtree（A 式投影），静止段尤甚。  
2. **换成冻结 Mid-only 图后，运动段排名翻转为轻度偏 B**（配对 NN 中位 Δ≈−3.7 mm，局部点面 Δ≈−1.7 mm；左转不再像 V1 那样 B 差 +4 mm）。静止段 A≈B。  
3. **相对已发布 `t_odin=H2+span` 的 `b_clock` 扫频，NN 残差呈稳定 U 形，谷底在 ≈0 ms**；这是**几何最优偏移**，**不是**已标定的物理时钟。固定墙符号距离指标本轮失败（~3.4 m 且几乎平坦）。  
4. **仍只构成对「Odin_end→Mid_end 动态重表达」的弱支持**，不足以改正式 Stage2。  
5. **不具备修改正式 Stage2 的证据**（效应量小、物理 `b_clock` 未独立标定、墙面绝对指标失效）。

---

## 1. 评价偏差审查（源码 + 在线审计）

### 1.1 执行顺序（泄漏已确认）

```3266:3274:/home/ll/FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp
                /*** add the feature points to map kdtree ***/
                map_incremental();

                // Shadow cross-time SE3 test (default off). After map update so
                // residuals use Mid-updated ikdtree; never writes Odin / Stage2.
                if (odin_cross_time_shadow_en)
                    shadow_odin_cross_time_eval(feats_undistort, state_stage1_for_odin_,
                                                pubOdinXtformA_, pubOdinXtformB_,
                                                pubOdinXtformMet_);
```

- 影子评分读的是**同一** `ikdtree`。  
- `map_incremental()` 在 `lidar2_map_insert_en=true` 且门控通过时，会把 **A 式** `pointBodyToWorld` 后的 Odin 特征写入该树（`feats_down_body` intensity≥100）。  
- 因此：**当前帧 Odin 点可以进入被影子用来评价 A/B 的地图** → A 被「自插入」有利评价；历史帧若长期以 A 入图，还会形成**系统性偏向 A**。

相关门控（未改逻辑，仅审计）：

```1820:1844:/home/ll/FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp
    const bool block_odin_insert =
        !lidar2_map_insert_en || !odin_pose_update_allowed() || !last_odin_iekf_ok ||
        ...
        now_ros < odin_map_insert_block_until || sync_too_loose_for_map;
    ...
            last_map_odin_cand_++;
            if (block_odin_insert)
            {
                last_map_odin_blk_++;
                continue;
            }
            last_map_odin_ins_++;
```

### 1.2 实际插入比例（V2 bag，`/odin_xtform_metrics` 字段 17–20）

| 量 | 值 |
|----|----|
| 帧数 | 612 |
| `map_block_flag=1` 占比 | **≈49.7%** |
| `ins/cand` 中位 / 均值 | **1.0 / 0.50** |
| still 段 `ins/cand` 中位 | **1.0**（门控通过时整帧插入） |
| translate / combined 中位 | **0.0**（运动常被 `center_dt` / sync 门控挡住） |
| turn_left 中位 | 0.5 |

结论：泄漏**不是每帧都发生**，但在**静止与门控放宽时系统性发生**；V1 用 live ikdtree 评分时无法排除「A 自评」与「历史 A 入图」污染。

### 1.3 平面匹配集合与选择偏差

影子评分现为**同索引配对**（`ok_a && ok_b` 才进中位数；见 `shadow_odin_cross_time_eval`）：

| 在线指标（V2 bag） | 中位 |
|--------------------|------|
| `n_try`（子采样） | 2952 |
| `n_plane_A` / `B` / `both` | 694 / 687 / **482** |
| `planeA−planeB` | **+3**（几乎无偏） |
| `both / n_try` | **≈0.16** |

含义：

- A/B **能拟合平面的点数几乎一致**；V1「只对各自成功点取中位数」的选择偏差在本数据上不大（A−B≈3）。  
- 但 **有效平面点只占尝试点 ~16%**，指标仍是「条件于拟合成功」的残差；配对后消除了集合不一致，**不能**消除「难匹配区域被丢弃」的条件偏置。  
- Mid-only 离线：`frac_plane_both≈0.28`（局部 kNN 平面），`frac_nn_both≈0.88`；报告时同时给 NN / 点面 / 有效比例。

---

## 2. Mid-only 冻结图复评

### 2.1 方法

| 项 | 做法 |
|----|------|
| 测试云 | 复用 `/cloud_odin_xtform_{A,B}`（同帧同索引） |
| 参考图 | `/cloud_registered`（代码保证 **Mid-only**，见 `publish_mid_from_undistorted`） |
| 入图筛选 | \|\ω\|≤0.06 **且** 平移速度 ≤0.05 m/s（禁止「只看角速度当静止」） |
| 坐标系 | 同一 `map` + `/Odometry` |
| 指标 | 配对 NN、局部点面残差、有效比例、固定 RANSAC 墙有符号距离 |
| 分段 | still / turn_left / turn_right / translate / combined |

旧 V1 bag **缺** `/cloud_registered` → **未伪造**独立图；本轮补采 `/tmp/odin_xtform_v2/bag`。

地图摘要：`n_keep=101` 静帧，`n_pts=40000`，RANSAC 平面 4；`reject_w=172`，`reject_v=23`。

### 2.2 分段结果（配对；Δ = B−A，负=B更好）

| 片段 | n | NN_A | NN_B | ΔNN | resA | resB | Δres | B更好占比 | 在线 d_res |
|------|---|------|------|-----|------|------|------|-----------|------------|
| still | 229 | 0.120 | 0.120 | ≈0 | 0.085 | 0.085 | ≈0 | 45% | ≈0 |
| turn_left | 34 | 0.179 | 0.122 | **−3.2 mm** | 0.128 | 0.090 | **−2.7 mm** | **68%** | ≈0 |
| turn_right | 31 | 0.145 | 0.124 | **−4.8 mm** | 0.100 | 0.082 | −1.5 mm | 61% | −0.6 mm |
| translate | 73 | 0.135 | 0.130 | −0.2 mm | 0.099 | 0.094 | −0.2 mm | 52% | −0.3 mm |
| combined | 217 | 0.183 | 0.130 | **−6.9 mm** | 0.130 | 0.089 | **−3.5 mm** | **63%** | −0.5 mm |
| **motion all** | **379** | **0.167** | **0.128** | **−3.7 mm** | **0.119** | **0.089** | **−1.7 mm** | **60%** | −0.4 mm |

与 V1（live ikdtree）对比：

- V1 motion：`d_res≈+0.4 mm`，turn_left **+4.4 mm（B 更差）**  
- V2 Mid-only：motion **−1.7 mm**，turn_left **−2.7 mm（B 更好）**  
→ **泄漏/历史偏向足以解释 V1「B 无改善」的部分甚至方向性结论**；独立图后方向翻转，但幅度仍小。

### 2.3 失败 / 不可靠案例

1. **固定参考墙有符号距离**：A/B 中位均 ≈ **+3.43 m**，宽度指标近乎常数 → 本场景 RANSAC「墙」未对齐有效表面（失败，**不用于裁决**）。  
2. **body 逆变换近似**：`b_clock` 扫频用 `/Odometry` 近似 \(T_{WL}\)，忽略正式 \(T_{WI}T_{IL}\) → 扫频有系统误差，只作相对曲线。  
3. **在线 ikdtree 仍略偏 B**（本 bag `d_res` 中位 −0.11 mm）：与 V1 不一致，说明 **live 评分对运动内容/插入状态敏感**，不能单独当真理。

---

## 3. `b_clock` 敏感性（影子路径，不改滤波器）

定义（与文档约定一致）：

- 发布 `t_odin = H2 + span`（D 语义）  
- 候选：`t_query = t_odin + b_clock`  
- **禁止**默认 `b_clock=0` 或 `b_clock=b_pair` 当作物理真值  

本 bag：`dt=(H2+span)−Mid_end` 中位 **≈ −120 ms**；`b_pre≈b_pair` 中位 **≈ +113 ms** → 再次确认 **USB 端戳差被配对偏置吸收，≠ 可直接当 SE3 时钟**。

### 3.1 运动段扫频（相对 H2+span；NN 中位）

| b_clock / ms | NN p50 |
|-------------|--------|
| −150 | 0.195 |
| −120 | 0.183 |
| −100 | 0.178 |
| −80 | 0.167 |
| −60 | 0.155 |
| −40 | 0.148 |
| −20 | 0.138 |
| **0** | **0.129（谷）** |
| +20 | 0.132 |
| +40 | 0.140 |
| +60 | 0.148 |
| +80 | 0.155 |
| +100 | 0.165 |
| +120 | 0.174 |

- **几何最优区间**：约 **[−20, +20] ms**（相对已发布 H2+span），谷在 **0**。  
- 墙符号「res」扫频几乎平坦且绝对值 ~3.4 m → **作废**。  
- **几何最优 ≠ 物理时钟标定**：0 ms 只说明「在当前 odom 近似下，用发布的 H2+span 做 B 式投影 NN 最好」；真实 `b_clock` 仍需 IMU 互相关 / 外参静止法独立求。

### 3.2 Odin header vs Mid 时间轴

| 观测 | 解释 |
|------|------|
| `dt≈−120 ms ≈ −b_pair` | Odin USB `H2+span` 相对 Mid_end 系统偏早一个配对偏置量级 |
| 橙云 D 在静止 Mid 图上更好 | 用偏早姿态贴静态墙的可视化效应 |
| Stage2 / 影子相对 **当前 Mid 地图时间** | 需要的是物理对齐后的 \(T(\mathrm{Odin\_end})\)，不是裸 H2+span |

---

## 4. 五问裁决

| # | 问题 | 裁决 |
|---|------|------|
| 1 | V1「B 无改善」是否可能由泄漏/历史偏向造成？ | **是（很可能）**。顺序上当前帧可 A 式入图；~50% 帧 `ins/cand=1`；独立图后运动段方向翻转。 |
| 2 | Mid-only 后 A/B 优劣是否改变？ | **改变**：由「B 不更好 / 左转更差」→「运动轻度偏 B，静止持平」。效应量小（数 mm）。 |
| 3 | 不同 `b_clock` 是否有稳定最优区间？ | **有（几何）**：相对 H2+span 的 NN U 形，谷 ≈0±20 ms。**不是**物理标定完成。 |
| 4 | 是否仍支持 Odin_end→Mid_end 动态重表达？ | **弱支持 / 观察中**：独立图上运动 NN/点面略偏 B，但未达可上线阈值；时钟未标定。 |
| 5 | 是否具备改正式 Stage2 的证据？ | **否**。 |

---

## 5. 源码位置（可复核）

| 项 | 路径 |
|----|------|
| 入图→影子顺序 | `laserMapping_adaptive.cpp` ~3267–3274 |
| 入图审计计数 | 同文件 `last_map_odin_*`，`map_incremental` ~1825–1844 |
| 影子 A/B + 配对平面 | `shadow_odin_cross_time_eval` ~757–1028 |
| metrics v2 布局 | 同函数：字段 14–20 = planeA/B/both + map cand/ins/blk/flag |
| Mid-only 发布 | `publish_mid_from_undistorted` → `/cloud_registered` |
| 开关（默认 false） | `config/mid360_odin1.yaml` → `mapping.odin_cross_time_shadow_en` |
| 离线复评 | `scripts/odin_cross_time_eval_v2.py` |

---

## 6. 复现

```bash
# 临时打开影子后重启 mid360_odin1_multi.launch.py
# 录制（需含 Mid 云）：
ros2 bag record -o /tmp/odin_xtform_v2/bag --compression-mode none \
  /cloud_odin_xtform_A /cloud_odin_xtform_B /odin_xtform_metrics \
  /cloud_registered /Odometry /livox/imu

python3 /home/ll/3dnavigation/scripts/odin_cross_time_eval_v2.py \
  --bag /tmp/odin_xtform_v2/bag --out /tmp/odin_xtform_v2 \
  --b-clock-ms='-150,-120,-100,-80,-60,-40,-20,0,20,40,60,80,100,120'
```

录制后务必将 `odin_cross_time_shadow_en` **改回 false**（本轮已恢复）。

---

## 7. 后续最小路径（仍不改 Stage2）

1. 独立标定物理 `b_clock`（禁止用 `b_pair`）。  
2. 影子 B 用 `t = H2 + b_clock + span` 再跑 Mid-only；目标：运动 `d_res_pair ≤ −5 mm` 且静止 |Δ|≈0。  
3. 评价路径改为 **影子前不入 Odin** 或专用 Mid-only ikdtree，避免再泄漏。  
4. 达标后再提 `odin_stage2_xtform_en` 可回退开关。

---

## 8. 实验产物

| 路径 | 内容 |
|------|------|
| `/tmp/odin_xtform_v2/bag` | 90s 原始 bag |
| `/tmp/odin_xtform_v2/summary.json` | 本报告数值源 |
| `/tmp/odin_xtform_v2/eval.log` | 脚本日志 |
| `/tmp/odin_xtform_mixed/` | V1（无 Mid 云，仅对照） |
