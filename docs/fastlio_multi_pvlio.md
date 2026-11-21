# fastlio_multi_pvlio（新建，不碰 dual）

工作区：`/home/ll/fastlio_multi_pvlio_ws`  
包名：`fastlio_multi_pvlio`

## 意图

在**不修改** `pv_lio_dual_ws` 的前提下，新建一条线：

- **紧耦合**：FAST-LIO-MULTI Bundle（双雷达拼帧 + 主 IMU 去畸变 + **一次**联合 IEKF）
- **建图**：PV-LIO **体素地图**（不是 FAST-LIO ikd-Tree）
- **传感器**：Mid360 + Odin1 适配

官方 MULTI 源码只读参考：`fastlio_multi_pvlio_ws/reference/FAST_LIO_MULTI_ROS2_upstream/`

## 与 dual 的区别

| | dual (`pv_lio_dual_ws`) | 本树 |
|--|--|--|
| IEKF | Mid S1 → 可选 Odin S2 + 门控 | 设计为合并点云一次 Bundle；实践上 Mid↔Odin `end_dt>50ms` 会本帧 Mid-only |
| densify / residual_z | 有 | 有 residual_z（楼面 info 软化 + multilevel H_z；无 densify） |
| 地图 | PV 体素 | PV 体素 |

## 注意（RViz 图/路径对不齐）

`/Global_cloud_map` 点坐标与 `/path` 同属 `camera_init`。若 `frame_id` 误标 `world`，再经 `world→camera_init = Initial_R_wrt_G`（重力对齐），Fixed Frame=`camera_init` 时图会被多旋一次，路线看起来斜出走廊。

## 注意（末段漂）

`lidar2_soft_wait` 可能在一个 Mid 帧下攒多帧 Odin。若把 buffer 全部 `+=` 进 Bundle，会出现 `l2_pts≈2×` 的坏更新并触发 Divergence。现改为只取与 Mid 结束时刻最近的**一帧** Odin。

回程末段另见：Mid-only 后再接回 Bundle 时，Odin 可在 XY 几乎不动的情况下把 **Z 在 ~2s 内拉高 >1 m**。仅对**含 Odin** 的帧做 `|Δz|>0.15` / `d_upd>0.35` 拒更；拒更后仍写入 Mid 点（避免 skip-map → No Effective Points → IMU 崩，见 run6）。

run7 数据：同走廊 y≈−40 去程 z≈4.9 / 回程 z≈1.0（Δz≈3.9）、横向偏≈1.5 m → 叠影。曾加 `odin_bundle_z_band`，但 run9 中 band 跟着错误 Z 下沉 → `No Effective Points` → IMU 自由落体（z→−2900）。现默认 band=0；匹配失效时冻结 last_good 并短暂 Mid-only。

高度：默认 **关** `odin_z_replace`（硬改 Z 会撕裂 Bundle 图/XY，见 run11）。改为 dual 同款 `residual_z`：软化 Mid/Odin 楼面信息、单层压 H_z、跨层后恢复满 H_z。

**根因（运行数据）**：
1. Odin `lidar_end_time` 误用未排序的 `points.back().curvature` → end≈beg → `end_dt~100ms` 误丢 L2；应取 `max(curvature)`（与 dual 一致）。
2. Bundle 无楼面残差抑制 → 同走廊去/回程 Δz≈数米（run7 y≈−40：4.9 vs 1.0）。
3. 硬 `odin_z_replace` / `z_band` 分别破坏一致性或跟随错误 Z 下沉。

**run12（residual_z + max-span end_time，odin_z_replace=0，floor_info=0.45）**：
- joint≈89%，`end_dt_drop`≈3.5%，NoEff/freeze=0 → 基本全程 Bundle。
- y≈−40 去/回：Δx≈0.42 m、Δz≈1.42 m（优于 run7 的 Δz≈3.9 / Δx≈1.5）。
- 仍见远端走廊 Z 下沉（最低≈−4.8）。

**run14（1× + RViz，floor_info=0.30）**：joint≈86%，y≈−40 Δz≈−3.54（变差）。

**run15（map_mid_only + ml_h_min=0.75）**：失败 — 上楼不足（峰 z≈5.9）后 z→−34。已回退。

**run16（回退 run12 量级 + L2 拒更略松，1×+RViz）**：`/tmp/fastlio_multi_pvlio_run16/`  
`map_mid_only=0 ml_h_min=1 floor_info=0.45`；joint≈89.5%，z∈[−5.2,9.18]，Reject=2；  
y≈−40 Δx≈0.89 / Δz≈−2.51。截图 `rviz_shot{1,2,3}.png`。

**run28–30**：`odin_xy_factor` 短窗 yaw 估错 → 与 IEKF 对拉 → 地图叠影。已弃用 Odin 位姿劫持。

**run32**：官方 MULTI Bundle（`end_time=max(curvature)`、每 Mid 只拼 1 帧 Odin）+ 轻度楼面残差软化；无 odin_z/xy factor。

## 三源对照（同 bag `multifloor_20261120_1031_1037`，~360s @1×）

数据目录：`/tmp/compare_mid_odin_bundle/`（`mid_only/`、`odin_onboard_traj.csv`、`bundle_run32_traj.csv`、`ROOTCAUSE.txt`）。

| 源 | 算法 | path2d | Z 范围 | 上楼 \|z\|>0.5 时刻 |
|--|--|--|--|--|
| Mid-only | PV-LIO `mapping_mid360_go2` | 199.4 m | [−1.81, **3.56**] | **t≈54 s** |
| Odin 自带 | bag `/odin1/odometry` | 199.0 m | [−0.07, **3.87**] | **t≈53 s** |
| Bundle run32 | Mid+Odin 紧耦合 | 195.6 m | [−4.90, **9.17**] | **t≈14 s**（提前 ~40 s） |

对齐：Odin→Mid yaw≈**+13.5°** 后，Odin↔Mid `RMSE_xy≈0.80 m`、`RMSE_z≈1.05 m`（两套单雷达一致）。  
Mid↔Bundle：`RMSE_xy≈1.54 m`、`RMSE_z≈3.42 m`（**Z 是主矛盾**）。

**吸烟枪（t=15~50，尚未上楼）**：Mid Z≈0、Odin Z≈0.2，Bundle Z 已拉到 ~5 m。楼梯段 Bundle 峰高 ≈ Mid/Odin 的 **2.5×**。

**根因**：不在 Mid 单雷达，也不在 Odin 自带建图。Bundle 在 Mid `extrinsic_R=I` 时仍对 Odin 乘 CAD `Ry(-13°)`，相对重力对齐的 Mid 多了一层 pitch → 路径上 Z 以 ≈tan(13°) 假爬升。合并 `VoxelGrid` 还会平均掉 Odin intensity=500 标签。

**run35 修复（紧耦合 + Odin 进地图）**：
1. Mid IMU↔L1 改为官方链 `Ry(+13°)` + Unitree `T≈[0.187,0,0.0803]`；L2 保持 `Ry(-13°)` / dual `Tz=0.078`。
2. Mid/Odin **分开** downsample 后再拼接，保留 Odin 标签。
3. `bundle_map_mid_only=false`（Odin 点云进体素图）；IEKF 仍联合（joint≈89.5%）。
4. 暂 `odin_floor_info_scale=0`（楼面残差仍偏敏感）；无 odin_z/xy factor。

| | Z 峰 | \|z\|>0.5 时刻 | Mid↔Bundle RMSE_z | RMSE_xy |
|--|--|--|--|--|
| run32 | 9.17 | t≈14 s | 3.42 | 1.54 |
| **run35** | **4.39** | t≈26 s | **1.75** | **0.74** |
| Mid / Odin | 3.56 / 3.87 | t≈54 / 53 s | — | — |

数据：`/tmp/fastlio_multi_pvlio_run35/`。末段仍有走廊 Z 下沉（z_min≈−5），后续可再调 residual_z。

**run43–44（相对 Δz replace）**：`z = z0_mid + (z_odin − z0_odin)` 可跟楼梯节奏，但全程比 Odin 低约 **0.20 m**（锚在 Mid 初值≈0）。观感高度偏矮。

**run45**：改为 **绝对** `pose.z = /odin1/odometry.z`（XY/姿态仍由 Bundle IEKF），对齐 Odin 自带高度数字。

**run47（最小）**：PV-LIO Bundle 照常；**仅**在更新末尾 `pose.z = /odin1/odometry.z`。去掉 H_z 清零 / 预锁 Z / `ba_z` 等防御改动。

**截图根因（坐标/外参，非 odom 数字）**：`/Odometry.z` 已跟 Odin，但 RViz 楼面叠层。同帧 `/cloud_registered`：Mid 近地 z≈3.45、Odin≈3.10（Δ≈0.35 m）。`Mid Ry(+13°)` 相对 go2/`dual` 的 `R=I` 把 Mid 楼面抬斜；远端约 `x·sin(13°)`。

**run48**：外参改回 **dual/go2**（Mid `R=I` + 原 L2），仍只 `pose.z←Odin`。图仍叠层——Odin 点云进 Bundle 与 Mid 外参/俯仰不一致。

**run49**：按“Z 用 Odin、其它用 PV-LIO”→ **`multi_lidar: false`**（纯 Mid 建图）+ `pose.z←/odin1/odometry`；关 residual_z；噪声对齐 go2。

**dual_run_z**：dual + 绝对 Odin Z → 图被剪开（与 tune 注释一致）。

**dual_run_z2**：dual 建图不变；Z 用 **Odin Δz**（`z=z_mid0+(z_odin−z_odin0)`），跟楼梯、不撕 Mid 楼面锚点。

**固定点开始漂（dual_run_z2 根因）**：约 **t≈123s / dual y≈−18~−24**（二楼走廊）。
1. 行走 `gyr` 触发 `skip=ok_map_gyr` → **关 S2**，只剩 Mid S1。
2. 每帧 `odin_z_replace` 硬改 Z，与 Mid 楼面高度不一致 → Mid 沿走廊错配，**v 飙到 ~3.6 m/s**（同段 Mid-only/Odin 仅 ~0.9）。
3. tune 里 **`iekf_max_vel=0`（不限速）** + **`odin_pose_hold_on_bad=false`** → 无法兜底。
4. 随后大量 `No Effective Points`，`d_frame≈4.9 m` IMU 滑行，y 冲到 −195（path@180：dual 274 m vs Mid/Odin ~102 m）。

**dual_run_z3**：`iekf_max_vel=1.2`、`odin_s2_gyr_gate=0.40`、`odin_pose_hold_on_bad=true`、`consec_bad_freeze=3`；仍 Odin Δz replace。XY 控住，但走廊段姿态被拧翻（live roll≈168° vs Odin≈0°）→ 点云朝下。

**dual_run_z4**：Z replace 同时 **`odin_att_replace`（仅 roll/pitch，保 Mid yaw）**。`odin_R_align=Mid_R0×Odin_R0ᵀ` 把 Mid/Odin 重力系差灌成约 **+14° 假俯仰** → 一楼就往 −X 拉长（path@60 ≈39.6 m vs Odin ≈27.5 m），比 z3 更早漂。

**dual_run_z5**：关 `odin_att_replace`；保留 Odin Δz + `iekf_max_vel` / S2 gyr / pose_hold（同 z3 策略）。

## 构建 / 运行

见 `/home/ll/fastlio_multi_pvlio_ws/README.md`。
