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

**根因**：不在 Mid 单雷达，也不在 Odin 自带建图。虚假 +Z 来自 Bundle 拼帧（Odin 经 `extrinsic_*_L2_wrt_L1` → Mid IMU 去畸变 → 一次 IEKF）产生的双雷达楼面/墙面残差冲突。下一步应先核 L2→L1 外参（尤其 pitch≈13° 的 `extrinsic_R_L2`）与拼帧几何，**不要**再上 odin_z/xy factor。

## 构建 / 运行

见 `/home/ll/fastlio_multi_pvlio_ws/README.md`。
