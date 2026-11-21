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

**run12（residual_z + max-span end_time，odin_z_replace=0）**：
- joint≈89%，`end_dt_drop`≈3.5%，NoEff/freeze=0 → 基本全程 Bundle。
- y≈−40 去/回：Δx≈0.42 m、Δz≈1.42 m（优于 run7 的 Δz≈3.9 / Δx≈1.5）。
- 仍见远端走廊 Z 下沉（最低≈−4.8）：multilevel 后满 H_z + 联合地图含 Odin 楼面。run13 起 multilevel 仍保留轻度 H_z 压制 + `floor_info_scale=0.30`。

## 构建 / 运行

见 `/home/ll/fastlio_multi_pvlio_ws/README.md`。
