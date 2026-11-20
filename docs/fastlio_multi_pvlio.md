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
| densify / residual_z | 有 | 无（纯 Bundle+体素） |
| 地图 | PV 体素 | PV 体素 |

## 注意（RViz 图/路径对不齐）

`/Global_cloud_map` 点坐标与 `/path` 同属 `camera_init`。若 `frame_id` 误标 `world`，再经 `world→camera_init = Initial_R_wrt_G`（重力对齐），Fixed Frame=`camera_init` 时图会被多旋一次，路线看起来斜出走廊。

## 注意（末段漂）

`lidar2_soft_wait` 可能在一个 Mid 帧下攒多帧 Odin。若把 buffer 全部 `+=` 进 Bundle，会出现 `l2_pts≈2×` 的坏更新并触发 Divergence。现改为只取与 Mid 结束时刻最近的**一帧** Odin。

## 构建 / 运行

见 `/home/ll/fastlio_multi_pvlio_ws/README.md`。
