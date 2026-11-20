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
| IEKF | Mid S1 → 可选 Odin S2 + 门控 | 合并点云 **全程一次 Bundle** |
| densify / residual_z | 有 | 无（纯 Bundle+体素） |
| 地图 | PV 体素 | PV 体素 |

## 构建 / 运行

见 `/home/ll/fastlio_multi_pvlio_ws/README.md`。
