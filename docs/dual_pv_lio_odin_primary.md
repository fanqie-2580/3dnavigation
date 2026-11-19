# 双雷达 PV-LIO：Odin1 主雷达分叉（实验，当前不推荐）

目录：`/home/ll/pv_lio_dual_odin_primary_ws`（包名 `pv_lio_odin_pri`）。**未改**原 Mid 主树 `pv_lio_dual_ws`。

走廊去程 bag 验证（2026-11-19）：即便关闭 Mid 第二段，约 45 s 净高度约 +22 cm、峰峰值约 38 cm，频繁 reject_s1。主因是 Odin IMU 做主传播噪声过大，以及 Mid↔Odin 末端时刻偏置约 150–200 ms。

日常请用 Mid 主树；爬楼应在 Mid 主方案上做高度约束自适应，而不是换主雷达。
