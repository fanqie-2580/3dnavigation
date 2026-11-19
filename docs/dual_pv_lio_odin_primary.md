# 双雷达 PV-LIO：Odin1 主雷达分叉（新建目录）

原 Mid 主树 `/home/ll/pv_lio_dual_ws` **未改**。Odin 主雷达实验在：

```text
/home/ll/pv_lio_dual_odin_primary_ws
```

包名 `pv_lio_odin_pri`。思路对齐官方 FAST-LIO-MULTI：以 L1 为时间轴与主更新，L2 末端时刻配对后降权进入联合更新。

- L1：Odin1 点云 + Odin IMU（S1 锁定、默认写入匹配地图）
- L2：Mid360 + Mid IMU（软等待配对、S2 联合、强度标记为副雷达）
- 外参：Odin↔IMU 写入 `extrinsic_T/R`；Mid 相对 Odin 为原「Odin 相对 Mid」的逆
- 高度：默认关掉平坦硬顶与 `odin_z_factor`，减轻爬楼被压死；走廊需再锁高度时只改本分叉的 `dual_odin_mid_tune.py`

编译与启动见该目录 `README.md`。与 Mid 主树不要在同一 shell 同时 `source` 两套 `install`。
