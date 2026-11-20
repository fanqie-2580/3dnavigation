# 官方 PV-LIO 最小改动：FAST-LIO-MULTI Bundle 语义

工作区：`/home/ll/pv_lio_multi_ws`（包名 `pv_lio_multi`）。  
基线：官方 PV-LIO（`backup_before_dual`），**建图仍用 PV-LIO 体素**，不跑 FAST-LIO-MULTI 二进制。

与日常 `pv_lio_dual_ws` 无关；dual 的异质门控 / 两段 IEKF / 高度带限仍保留在原树。

## 相对官方 PV-LIO 改了什么

| 改动 | 对应官方 MULTI |
|------|----------------|
| `MeasureGroup` 增加 `lidar2` + 起止时刻 | 双雷达测量组 |
| Bundle `sync_packages`：L2→L1 外参后并入同一帧 | 主雷达时间窗拼帧 |
| `UndistortPclMultiLiDAR`（主 IMU 去畸变两路） | Bundle 去畸变 |
| 合并点云一次 IEKF + 写入体素地图 | 同质一次更新 + 共地图 |
| Odin `lidar_type=6` 预处理 | 仅消息格式（官方无 Odin） |
| `lidar2_soft_wait_sec`（默认 0.18） | Mid+Odin 实用补丁；官方无 |

刻意**没有**移植 dual 的：双 IMU、Odin 独立陀螺去畸变、S1/S2、残差门、高度带限、速度钳位。

## 运行（走廊 bag）

```bash
source /opt/ros/humble/setup.bash
source /home/ll/pv_lio_multi_ws/install/setup.bash
ros2 launch pv_lio_multi mapping_mid360_odin1_multi.launch.py rviz:=true use_sim_time:=true

# 另一终端
ros2 bag play /home/ll/pv_lio_dual_ws/bags/corridor_20261118_outbound_1014_1017 --clock --rate 1.0
```

配置：`config/mid360_odin1_multi.yaml`（外参与 dual 地面标定一致）。

## 预期

同质双雷达上应接近官方 MULTI Bundle。Mid+Odin 异质场景可能出现航向/高度被副雷达拉偏——那正是 dual 加防护的原因；本包用于对照「官方同质紧耦合在 PV-LIO 上的最小形态」。
