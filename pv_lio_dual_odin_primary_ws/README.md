# PV-LIO dual — Odin1 主雷达分叉

与 `/home/ll/pv_lio_dual_ws`（**Mid 主雷达**）物理分离。本目录不改原仓库源码。

## 相对官方 FAST-LIO-MULTI

官方以 `lid_topic` / `imu_topic` 为 **L1 时间轴**，`lid_topic2` 为副雷达拼帧。本分叉按同一约定：

| 角色 | 传感器 | 话题 |
|------|--------|------|
| L1 主 | Odin1 | `/odin1/cloud_raw` + `/odin1/imu` |
| L2 副 | Mid360 | `/livox/lidar` + `/livox/imu` |

外参：`extrinsic_*` 为 Odin↔Odin IMU；`extrinsic_*_L2_wrt_L1` 为 Mid 相对 Odin（由原 Mid 主树外参求逆）。

高度：默认关闭走廊平坦硬顶 / `odin_z_factor`（便于爬楼）；走廊若再漂，可在 `dual_odin_mid_tune.py` 里单独打开。

## 编译

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash   # livox_ros_driver2
cd /home/ll/pv_lio_dual_odin_primary_ws
colcon build --symlink-install --packages-select pv_lio_odin_pri
```

**不要**与 Mid 主树 `pv_lio_dual_ws` 的 `install/setup.bash` 叠在同一 shell。

## 运行

```bash
source /home/ll/pv_lio_dual_odin_primary_ws/install/setup.bash
ros2 launch pv_lio_odin_pri mapping_dual_odin_primary.launch.py rviz:=true
# bag 回放加：use_sim_time:=true
```

调参：`src/PV-LIO_ROS2/config/dual_odin_mid_tune.py`（改完重启 launch）。

## 包名

`pv_lio_odin_pri`（可执行文件仍为 `pv_lio_node`）。

## 验证结论（2026-11-19 走廊去程 bag）

当前 Odin 主雷达分叉**不稳定，不建议日常使用**：

- 即便关闭 Mid S2（等价单 Odin LIO），约 45 s 内净高度已约 **+22 cm**，峰峰值约 **38 cm**，且频繁 `reject_s1`。
- 根因主要是 **Odin IMU 做主状态传播**（加速度噪声极大），外加 Mid↔Odin 末端时刻系统性偏置约 **150–200 ms**。
- 日常请继续用 Mid 主树 `/home/ll/pv_lio_dual_ws`；爬楼应在 Mid 主树上做「平地锁高度 / 爬楼放宽」自适应，而不是换主雷达。
