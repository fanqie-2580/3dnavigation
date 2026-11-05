# FAST_LIO_MULTI Mid360+Odin（干净上游改动版）

源码目录：`/home/ll/FAST_LIO_MULTI_ROS2_mid_odin`  
编译工作空间：`/home/ll/fast_lio_multi_mid_odin_ws`  
上游基准：`Draxran/FAST_LIO_MULTI_ROS2` @ `e7864a6`

## 相对上游的改动

- `preprocess`：`lidar_type=4`（XYZ_ONLY）+ 读取 Odin `offset_time`→`curvature(ms)`，可选 `confidence` 过滤
- `scripts/odin_raw_downsample_bridge.py`：降采样并保留 `offset_time`（**默认不用 Mid odom 重打 stamp**）
- `config/mid360_odin1.yaml`：`multi_lidar: true`，Mid CustomMsg + Odin PointCloud2
- `launch/mid360_odin1_multi.launch.py`：adaptive + downsample bridge
- adaptive：L2 stamp 与 Mid 差 >1s 时丢弃（防跨时钟）

## 运行前

1. Odin `custom_map_mode: 0`
2. Odin `use_host_ros_time: 2`
3. 确认 `/odin1/cloud_raw` 与 `/livox/lidar` stamp 同量级

## 编译 / 启动

```bash
cd /home/ll/fast_lio_multi_mid_odin_ws
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
colcon build --symlink-install --packages-select fast_lio_multi
source install/setup.bash

ros2 launch fast_lio_multi mid360_odin1_multi.launch.py update_method:=adaptive
```
