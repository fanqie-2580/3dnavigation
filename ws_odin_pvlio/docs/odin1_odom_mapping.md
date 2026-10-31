# Odin1 建图（使用 Odin 自带里程计）

默认建图不再用 PV-LIO / CT-LIO 的里程计，而是用 Odin **`custom_map_mode=1`（SLAM，带回环）** 输出的 `/odin1/odometry`，把 `/odin1/cloud_raw` 叠成地图。

> mode `0` 仅外部 LIO 用，位姿易卡住导致重影；室内用 Odin 自带位姿建图请保持 `1`。

## 拓扑

- 拓展坞（ROS1 Noetic）：`host_sdk_sample` + `odin_ros1_tcp_server.py`（`:5601`）
- Jetson（ROS2 Humble）：`odin_ros2_tcp_client.py` + `odin_odom_mapper.py` + RViz

## 一键启动

```bash
bash /home/ll/ws_odin_pvlio/scripts/02_run_mapping.sh
```

环境变量：

| 变量 | 默认 | 说明 |
|------|------|------|
| `MAPPER` | `odin_odom` | `odin_odom` / `ct_lio` / `pv_lio` |
| `ODIN_SOURCE` | `bridge` | `bridge` 或 `usb` |
| `RVIZ` | `true` | 是否开 RViz |

## 话题

- 输入：`/odin1/odometry`、`/odin1/cloud_slam`（已在 `odom` 系，**不再二次变换**）
- 输出：`/cloud_registered`、`/Laser_map`、`/Odometry`、`/path`
- TF：`map`→`odom`(static)→`base_link`

> 糊图常见原因：对已在 `odom` 系的 `cloud_slam` 再乘里程计。mode=1 时桥接默认转发 `cloud_slam`。

## 存图

走完一圈后：

```bash
bash /home/ll/ws_odin_pvlio/scripts/06_save_map.sh
# 或
source /home/ll/ws_odin_pvlio/env.sh
ros2 service call /save_map std_srvs/srv/Trigger
```

默认文件：`/home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd`  
`06_save_map.sh` 会额外复制一份带时间戳的副本。

## 分割（给 dddmr）

存图后把全局 PCD 拆成地面/障碍两份（`FIELDS x y z`，可被 dddmr `pcl_publisher` 直接加载）：

```bash
bash /home/ll/ws_odin_pvlio/scripts/07_segment_map.sh
# 或指定输入
bash /home/ll/ws_odin_pvlio/scripts/07_segment_map.sh /home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd
```

输出：

- `/home/ll/ws_odin_pvlio/maps/segmentation_ground_map.pcd` → dddmr `mapground`
- `/home/ll/ws_odin_pvlio/maps/segmentation_obstacle_map.pcd` → dddmr `mapcloud`

拷到导航数据目录：

```bash
cp /home/ll/ws_odin_pvlio/maps/segmentation_ground_map.pcd \
   /home/ll/ws_odin_pvlio/maps/segmentation_obstacle_map.pcd \
   /home/ll/dddmr_navigation/src/dddmr_global_planner/data/
```

可调环境变量：`VOXEL` `DISTANCE` `MAX_ANGLE` `MAX_HEIGHT` `MIN_OBSTACLE_HEIGHT`。

> 分割只处理静态全局图。以后用 Odin 导航时，还需要把 dddmr 实时 `lidar_topic` / `sensor_frame` 改成 Odin 点云（另做联调）。

## 停止

```bash
bash /home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
```
