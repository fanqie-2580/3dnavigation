# Odin1 建图（使用 Odin 自带里程计）

默认建图不再用 PV-LIO / CT-LIO 的里程计，而是用 Odin `custom_map_mode=0` 输出的 `/odin1/odometry`，把 `/odin1/cloud_raw` 叠成地图。

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

- 输入：`/odin1/odometry`、`/odin1/cloud_raw`
- 输出：`/cloud_registered`、`/Laser_map`、`/Odometry`、`/path`
- TF：`map`→`odom`(static)→`base_link`

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

## 停止

```bash
bash /home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
```
