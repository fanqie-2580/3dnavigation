# 留形 Odin1 + CT-LIO 建图（ROS 2 Humble）

| 组件 | 路径 | 作用 |
|------|------|------|
| `odin_ros_driver` | `/home/ll/ws_odin_pvlio` | Odin1 USB 驱动 → `/odin1/cloud_raw`、`/odin1/imu` |
| `ct_lio` | `/home/ll/ct_lio_to_ros2-master` | CT-LIO 建图（`lidar_type: 6` Odin1） |
| bringup | `odin_pv_lio_bringup` | 一键 launch |

> 驱动需 **Odometry 模式**：`custom_map_mode: 0`（已配置）。  
> **Odin 数据线必须插在 Jetson**（本机），不要只插拓展坞——CT-LIO 在本机 ROS2 跑。

---

## 编译

```bash
bash /home/ll/ws_odin_pvlio/scripts/01_build_all.sh
```

只改了 ct-lio 时可：

```bash
SKIP_ODIN_BUILD=1 bash /home/ll/ws_odin_pvlio/scripts/01_build_all.sh
```

---

## 建图

```bash
lsusb | grep 2207:0019   # 应能看到
bash /home/ll/ws_odin_pvlio/scripts/02_run_mapping.sh
```

会启动：`host_sdk_sample` →（约 3s）`ct_lio_eskf` + RViz。

检查：

```bash
source /home/ll/ws_odin_pvlio/env.sh
ros2 topic hz /odin1/imu
ros2 topic hz /odin1/cloud_raw
ros2 topic hz /cloud_registered
```

保存地图：

```bash
source /home/ll/ws_odin_pvlio/env.sh
ros2 service call /save_map std_srvs/srv/Trigger {}
```

默认路径：`/home/ll/ws_odin_pvlio/maps/odin1_ct_lio_map.pcd`

关闭：

```bash
bash /home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
```

---

## 参数

- CT-LIO：`/home/ll/ct_lio_to_ros2-master/config/mapping_odin1.yaml`
- Odin：`.../odin_ros_driver/config/control_command.yaml`（`custom_map_mode: 0`）

Mid-360 原配置仍用 `config/mapping.yaml` + `run_eskf.launch.py`，互不影响。
