# 留形 Odin1 + CT-LIO 建图（ROS 2 Humble）

| 组件 | 路径 | 作用 |
|------|------|------|
| `odin_ros_driver` | 拓展坞 ROS1 Noetic | USB 驱动 → `/odin1/cloud_raw`、`/odin1/imu` |
| TCP 桥 | `ws_odin_pvlio/bridge/` | 坞 ROS1 → 本机 ROS2（端口 `5601`） |
| `ct_lio` | `/home/ll/ct_lio_to_ros2-master` | CT-LIO（`lidar_type: 6` Odin1） |

> 驱动需 **Odometry 模式**：`custom_map_mode: 0`。  
> **默认模式：Odin 插在拓展坞**，本机通过 TCP 桥收数。若 USB 直接插本机：`ODIN_SOURCE=usb bash scripts/02_run_mapping.sh`。

网络：拓展坞 `192.168.123.18`，本机网口约 `192.168.123.222`。

---

## 编译

```bash
bash /home/ll/ws_odin_pvlio/scripts/01_build_all.sh
# 或只编 ct-lio：
SKIP_ODIN_BUILD=1 bash /home/ll/ws_odin_pvlio/scripts/01_build_all.sh
```

---

## 建图（桥接，推荐）

```bash
bash /home/ll/ws_odin_pvlio/scripts/02_run_mapping.sh
```

会：坞上启驱动+TCP server → 本机 TCP client → `ct_lio_eskf`。

检查：

```bash
source /home/ll/ws_odin_pvlio/env.sh
ros2 topic hz /odin1/imu
ros2 topic hz /odin1/cloud_raw
ros2 topic hz /cloud_registered
ros2 topic hz /odom
```

保存地图：

```bash
ros2 service call /save_map std_srvs/srv/Trigger {}
# → /home/ll/ws_odin_pvlio/maps/odin1_ct_lio_map.pcd
```

关闭：

```bash
bash /home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
```

单独启/停坞桥：

```bash
bash /home/ll/ws_odin_pvlio/scripts/04_start_dock_odin_bridge.sh
bash /home/ll/ws_odin_pvlio/scripts/05_stop_dock_odin_bridge.sh
```

---

## 参数

- CT-LIO：`ct_lio_to_ros2-master/config/mapping_odin1.yaml`
- Odin（坞）：`ws_odin/.../control_command.yaml`（`custom_map_mode: 0`）
