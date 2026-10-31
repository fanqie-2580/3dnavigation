# 留形 Odin1 + PV-LIO 建图（ROS 2 Humble）

工作空间：`/home/ll/ws_odin_pvlio`

| 组件 | 来源 | 作用 |
|------|------|------|
| `odin_ros_driver` | [manifoldsdk/odin_ros_driver](https://github.com/manifoldsdk/odin_ros_driver) | Odin1 USB 驱动，发 `/odin1/cloud_raw`、`/odin1/imu` |
| `pv_lio` | [shuoyang1215/PV-LIO_ROS2](https://github.com/shuoyang1215/PV-LIO_ROS2) | PV-LIO 建图 |
| `odin_pv_lio_bringup` | 本机自写 | 点云格式转换 + 一键 launch |

> 用 **外部 PV-LIO** 建图时，驱动需处于 **Odometry 模式**（`custom_map_mode: 0`），不要用 Odin 自带 SLAM 抢数据。当前配置已按此改好。

---

## 0. 一次性准备

### 0.1 USB 权限（已写入，插拔一次生效）

```bash
# 已安装文件：/etc/udev/rules.d/99-odin-usb.rules
# Vendor 2207 / Product 0019
sudo udevadm control --reload
sudo udevadm trigger
```

确认设备：

```bash
lsusb | grep 2207
```

### 0.2 环境（重要）

本机其它工作空间的 `setup.bash` 可能链式带上 Autoware。请**只用**本仓库的 `env.sh`：

```bash
source /home/ll/ws_odin_pvlio/env.sh
```

### 0.3 编译（已成功过一次；改代码后再编）

```bash
bash /home/ll/ws_odin_pvlio/scripts/01_build_all.sh
```

或手动：

```bash
# 建议在干净终端：unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
# 编译 PV-LIO 需要 livox_ros_driver2（仅依赖，运行 Odin 建图不启 Livox）
source /home/ll/ws_livox/install/local_setup.bash 2>/dev/null || \
  export AMENT_PREFIX_PATH=/home/ll/ws_livox/install/livox_ros_driver2:$AMENT_PREFIX_PATH
cd /home/ll/ws_odin_pvlio/src/odin_ros_driver && bash ./script/build_ros2.sh
# odin 官方脚本可能在 install/ 留下 COLCON_IGNORE，编完删掉：
rm -f /home/ll/ws_odin_pvlio/install/COLCON_IGNORE /home/ll/ws_odin_pvlio/build/COLCON_IGNORE
cd /home/ll/ws_odin_pvlio
colcon build --symlink-install --packages-select pv_lio odin_pv_lio_bringup
```

---

## 1. 建图步骤（推荐）

### 终端 A：启动

```bash
# 先 USB3 插入 Odin1，确认：
lsusb | grep 2207

bash /home/ll/ws_odin_pvlio/scripts/02_run_mapping.sh
```

等价于：

```bash
source /home/ll/ws_odin_pvlio/env.sh
ros2 launch odin_pv_lio_bringup odin1_pv_lio_mapping.launch.py
```

会启动：

1. Odin 驱动  
2. `odin_cloud_to_pvlio`：`/odin1/cloud_raw` → `/odin1/points_pvlio`  
3. `pv_lio_node`（约 3s 后）  
4. RViz  

### 检查数据

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_odin_pvlio/install/setup.bash
ros2 topic hz /odin1/imu
ros2 topic hz /odin1/cloud_raw
ros2 topic hz /odin1/points_pvlio
ros2 topic list | grep odin1
```

### 走动建图

手持 / 装在机器人上缓慢行走，看 RViz 轨迹与点云是否正常累积。

### 保存地图

配置里已开 `pcd_save_en: true`，路径：

```text
/home/ll/ws_odin_pvlio/maps
```

**正常 Ctrl+C 结束** `pv_lio_node`（或跑 stop 脚本）后，会写出累积 PCD（`interval: -1` 为整图一份）。

### 关闭

```bash
bash /home/ll/ws_odin_pvlio/scripts/03_stop_mapping.sh
```

---

## 2. 分步启动（调试用）

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/ws_odin_pvlio/install/setup.bash

# 1) 仅驱动
ros2 launch odin_ros_driver odin1_ros2.launch.py

# 2) 转换
ros2 run odin_pv_lio_bringup odin_cloud_to_pvlio

# 3) PV-LIO（已有转换后的点云时可不启驱动）
ros2 launch pv_lio mapping_odin1.launch.py
# 或
ros2 launch odin_pv_lio_bringup odin1_pv_lio_mapping.launch.py start_odin_driver:=false
```

---

## 3. 关键配置

| 文件 | 说明 |
|------|------|
| `odin_ros_driver/config/control_command.yaml` | `custom_map_mode: 0`；raw cloud + imu 开；自带 slam/render/rgb 关 |
| `odin_pv_lio_bringup/config/odin1_pvlio.yaml` | PV-LIO 话题/外参/存图 |
| `PV-LIO_ROS2/config/odin1.yaml` | 同上副本，供 `mapping_odin1.launch.py` |

外参：Odin1 模组内置 IMU+雷达，初始设为单位阵，并开 `extrinsic_est_en: true` 在线估计。若厂家给了标定，再改 `extrinsic_T` / `extrinsic_R`（**雷达在 IMU 坐标系下的位姿**）。

---

## 4. 与 MID360 / FAST-LIO 栈的关系

- 本流程**不依赖** Livox 实时驱动；仅编译 PV-LIO 时需 source `ws_livox`（包依赖）。  
- 导航联调仍可用原有 `docs/go2_mapping_nav_commands.md`；Odin 地图进导航前需转成你们 dddmr / 定位用的 PCD 格式。

---

## 5. 常见问题

1. **插上没话题**：查 `lsusb | grep 2207`、USB3、udev；看驱动终端是否连上设备。  
2. **`Failed to find match for field 'time'`**：转换节点未起或字段不对；确认 `/odin1/points_pvlio` 有数据。  
3. **漂 / 飞**：检查时间同步、IMU 是否在发；必要时关 `extrinsic_est_en` 并手填外参。  
4. **想用 Odin 自带建图**：把 `custom_map_mode` 改回 `1`，用官方 `./set_param.sh save_map 1`（与 PV-LIO 二选一）。
```
