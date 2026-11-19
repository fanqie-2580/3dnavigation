# Dual PV-LIO 走廊包回放命令（已验证）

> 验证日期：2026-11-19  
> 配置：当前 `dual_mid_odin_tune.py`（`odin_z_factor_band=±2cm`，`residual_z_hard_band=2.5cm`）  
> 环境：宿主机 ROS 2 Humble + CycloneDDS；RViz 用 `DISPLAY=:1`

## 包路径

| 航段 | Bag |
|------|-----|
| 去程 | `/home/ll/pv_lio_dual_ws/bags/corridor_20261118_outbound_1014_1017` |
| 回程 | `/home/ll/pv_lio_dual_ws/bags/corridor_20261118_return_1017_1019` |

## 每个终端先执行

```bash
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/pv_lio_dual_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY=:1
```

## 去程

**终端 A — 建图 + RViz**

```bash
ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=true use_sim_time:=true
```

**终端 B — 播包（等 A 起来后再播）**

```bash
ros2 bag play /home/ll/pv_lio_dual_ws/bags/corridor_20261118_outbound_1014_1017 --clock --rate 1.0
```

**终端 C — 播完后保存地图**

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

## 回程

流程同去程：先 A，再播回程包，再 `save_map`。

```bash
ros2 bag play /home/ll/pv_lio_dual_ws/bags/corridor_20261118_return_1017_1019 --clock --rate 1.0
```

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

## PCD 保存位置

`/save_map` 异步写入（yaml：`pcd_save.save_path`）：

```text
/home/ll/pv_lio_dual_ws/maps/map_YYYYMMDD_HHMMSS.pcd
```

本次验证产物（别名拷贝，便于查找）：

| 航段 | PCD | Z 曲线 | Z CSV |
|------|-----|--------|-------|
| 去程 | `/home/ll/pv_lio_dual_ws/maps/corridor_20261118_outbound_zhard.pcd` | `..._outbound_zhard_z.png` | `..._outbound_zhard_z.csv` |
| 回程 | `/home/ll/pv_lio_dual_ws/maps/corridor_20261118_return_zhard.pcd` | `..._return_zhard_z.png` | `..._return_zhard_z.csv` |
| 合并图 | — | `/home/ll/pv_lio_dual_ws/maps/corridor_20261118_zhard_z_both.png` | — |

对应原始落盘：

- 去程：`/home/ll/pv_lio_dual_ws/maps/map_20261119_142554.pcd`
- 回程：`/home/ll/pv_lio_dual_ws/maps/map_20261119_142820.pcd`

## 本次回放 Z 摘要

| 航段 | path | net dz | ptp | reject / no_eff |
|------|------|--------|-----|-----------------|
| 去程 | 107.4 m | +1.5 cm | 4.5 cm | 0 / 0 |
| 回程 | 96.4 m | −1.6 cm | 2.8 cm | 0 / 0 |

## 无 RViz 快速回放（可选）

```bash
ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=false use_sim_time:=true
```

播包与 `save_map` 命令同上。

## 注意

- 不要在同一 shell 里同时 `source` Mid / Odin 其它工作空间的 `install/setup.bash`（包名冲突）。
- `use_sim_time:=true` 必须与 `ros2 bag play ... --clock` 一起用。
- `save_map` 返回 `success=True` 只表示异步保存已启动；大图写盘可能还需几秒。
