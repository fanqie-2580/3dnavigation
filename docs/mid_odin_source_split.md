# Mid360 / Odin1 源码物理分叉

改一侧时不要动另一侧。包名相同，**禁止在同一 shell 同时 source 两侧 `install/setup.bash`**。

## 路径对照

| 角色 | Mid360 | Odin1 |
|------|--------|-------|
| 全局规划 | `/home/ll/dddmr_navigation` | `/home/ll/dddmr_navigation_odin1` |
| Docker | `dddmr_dev` | `dddmr_odin1` |
| 局部规划 | `/home/ll/ego-planner-swarm-ros2_version` | `/home/ll/ego-planner-swarm-odin1` |
| 全局 launch | `dddmr_global_ego_docker.launch.py` | `dddmr_global_ego_odin1.launch.py` |
| 局部 launch | `dddmr_reference_tracking.launch.py` | `dddmr_reference_tracking_odin1.launch.py` |
| 定位 | `localization_3d_go2.launch.py` | `localization_3d_odin1.launch.py` |
| 建图 | Livox + FAST-LIO mid360 | `ws_odin_pvlio` 脚本 |
| 运动桥 | `go2_bridge_node`（共用） | 同左 |

## 一键启动

```bash
# Mid360
bash /home/ll/3dnavigation/scripts/11_start_mid360_nav.sh

# Odin1
bash /home/ll/ws_odin_pvlio/scripts/11_start_odin1_nav.sh
```

## Odin1 专用配置（在 Odin 树内）

- `p2p_move_base/config/p2p_move_base_localization_odin1.yaml`
- `p2p_move_base/config/dddmr_global_ego_frames_odin1.yaml`
- `p2p_move_base/rviz/p2p_move_base_localization_odin1.rviz`
- maps: `dddmr_global_planner/data/odin1/`
- EGO RViz: `odin1_navigation.rviz`

## 编译

```bash
# Mid DDDMR
docker exec -it dddmr_dev bash -lc 'cd /root/dddmr_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install'

# Odin DDDMR
docker exec -it dddmr_odin1 bash -lc 'cd /root/dddmr_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install'

# Mid EGO
cd /home/ll/ego-planner-swarm-ros2_version && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Odin EGO
cd /home/ll/ego-planner-swarm-odin1 && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```
