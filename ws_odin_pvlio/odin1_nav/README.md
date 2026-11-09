# Odin1 定位 / 导航（独立于 Mid-360）

Mid-360 的 launch / 地图 / 分割结果**不改动**。本目录与下列新增文件专供 Odin1：

- 地图：`maps/odin1_odom_map.pcd`、`segmentation_*.pcd`
- 分割图副本：`/home/ll/dddmr_navigation/src/dddmr_global_planner/data/odin1/`
- 定位 launch：`open3d_loc/launch/localization_3d_odin1.launch.py`（新增）
- 导航 launch：`p2p_move_base/launch/dddmr_global_ego_odin1.launch.py`（新增）

**逐步命令行（建图/分割/定位/导航）+ 与 Mid360 单独 / 融合版防冲突说明**：  
[`/home/ll/3dnavigation/docs/odin1_mid360_separate_pipelines.md`](/home/ll/3dnavigation/docs/odin1_mid360_separate_pipelines.md)

## 一键启动

```bash
bash /home/ll/ws_odin_pvlio/scripts/11_start_odin1_nav.sh
```

在定位 RViz 用 **2D Pose Estimate** 给初值；DDDMR RViz 用 **Publish Point** 下目标。
