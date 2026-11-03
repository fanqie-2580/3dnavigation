# 全局地图天花板稀疏化 & Odin1 / Mid360 对齐说明

更新：2026-11-03（对照当前 `odin1_odom_map.pcd` 实测）

---

## 1. 全局地图点太多、天花板太密

### 实测（处理前）

| 地图 | 点数 | z≥2.5m 占比 |
|------|------|-------------|
| 定位图 `odin1_odom_map.pcd` | 659602 | **35.4%**（天花板） |
| 全局障碍 `segmentation_obstacle_map.pcd` | 254890 | 15.5% |
| 全局地面 `segmentation_ground_map.pcd` | 74370 | 0%（已是纯地面） |

地面图本身没问题；点多主要来自 **障碍图 / 定位图里的天花板与高墙**。

### 已做的处理（2026-11-03）

分割脚本增加高度带策略（相对地面高度）：

- `keep_below=1.8`：1.8 m 以下障碍**保留原密度**（墙、家具）
- `ceiling_voxel=0.40`：1.8–2.4 m **大幅体素稀疏**
- `drop_above=2.4`：2.4 m 以上**直接丢掉**

处理后：

| 地图 | 点数 | z≥2.5m |
|------|------|--------|
| 障碍图（已部署到 dddmr） | **171343** | **0** |
| 定位图瘦身后副本 `odin1_odom_map_thin_ceiling.pcd` | **251421** | **0** |

重新分割命令：

```bash
# 默认参数已含天花板稀疏
bash /home/ll/ws_odin_pvlio/scripts/07_segment_map.sh \
  /home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd

cp /home/ll/ws_odin_pvlio/maps/segmentation_*.pcd \
  /home/ll/dddmr_navigation/src/dddmr_global_planner/data/

# 可选：给定位用的瘦身图
python3 /home/ll/ws_odin_pvlio/scripts/07b_thin_loc_map.py \
  --input /home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd \
  --output /home/ll/ws_odin_pvlio/maps/odin1_odom_map_thin_ceiling.pcd
```

可调环境变量：`KEEP_BELOW` `CEILING_VOXEL` `DROP_ABOVE` `VOXEL` `MAX_HEIGHT`。

旧障碍图备份在：

```text
/home/ll/dddmr_navigation/src/dddmr_global_planner/data/backup_before_ceiling_thin_*
```

---

## 2. 为什么 Odin1 建的图，用 Mid360 定位经常对不齐

当前常见用法是：

```text
建图：Odin1（稠密视觉/深度点云 + Odin 自带里程计）
定位：Mid360 LiDAR 实时点云  ↔  open3d_loc 匹配 Odin1 地图
```

这是**跨传感器匹配**，不对齐是预期现象，不是单纯“初值差一点”。

主要原因：

1. **模态不同**  
   Odin1 点云稠密、噪声分布像深度相机；Mid360 是稀疏旋转激光线。同一堵墙在两种云里几何细节、空洞、厚度不一致，ICP / NDT 容易漂。

2. **视野与采样不同**  
   Odin1 前方视场密、两侧/后方弱；Mid360 近 360°。地图里大量“只被 Odin 看见”的天花板/近距结构，Mid360 扫不到，匹配权重被这些点拉偏。

3. **坐标系 / 重力轴不完全一致**  
   两套里程计零位、俯仰外参、地面高度零点若差几度/几厘米，室内长廊会放大成横向漂移。

4. **建图与定位链路本就分开设计**  
   仓库里已有 **Odin1 专用导航**：`localization_3d_odin1` + `dddmr_global_ego_odin1`（见 `ws_odin_pvlio/odin1_nav/README.md`）。  
   Mid360 链路本应用 Mid360 自己的图（或对齐后的图），不该默认拿生 Odin 全图硬匹配。

---

## 3. Odin1 和 Mid360 能怎么“融合”

按投入从低到高：

### A. 不融合：同传感器闭环（最稳，推荐先做）

| 用途 | 做法 |
|------|------|
| Mid360 导航 | Mid360 建图 → Mid360 定位 → 现有 EGO/DDDMR |
| Odin1 导航 | Odin1 建图 → `11_start_odin1_nav.sh` 全 Odin 链路 |

全局规划仍可用 Odin 分割出的地面/障碍（几何当静态层），但**定位不要跨传感器硬套**。

### B. 离线对齐成“一张图”（实用折中）

1. Mid360 扫一遍得到 `mid360_map.pcd`  
2. 与 `odin1_odom_map.pcd` 做一次全局 ICP / 手动粗对齐 + 精配准，得到 `T_odin_mid`  
3. 导航时：
   - **定位**用 Mid360 实时云 ↔ 对齐后的 Mid360 图（或把 Odin 障碍图变换到 Mid360 系当全局层）
   - **全局规划**继续用分割后的地面/障碍（变换到同一 `map`）

适合：Odin 图几何完整、但日常只背 Mid360 跑。

### C. 在线松耦合（中等难度）

- 标定固定外参：`T_base_odin`、`T_base_mid360`（已有 Mid360 `body→base`；Odin 需单独标定）  
- Mid360 做主定位（FAST-LIO / open3d_loc）  
- Odin 点云仅作局部障碍/语义，不进全局位姿优化  
- 或 EKF：Mid360 位姿为主，Odin 视觉里程计当相对约束

### D. 在线紧耦合 SLAM（工作量大）

统一状态估计里同时吃两路点云 + IMU。当前仓库**没有**现成可直接跑的紧耦合节点，需要新开工程（如 FAST-LIO 多雷达 / LIO-SAM 扩展），不建议作为近期路径。

---

## 4. 建议你怎么选

1. **马上减负**：已部署瘦身后的 `segmentation_obstacle_map.pcd`；全局 RViz 天花板应明显变稀。  
2. **定位对齐**：  
   - 继续 Mid360 导航 → 用 Mid360 重绘定位图，或试用  
     `maps/odin1_odom_map_thin_ceiling.pcd`（天花板少了，跨传感器仍可能漂）  
   - 要吃 Odin 图精度 → 走纯 Odin 定位链路，不要用 Mid360 去对 Odin 全图。  
3. **真要融合**：先做 **B 离线对齐**；标定好转到 **C 松耦合**。不要期望“同一张生 Odin 图 + Mid360 实时云”长期稳定。

---

## 5. 相关路径

```text
分割脚本:     /home/ll/ws_odin_pvlio/scripts/07_segment_map.py
瘦身定位图:   /home/ll/ws_odin_pvlio/scripts/07b_thin_loc_map.py
Odin 建图说明: /home/ll/ws_odin_pvlio/docs/odin1_odom_mapping.md
Odin 导航:     /home/ll/ws_odin_pvlio/odin1_nav/README.md
Mid360 定位→导航: /home/ll/3dnavigation/docs/go2_localization_to_navigation.md
```
