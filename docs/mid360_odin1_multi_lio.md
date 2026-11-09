# Mid360 + Odin1：相对官方 FAST_LIO_MULTI 的全程紧耦合

工作区：`/home/ll/fast_lio_multi_ws`（包名 `fast_lio_multi`）  
配置：[`config/mid360_odin1.yaml`](/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/config/mid360_odin1.yaml)  
启动：[`launch/mid360_odin1_multi.launch.py`](/home/ll/fast_lio_multi_ws/src/FAST_LIO_MULTI_ROS2/launch/mid360_odin1_multi.launch.py)

> **当前默认**：`multi_lidar: true` + `lidar2_update_en: true` + `update_method:=adaptive`，即副雷达全程参与 IEKF 更新与地图写入（紧耦合）。  
> 旧文档里“Odin 只独立累积、不进滤波器”的说法已过时。

---

## 1. 目标与概念

相对官方 FAST_LIO_MULTI，在 Go2 四足上把**主雷达**换成 Livox Mid360，把**副雷达**换成奥比中光 Odin1，目的是让这两台不同厂家、不同数据格式的雷达，仍然跑进官方那一套“多雷达一起算位姿、一起建地图”的流程，并且**全程都是紧耦合**。

| 词 | 含义 |
|----|------|
| **位姿** | 机器人在空间里的位置 + 朝向 |
| **建地图** | 把各时刻点云按估计位姿叠到同一张全局点云图 |
| **紧耦合** | 两路雷达的点云残差都进入**同一个**滤波器改正位姿，而不是主雷达算完位姿后副雷达只做显示叠加 |

官方 FAST_LIO_MULTI 正是这种紧耦合框架：在 FAST-LIO2 单雷达惯性里程计之上扩展第二路雷达。官方演示常见两台 Livox（或 Mid360 + Avia），数据都能较直接订阅进节点；Odin1 发出的是 ROS 标准 `sensor_msgs/PointCloud2`，字段含义和时间戳习惯都与 Livox `CustomMsg` 不同，**不能原样套用官方第二路输入**。

---

## 2. 传感器角色与开关

| 角色 | 话题 | 说明 |
|------|------|------|
| 主雷达 | `/livox/lidar` | Livox CustomMsg |
| 主 IMU | `/livox/imu` | Mid360 板载 IMU：加速度 + 角速度，用于帧间预测与帧内去畸变 |
| 副雷达（经桥接） | `/odin1/cloud_raw` → `/odin1/cloud_raw_ds` | **不要**把原始 `/odin1/cloud_raw` 直接塞进建图节点 |

配置里两个关键开关（须**都打开**才是全程紧耦合）：

| 参数 | 作用 |
|------|------|
| `common.multi_lidar: true` | 启用双雷达架构 |
| `mapping.lidar2_update_en: true` | 副雷达点云真正参与滤波器更新和地图写入 |

滤波器是**迭代扩展卡尔曼滤波（IEKF）**：先用 IMU 预测，再用当前帧点云与已有地图做点到平面匹配，迭代修正位姿。地图在内部用 **ikd-tree**（可增量更新的三维点云索引树）存储；对外常见话题是 `/Laser_map`、`/Odometry`。

更新策略选官方 **adaptive**：

- 平时可“谁来了谁更新”，缩短仅靠 IMU 空推的时间；
- 有效特征不够时，自动改成 **bundle**：把时间接近的 Mid 一帧和 Odin 一帧绑在一起更新。

---

## 3. Odin 预处理（`lidar_type2: 4` / XYZ_ONLY）

预处理把原始消息变成算法内部统一点云格式。针对 Odin 增加了类型代号 **4（XYZ_ONLY）** 解析：

1. 按 PointCloud2 字段字节偏移读 `x,y,z`；
2. 兼容 `intensity`、`confidence`、点内相对时间 `offset_time`（有的驱动叫 `time` / `t`）；
3. FAST-LIO 习惯把“相对本帧起始的时间”存在点的 **curvature**，单位是**毫秒**；Odin 文档里 `offset_time` 常以**秒**给出，读取后要 **×1000** 再写入；
4. 置信度低于阈值（常用 **30**）的点丢掉；过近盲区 / 可选过远点过滤；按抽样步长降点数；
5. 若帧内时间顺序乱，按时间重排，便于估计本帧扫描结束时刻。

这样副雷达才能进入与 Mid360 相同的流程：

> **IMU 预测 → 去畸变 → 与地图匹配 → 更新位姿**

**去畸变**：扫完一帧期间机器人可能在动，用 IMU 把各点扳回同一时刻坐标系，否则一帧内部会糊成拖影。

---

## 4. 降采样桥：`odin_raw_downsample_bridge`

坞上 Odin 桥常把已经变换到 `odom`/`map` 的点云（如 `cloud_slam`）仍以 `/odin1/cloud_raw` 发出。若把这种“已带位姿的点云”再当作雷达系输入紧耦合，相当于**位姿乘两次**，地图会错位。

桥接节点职责：

1. 看 `frame_id`：若是 `odom`/`map` 等全局系，用时间最近的 `/odin1/odometry`，再结合 Odin 雷达相对其 IMU 的固定安装，把点**反算回雷达本体系**；若本就是雷达系真原始点，则跳过；
2. 置信度过滤 + **体素降采样**（小立方格留代表点，降低算力，尽量保留逐点时间）；
3. 按本帧云时间戳，在 Mid `/Odometry` 历史里找接近戳重打时间，**|Δt| 大约 ≤ 50 ms**，避免“几何是几秒前、时间写成当前”；
4. 过旧缓存丢弃；发出去的副雷达时间戳一路增大，防止建图缓冲误判时间回绕；
5. 发布节奏尽量跟 Mid 里程计节拍对齐；Mid 还没出里程计或短暂卡住时，**降频直接发布**，避免“桥接等里程计、建图又等副雷达”死锁。

输出：`/odin1/cloud_raw_ds`（配置里 `lid_topic2`）。

---

## 5. 建图节点：`laserMapping_adaptive`

打开副雷达更新后，Odin 帧进入第二路缓冲。做 **bundle** 绑定时：

- 以当前 Mid 帧时间为基准，在 Odin 缓冲找时间最近的一帧；
- 若最近也超过约 **80 ms**，丢掉更旧那一路队头再重配，避免差太远的两帧硬拧在一起；
- 配对成功后，用标定好的 **Mid 相对 Odin 外参**，把 Odin 点变到 Mid 雷达系，再与 Mid 点、IMU 一起做 IEKF 更新。

**外参**：两台雷达之间固定的旋转和平移，需事先标定或按安装尺寸写入 `extrinsic_*_L2_wrt_L1` 等。

**启动顺序逻辑**（已改）：

- **不再**要求必须先等到第一帧 Odin 才能建图；
- Mid 点数够 → 先用 Mid 初始化地图树并持续输出里程计；
- Odin 随后到达 → 再以全程紧耦合加入更新和地图写入。

从而打破：“没有里程计就不发副雷达、没有副雷达就不出里程计”的死锁。

---

## 6. 与仓库文件对应

| 环节 | 路径 |
|------|------|
| 双雷达 YAML | `FAST_LIO_MULTI_ROS2/config/mid360_odin1.yaml` |
| adaptive 建图 | `src/laserMapping_adaptive.cpp` |
| Odin PointCloud2 预处理 | preprocess 中 `lidar_type2: 4` |
| 降采样 / 反投影桥 | `scripts/odin_raw_downsample_bridge.py` |
| 导航别名 | `scripts/multi_lio_nav_bridge.py` → `/baselink2map`、`/cloud_registered_1`、`/cloud_registered_body_1` |
| 一键 launch | `launch/mid360_odin1_multi.launch.py` |

相关说明：[`odin_attitude_measure.md`](./odin_attitude_measure.md)、[`odin1_mid360_map_and_fusion.md`](./odin1_mid360_map_and_fusion.md)、[`go2_localization_to_navigation.md`](./go2_localization_to_navigation.md)。

---

## 7. 启动与验收

每个终端先：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
```

顺序：

1. Livox Mid360（`/livox/lidar`、`/livox/imu`）  
2. Odin TCP 桥（`/odin1/cloud_raw`、`/odin1/odometry`、`/odin1/imu`）  
3. 融合：

```bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/fast_lio_multi_ws/install/setup.bash

ros2 launch fast_lio_multi mid360_odin1_multi.launch.py \
  update_method:=adaptive \
  config:=mid360_odin1.yaml \
  enable_odin_timed:=true \
  enable_frame_fuse:=true \
  enable_nav_bridge:=true \
  enable_odin_system_odom:=false
```

检查：

```bash
ros2 param get /laserMapping_multi common.multi_lidar      # true
ros2 param get /laserMapping_multi mapping.lidar2_update_en # true
ros2 topic hz /Odometry
ros2 topic hz /odin1/cloud_raw_ds
ros2 topic hz /baselink2map
ros2 topic hz /cloud_registered_1
```

日志侧应能看到双雷达开启、`lidar_type` 主=1 / 副=4，以及 adaptive / bundle 更新信息。

接到 DDDMR / EGO 时仍订：

- odom：`/baselink2map`  
- cloud：`/cloud_registered_1`（body 侧常用 `/cloud_registered_body_1`）

---

## 8. 风险备忘

- **全局系误当雷达系**：务必经 `odin_raw_downsample_bridge`；禁止把 `/odin1/cloud_slam`（已带位姿）直接喂 MULTI。  
- **外参不准**：Odin 进 IEKF/ikdtree 会在运动中把位姿和地图拉歪；先粗标定再开 `lidar2_update_en`。  
- **时间错配**：桥接重打戳 |Δt| 门限、bundle 配对门限过松都会导致转弯拖影或错层。  
- **与旧 open3d_loc 并存**：不要同时发两套 `/baselink2map`。  
- RViz 长 Decay 看当前扫描会像“地图在转”；主看 `/Laser_map` / 累积图，Decay=0。
