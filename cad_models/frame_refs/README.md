# 外参测量：两个坐标系在哪里

测量目标：`T_base_odin`（Odin 相对 Go2 机身），不测 Mid（官方安装用名义值）。

## ① Go2 机身系 = `base` / `base_link`

- 官方文档：https://support.unitree.com/home/zh/developer/about_Go2  
  章节「坐标系，关节旋转轴与关节零点」
- 本地图：`go2_joint_frames_official.png`
- 原点：机身几何中心附近（URDF `base` / ROS `base_link`）
- 轴向（站立、正视前方）：
  - **X 红：前**
  - **Y 绿：左**
  - **Z 蓝：上**
- 实物参考：躯干中心；四髋关节相对躯干对称，可用于对齐卷尺/SW。

## ② Odin1 点云系 = `L`（lidar）

- 官方文档：https://manifoldtechltd.github.io/wiki/odin_series/odin1/5.%20Data%20output_.html  
  §5.1 坐标系定义
- 本地图：`odin1_coordinate_official.png`、`odin1_structure2.png`
- 三个系：
  - **I**：IMU（设备本体基准）
  - **L**：点云 / `/odin1/cloud_raw`（**外参请量这个**）
  - **C**：相机（OpenCV：X 右、Y 下、Z 前）
- **L 原点**：中间那颗雷达窗口/镜头中心附近  
- **L 轴向**（正视设备前面）：**X 前、Y 右、Z 上**（与 I 平行，仅平移）
- 固定外参（不必再量）`T_imu_lidar`：

```text
t = [-0.02663, 0.03447, 0.02174] m
R = I
```

即 `P_imu = T_imu_lidar * P_lidar`。

## 量什么

在 SW / 实机上量 **Go2 `base` → Odin `L`**：

1. 平移：Odin `L` 原点在 `base` 下的 `(x, y, z)`（米）
2. 姿态：主要确认 Odin 是否相对机身有 **yaw**（绕 Z）；官方安装一般 pitch/roll≈0

得到 `T_base_L` 后，再按名义 `T_base_mid` 换成融合用的 `T_L1_L2`（Mid→Odin）。

## 源链接

| 图 | 来源 |
|----|------|
| Go2 关节/机身轴 | Unitree 关于 Go2 |
| Odin I/L/C | Manifold Odin1 Wiki 数据输出 |
| Odin 尺寸/IMU 孔位 | 同 Wiki structure 图 |
