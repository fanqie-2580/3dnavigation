# 拓展坞 Humble Docker（R35，不升级系统）

目标：拓展坞保持 Ubuntu 20.04 / L4T **R35.3.1**，用 Docker 跑 ROS 2 Humble 导航栈；Odin USB 驱动仍在宿主机 Noetic。

## 为什么不用本机 `dddmr:l4t_r36`

| 机器 | L4T | 可用镜像 |
|------|-----|----------|
| Orin 本机 `192.168.123.222` | R36.4 | `dddmr:l4t_r36` |
| 拓展坞 `192.168.123.18` | R35.3.1 | **`dustynv/ros:humble-desktop-l4t-r35.3.1`** |

R35/R36 容器与宿主机 CUDA/用户态不兼容，禁止把 `l4t_r36` 镜像拷到坞上跑。

## 角色划分

```
宿主机 Noetic（已有）
  odin_ros_driver / host_sdk  → /odin1/cloud_slam, imu, odom
  odin_ros1_tcp_server        → :5601（可选，本机容器也可用 ros1_bridge）

Docker Humble（--network host --runtime nvidia）
  定位 / EGO / DDDMR / go2_bridge / RViz2
```

数据面：`--network host`，与宿主机同网栈，点云/DDS 无明显额外损耗。

## 目录

| 路径 | 用途 |
|------|------|
| `/home/unitree/docker_nav/` | 启停脚本、日志 |
| `/home/unitree/humble_nav_ws/` | 挂进容器的源码工作区 |
| `/home/unitree/odin_bridge/` | 宿主机 Odin/TCP（保持不动） |

## 拉镜像（坞上 Docker Hub 常超时，走镜像）

```bash
sudo docker pull docker.m.daocloud.io/dustynv/ros:humble-desktop-l4t-r35.3.1
sudo docker tag docker.m.daocloud.io/dustynv/ros:humble-desktop-l4t-r35.3.1 \
  dustynv/ros:humble-desktop-l4t-r35.3.1
# 日志：tail -f /home/unitree/docker_nav/pull_humble.log
```

## 一键启容器

```bash
bash /home/unitree/docker_nav/start_humble_nav.sh
# 进入（未重新登录前可能要 sudo）：
sudo docker exec -it humble_nav bash
```

## 容器内环境

dustynv 镜像的 Humble 是源码装的，入口是：

```bash
source /opt/ros/humble/install/setup.bash   # 不是 /opt/ros/humble/setup.bash
```

## 容器内首次编译（拉完镜像、同步源码后）

```bash
sudo docker exec -it humble_nav bash -lc '
  source /opt/ros/humble/install/setup.bash
  cd /root/humble_nav_ws
  # 按需选包，勿一次全编 dddmr
  colcon build --symlink-install --packages-select <pkg>
'
```

注意：dustynv Humble 基于 20.04，**多数 `apt install ros-humble-*` 不可用**，依赖需源码编译。

## 与本机联调差异

- 坞上容器名：`humble_nav`（本机仍是 `dddmr_dev`）
- 坞上 Odin 在本机 USB，不必再 TCP 到 222；容器内可直接 `ros1_bridge` 或继续用现有 TCP 环回
- 本机 R36 上验证通过的 launch/参数可同步到 `humble_nav_ws`，在坞上重新编译
