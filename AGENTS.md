# Codex 工作区说明

> **推送到 GitHub（必读）**：源码改动后同步推送流程见  
> [`docs/push_to_github.md`](./docs/push_to_github.md)。  
> 上传根目录：`/home/ll/3dnavigation_github_upload`；远端：`https://github.com/fanqie-2580/3dnavigation`（HTTPS + PAT，不用 SSH）。

这个工作区用于我的机器人导航、规划、定位相关项目联调。 请严格按照本文件理解项目结构、运行环境和修改规则。

## 0. 当前工作区包含的项目

当前 Codex 工作区根目录建议为：

```bash /home/ll/codex_robot_ws ```

该目录下应包含 Mid360 三软链，以及 Odin1 分叉（物理独立，改一侧勿动另一侧）：

```text
dddmr_navigation -> /home/ll/dddmr_navigation
ego-planner-swarm-ros2_version -> /home/ll/ego-planner-swarm-ros2_version
FAST_LIO_LOCALIZATION_HUMANOID-humble -> /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble
dddmr_navigation_odin1 -> /home/ll/dddmr_navigation_odin1
ego-planner-swarm-odin1 -> /home/ll/ego-planner-swarm-odin1
```

分叉说明见 [`docs/mid_odin_source_split.md`](./docs/mid_odin_source_split.md)。  
**禁止**在同一 shell 同时 `source` Mid 与 Odin 的 `install/setup.bash`（包名相同会互相覆盖）。

不要在 `/home/ll` 家目录直接进行全局分析或全局修改，因为家目录下还有很多无关项目，例如 autoware、unitree、ws_livox、ros_ws、bag 文件等。

## 1. 项目职责

### 1.1 dddmr_navigation（Mid360 全局规划）

`dddmr_navigation` 是 Mid360 全局规划树。

宿主机路径：

```bash /home/ll/dddmr_navigation ```

Docker：`dddmr_dev`，容器内路径 `/root/dddmr_ws`。

- 修改代码：宿主机 `/home/ll/dddmr_navigation`
- 编译/运行：优先 `dddmr_dev` 内
- Odin1 **不要**改此树；Odin 用 `dddmr_navigation_odin1` + `dddmr_odin1`

### 1.1b dddmr_navigation_odin1（Odin1 全局规划）

宿主机：`/home/ll/dddmr_navigation_odin1`  
Docker：`dddmr_odin1` → `/root/dddmr_ws`  
入口：`dddmr_global_ego_odin1.launch.py`（Odin 专用 yaml/rviz/maps）

### 1.2 ego-planner-swarm-ros2_version（Mid360 局部规划）

宿主机路径：

```bash /home/ll/ego-planner-swarm-ros2_version ```

该项目在宿主机上直接编译和运行。

主要关注：

- 局部规划 - 轨迹生成 - swarm 多机逻辑 - odom / map / trajectory 相关 topic - launch 文件和参数文件

### 1.2b ego-planner-swarm-odin1（Odin1 局部规划）

宿主机：`/home/ll/ego-planner-swarm-odin1`  
入口：`dddmr_reference_tracking_odin1.launch.py` + `odin1_navigation.rviz`  
与 Mid EGO 树物理分离；调 Odin 参数只改此树。

### 1.3 FAST_LIO_LOCALIZATION_HUMANOID-humble

`FAST_LIO_LOCALIZATION_HUMANOID-humble` 是定位相关项目。

宿主机路径：

```bash /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble ```

该项目在宿主机上直接编译和运行。

主要关注：

- LiDAR / IMU / odom - map、odom、base_link 等坐标系 - 点云 topic - localization 输出 - tf 关系 - launch 和 config 参数

## 2. dddmr_navigation 的 Docker 环境

dddmr 使用的 Docker 镜像是：

```bash dddmr:l4t_r36 ```

用户当前常用启动命令是：

```bash docker run -it --rm \
    --network host --runtime nvidia --privileged \ -v /dev:/dev \ -v /etc/localtime:/etc/localtime:ro \ -v /etc/timezone:/etc/timezone:ro \ -v /home/ll/dddmr_navigation:/root/dddmr_ws \ -v 
    /home/ll/dddmr_bags:/root/dddmr_bags \ -v /tmp/.X11-unix:/tmp/.X11-unix \ -e DISPLAY=$DISPLAY \ dddmr:l4t_r36 /bin/bash
```

注意：

- `/home/ll/dddmr_navigation` 挂载到容器内 `/root/dddmr_ws` - `/home/ll/dddmr_bags` 挂载到容器内 `/root/dddmr_bags` - 容器使用 `--network host` - 容器使用 `--runtime nvidia` - 容器使用 
`--privileged` - 容器使用 Jetson / L4T 相关环境 - 用户当前命令包含 `--rm`，容器退出后会自动删除

如果需要 Codex 执行 Docker 内验证命令，优先检查当前是否有正在运行的 dddmr 容器：

```bash docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}' ```

如果没有正在运行的 dddmr 容器，不要擅自启动复杂硬件相关容器。应先提示用户启动容器，或者给出建议命令。

如果用户允许使用固定容器名，推荐容器名为：

```bash dddmr_dev ```

推荐的非 `--rm` 启动方式是：

```bash docker run -it \
    --name dddmr_dev \ --network host --runtime nvidia --privileged \ -v /dev:/dev \ -v /etc/localtime:/etc/localtime:ro \ -v /etc/timezone:/etc/timezone:ro \ -v 
    /home/ll/dddmr_navigation:/root/dddmr_ws \ -v /home/ll/dddmr_bags:/root/dddmr_bags \ -v /tmp/.X11-unix:/tmp/.X11-unix \ -e DISPLAY=$DISPLAY \ dddmr:l4t_r36 /bin/bash
```

进入正在运行的容器：

```bash docker exec -it dddmr_dev bash ```

如果容器名不是 `dddmr_dev`，请通过 `docker ps` 查找实际容器名，不要猜。

Odin1 全局规划容器固定名：`dddmr_odin1`（挂载 `/home/ll/dddmr_navigation_odin1`）。  
进入：`docker exec -it dddmr_odin1 bash`。不要用 `dddmr_dev` 跑 Odin1。

## 3. 编译和验证命令

### 3.1 dddmr_navigation 编译方式

Mid360：`dddmr_dev` 内编译。

```bash docker exec -it dddmr_dev bash -lc ' cd /root/dddmr_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install ' ```

Odin1：`dddmr_odin1` 内编译。

```bash docker exec -it dddmr_odin1 bash -lc ' cd /root/dddmr_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install ' ```

如果只改了某个 package，优先使用局部编译，例如：

```bash docker exec -it dddmr_dev bash -lc ' cd /root/dddmr_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select <package_name> ' ```

如果需要运行 dddmr 已编译环境：

```bash docker exec -it dddmr_dev bash -lc ' cd /root/dddmr_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg list ' ```

不要在宿主机上直接假设下面命令一定可用：

```bash cd /home/ll/dddmr_navigation colcon build ```

dddmr 的构建验证应优先通过 Docker。

### 3.2 ego-planner-swarm-ros2_version 编译方式

ego 在宿主机编译：

```bash cd /home/ll/ego-planner-swarm-ros2_version source /opt/ros/humble/setup.bash colcon build --symlink-install ```

如果只改了某个 package，优先局部编译：

```bash cd /home/ll/ego-planner-swarm-ros2_version source /opt/ros/humble/setup.bash colcon build --symlink-install --packages-select <package_name> ```

### 3.3 FAST_LIO_LOCALIZATION_HUMANOID-humble 编译方式

FAST_LIO 在宿主机编译：

```bash cd /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble source /opt/ros/humble/setup.bash colcon build --symlink-install ```

如果只改了某个 package，优先局部编译：

```bash cd /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble source /opt/ros/humble/setup.bash colcon build --symlink-install --packages-select <package_name> ```

## 4. 运行环境约束

默认 ROS 版本：

```text ROS 2 Humble ```

dddmr Docker 内也默认按 ROS 2 Humble 处理。

如果涉及 DDS / RMW，先检查当前环境，不要擅自修改：

```bash echo $RMW_IMPLEMENTATION ```

如果需要 Cyclone DDS，优先使用：

```bash RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ```

但不要在没有用户要求的情况下随意修改系统级 `.bashrc`、Dockerfile 或全局环境变量。

## 5. 修改代码前必须做的事

在修改任何代码之前，先完成下面步骤：

1. 明确本次任务涉及哪个项目： - `dddmr_navigation` - `ego-planner-swarm-ros2_version` - `FAST_LIO_LOCALIZATION_HUMANOID-humble` - 或多个项目联调

2. 先搜索相关文件，不要凭空猜路径。

推荐搜索命令：

```bash rg "关键字" . find . -name "*.launch.py" find . -name "*.yaml" find . -name "CMakeLists.txt" find . -name "package.xml" ```

3. 如果涉及 ROS topic、frame、参数、launch，需要搜索引用关系：

```bash rg "topic_name" . rg "frame_id" . rg "base_link|odom|map" . rg "declare_parameter|get_parameter" . rg "Node\\(" . ```

4. 修改前先说明计划修改哪些文件，以及为什么修改。

不要一上来就大面积改代码。

## 6. 严禁或谨慎修改的内容

除非用户明确要求，否则不要修改这些目录或文件：

```text build/ install/ log/ .git/ .cache/ node_modules/ __pycache__/ *.bag *.db3 *.mcap *.onnx *.pt *.pth *.engine *.tar.gz *.zip ```

不要修改大型数据文件、rosbag、模型权重、地图数据，除非用户明确要求。

不要随意删除文件。

不要随意重命名 package。

不要随意改变 topic 名、frame_id、namespace、参数名、消息类型。

不要随意修改 `package.xml` 和 `CMakeLists.txt` 的依赖；如果确实需要新增依赖，必须说明原因。

## 7. ROS 2 修改规则

涉及 ROS 2 代码时，请遵守：

- 修改 topic 名之前，先全项目搜索发布者和订阅者 - 修改 frame_id 之前，先分析 tf tree 关系 - 修改参数名之前，先搜索参数声明、读取、yaml 配置和 launch 传参 - 修改 launch 文件之前，先确认该 
launch 是否被其他 launch include - 修改 message 或 service 之前，先搜索所有使用位置 - 修改 namespace 或 robot id 之前，先确认多机或多节点通信关系 - 不要把临时调试 topic 写死到正式代码中 - 
不要为了让代码能编译而注释掉核心逻辑

常见需要重点关注的坐标系名称：

```text map odom base_link base_footprint livox_frame camera_link imu_link ```

如果这些 frame 名出现不一致，要先报告问题并给出分析，再修改。

## 8. 三个项目联调时的分析顺序

如果任务涉及 dddmr、ego、FAST_LIO 三者联调，按下面顺序分析：

1. FAST_LIO 输出什么定位信息： - odom topic - pose topic - path topic - map frame - odom frame - base frame - tf 发布情况

2. ego 需要什么输入： - odom - point cloud / map - goal - trajectory - drone_id / namespace - frame_id

3. dddmr_navigation 需要什么输入和输出： - sensor 输入 - map / localization 输入 - cmd_vel 或控制输出 - navigation goal - global/local planner 接口

4. 对比三者： - topic 名是否一致 - frame_id 是否一致 - QoS 是否可能不匹配 - namespace 是否一致 - ROS_DOMAIN_ID 是否一致 - RMW_IMPLEMENTATION 是否一致 - 是否一个在 
   Docker，一个在宿主机导致网络或环境变量不同

由于 dddmr 使用 `--network host`，Docker 和宿主机 ROS 2 节点理论上可以通过 host 网络通信，但仍需要检查：

```bash echo $ROS_DOMAIN_ID echo $RMW_IMPLEMENTATION ros2 topic list ros2 node list ros2 topic echo <topic_name> ros2 topic info <topic_name> -v ros2 run tf2_tools view_frames ```

## 9. Docker 与宿主机通信注意事项

dddmr 在 Docker 中运行，ego 和 FAST_LIO 在宿主机运行。

因此联调问题可能来自：

- Docker 内和宿主机的 `ROS_DOMAIN_ID` 不一致 - Docker 内和宿主机的 `RMW_IMPLEMENTATION` 不一致 - DDS 发现问题 - QoS 不匹配 - frame_id 不匹配 - topic namespace 不匹配 - Docker 内没有 source 
正确的 ROS workspace - 宿主机和 Docker 启动顺序导致 topic 没有出现

检查 Docker 内 ROS 环境时：

```bash docker exec -it dddmr_dev bash -lc ' echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID echo RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION source /opt/ros/humble/setup.bash cd /root/dddmr_ws [ -f 
install/setup.bash ] && source install/setup.bash ros2 node list ros2 topic list ' ```

检查宿主机 ROS 环境时：

```bash echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID echo RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION source /opt/ros/humble/setup.bash ros2 node list ros2 topic list ```

## 10. 代码风格要求

### C++

- 保持原项目代码风格 - 不要无意义地重排格式 - 不要大面积重构 - 优先小范围修改且尽量避免写兜底代码 - 添加必要注释，但不要写过多废话注释 - 注意 ROS 2 Humble API 兼容性 - 注意 rclcpp 参数、QoS、callback 
group、timer、subscription 生命周期

### Python

- 保持 shebang 和可执行权限 - 不要随意改变脚本入口 - 修改 launch.py 时注意 Python 语法和 LaunchConfiguration 使用方式 - 不要把绝对路径写死，除非项目原本就是这样设计

### YAML / 参数文件

- 保持缩进 - 不要删除未知参数 - 修改参数前先搜索读取位置 - 修改后说明该参数影响哪个节点

### CMakeLists.txt / package.xml

- 新增依赖必须说明原因 - 修改后给出对应 package 的编译命令 - 不要删除看似没用但可能运行时需要的依赖

## 11. 完成任务后的输出格式

每次完成修改后，请按下面格式汇报：

```text 修改摘要： - 修改了什么 - 为什么这么改

涉及文件： - path/to/file1 - path/to/file2

验证方式： - 已运行的命令 - 命令结果摘要

如果未验证： - 说明没有验证的原因 - 给出用户可以复制执行的验证命令

风险和注意事项： - topic/frame/参数变化 - Docker/宿主机环境差异 - 需要用户手动确认的地方 ```

## 12. 优先采用的工作方式

默认工作方式：

1. 先阅读相关文件 2. 先分析，不要急着改 3. 给出修改计划 4. 小范围修改 5. 编译或给出编译命令 6. 汇报修改结果和风险

如果用户说“先不要改代码”，只能分析，不能写文件。

如果用户说“直接改”，也要尽量保持小范围修改。

如果不确定项目运行方式、容器名、launch 入口、topic 含义，应先询问或搜索，不要猜。

## 13. 常用路径速查

宿主机：

```bash /home/ll/dddmr_navigation /home/ll/dddmr_bags /home/ll/ego-planner-swarm-ros2_version /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble /home/ll/codex_robot_ws ```

Docker 内：

```bash /root/dddmr_ws /root/dddmr_bags ```

推荐 Codex 启动目录：

```bash cd /home/ll/codex_robot_ws codex ```

## 14. 不要做的事

- 不要在 `/home/ll` 下全局搜索并修改所有项目 - 不要修改无关项目，例如 autoware、unitree_ros2、ws_livox、ros_ws，除非用户明确要求 - 不要修改 bag 文件和模型文件 - 不要假设 dddmr 
可以在宿主机直接运行 - 不要假设 Docker 容器名一定存在，先用 `docker ps` 确认 - 不要自动删除 Docker 容器、镜像、数据卷 - 不要执行危险命令，例如 `rm -rf`、`docker system 
prune`，除非用户明确要求并确认 - 不要在没有说明的情况下改 `.bashrc`、系统代理、Dockerfile、全局环境变量 - 不要为了快速通过编译而屏蔽核心功能代码

## 15. 用户当前重点项目

用户当前主要使用：

```text dddmr_navigation ego-planner-swarm-ros2_version FAST_LIO_LOCALIZATION_HUMANOID-humble ```

其中：

```text dddmr_navigation：宿主机修改代码，Docker 内运行和编译 ego-planner-swarm-ros2_version：宿主机运行和编译 FAST_LIO_LOCALIZATION_HUMANOID-humble：宿主机运行和编译 ```

请始终按照这个环境理解任务。

