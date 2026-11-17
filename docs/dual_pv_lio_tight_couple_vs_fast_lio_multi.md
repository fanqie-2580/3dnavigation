# 当前双雷达 PV-LIO 紧耦合改动对照 FAST-LIO-MULTI

对照对象有两层。一层是官方上游 `FAST_LIO_MULTI_ROS2_upstream`（原版双雷达 Bundle / Async 紧耦合骨架）；另一层是本机为 Mid360+Odin1 长期调过的 `FAST_LIO_MULTI_ROS2_mid_odin`（在官方骨架上加了 Odin 原生去畸变、两段式 IEKF、残差裁剪、陀螺门控、建图插入门控等）。当前运行的紧耦合建图是 `pv_lio_dual_ws` 里的 PV-LIO 分叉，不是再跑 FAST-LIO-MULTI 二进制；但紧耦合语义刻意按 mid_odin 那套“Mid 先锁、Odin 软更新”移植过来，同时因为底层是 PV-LIO 的体素地图与对角 IEKF，实现形态并不等同于把 mid_odin 源码原样搬过去。下文按数据通路、时间同步、去畸变、两段式更新、观测权重、建图与可视化、外参与工程化，把“官方 MULTI 有什么 → mid_odin 改成什么 → 现在 PV-LIO dual 又改成什么”写成连贯说明。

## 一、问题起点：官方 FAST-LIO-MULTI 的紧耦合是什么

官方 FAST-LIO-MULTI 的核心不是“两个独立里程计拼图”，而是同一套 IEKF 状态里吃两路雷达观测。`MeasureGroup` 自带 `lidar` / `lidar2`、两套起止时刻，IMU 默认仍是主雷达那一路。同步有两条路：`sync_packages_async` 按时间先后单独处理某一雷达；`sync_packages_bundle` 以主雷达为轴，把时间窗内的副雷达点云拼进来，再用固定外参 `LiDAR2_wrt_LiDAR1` 刚体变到主雷达坐标系，然后和主雷达点一起做去畸变与点面残差。也就是说，官方紧耦合的默认形态是“先几何对齐到同一 body，再一次性或交替做状态更新”，副雷达并没有独立 IMU 时间轴，也没有专门的“第二段软更新 + 大残差裁剪 + 运动门控建图”这一整套 Odin 特化逻辑。地图侧是 ikd-tree，位姿真值是 IEKF 输出的 `/Odometry`，副雷达成功参与更新后也可以进同一棵地图树。

这对两台同质机械/固态雷达、时间同源或近似同源的场景是成立的。落到 Mid360 + 留形 Odin1 时会撞上三件事：Odin 点云时间戳更像 USB 接收时刻而不是采集时刻；Odin 自带 IMU 与 Mid IMU 噪声尺度差一个数量级；Odin 几何密度高、近处平面强，一旦和 Mid 同时进同一轮 IEKF，很容易把 Mid 已经锁好的航向/高度“拧走”。于是 mid_odin 分叉在官方 Bundle 之上长出了一套 Odin 专用通路，而当前 PV-LIO dual 又把这套通路的思想迁到了体素建图框架里。

## 二、数据模型：从官方双雷达 Measure 到“双 LiDAR + 双 IMU”

官方 `MeasureGroup` 只有一路 `imu`。mid_odin 为了让 Odin 在自己的时钟上做扫描内去畸变，增加了 `imu2`、`lidar_beg_time2` / `lidar_end_time2`，并订阅 `/odin1/imu`。当前 PV-LIO dual 的 `common_lib.h` 同样扩展了 `lidar2`、`lidar2_fresh`、`imu2` 以及第二套起止时刻；这是相对原版单雷达 PV-LIO 的结构性改动，语义上对齐 mid_odin，而不是对齐官方“副雷达共用主 IMU”的假设。

订阅侧，官方 MULTI 期望两路都是可预处理的雷达消息；mid_odin 实际吃 Mid 的 Livox CustomMsg 与 Odin 的 PointCloud2（还曾走过 bridge 话题 `/odin1/cloud_l2_body`）。PV-LIO dual 直接订 `/livox/lidar`、`/livox/imu`、`/odin1/cloud_raw`、`/odin1/imu`，并在 preprocess 里为 Odin 增加了独立类型分支：读 `offset_time`、按 `confidence` 门控、第二套 `point_filter_num2` / 盲区 / 最远距。官方没有 Odin confidence；mid_odin 有；dual 也有，并把第二路最远距收得更紧（约 20 m），避免 Odin 远点把 Mid 实时配对拖垮。

外参方面，官方用“各雷达到各自 IMU”再合成 `LiDAR2_wrt_LiDAR1`，或直接给雷达间外参。mid_odin 与 dual 都显式维护 Mid 雷达相对 Mid IMU、Odin 雷达相对 Odin IMU、以及 Odin 雷达相对 Mid 雷达的刚体。dual 的 yaml 里还固化了地面标定过的 `extrinsic_*_L2_wrt_L1`（含约 −13.8° 的俯仰）和 Odin 雷达到 Odin IMU 的小平移；紧耦合更新时外参估计默认关闭，避免 Odin 残差去“学”外参把状态拧歪。

## 三、时间同步：官方 Bundle 窗 → mid_odin 的 bias/soft-wait → dual 的 end-time 配对

官方 Bundle 大致是：主雷达入队后，在一定时间邻域内把副雷达帧拼进去，用副雷达 header 与主雷达 beg 的接近程度做关联，没有专门的“Odin USB 滞后补偿”和“等晚到帧”的工程参数。mid_odin 在长期踩坑后加了三件关键工具：固定或在线估计的 `lidar2_stamp_bias_sec`（曾默认约 65 ms，用来把 Odin stamp 拉回近似采集时刻）、`lidar2_max_sync_dt` 紧窗、以及 `lidar2_soft_wait_sec` 短等待，让 Mid 帧先占坑、晚到一点的 Odin 仍能配上，而不是直接 Mid-only。配对度量也从简单 beg 对齐演进到更关心扫描中心或末端是否落在同一物理时刻，因为“整帧用错了 Mid 位姿”会在转动时造成橙色 Odin 先错后跳。

当前 PV-LIO dual 继承了 soft-wait 与双缓冲思想，但参数和打分更贴近实测相位：stamp bias 默认置零（现场测过 header 偏差接近 0，再硬加 65 ms 反而制造约 60 ms 的末端错位）；同步窗放宽到约 0.20 s；soft-wait 约 0.18 s，大于一个 Odin 周期，专门对付相位漂移到百毫秒级时的“静止画面闪烁、Odin 时有时无”。配对打分以“Odin 末端经 bias 映射到 Mid 钟后，与 Mid 末端之差”为主，并记录 `bundle_end_dt`；后续 Stage-2 是否允许进 IEKF，首先看这个末端差是否小于可配门限。官方没有 `end_dt` 门控；mid_odin 有中心差 / sync_dt / map_insert 专用 dt；dual 把“能否紧耦合更新”直接绑在 end_dt 与陀螺门上，逻辑更短，也更偏工程开关。

另外，官方与 mid_odin 的 Bundle 仍可能把副雷达点变换后并进同一处理云；dual 明确禁止把 Odin 并进 Mid 的 `feats_undistort`。Odin 只走独立 deskew → body 外参 → 显示/建图/Stage-2 子采样云。这是相对官方“合成一朵云再更新”的关键分叉，也是为了消灭早期“单次更新被 Odin 猛拽”的路径。

## 四、去畸变：官方共用主 IMU → mid_odin 的 Odin-native → dual 的陀螺-only 与 Mid-end 对齐

官方 `UndistortPclMultiLiDAR` 的假设是：副雷达点已经（或即将）落在主雷达时间语义下，用主 IMU 积分链把各点补偿到统一末端。这对同源 IMU、近似同步的双雷达合理。mid_odin 发现 Odin 若仍按 Mid 时间曲率去畸变，等于系统性地忽略 header 差与 bias，转动误差近似角速度乘时间偏差；因此增加了 `UndistortOdinNative`：在 Odin 雷达系内，用 `/odin1/imu` 积分，把点补偿到该 Odin 扫描末端，再整体用 `T_L2_wrt_L1` 变到 Mid body。橙色可视化还可以按 Odin 末端在 Mid 钟上的时刻去查 Stage-1 位姿，避免整帧用 Mid 末端位姿投影造成的航向涂抹。

当前 PV-LIO dual 同样实现了 `UndistortOdinNative`，并在 mid_odin 基础上又改了两处关键行为。第一，目标时间可以追 Mid 末端在 Odin 钟上的对应时刻，而不是死钉 Odin 自身末端；注释里写得很清楚：若先 deskew 到 Odin 末端，再用 Mid 末端位姿投世界，残余 Δt 乘偏航会把旧几何画到错误航向上。第二，Odin 加速度被故意丢掉，只做陀螺旋转去畸变。原因是实测 `/odin1/imu` 加速度噪声极大，带加速度的相对积分会在静止 body 云上凭空泵出几十厘米的 Z 起伏；Mid 那路仍走完整 IMU 传播。准静态且 `|end_dt|` 很小（约 30 ms 内）时，dual 甚至跳过 Odin deskew（`ok_nodeskew`），避免噪声 IMU 把已经对齐的帧拧花。这些都不是官方 MULTI 的行为；mid_odin 的 native deskew 仍整合加速度；dual 更激进地按传感器噪声剖面裁剪了物理模型。

IMU 初始化重力对齐方面，dual 的注释写明是对齐 mid_odin：用加速度均值估计重力方向，把世界系拉到与 RViz 网格水平，避免 CloudMap 整幅倾斜。原版 PV-LIO 没有这套双雷达场景下的“水平世界”约定。

## 五、紧耦合更新：官方一次/交替更新 → mid_odin 两段式 → dual 的 Stage-2 软更新与回滚

这是“紧耦合”一词在本项目里的核心含义。

官方 Bundle 路径通常是去畸变后，对融合特征做 `update_iterated_dyn_share_modified`；Async 路径则按当前是雷达 1 还是雷达 2 更新。副雷达观测与主雷达观测在信息形式上同质，主要靠外参与时间对齐保证一致，并没有“先只让主雷达说话、再让副雷达轻轻说一句、说错了就撤回”的标准流程。

mid_odin 在 Bundle 且允许点更新时，显式做两段：`residual_select = 1` 只用 Mid 点面残差锁住位姿；然后若 `lidar2_point_update_en`、Bundle 已配对、姿态门允许，再把 `residual_select = 2`，用放大后的标量协方差 `R_odin = LASER_POINT_COV * odin_cov_scale` 跑第二轮迭代更新。Stage-2 里对 Odin 点做大残差剔除（默认约 6 cm），并按点相对 Mid 末端的时间、角速度、力臂给每点加时序不确定性，让时间越不对齐的点越“软”。此外还曾试验过用 `/odin1/odometry` 做相对位姿因子的软拉拽，配置上后来默认关掉，因为会抖。地图插入与橙色显示尽量挂在 Stage-1 位姿或 Odin 末端查询位姿上，避免 Stage-2 微拧航向后可视化跟着抽一下。

当前 PV-LIO dual 把同一哲学搬进 `voxelMapping.cpp`，但实现接口换成 PV-LIO 的 `update_iterated_dyn_share_diagonal` 与体素平面观测。流程是：先只用 Mid 降采样特征做第一段 IEKF；若更新步长/转角过大则整段回滚，并禁止本帧 Stage-2，连续失败还能冻结到上一帧好位姿，防止错误 IMU 积分把坏平面写进体素地图。只有 `odin_s2_enable`、末端差 `|end_dt|` 不超过门限、帧平均陀螺低于门限时，才把 Odin body 云子采样到至多约 400 点，替换临时特征云做第二段更新。观测模型里若处于 `iekf_odin_stage`，点面距离超过 `odin_max_residual` 的行被置零雅可比并赋极小信息权重，等价于 mid_odin 的残差裁剪。第二段结束后比较相对 Mid 锁定态的平移/旋转增量，以及被裁剪点比例；超门限就 `change_x` 回到 Mid 锁定态，计为 `reject_s2`，否则 `accept_s2`。体素地图增量始终改回 Mid 降采样云，Stage-2 的 Odin 子采样云绝不直接当地图写入源——这一点比 mid_odin“IEKF 成功后再决定是否 insert Odin 到 ikdtree”更干净地分离了“状态更新用的点”和“地图几何用的点”。

相对 mid_odin 的默认调参，dual 为了让步行走廊也能跑真 Stage-2，把陀螺门从“近静止才更新”放宽到大约 2 rad/s，末端差门从约 50 ms 放到约 120 ms，残差裁剪约 8 cm，裁剪比例容忍到约 0.9，并给 Stage-2 单独更紧的跳变拒绝（约 3 cm / 0.04 rad）。信息权重用 `lidar2_cov_scale`（yaml 默认 100，对齐 MULTI 量级）再乘软因子，并按点距参考距离的平方放大协方差，带一点 MA-LIO 式“远点更不信”的味道；`odin_info_weight` 再整体调节 Odin 说话的响度。官方没有这些门；mid_odin 有残差裁剪与 cov scale，但 Stage-2 默认更强调“有 Bundle 就软更新”，运动抑制更多落在建图插入与姿态门，而不是像 dual 早期那样用很紧的陀螺门直接不跑 Stage-2。dual 现在的 `ok_map_gyr` / `ok_map` / `ok_s2` 标签，就是在说：Odin 几何也许还能用于显示或加图，但本帧有没有真正把 Odin 残差写进 IEKF。

## 六、建图与显示：官方 ikdtree 同树插入 → mid_odin 多级 insert 门控 → dual 的 Global 体素与 require_s2

官方成功耦合后，副雷达点可以进入与主雷达同一套 ikd-tree 地图。mid_odin 为了防止“IEKF 窗内时间仍偏、转动中把错误 Odin 写进不可撤销的地图”，加了大量 insert 门控：同步 dt、中心 dt、静止陀螺放宽、运动陀螺收紧、近邻支撑、候选体素多帧提升、冷却时间等；并长期区分“紧耦合真值地图 `/Laser_map`”与“Python 软 densify 仅可视化”。橙色 `/cloud_odin_registered` 的位姿查询模式（A/D 等）也是 mid_odin 特有的可视化时间语义，不回写 IEKF。

PV-LIO dual 的地图真值路径不同：后端是 PV-LIO 体素地图，前端给用户看的是独立维护的 `global_cloud_map`（加 pending 缓冲）。Global 累积对 Mid 有跳变门、hold 帧禁写、可选“必须静止才加密”、静止用稠密去畸变云、运动用降采样云等策略；对 Odin 则由 `odin_global_insert_en` 与 `odin_global_require_s2` 控制——当前紧耦合建图默认要求本帧 `accept_s2` 才把全量 Odin body 云并进 Global，避免“S2 拒识或根本没跑 S2、却仍按错误位姿把 Odin 铺进地图”造成双地面或错层。显示用的 Body 云可以单独打开 `odin_body_pub_en`，让用户看见 Odin 对齐情况，这与是否写入 Global 解耦。叶大小约 8 cm，并且对历史地图做周期性异步全量 VoxelGrid 重映射，修复早期“只对 pending 体素过滤导致 Mid360 非重复扫描呈团块”的问题。官方 MULTI 没有这套 Global PCD 显示链路；mid_odin 的密度与去重在 ikdtree 与 filter_size_surf 上解决。

保存地图方面，dual 提供异步 `/save_map`，避免大 PCD 阻塞实时线程；这属于工程化增量，不是 MULTI 算法本体。

## 七、质量门与失败语义：官方较少传感器特化 → mid_odin / dual 的稀疏、Z 门、hold

官方假设副雷达帧质量大致可用。mid_odin 与 dual 都面对 Odin 偶发稀疏、置信度低、USB 抖动。dual 在 commit Odin body 时检查最少点数、相对上一帧好帧的点数比例、body 系 Z 窗（大约 −2 m 到 3.5 m），不通过则尝试在准静态下短时 hold 上一帧好 Odin 仅用于显示，并明确 hold 帧不得写入 Global，否则会在新 Mid 位姿下留下永久鬼影。缺配对、缺 imu2、deskew 失败、稀疏、Z 裁空，都会在日志里变成可读的 `skip=` 标签。这些失败语义是 mid_odin 长期联调风格的延续，官方上游没有对等的 Odin 词汇表。

## 八、运行时调参与入口：官方 yaml → dual 的 tune.py

官方与 mid_odin 主要靠 yaml 与 launch 参数。dual 额外增加 `config/dual_mid_odin_tune.py`，经 launch 灌进 `dual_tune.*` 参数，改 Stage-2 门限、Global 策略、Odin 质量门后只需重启 launch、不必重编译。这是把 mid_odin 里大量 `mapping.lidar2_*` 开关，收成一套面向“现在这台 Mid+Odin 步行建图”的短清单。当前清单的意图很明确：陀螺门与 end_dt 门宽到足够在走廊步行中出现 `accept_s2`；Global 只在真接受 Stage-2 后吃 Odin；Body 始终能看见 Odin 以便人工判断对齐。

## 九、没有迁过去或故意不同的部分

对照 mid_odin，有几块 dual 没有原样搬迁。没有 ikd-tree，也就没有候选体素多帧提升、近邻支撑 insert、橙色 t_query A/B 影子评估那一套；Global 体素是另一条产品路径。没有启用 `/odin1/odometry` 位姿因子。没有把 Odin 点按 Mid IMU 时间曲率做交叉传感器去畸变的遗留路径（那条在 mid_odin 文档里被证明是涂抹根因之一）。没有官方 Async“当前帧只处理雷达 2”的调度；dual 始终以 Mid 帧为轴，Odin 只是可缺席的附着观测。地图叶栅格与异步 remap、PCD 保存、bag 回放 `use_sim_time` 等，是 PV-LIO 工作流自己的工程层。

对照官方 MULTI，dual（以及 mid_odin）都不再把“两雷达点云简单拼进一次同质更新”当作默认紧耦合定义；默认定义变成：主雷达硬约束锁状态，副雷达在时间与运动门通过后，以降权、裁残差、可回滚的第二段更新进入同一 IEKF，并且副雷达进地图的条件可以严于进滤波器的条件。

## 十、一句话收束

官方 FAST-LIO-MULTI 提供了双雷达进同一 IEKF 的 Bundle/Async 骨架与雷达间外参变换；本机 mid_odin 分叉在此之上补齐了 Odin 时钟、原生去畸变、两段式点面更新、残差与时间加权、以及严格的副雷达建图门控；当前 PV-LIO dual 紧耦合则是把 mid_odin 的两段式软耦合语义迁到 PV-LIO 体素框架：双 IMU 测量组、end-time 配对与 soft-wait、Odin 陀螺-only 去畸变并对齐 Mid 末端、Mid 更新可回滚、Odin Stage-2 子采样软更新可拒绝、Body 显示与 Global 加图分离且默认要求 `accept_s2` 才密化 Odin。算法血统上它是 FAST-LIO-MULTI 紧耦合思想的 PV-LIO 实现，不是官方 MULTI 源码的逐行移植，也刻意丢掉了官方“副雷达与主雷达一次性同质融合”里最容易在 Mid+Odin 上拧坏航向与高度的那部分假设。
