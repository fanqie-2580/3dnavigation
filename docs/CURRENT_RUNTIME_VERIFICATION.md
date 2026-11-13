# CURRENT_RUNTIME_VERIFICATION

- **时间**：2026-11-13 14:22–14:30（融合进程自 14:22:04 起；本报告只读）
- **主机**：Jetson Orin NX，ROS 2 Humble，`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- **约束**：未改代码 / 参数 / 系统时间；证据取自运行中进程、`/tmp/mid_odin_fusion.log`、话题抽样、源码与 GitHub `cc2ce811`
- **目标提交**：`cc2ce811de407d18aeee20601273c6fce3c67889` — *Query orange Mid pose via Bundle center relation, not end2+bias.*

---

## 总表（先读）

| 项 | 结论 |
|----|------|
| 二进制是否含 `cc2ce81` 的 `t_query` | **已生效**（编译 12:05，进程 14:22 加载；日志有 `expect≈`） |
| 左右转扇形拖影（视觉） | **本 Agent 未目视 RViz**；运动学上若仍跟 `center_dt` 错位，转角扇形可达约 **0.7–1.5°（中位）/ 最高 ~2–3°** |
| 问题首次出现层级 | **非 raw 几何复制链**；注册位姿时间语义 / Bundle `center_dt` 残差是主嫌疑；deskew 偏置混用为次级未定量项 |
| Mid `bg/ba` 用于 Odin native deskew | **源码确认混用**；合理性 **存疑**，缺独立 Odin 偏置实测 |
| `vs_mid_end≈expect` 能否证明物理正确 | **不能**；只证明代数恒等式；物理对齐取决于 `center_dt→0` |
| Stage1↔Stage2 位姿修正量 | **无法确认**（无运行时日志） |
| Odin 是否进图 | **部分可以**：`|center|≤25ms` 约半数帧才过写图时间门；Stage2 成功与否未打点 |

**已修复**：`t_query` 与 `center_dt` 解耦（`corr(|vs|,|center|)≈0`）；imu2≈38–46；`|ω|` 门控已关。  
**仍存在**：Bundle `|center_dt|` 中位 ~25ms（转角更大）；写图被 center 门砍半；Odin raw 头年龄 ~145ms（USB 语义）。  
**无法确认**：RViz 扇形是否仍可见；Stage2 Δpose；Odin 独立 bg/ba；`last_odin_iekf_ok` 真实比例。

---

## 1. 运行程序是否已部署 GitHub `cc2ce811`（含最新 `t_query`）

### 实测值

| 对象 | 值 |
|------|-----|
| 远端提交 | `cc2ce811`（2026-11-13 12:06:39 +0800） |
| 本机源码 `laserMapping_adaptive.cpp` mtime | 2026-11-13 **12:01:17** |
| 构建产物 `build/.../laserMapping_adaptive` | 2026-11-13 **12:05:26**（晚于源码、早于/贴近 push） |
| 运行 PID 8693 启动 | 2026-11-13 **14:22:04**，加载上述 build（symlink-install） |
| install/源码/upload 三方 `mid360_odin1.yaml` md5 | 相同 `e323a170…` |
| 本地源 vs upload 源 `laserMapping_adaptive.cpp` | **逐字节相同** |
| 运行日志特征串 | `[odin-pose-at-end] vs_mid_end=… expect≈…`（仅新公式会打 `expect≈`） |

启动参数回显（`/tmp/mid_odin_fusion.log` L20）：

```text
lidar2_update_en: true … stamp_bias=65ms odin_native_deskew=true odin_pose_at_end=true
imu2=/odin1/imu odin_cov_scale=100 … odin_map_insert=true insert_max_dt=55ms
still_dt=55ms@|w|<0.12 insert_gyr_gate=0rad/s cooldown=0s
```

### 日志证据

```text
[odin-pose-at-end] vs_mid_end=-1.142ms(中位) center=±十几～五十ms expect≈(与 vs 差 <0.0002ms)
corr(|vs_mid_end|, |center|) = 0.004
```

### 源码位置

- 公式：`FAST_LIO_MULTI_ROS2_mid_odin/src/laserMapping_adaptive.cpp` ≈L2597–2640  
  `t_odin_end_mid = mid_center + (odin_span - odin_med)`  
  `expect≈ (odin_span - odin_med - 0.5*mid_span)`
- 提交 diff：`cc2ce81` 仅改该文件相关 25 行 + 文档

### 判断结论

**已修复 / 已部署**：当前运行的 `laserMapping_adaptive` **已编译并加载**含 `cc2ce81` 的 `t_query` 逻辑；不是旧的 `end2+bias` 路径。

---

## 2. 最新一次主动左右转：扇形 / 滞后量化

### 实测值（融合日志全程，非目视）

| 指标 | 值 |
|------|-----|
| Bundle tight / Mid-only | 4199 / ~100 → **紧耦通过率 ≈ 97.6%** |
| `gyr_scan` | 中位 0.013，p90 0.11，**max 1.61 rad/s** |
| 转角帧 `gyr≥0.25` | **≈6–8%** 帧；识别到 **21** 段连续转角 episode |
| 转角时 `|center_dt|` | 中位约 **30–38 ms**（多段），最大约 **52 ms** |
| 若橙色位姿仍错 `center_dt`：扇形角 | 典型 episode **中位 0.7–1.5°，峰值 ~2.1–3.4°** |
| 若仅错 `t_query−Mid_end≈1.2ms`：扇形角 | **中位 ~0.03°，可忽略** |
| 旧 `±50ms` 量级错误 | 同转角下 **中位 ~1.4°，峰值 ~4.6°** |

近期 densify 曾报 `yaw≈7 deg/s`（有过主动转），但 **14:28 左右 10s 话题抽样时机器人近乎静止**（`|ω|_max≈0.023`，`Δyaw≈0.1°`）——**本轮 Agent 未捕捉到与日志高峰重合的实时转角点云几何**。

### 日志证据

```text
Bundle tight … center=-37.5ms … gyr_scan=0.675…   # 典型转角段
[odin-pose-at-end] vs_mid_end=-1.43ms center=-41.6ms expect≈-1.43ms  # 同段附近
Bundle Mid-only … gyr_scan=1.28571                 # 转中偶发丢配
```

### 源码位置

- Bundle 打印：`laserMapping_adaptive.cpp` ≈L1280–1290  
- 扇形角估算：`θ ≈ |ω| · |Δt|`（运动学上界，非点云拟合）

### 判断结论

| 分类 | 内容 |
|------|------|
| **已修复** | `t_query` 不再跟着 `center_dt` 漂 ±50ms；代数上 `vs≈expect≈−1…−4ms` |
| **仍存在** | 转角时 **`center_dt` 本身仍常达 30–40ms**；若“物理 Odin 端时刻”仍依赖中心配对，则 **橙色整帧位姿仍可错 ~ω·center** → **扇形/整体偏航量级 1° 上下仍可能** |
| **无法确认** | RViz 上橙色是否仍肉眼可见扇形（需人眼或离线点云扇形度量） |

---

## 3. 点云链路：raw → bridge → deskew → registered

### 实测值

| 话题 | Hz（抽样） | header age | 相对 raw 的 stamp |
|------|------------|------------|-------------------|
| `/odin1/cloud_raw` | ~6–10（不稳；`topic delay`≈**145ms**；Python 长订可到 260ms+） | USB/host 语义固有偏晚 | — |
| `/odin1/cloud_raw_ds` | ~11 | ≈ raw（`topic delay`≈146ms） | **stamp_dt med = 0** |
| `/odin1/cloud_l2_body` | ~8 | med≈152–180ms，max≈0.34s | **stamp_dt med = 0** |
| deskew 后（进程内 `feats_undistort` Odin 子集） | 无独立话题 | — | 见 §4 |
| `/cloud_odin_registered` | ~9.7 | med≈**95–100ms** | vs raw stamp med **+7ms**（**已改写为 Mid 时钟 `t_query`**） |
| `/cloud_registered`（Mid） | ~9.8 | med≈100ms | vs odin_reg stamp med **~1.6ms** |

Bridge（累计末行量级）：`pub≈cloud`，`drop_busy=0`，`skip_sync=7`；`l2_match=causal` **med≈52ms，p90≈90ms，max≈100ms**。  
Densify：`pub/cloud≈79%`，`skip_pose≈13%`，`skip_accum≈8%`（软可视化，非 IEKF 真值）。

### 日志证据

```text
cpp_bridge … drop_busy=0 … l2_match=causal:46–99ms
cpp_densify … skip_pose≈13% skip_accum≈8%
[odin-native deskew] imu2=39 poses=40 span=94.37ms pts≈57xx
```

### 源码位置

- Bridge / densify：launch 内 `odin_raw_downsample_bridge`、`odin_mid_densify`  
- Native deskew：`IMU_Processing.hpp` `UndistortOdinNative` ≈L355–531  
- 橙色发布：`publish_odin_from_undistorted` + `state_point_odin_pub_` ≈L1531+、L2687–2692

### 判断结论

| 层级 | 滞后/变形 | 判断 |
|------|-----------|------|
| raw | ~145ms header age；帧率抖动 | **驱动/时戳语义问题**，非融合 deskew；单帧几何此处未测变形 |
| ds / l2_body | **同 stamp**；墙钟处理 ~1ms；causal Mid 匹配 50–100ms | **未引入额外 stamp 漂移**；配对时延是 Mid 位姿选择问题 |
| native deskew | span 固定 ~94.4ms，imu2 充足 | **有执行**；变形取决于偏置/重力假设（§4） |
| `/cloud_odin_registered` | age 反低于 raw（因 stamp 换成 Mid/`t_query`） | **时间基已换**；扇形/整体偏航若仍在，**首次表现在“用 Mid 位姿把 deskew 后 Odin 投到 map”这一级**，不是 raw→ds 拷贝链 |

**仍存在**：raw 固有 ~150ms 晚戳 + Bundle 中心残差。  
**无法确认**：deskew 前后点云几何差（无 before/after 话题）。

---

## 4. `UndistortOdinNative`：Mid `bg/ba` 与加速度尺度

### 实测值

- 调用：`UndistortPclMultiLiDAR` → `UndistortOdinNative(pcl_2, meas, st.bg, st.ba)`（`st =` Mid KF 状态）
- `acc_scale = G_m_s2 / mean_acc.norm()`，而 **`mean_acc` 来自 Mid IMU 初始化**（同 `ImuProcess`）
- 积分：局部相对系，`grav≈0`（注释）；`angvel -= bg`，`acc = acc*acc_scale - ba`
- 运行：`imu2≈38–46`，`poses≈39–40`，`span≈94.37ms`（稳定，几乎无 fallback 日志）

### 日志证据

```text
[odin-native deskew] imu2=39 poses=40 span=94.3683ms pts=5780
```

（本次日志中几乎未见 `fallback→Mid-IMU path`。）

### 源码位置

- `IMU_Processing.hpp` L355–413（偏置与尺度）、L431（无重力相对积分）、L790–796（传入 `st.bg/st.ba`）
- Mid `mean_acc` 初始化：同文件 `IMU_init` ≈L176+

### 判断结论

| 分类 | 内容 |
|------|------|
| **已修复** | Odin 扫描内 deskew 走 `/odin1/imu` 时间轴（不再逐点映射 Mid IMU 时间） |
| **仍存在 / 存疑** | **直接使用 Mid 的 `bg/ba` 与 Mid `mean_acc` 尺度**；两 IMU 零偏与加速度标定不同则转角 deskew 残留 ∝ω、a |
| **无法确认** | 是否“需要”独立估 Odin 偏置/重力——缺 Odin 静止标定与 deskew 前后误差对比；逻辑上 **独立偏置更合理**，本轮无数据证明其是当前扇形主因 |

重力：相对短时积分不显式加 `g`；若 `ba` 错，表现为虚假线加速度，对纯转扇形次要、对猛烈加减速更敏感。

---

## 5. 物理时间戳 ↔ Bundle 中心 ↔ `t_query` ↔ Mid 位姿历史

### 实测值

| 量 | 含义 | 实测 |
|----|------|------|
| `signed_dt` / `dt` | `(L2_header+bias) − Mid_beg` | tight 窗内 `|dt|≤55ms`（日志无 >55） |
| `center_dt` | `(L2+bias+odin_med) − Mid_center` | **全时段 \|center\| med≈24.7ms，p90≈44.6ms，max≈54.6ms** |
| `vs_mid_end` | `t_query − Mid_end` | **med≈−1.14ms**（范围约 −4.6～+2.5） |
| `expect≈` | `span−med−mid_span/2` | 与 `vs` **\|差\|<0.0002ms**（恒等式） |
| `corr(\|vs\|,\|center\|)` | 解耦检验 | **0.004 ≈ 0** |
| Mid hist | `push_mid_pose_hist(lidar_end_time, state_stage1)`，最长 ~0.6s / 50 帧 | 日志 `hist=6～7` |
| `sample_mid_pose_at` | 在 Mid **end 离散样本**上 slerp/短外推 | `t_query≈Mid_end−1ms` → **几乎取当前 Stage1** |

### 日志证据

见 §1–2；转角段常同时出现 `center≈−30～−45ms` 与 `vs_mid_end≈−1.4ms`。

### 源码位置

- 配对与 `center_dt`：≈L1012–1037、L1086–1095（bias EMA）  
- `t_query`：≈L2610–2624  
- hist：≈L540–592、L2598

### 判断结论

**不能**因为 `vs_mid_end≈expect≈−1ms` 就认定物理正确。

正确说法：

1. **已证明**：实现的是“相对 Mid_end 的中心公式”，且与 `center_dt` **脱钩**。  
2. **未证明**：`Mid_center` 在物理上等于 Odin 扫描中心。残差就是日志里的 **`center_dt`（常 25ms，转角常 30–40ms）**。  
3. 若 `center_dt=ε`，则 `t_query` 相对真 Odin-end 的误差 **仍约是 ε**，只是不再错误地再叠一层 `end2+bias` 漂移。  
4. Mid hist 只有 **扫描 end 的 Stage1**，对 −1ms 查询几乎不提供“扫尾姿态”分辨率；真正敏感的是 **配对中心是否准**。

| 分类 | |
|------|--|
| **已修复** | 查询时刻不再跟踪 EMA/`center` 锯齿到 ±50ms |
| **仍存在** | **`\|center_dt\|` 系统性偏大**（中位~25ms）→ 物理端时刻仍可能偏 |
| **无法确认** | bias EMA 稳态值、Odin header 与真实采集时刻的固定偏置是否已在 0.065 窗内吃干净 |

---

## 6. Stage1 vs Stage2 位姿修正量

### 实测值

**无。** 进程不打印 Stage1→Stage2 的 ΔR/Δt；计数器 `n_odin_iekf_updates` 仅内存累加、不落盘。

设计行为（源码）：

- Stage1：`residual_select=1`，只 Mid 点  
- Stage2：`residual_select=2`，Odin 点，`R *= lidar2_residual_cov_scale`（**100**）+ 时间加权  
- `/Odometry` 与 Mid `/cloud_registered`：用 **Stage2 后** `state_point`  
- `/cloud_odin_registered`：用 **`state_point_odin_pub_` = Stage1 hist @ `t_query`**（刻意不让 Stage2 拧橙色）

### 源码位置

≈L2566–2692；注释 L167、L2572–2573。

### 判断结论

| 分类 | |
|------|--|
| **已修复（设计）** | 橙色与 Stage2 解耦，避免 Stage2 把橙云拧偏 |
| **仍存在（设计后果）** | Mid 注册云与橙云 **允许** 不一致，差分为 Stage2 修正 + `t_query` 位姿差 |
| **无法确认** | 实际 Δyaw/Δt 大小（需加一次性日志或录包对比同帧两状态） |

弱观测（`cov_scale=100`）暗示 Stage2 修正应偏小，但 **本轮无实测**。

---

## 7. 门控通过率 / Odin 有效观测 / 地图插入

### 实测值

| 门控 | 配置 | 运行表现 |
|------|------|----------|
| `lidar2_pose_gyr_gate` 等 | **全 0** | `odin_pose_update_allowed()` 不因 ω 拒绝 |
| Bundle 紧耦 | `max_sync_dt=55ms` + soft_wait 25ms | **≈97.6%** tight；**≈2.4%** Mid-only（含转中丢配） |
| Stage2 尝试条件 | `point_update && bundle && pose_allowed && cov>0 && pair_dt≥0` | 配置上应对几乎所有 tight 帧尝试 |
| Stage2 成功 `last_odin_iekf_ok` | `effect_feat_num>0` | **未打日志 → 比例未知** |
| 写图 `lidar2_map_insert_*` | `en=true`，`max_center_dt=0.025`，`gyr_gate=0` | **`\|center\|≤25ms`：全时段 ≈50.4%；转角 `gyr≥0.25` 时 ≈44.2%** |
| `/Laser_map` | 有发布 | 抽样 ~1.5MB/消息，说明 ikdtree 在长；**无法从带宽拆 Mid/Odin 点比** |

Densify 非地图真值：有效 pub≈79%。

### 日志 / 源码证据

- 启动行：`insert_gyr_gate=0`，`odin_map_insert=true`  
- 写图条件：`map_incremental` ≈L1382–1395：`block_odin_insert` 含 `!last_odin_iekf_ok` 与 `sync_too_loose_for_map`（center/dt）

### 判断结论

| 分类 | |
|------|--|
| **已修复** | `|ω|` 门控关闭 → 不是“转角完全不紧耦” |
| **仍存在** | **约一半 Bundle 帧因 `|center_dt|>25ms` 禁止 Odin 写 ikdtree**；转角更差 → 转中 Odin **常不进图**（这是时间门，不是旧 ω 门） |
| **无法确认** | Stage2 有效观测比例、实际插入的 Odin 点数占比 |

---

## 分类汇总

### 已经修复

1. `cc2ce81` 的 `t_query=Mid_center+(span−med)` 已编译进当前 PID 8693，并有 `expect≈` 运行证据。  
2. `t_query` 与 Bundle `center_dt` 数值解耦（相关≈0）。  
3. `/odin1/imu` Reliable 深队列下 `imu2≈39`，native deskew 稳定。  
4. `|ω|` 姿态/写图硬门关闭；紧耦帧率 ≈97.6%。  
5. raw→ds→l2 **同 stamp**，bridge 无积压性 `drop_busy`。

### 目前仍然存在

1. **`|center_dt|` 中位 ~25ms、转角常 30–40ms** → 物理中心对齐未完成；扇形运动学上界仍约 **1° 量级**。  
2. **写图 center 门** 使 ~50% 帧（转角更高）Odin 不能进 `/Laser_map`。  
3. Odin raw **~145ms** header 固有晚（`use_host_ros_time` 类语义），与 Mid 配对依赖 bias。  
4. Native deskew **混用 Mid `bg/ba` + Mid `mean_acc` 尺度**。  
5. 橙云用 Stage1@`t_query`、Mid 云/里程计用 Stage2 后状态 → 显示可不一致（设计如此）。

### 无法确认

1. **人眼/几何意义下扇形是否仍明显**（本轮无 RViz 定量、转角与采数未对齐）。  
2. Stage1→Stage2 实际 Δpose。  
3. `last_odin_iekf_ok` / Odin 残差有效点数比例。  
4. 独立 Odin IMU 偏置是否为当前残差主因。  
5. deskew 前/后同帧点云形变量。

---

## 唯一优先下一步实验

**固定一次左右转（峰值 `|ω|≥0.5`），同时录：**

1. `/tmp/mid_odin_fusion.log` 中该秒级窗口的 `center=` 与 `vs_mid_end=`；  
2. 同窗 `/cloud_odin_registered` 与 `/cloud_registered`（或 RViz 截图）；  

**判定标准（只做这一件事）：**  
若转角扇形角 ≈ `|ω|·|center_dt|`（约 1°）而非 ≈ `|ω|·1ms`（≪0.1°），则 **下一主因锁定为 Bundle 中心对齐/`stamp_bias`，而不是 `t_query` 公式本身**；若扇形 ≪ `ω·center` 仍接近 0，则改查 deskew 偏置或 Stage1/2 显示差。

---

## 附录：进程快照（核查时）

```text
host_sdk_sample (Odin USB)
livox_ros_driver2_node (Mid360)
laserMapping_adaptive  pid 8693  ← build 12:05:26
odin_raw_downsample_bridge
odin_mid_densify
rviz2  mid360_odin1_multi.rviz
```

主日志：`/tmp/mid_odin_fusion.log`  
GitHub：https://github.com/fanqie-2580/3dnavigation/commit/cc2ce811de407d18aeee20601273c6fce3c67889
