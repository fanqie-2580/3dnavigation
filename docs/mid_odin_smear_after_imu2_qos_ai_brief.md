# Mid+Odin 扇形仍在：IMU2 QoS 补满之后再分析（给另一 AI）

> 日期：2026-11-13（第三轮）  
> 用途：`sub_imu2_` 已改为 **RELIABLE KeepLast(4000)**，native `imu2` 已从中位 ~25 升到 ~39，**用户确认转动时 Odin 仍有扇形拖影**。请重新找根因；上一刀「样本饥饿」已基本打中计数，但**不足以解释残留扇形**。  
> 工作区：`/home/ll/FAST_LIO_MULTI_ROS2_mid_odin`  
> 诊断：`docs/lidar_fusion_diagnosis.md`（§10u–10w）  
> 前两轮 brief：`mid_odin_timestamp_smear_ai_brief.md`、`mid_odin_smear_after_pose_at_end_ai_brief.md`

---

## 0. 请你先做的事

1. **先结论**：QoS 补满后残留扇形更像什么（native 积分模型 / A 时钟 / t_query·bias / Mid 插值 / 外参 / 观感伪影）。  
2. 只读核对 `UndistortOdinNative` + pose-at-end 的时间与运动模型是否自洽。  
3. **未确认前不要大改**；给唯一最小下一刀 + 可证伪指标。  
4. 禁止：|ω| 门 / cooldown / 冻 Mid；禁止再盲目加大 `stamp_bias`；不要重复「再加深 KeepLast」除非证明仍在丢样。

重点文件：

```text
src/IMU_Processing.hpp::UndistortOdinNative          (~355+)
src/laserMapping_adaptive.cpp                        # imu2 sync / sample_mid_pose_at / publish
src/odin_raw_downsample_bridge.cpp
config/mid360_odin1.yaml
```

---

## 1. 已落地且本轮仍在跑的三刀

| 刀 | 状态 | 现场证据 |
|----|------|----------|
| §10u Odin-native `/odin1/imu` deskew | 开 | `[odin-native deskew] … span=94.3683ms`；无 fallback / center-lock |
| §10v 橙云挂 `T_map(t_odin_end_mid)` | 开 | `[odin-pose-at-end] vs_mid_end≈center`（\|Δ\| med≈1.5 ms） |
| §10w `sub_imu2_` RELIABLE KeepLast(4000) | **已上** | 启动日志 `QoS=RELIABLE,KeepLast(4000)`；`ros2 topic info` 订阅端 Reliability=RELIABLE |

用户现象：**转动时扇形拖影仍在**（相对 Mid `/cloud_registered`）。默认 Decay=0、单帧橙云（此前已确认过）。

---

## 2. QoS 前后对比（关键负证据）

| 量 | QoS 前（上一轮 AI） | QoS 后（本轮 `/tmp/mid_odin_fusion.log`） |
|----|---------------------|------------------------------------------|
| `imu2` 中位 | ~25（~55% 理论） | **39**（min 38 / p95 46 / max 46），n=23 |
| `poses` 中位 | — | **40** |
| 理论窗 | ~45（94.4 ms + ~20 ms 前置） | 已接近；**不再像严重饥饿** |
| `gyr_scan` | 近静 | 本轮 **p95=0.455，max=1.45** → 确有主动转 |
| Mid-only | — | 24 次（相对 Bundle 日志量不高） |
| fallback / center-lock | — | **0** |

结论：若扇形仍在，**主因不应再写「KeepLast(5) 丢样」**。要么还剩小缺口（39 vs 45），要么根因已转到 **积分模型 / 时间映射 / 外参 / A**。

---

## 3. 请优先审查的剩余假设（排序请你重排）

### H1. `UndistortOdinNative` 运动模型瑕疵（高嫌疑）

源码要点（`IMU_Processing.hpp`）：

- 用 **Mid IEKF 的 `bg/ba`** 减 Odin IMU（`UndistortPclMultiLiDAR` 调用 `UndistortOdinNative(..., st.bg, st.ba)`）。  
- `acc_scale` 用 **Mid `mean_acc`**。  
- 相对系从 Identity@beg 积分；短窗内重力近似为 0。  
- 补偿到 **Odin scan end**，再刚体 `T_L2_wrt_L1`。

问题：Odin IMU 与 Mid IMU **bias/轴系/刻度**不同时，弯道上系统性帧内弯；QoS 补满后只会「更稳定地用错模型」。

### H2. A：Odin cloud header 与 `/odin1/imu` 仍有固定/慢漂偏置

驱动同用 `make_aligned_stamp()`，但 capture 起点未必重合。  
本轮已有高 `gyr_scan`，可用主动转做互相关 / 「平面残差 vs offset_time」。

### H3. C 残留：`t_query = end2 + bias` 的 bias 非物理时钟

上一轮已指出：`vs_mid_end≈center` 只证明实现按设计跑；bias EMA 来自离散中心配对且曾逼近 109–120 ms。  
整帧刚体错位表现为 δψ∝ω·Δt；**帧内扇形**更需 H1/H2。请用指标拆开。

### H4. Mid 位姿历史 ~10 Hz + 短外推

`vs_mid_end` 可正可负（本轮 −35…+42 ms）。正值走外推。可造成整帧抖/偏，通常不是「扇骨」状帧内弯。

### H5. Mid-only hold

有，但次数有限；hold 是旧整帧重发，不像扫描纹理扇形。

### H6. offset 反号 / 双重 deskew / densify / Decay

前几轮基本排除；除非本轮有新证据，不要回头当主因。

### H7. 外参 / Odin 稀疏 32 切片纹理

静态基线；高 ω 下扇形若随 ω 明显变厚，不能只用纹理解释。

---

## 4. 当前时间链（简图）

```text
/odin1/cloud_l2_body: header=USB/host, offset∈[0,94.368]ms
/odin1/imu ~400Hz RELIABLE → sub_imu2_ RELIABLE/4000 → meas.imu2
UndistortOdinNative → Odin_end body → T_L2_wrt_L1
橙云: T_map(end2+bias) from Mid Stage1 hist; header=t_query
```

---

## 5. 必须逐条回答

1. **QoS 后扇形仍在，主因排序是什么？** 每条一句证据。  
2. **imu2≈39 是否已够？** 若仍差 6 个，缺口能否单独解释 1 rad/s 级扇形？给出数量级。  
3. **如何一刀分清 H1 vs H2 vs H3？**  
   - 残差 vs `offset_time`（帧内）  
   - δψ vs `center_dt·ωz`（整帧）  
   - 关 native deskew（仅刚体）对照  
4. **唯一最小下一刀**（只动一条链）：例如  
   - native 改用 Odin 自身静止估 bg，或纯 gyro 相对积分不用 Mid ba；  
   - 或冻结 bias EMA，改固定双 IMU 相关 `b_clock`；  
   - 或离线正号/反号/无 deskew 三档比厚度斜率。  
5. **改了也消不掉的情况**列清。

---

## 6. 建议立刻可做的对比实验（用户可转）

```bash
ros2 bag record -o /tmp/odin_smear_qos_full \
  /livox/imu /odin1/imu \
  /odin1/cloud_l2_body /Odometry \
  /cloud_registered /cloud_odin_registered
```

主动左右转 30–60 s。离线至少出：

| 指标 | 含义 |
|------|------|
| 同墙残差对 offset_time 的斜率（按 \|ω\| 分层） | 帧内弯 |
| Odin↔Mid ICP δψ 对 center_dt·ωz 斜率 | 整帧时刻 |
| 同墙 p90 对 \|ωz\| 斜率 | 总拖影强度 |
| 每帧 imu2 数、最大 IMU 间隔 | 是否仍有空洞 |

---

## 7. 一句话现状

> QoS 已把 native `imu2` 从 ~25 拉到 ~39，转弯 `gyr_scan` 可达 >1 rad/s，但用户仍见 Odin 扇形。请在「样本饥饿」基本排除后，找出真正主因，并给出可证伪的最小下一刀。
