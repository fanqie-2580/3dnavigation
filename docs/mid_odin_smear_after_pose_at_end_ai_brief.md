# Mid+Odin 转弯仍拖影：pose-at-end 之后再分析（给另一 AI）

> 日期：2026-11-13（第二轮）  
> 用途：现场在 **Odin-native deskew + 橙云挂 Odin-end Mid 位姿** 都已上线后，**转动时 Odin 仍有拖影**。请重新判型：C 是否其实没打中 / 打偏，还是剩余主因已变成 A/B 或其他。  
> 工作区：`/home/ll/FAST_LIO_MULTI_ROS2_mid_odin`  
> 上一轮 brief：`docs/mid_odin_timestamp_smear_ai_brief.md`  
> 诊断长文：`docs/lidar_fusion_diagnosis.md`（§10u–10v）

---

## 0. 请你先做的事

1. **先结论**：当前拖影更像帧内弯曲（A/B）还是整帧仍错位（C 未真正修好 / 新变种），还是观感伪影。  
2. 只读核对下面「已改代码的时间语义」是否自洽；标出仍可能错的符号/时钟域。  
3. **未确认前不要大改**；给出唯一最小下一刀 + 可证伪实验。  
4. 禁止：|ω| 门 / cooldown / 冻 Mid 作主方案；禁止再盲目加大 `stamp_bias`。

重点文件：

```text
src/IMU_Processing.hpp                 # UndistortOdinNative
src/laserMapping_adaptive.cpp          # sample_mid_pose_at / publish_odin_* / Bundle
src/odin_raw_downsample_bridge.cpp
config/mid360_odin1.yaml
```

---

## 1. 现场现象（本轮）

- 用户：转动时 Odin1 **仍有拖影**（相对 Mid）。  
- RViz：`/cloud_odin_registered` 仍由 `laserMapping_multi` 发；**2 个 RViz 订阅**（两份 Odin display 叠色，已知不产生不同时间位置）。  
- 请默认 Decay=0、单帧对象（上轮已确认过；若需可再让用户截图确认）。

---

## 2. 已落地且日志证明在跑的两刀

| 刀 | 参数 | 意图 | 现场证据 |
|----|------|------|----------|
| §10u native deskew | `odin_native_deskew_en: true` | Odin 帧内用 `/odin1/imu`，不再逐点映到 Mid IMU | `[odin-native deskew] imu2=… poses=… span=94.3683ms`；`extend=0ms`；**无** `center-lock` / `fallback` 计数 |
| §10v pose-at-end | `odin_pose_at_scan_end_en: true` | 橙云不用 Stage1@Mid_end，改用 Mid 轨迹在 `t_odin_end_mid` 的插值 | `[odin-pose-at-end] vs_mid_end≈center`（例 −10.3 ms / −9.7 ms；−48.1 / −48.6） |

上一轮结论曾是：**C/F 整帧挂错 Mid_end 是确定结构缺口**；最小刀就是 10v。  
**现在 10v 已上，拖影仍在 → 必须重排根因。**

---

## 3. 当前橙云时间语义（请验算对不对）

```text
Odin native:
  点时间 = L2_header + offset_time
  补偿到 Odin_scan_end = L2_header + max(offset) ≈ header + 94.368 ms
  然后 T_L2_wrt_L1 → Mid L1 系（仍是「Odin_end 时刻的刚体云」）

橙云发布（10v）:
  t_odin_end_mid = lidar_end_time2 + lidar2_stamp_bias_sec
                 = (L2_header + span) + bias
  从 Mid Stage1 历史（每帧一个 @ Mid_end）slerp/短外推得到 T_map_mid(t)
  P_world = T_map_mid(t) · P_odin_body
  header.stamp = t_odin_end_mid
```

### 3.1 请重点质疑的点

**Q1. `t = end2 + bias` 是否等于「Odin_end 在 Mid 时钟上的真时刻」？**  
bias EMA 是为 **scan-center** 配对训的，不是为 end 训的。若 center 对齐正确，end 应对齐到 `mid_center + (span - odin_med)`，等价于 `end2+bias` 仅当 med 定义一致。日志里 `vs_mid_end ≈ center` 很紧——这符合「查询时刻相对 Mid_end 的偏移 ≈ center_dt」的推导，也说明 **实现在按设计跑**；但若设计本身用错了「该挂的时刻」，拖影会原样保留。

**Q2. Mid 位姿历史只有 ~10 Hz 的 Mid_end 样本**  
查询点常在相邻 Mid_end 之间或之外 ±50 ms。短外推用相邻相对旋转估 ω。转弯急时插值/外推误差是否仍够造成「拖影观感」？

**Q3. C 修的是整帧刚体；若拖影是帧内扇形，10v 本来就不该消掉**  
需要用「残差 vs offset_time」区分整帧平移 vs 帧内弧。

**Q4. Mid-only hold**  
日志仍有 `Bundle Mid-only (hold Odin viz)`：会重发上一帧橙云地图快照。快转中若频繁 Mid-only，观感像拖影/粘滞——请判断是否主因。

**Q5. Native deskew 实现瑕疵**  
`UndistortOdinNative` 用 Mid 的 `bg/ba` 与 `mean_acc` 尺度；相对系积分从 Identity@beg；重力在短窗近似为 0。是否在高 yaw-rate 下系统性弯云？

**Q6. A：Odin cloud header 与 `/odin1/imu` 仍可能不同源**  
上一轮无法在静止段证实；**本轮用户已在转**，应用主动转 bag 做互相关。

---

## 4. 现场日志摘要（`/tmp/mid_odin_fusion.log`）

```text
odin_native_deskew=true odin_pose_at_end=true imu2=/odin1/imu
[odin-pose-at-end] vs_mid_end≈center（抽样 ±1～50 ms，二者接近）
[odin-native deskew] span=94.3683ms，imu2 约 10–46
Bundle center 仍大幅扫动并符号翻转（例 -50 ms → +45 ms）
偶发 Bundle Mid-only (hold Odin viz)
无 center-lock / fallback 日志（native 主路径）
```

启动命令（仍在跑）：

```text
laserMapping_adaptive … mid360_odin1.yaml
```

---

## 5. 必须逐条回答

### 1) 10v 之后拖影还在，最可能是什么？

给出排序（例：A ≫ 错误的 t 映射 ≫ Mid 插值 ≫ Mid-only hold ≫ B ≫ 外参），每条一句证据。

### 2) 10v 是否「语义正确但不够」还是「语义仍错」？

- 若语义正确：预测 δψ 对 `center_dt·ωz` 的斜率应≈0；请给出离线验证公式。  
- 若语义仍错：指出正确的 `t_query` 公式（用 mid_center / odin_med / span / bias 写清）。

### 3) 如何一刀分清「整帧」vs「帧内」？

主动左右转录 bag（至少 30–60 s）：

```bash
ros2 bag record -o /tmp/odin_smear_turn2 \
  /livox/imu /odin1/imu \
  /odin1/cloud_l2_body /Odometry \
  /cloud_registered /cloud_odin_registered
```

指标：

| 指标 | 指向 |
|------|------|
| 同墙平面残差对 `offset_time` 呈线性/弧形 | 帧内 A/B |
| 残差对 offset 平直，但整帧 δψ ∝ ωz 或 ∝ center_dt·ωz | 整帧时刻/插值仍错 |
| δψ 与 center_dt·ωz 解耦，但厚度仍随 \|ω\| 涨 | native deskew 模型/外参/双重补偿 |
| Mid-only 占比高且 hold 云与 live 交错 | hold 观感 |

### 4) 唯一最小下一刀是什么？

只允许动一条语义链（例如：改 `t_query`、或修 cloud–IMU 对齐、或关 hold 改为空、或离线正反号实验）。说明预期哪条指标先变。

### 5) 哪些情况再怎么改 stamp 也消不掉？

（外参力臂、Odin 稀疏纹理、双重 deskew、坏点/置信度等）

---

## 6. 约束

- Mid 始终前进；不要等齐。  
- 不要叠 `align_l2_stamp_to_mid=true` 与非零 bias，除非先证明不会钉死 Bundle。  
- 橙云与写图可以分开讨论：先修可视化几何，再谈 Stage2/ikdtree。

---

## 7. 一句话现状

> Native 帧内 deskew 与「挂 Odin-end Mid 位姿」都已按设计上线（`vs_mid_end≈center`），但用户转动时仍见 Odin 拖影。请判定是 **C 的时刻公式仍错**、**帧内 A/B**、**插值/hold 伪影**，还是其它，并给出可证伪的最小下一刀。
