# 2026-11-06 写入融合的 CAD / 官方外参

## 输入

1. 手工/CAD：`T_base_odinIMU`（Odin IMU 相对 Go2 `base_link`）

```text
R = I
t = [0.313653617, -0.028973900, 0.104417573] m
```

2. 官方 Odin：`T_odinIMU_L` t=`[-0.02663, 0.03447, 0.02174]` R=I  
3. 宇树官方 Mid：在 Go2 机身 IMU 下 t=`[0.187, 0, 0.0803]`、`Ry(+13°)`  
4. URDF：`T_base_go2IMU` t=`[-0.02557, 0, 0.04232]` R=I  
5. Mid 板载：`T_midIMU_midL` 保持 `[-0.011, -0.02329, 0.04412]` R=I

## 写入系统

```text
T_L1_L2 = inv(T_base_midL) @ T_base_odinIMU @ T_odinIMU_L
t ≈ [0.12158, 0.00550, 0.03170] m
R = Ry(-13°)
```

- `use_yaw_only_body: false`（L2 已含相对 Mid 安装俯仰补偿）  
- `body→base_link`：t≈`[-0.1407, -0.0233, -0.1117]`，pitch≈`-13°`

文件：`mid360_odin1.yaml`、`mid360_odin1_multi.launch.py`
