# Dual Mid360 + Odin1 PV-LIO runtime knobs (edit → restart launch, no rebuild).
# Loaded by mapping_dual_mid_odin.launch.py → ROS params under dual_tune.*.
#
# Multifloor/stairs: Odin Z only. Att replace OFF — Euler RP mixed pitch~50°
# and full att overwrote yaw → both caused visible drift.

TUNE = {
    # --- Odin cloud quality (mild) ---
    "odin_min_pts": 2000,
    "odin_min_frac": 0.20,
    "odin_body_z_min": -3.0,
    "odin_body_z_max": 4.0,
    "odin_static_gyro": 0.08,

    # --- Official-like joint Bundle (skip Mid-only S1 when Odin ready) ---
    "odin_s2_enable": True,
    "odin_joint_only_en": True,
    "odin_iekf_max_pts": 2000,
    "odin_iekf_max_end_dt": 0.25,
    "odin_info_weight": 1.0,
    "odin_range_ref": 8.0,
    "odin_max_residual": 0.50,
    "odin_s2_gyr_gate": 5.0,
    "odin_s2_max_clip_ratio": 1.0,
    "shallow_s1_en": True,
    "shallow_s1_end_dt": 0.12,
    "shallow_s1_gyr": 0.20,
    "shallow_s1_iters": 2,

    # --- Odin into matching voxel map (official co-map) ---
    "odin_wall_nz_thr": 0.40,
    "odin_wall_info_boost": 1.0,
    "odin_floor_info_scale": 1.0,
    "odin_voxel_insert_en": True,
    "odin_voxel_require_still": False,
    "odin_voxel_max_clip": 1.0,
    "odin_voxel_max_pts": 400,
    "odin_voxel_body_z_min": -0.05,
    "odin_voxel_cov_scale": 2.0,

    # --- Jump / vel / freeze ---
    # Stairs: Mid NEP → IMU coast; freeze=100 allowed ~10s of ~50m/s blowup.
    # Hold Odin XYZ on bad (see odin_pose_hold_on_bad); freeze is fallback only.
    "iekf_max_upd": 10.0,
    "iekf_max_rot": 10.0,
    "iekf_max_vel": 0.0,
    "iekf_s2_max_upd": 10.0,
    "iekf_s2_max_rot": 10.0,
    "consec_bad_freeze": 5,

    # --- Global display map ---
    "map_jump_upd": 10.0,
    "map_jump_frame": 10.0,
    "map_max_range": 30.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": False,
    "odin_global_insert_en": True,
    "odin_global_require_s2": False,
    "odin_body_pub_en": True,

    # --- Multifloor: Odin odom Z + attitude (init-aligned). ---
    "residual_z_en": False,
    "residual_z_flat_z_alpha": 0.0,
    "residual_z_flat_vz_alpha": 0.0,
    "residual_z_flat_ba_z_alpha": 0.0,
    "residual_z_flat_max_step": 0.0,
    "residual_z_hard_band": 0.0,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_floor_info_scale": 1.0,
    "residual_z_h_dampen": 1.0,
    "residual_z_dz_tol": 0.05,
    "residual_z_floor_ratio_thr": 0.03,
    "odin_z_factor_en": False,
    "odin_z_replace_en": True,
    # Bad Mid match (stairs): hold fusion XYZ from Odin Δpose.
    "odin_pose_hold_on_bad": True,
    # flat_hold OFF: re-anchoring made fusion sink meters while Odin stayed high.
    "odin_z_flat_hold_en": False,
    "odin_z_flat_vz_thr": 0.03,
    "odin_att_replace_en": False,
    "odin_att_rp_only": True,
    "odin_odom_topic": "/odin1/odometry",
    "odin_z_factor_alpha": 0.0,
    "odin_z_factor_max_dt": 0.30,
    "odin_z_factor_max_innov": 0.0,
    "odin_z_factor_band": 0.0,
    "odin_z_factor_catch_err": 0.0,
    "odin_z_factor_catch_frac": 1.0,
    "odin_z_factor_catch_max": 0.0,
    "odin_z_factor_catch_alpha": 0.0,
    "odin_z_factor_catch_min_feat": 30,
    "odin_z_factor_one_sided": False,
    "odin_z_factor_floor_scale": 1.0,
}


def as_node_params():
    """Flatten for launch_ros Node(parameters=[..., as_node_params()])."""
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
