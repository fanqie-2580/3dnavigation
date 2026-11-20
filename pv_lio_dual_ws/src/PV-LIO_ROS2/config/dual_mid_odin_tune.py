# Dual Mid360 + Odin1 — tight couple (Mid S1 → joint Mid+Odin S2).
#
# Safety from dual_bal / dual_go2mid lessons:
#   - soft_wait=0 (yaml): never block Mid waiting for Odin
#   - Mid filter=2 / voxel=0.25: dual-safe CPU (filter=1 diverged)
#   - residual_z multilevel: stairs + floor re-lock
#   - NO odin_z/xy_factor/replace (pose hijack shears map)
# Fusion policy:
#   - S1: Mid IEKF lock; S2: joint Mid+Odin when time+gyr gates pass
#   - Reject S2 on jump/clip; moving hard → densify-only that frame
#   - Odin also Global densify under Mid pose

TUNE = {
    # --- Odin cloud quality ---
    "odin_min_pts": 2000,
    "odin_min_frac": 0.20,
    "odin_body_z_min": -3.0,
    "odin_body_z_max": 4.0,
    "odin_static_gyro": 0.08,

    # --- Tight couple: Mid S1 then joint S2 (not joint-only) ---
    "odin_s2_enable": True,
    "odin_joint_only_en": False,
    "odin_iekf_max_pts": 1200,
    "odin_iekf_max_end_dt": 0.20,
    "odin_info_weight": 0.12,
    "odin_range_ref": 8.0,
    "odin_max_residual": 0.05,
    # Allow mild walk; hard turn/stairs still densify-only (gyr gate).
    "odin_s2_gyr_gate": 0.22,
    "odin_s2_max_clip_ratio": 0.35,
    "shallow_s1_en": True,
    "shallow_s1_end_dt": 0.05,
    "shallow_s1_gyr": 0.08,
    "shallow_s1_iters": 1,

    # --- Do NOT insert Odin into matching voxels (keeps Mid planes clean) ---
    "odin_wall_nz_thr": 0.40,
    "odin_wall_info_boost": 1.8,
    "odin_floor_info_scale": 0.35,
    "odin_voxel_insert_en": False,
    "odin_voxel_require_still": True,
    "odin_voxel_max_clip": 0.4,
    "odin_voxel_max_pts": 400,
    "odin_voxel_body_z_min": -0.05,
    "odin_voxel_cov_scale": 2.0,

    # --- Jump / freeze (same as mid360_go2) ---
    "iekf_max_upd": 10.0,
    "iekf_max_rot": 10.0,
    "iekf_max_vel": 0.0,
    "iekf_s2_max_upd": 10.0,
    "iekf_s2_max_rot": 10.0,
    "consec_bad_freeze": 5,

    # --- Global densify (paired Odin); S2 reject must not block map fill ---
    "map_jump_upd": 10.0,
    "map_jump_frame": 10.0,
    "map_max_range": 30.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": True,
    "odin_global_insert_en": True,
    "odin_global_require_s2": False,
    "odin_body_pub_en": False,

    # --- go2 residual_z_v6.3 (multilevel → full H_z locks each floor) ---
    "residual_z_en": True,
    "residual_z_h_dampen": 0.12,          # single-level only; multilevel forces 1.0
    "residual_z_multilevel_thr": 1.5,
    "residual_z_vz_alpha": 0.75,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_dz_tol": 0.06,
    "residual_z_floor_ratio_thr": 0.03,
    "residual_z_floor_info_scale": 0.45,
    # single-level corridor pin (disabled once multilevel); keep vz/ba soft
    "residual_z_flat_z_alpha": 0.12,
    "residual_z_flat_vz_alpha": 1.0,
    "residual_z_flat_ba_z_alpha": 0.06,
    "residual_z_flat_max_step": 0.0,
    "residual_z_hard_band": 0.0,

    # --- Never hijack Mid pose with Odin odom ---
    # odin_xy_factor OFF: Mid/Odin frames differ in yaw after gravity-align;
    # raw Δxy pull sheared the map (z→-38). Return shrink was from coarse Mid
    # params (voxel 0.5); aligning to mid360_go2 matches Odin path length.
    "odin_z_factor_en": False,
    "odin_z_replace_en": False,
    "odin_xyz_replace_en": False,
    "odin_xy_factor_en": False,
    "odin_xy_factor_alpha": 0.30,
    "odin_xy_factor_max_step": 0.05,
    "odin_pose_hold_on_bad": False,
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
