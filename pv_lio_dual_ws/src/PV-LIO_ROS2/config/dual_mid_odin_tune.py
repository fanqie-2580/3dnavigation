# Dual Mid360 + Odin1 — Mid-primary (match mid360_go2 residual_z), Odin densify-only.
#
# Lesson from single Mid360 multifloor run:
#   - residual_z H_z damp + NO flat pin → stairs OK, map clean
#   - odin_z_replace / pose_hold / att → Z/XY desync, shear, NEP
# Fusion policy:
#   - Pose/LIO = Mid only (same residual_z as mid360_go2)
#   - Odin points → Global densify under Mid pose (no IEKF / no pose hijack)

TUNE = {
    # --- Odin cloud quality ---
    "odin_min_pts": 2000,
    "odin_min_frac": 0.20,
    "odin_body_z_min": -3.0,
    "odin_body_z_max": 4.0,
    "odin_static_gyro": 0.08,

    # --- Mid-only IEKF (do NOT joint-update with Odin on stairs) ---
    "odin_s2_enable": False,
    "odin_joint_only_en": False,
    "odin_iekf_max_pts": 2000,
    "odin_iekf_max_end_dt": 0.25,
    "odin_info_weight": 0.15,
    "odin_range_ref": 8.0,
    "odin_max_residual": 0.05,
    "odin_s2_gyr_gate": 0.12,
    "odin_s2_max_clip_ratio": 0.4,
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

    # --- Global: Mid + Odin densify under Mid pose ---
    "map_jump_upd": 10.0,
    "map_jump_frame": 10.0,
    "map_max_range": 30.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": True,
    "odin_global_insert_en": True,
    "odin_global_require_s2": False,
    "odin_body_pub_en": True,

    # --- go2 residual_z (identical spirit to mid360_go2_tune) ---
    "residual_z_en": True,
    "residual_z_h_dampen": 0.12,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_dz_tol": 0.06,
    "residual_z_floor_ratio_thr": 0.03,
    "residual_z_floor_info_scale": 0.45,
    "residual_z_flat_z_alpha": 0.0,
    "residual_z_flat_vz_alpha": 0.75,
    "residual_z_flat_ba_z_alpha": 0.06,
    "residual_z_flat_max_step": 0.0,
    "residual_z_hard_band": 0.0,

    # --- Never hijack Mid pose with Odin odom ---
    "odin_z_factor_en": False,
    "odin_z_replace_en": False,
    "odin_xyz_replace_en": False,
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
