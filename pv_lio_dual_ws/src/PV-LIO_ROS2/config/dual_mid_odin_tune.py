# Dual Mid360 + Odin1 — push toward official MULTI Bundle.
# Joint equal-weight IEKF + Odin in matching voxels.
# Sole intentional non-official piece: pose Z := /odin1/odometry (corridor sink).

TUNE = {
    # Mild quality only (official has none)
    "odin_min_pts": 800,
    "odin_min_frac": 0.10,
    "odin_body_z_min": -4.0,
    "odin_body_z_max": 5.0,
    "odin_static_gyro": 0.15,

    # Official Bundle: one joint update, equal weight, almost always enter IEKF
    "odin_s2_enable": True,
    "odin_joint_only_en": True,
    "odin_equal_weight_en": True,
    "odin_iekf_max_pts": 4000,
    "odin_iekf_max_end_dt": 0.40,
    "odin_info_weight": 1.0,
    "odin_range_ref": 8.0,
    "odin_max_residual": 10.0,
    "odin_s2_gyr_gate": 10.0,
    "odin_s2_max_clip_ratio": 1.0,
    "shallow_s1_en": False,
    "shallow_s1_end_dt": 0.12,
    "shallow_s1_gyr": 0.20,
    "shallow_s1_iters": 1,

    # Odin into matching map like official co-map
    "odin_wall_nz_thr": 0.40,
    "odin_wall_info_boost": 1.0,
    "odin_floor_info_scale": 1.0,
    "odin_voxel_insert_en": True,
    "odin_voxel_require_still": False,
    "odin_voxel_max_clip": 1.0,
    "odin_voxel_max_pts": 800,
    "odin_voxel_body_z_min": -2.0,
    "odin_voxel_cov_scale": 1.0,

    # No jump / vel / freeze
    "iekf_max_upd": 10.0,
    "iekf_max_rot": 10.0,
    "iekf_max_vel": 0.0,
    "iekf_s2_max_upd": 10.0,
    "iekf_s2_max_rot": 10.0,
    "consec_bad_freeze": 100,

    "map_jump_upd": 10.0,
    "map_jump_frame": 10.0,
    "map_max_range": 40.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": False,
    "odin_global_insert_en": True,
    "odin_global_require_s2": False,
    "odin_body_pub_en": True,

    # Z: only non-official post step (IEKF Z sinks ~1–3 m on this bag)
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
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
