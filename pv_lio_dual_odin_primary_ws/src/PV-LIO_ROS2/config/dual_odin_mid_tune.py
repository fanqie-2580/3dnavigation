# Odin-primary dual: Odin L1 + Mid L2.
# L1-only Z hacks (hard_band / drop_iekf_z) made XY path 108→500m+ — not usable.
# Use Mid as L2 for XY lock; same corridor Z recipe as Mid-primary dual.

TUNE = {
    "odin_min_pts": 4000,
    "odin_min_frac": 0.35,
    "odin_body_z_min": -2.0,
    "odin_body_z_max": 3.5,
    "odin_static_gyro": 0.05,

    # L2=Mid joint S2 (variable names still say odin_* = secondary)
    "odin_s2_enable": True,
    "odin_iekf_max_pts": 1200,
    "odin_iekf_max_end_dt": 0.18,
    "odin_info_weight": 0.65,
    "odin_range_ref": 6.0,
    "odin_max_residual": 0.12,
    "odin_s2_gyr_gate": 3.0,
    "odin_s2_max_clip_ratio": 0.95,
    "shallow_s1_en": True,
    "shallow_s1_end_dt": 0.10,
    "shallow_s1_gyr": 0.15,
    "shallow_s1_iters": 2,

    "odin_wall_nz_thr": 0.40,
    "odin_wall_info_boost": 1.0,
    "odin_floor_info_scale": 0.30,
    "odin_voxel_insert_en": False,
    "odin_voxel_max_clip": 0.20,
    "odin_voxel_max_pts": 100,
    "odin_voxel_body_z_min": 0.15,
    "odin_voxel_cov_scale": 8.0,

    # Odin S1 often jumps; S2 Mid must be allowed a larger correction
    # (prev s2_max_upd=0.06 → accept_s2=1/43, path 197m≈2×).
    "iekf_max_upd": 0.08,
    "iekf_max_rot": 0.08,
    "iekf_max_vel": 1.5,
    "iekf_s2_max_upd": 0.20,
    "iekf_s2_max_rot": 0.15,
    "consec_bad_freeze": 2,

    "map_jump_upd": 0.05,
    "map_jump_frame": 0.12,
    "map_max_range": 25.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": False,
    "odin_global_insert_en": True,
    "odin_global_require_s2": True,
    "odin_body_pub_en": True,

    "residual_z_en": True,
    "residual_z_flat_z_alpha": 0.32,
    "residual_z_flat_vz_alpha": 1.0,
    "residual_z_flat_ba_z_alpha": 0.30,
    "residual_z_flat_max_step": 0.020,
    "residual_z_hard_band": 0.025,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_floor_info_scale": 0.42,
    "residual_z_h_dampen": 0.48,
    "residual_z_dz_tol": 0.05,
    "residual_z_floor_ratio_thr": 0.03,
    "odin_z_factor_en": True,
    "odin_odom_topic": "/odin1/odometry",
    "odin_z_factor_alpha": 0.60,
    "odin_z_factor_max_dt": 0.30,
    "odin_z_factor_max_innov": 0.025,
    "odin_z_factor_band": 0.020,
    "odin_z_factor_catch_err": 0.0,
    "odin_z_factor_catch_frac": 1.0,
    "odin_z_factor_catch_max": 0.05,
    "odin_z_factor_catch_alpha": 0.50,
    "odin_z_factor_catch_min_feat": 30,
    "odin_z_factor_one_sided": False,
    "odin_z_factor_floor_scale": 0.30,
}


def as_node_params():
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
