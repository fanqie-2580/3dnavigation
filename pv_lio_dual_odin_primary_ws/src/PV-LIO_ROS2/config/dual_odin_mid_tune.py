# Odin-primary dual runtime knobs (edit → restart launch, no rebuild).
# Loaded by mapping_dual_odin_primary.launch.py → dual_tune.*.
#
# Naming inherited from Mid-primary fork: "odin_*" knobs mostly mean
# "secondary LiDAR" (now Mid360). Primary is Odin1.

TUNE = {
    # --- Secondary (Mid) cloud quality / hold ---
    "odin_min_pts": 2000,
    "odin_min_frac": 0.25,
    "odin_body_z_min": -2.0,
    "odin_body_z_max": 3.5,
    "odin_static_gyro": 0.08,

    # --- Stage-2: Odin S1 lock + soft Mid joint ---
    "odin_s2_enable": True,
    "odin_iekf_max_pts": 1200,
    "odin_iekf_max_end_dt": 0.18,
    "odin_info_weight": 0.55,
    "odin_range_ref": 8.0,
    "odin_max_residual": 0.12,
    "odin_s2_gyr_gate": 3.0,
    "odin_s2_max_clip_ratio": 0.95,
    "shallow_s1_en": True,
    "shallow_s1_end_dt": 0.10,
    "shallow_s1_gyr": 0.20,
    "shallow_s1_iters": 2,

    # Secondary walls/floors (Mid) — keep soft floors to limit Mid floor Z bias
    "odin_wall_nz_thr": 0.40,
    "odin_wall_info_boost": 1.0,
    "odin_floor_info_scale": 0.35,
    "odin_voxel_insert_en": False,
    "odin_voxel_max_clip": 0.20,
    "odin_voxel_max_pts": 100,
    "odin_voxel_body_z_min": 0.15,
    "odin_voxel_cov_scale": 8.0,

    "iekf_max_upd": 0.08,
    "iekf_max_rot": 0.08,
    "iekf_max_vel": 1.3,
    "iekf_s2_max_upd": 0.060,
    "iekf_s2_max_rot": 0.070,
    "consec_bad_freeze": 2,

    "map_jump_upd": 0.05,
    "map_jump_frame": 0.12,
    "map_max_range": 25.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": False,
    # Secondary Mid into global only after good S2
    "odin_global_insert_en": True,
    "odin_global_require_s2": True,
    "odin_body_pub_en": True,

    # Stairs-friendly Z: no hard flat clamp; mild residual only.
    # Primary is Odin — do not band-limit height to corridor flat_ref.
    "residual_z_en": False,
    "residual_z_flat_z_alpha": 0.0,
    "residual_z_flat_vz_alpha": 0.0,
    "residual_z_flat_ba_z_alpha": 0.0,
    "residual_z_flat_max_step": 0.0,
    "residual_z_hard_band": 0.0,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_floor_info_scale": 0.50,
    "residual_z_h_dampen": 0.70,
    "residual_z_dz_tol": 0.08,
    "residual_z_floor_ratio_thr": 0.03,
    "odin_z_factor_en": False,
    "odin_odom_topic": "/odin1/odometry",
    "odin_z_factor_alpha": 0.0,
    "odin_z_factor_max_dt": 0.30,
    "odin_z_factor_max_innov": 0.05,
    "odin_z_factor_band": 0.0,
    "odin_z_factor_catch_err": 0.0,
    "odin_z_factor_catch_frac": 1.0,
    "odin_z_factor_catch_max": 0.05,
    "odin_z_factor_catch_alpha": 0.50,
    "odin_z_factor_catch_min_feat": 30,
    "odin_z_factor_one_sided": False,
    "odin_z_factor_floor_scale": 0.30,
}


def as_node_params():
    """Flatten for launch_ros Node(parameters=[..., as_node_params()])."""
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
