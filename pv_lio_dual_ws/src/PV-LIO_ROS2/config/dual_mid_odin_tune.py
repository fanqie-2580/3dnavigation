# Dual Mid360 + Odin1 PV-LIO runtime knobs (edit → restart launch, no rebuild).
# Loaded by mapping_dual_mid_odin.launch.py → ROS params under dual_tune.*.

TUNE = {
    # --- Odin cloud quality / hold ---
    "odin_min_pts": 4000,
    "odin_min_frac": 0.35,
    "odin_body_z_min": -2.0,
    "odin_body_z_max": 3.5,
    "odin_static_gyro": 0.05,

    # --- Stage-2 soft Odin IEKF ---
    "odin_s2_enable": True,
    "odin_iekf_max_pts": 400,
    "odin_iekf_max_end_dt": 0.12,
    "odin_info_weight": 0.20,
    "odin_range_ref": 6.0,
    "odin_max_residual": 0.08,
    "odin_s2_gyr_gate": 2.0,
    "odin_s2_max_clip_ratio": 0.90,

    # --- IEKF jump reject ---
    "iekf_max_upd": 0.08,
    "iekf_max_rot": 0.08,
    "iekf_s2_max_upd": 0.03,
    "iekf_s2_max_rot": 0.04,
    "consec_bad_freeze": 2,

    # --- Global display map ---
    "map_jump_upd": 0.05,
    "map_jump_frame": 0.12,
    "map_max_range": 25.0,
    "map_require_still": False,
    "map_still_gyr": 0.12,
    "map_use_downsample": False,
    "odin_global_insert_en": True,
    "odin_global_require_s2": True,
    "odin_body_pub_en": True,

    # --- Corridor Z (root-cause fix) ---
    # Return: fusion tracked Odin's own -14cm sink 1:1. Mid floors at full
    # weight keep re-pulling Z. Point-LIO-style: inflate floor noise; go2-style:
    # clipped flat hold + ba.z damp; factor targets flat_ref not Odin Δz.
    "residual_z_en": True,
    "residual_z_flat_z_alpha": 0.45,
    "residual_z_flat_vz_alpha": 1.0,
    "residual_z_flat_ba_z_alpha": 0.40,
    "residual_z_flat_max_step": 0.025,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_floor_info_scale": 0.18,
    "residual_z_h_dampen": 0.30,
    "residual_z_dz_tol": 0.06,
    "residual_z_floor_ratio_thr": 0.03,
    "odin_z_factor_en": True,
    "odin_odom_topic": "/odin1/odometry",
    "odin_z_factor_alpha": 0.55,
    "odin_z_factor_max_dt": 0.30,
    "odin_z_factor_max_innov": 0.06,
    "odin_z_factor_catch_err": 0.06,
    "odin_z_factor_catch_frac": 1.0,
    "odin_z_factor_catch_max": 0.10,
    "odin_z_factor_catch_alpha": 0.90,
    "odin_z_factor_catch_min_feat": 10,
    "odin_z_factor_one_sided": True,
    "odin_z_factor_floor_scale": 0.12,
}


def as_node_params():
    """Flatten for launch_ros Node(parameters=[..., as_node_params()])."""
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
