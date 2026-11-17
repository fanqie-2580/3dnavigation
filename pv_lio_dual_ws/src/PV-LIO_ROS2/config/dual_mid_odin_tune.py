# Dual Mid360 + Odin1 PV-LIO runtime knobs (edit → restart launch, no rebuild).
# Loaded by mapping_dual_mid_odin.launch.py → ROS params under dual_tune.*.
# Defaults match known-good 2e98c1d behavior.

TUNE = {
    # --- Odin cloud quality / hold ---
    "odin_min_pts": 4000,          # sparse spike → display-hold
    "odin_min_frac": 0.35,         # vs last good Odin count
    "odin_body_z_min": -2.0,       # m, body-frame Z gate after T_L2
    "odin_body_z_max": 3.5,
    "odin_static_gyro": 0.05,      # rad/s; quasi-static (skip deskew)

    # --- Stage-2 soft Odin IEKF ---
    "odin_s2_enable": True,
    "odin_iekf_max_pts": 400,
    "odin_iekf_max_end_dt": 0.05,  # s; else Mid-only / Global densify path
    "odin_info_weight": 0.15,
    "odin_range_ref": 6.0,         # m; range² cov growth
    "odin_max_residual": 0.05,     # m; residual clip
    "odin_s2_gyr_gate": 0.12,      # rad/s; skip S2 when moving
    # Live: accept_s2 clip_r mean~0.82 → Odin≠Mid map but still densified (ghost walls).
    # FAST_LIO_MULTI / residual-outlier: reject S2 + skip Global Odin when clip high.
    # Live later: reject_s2 clip_r~0.85; early accept~0.45 still painted Odin layers.
    "odin_s2_max_clip_ratio": 0.55,

    # --- IEKF jump reject ---
    "iekf_max_upd": 0.08,          # m, Mid stage-1
    "iekf_max_rot": 0.08,          # rad
    "iekf_s2_max_upd": 0.015,      # m, S2 vs Mid-locked
    "iekf_s2_max_rot": 0.025,      # rad
    "consec_bad_freeze": 2,        # Mid-fail frames → freeze last good pose

    # --- Global display map ---
    "map_jump_upd": 0.05,          # m; still + large EKF step → skip Global
    "map_jump_frame": 0.12,        # reserved
    "map_max_range": 25.0,         # m; body-range clip for Global densify
    # False: accumulate Global while walking (user needs live densify).
    # True: only densify when |ω|<map_still_gyr (less smear, no walk growth).
    "map_require_still": False,
    "map_still_gyr": 0.12,         # rad/s; used only if map_require_still
    "map_use_downsample": True,    # Global uses Mid downs (not dense undistort)
    # True: Odin enters Global with Mid inserts.
    # require_s2=True blocked all walk densify (S2 skipped when |ω|≥gyr_gate).
    "odin_global_insert_en": True,
    "odin_global_require_s2": False,
    # Body MUST show Odin for live alignment check (display ≠ Global densify).
    "odin_body_pub_en": True,
}


def as_node_params():
    """Flatten for launch_ros Node(parameters=[..., as_node_params()])."""
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
