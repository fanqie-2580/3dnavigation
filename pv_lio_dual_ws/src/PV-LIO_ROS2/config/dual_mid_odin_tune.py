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

    # --- Stage-2 soft Odin IEKF (true tight couple while walking) ---
    "odin_s2_enable": True,
    "odin_iekf_max_pts": 400,
    # Was 0.05 → most walk frames fell to ok_map (end_dt>50ms). Widen for S2.
    "odin_iekf_max_end_dt": 0.12,
    "odin_info_weight": 0.20,
    "odin_range_ref": 6.0,         # m; range² cov growth
    "odin_max_residual": 0.08,     # m; slightly looser so walk clip_r can accept
    # Was 0.12 → ok_map_gyr dominated (~260/frame), S2 never ran while walking.
    "odin_s2_gyr_gate": 2.0,       # rad/s; allow S2 during corridor walk
    # Walk reject was clip~0.8 with max=0.55; raise so soft S2 can stick.
    "odin_s2_max_clip_ratio": 0.90,

    # --- IEKF jump reject ---
    "iekf_max_upd": 0.08,          # m, Mid stage-1
    "iekf_max_rot": 0.08,          # rad
    "iekf_s2_max_upd": 0.03,       # m, S2 vs Mid-locked (walk needs a bit more)
    "iekf_s2_max_rot": 0.04,       # rad
    "consec_bad_freeze": 2,        # Mid-fail frames → freeze last good pose

    # --- Global display map ---
    "map_jump_upd": 0.05,          # m; still + large EKF step → skip Global
    "map_jump_frame": 0.12,        # reserved
    "map_max_range": 25.0,         # m; body-range clip for Global densify
    # False: accumulate Global while walking (user needs live densify).
    # True: only densify when |ω|<map_still_gyr (less smear, no walk growth).
    "map_require_still": False,
    "map_still_gyr": 0.12,         # rad/s; used only if map_require_still
    # False: still frames use dense; moving frames use downs (code path).
    "map_use_downsample": False,
    # Tight-couple map: densify Odin only after accept_s2 (S2 now runs while walking).
    "odin_global_insert_en": True,
    "odin_global_require_s2": True,
    # Body MUST show Odin for live alignment check (display ≠ Global densify).
    "odin_body_pub_en": True,
}


def as_node_params():
    """Flatten for launch_ros Node(parameters=[..., as_node_params()])."""
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
