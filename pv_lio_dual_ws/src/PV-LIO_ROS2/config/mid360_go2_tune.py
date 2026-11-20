# go2 residual_z_v6.3 (Mid360-only). No Odin.
# Multifloor bag: keep H_z damp; disable flat_z pin (would fight stairs).
TUNE = {
    "odin_s2_enable": False,
    "odin_joint_only_en": False,
    "odin_z_factor_en": False,
    "odin_z_replace_en": False,
    "odin_xyz_replace_en": False,
    "odin_pose_hold_on_bad": False,
    "odin_z_flat_hold_en": False,
    "odin_att_replace_en": False,
    "odin_global_insert_en": False,
    "odin_voxel_insert_en": False,
    "residual_z_en": True,
    "residual_z_h_dampen": 0.12,
    "residual_z_floor_nz_thr": 0.85,
    "residual_z_dz_tol": 0.06,
    "residual_z_floor_ratio_thr": 0.03,
    "residual_z_floor_info_scale": 0.45,
    # go2 flat alphas fight stairs — zero z pull, still damp vz/ba
    "residual_z_flat_z_alpha": 0.0,
    "residual_z_flat_vz_alpha": 0.75,
    "residual_z_flat_ba_z_alpha": 0.06,
    "residual_z_flat_max_step": 0.0,
    "residual_z_hard_band": 0.0,
    "map_require_still": False,
    "map_use_downsample": True,
    "consec_bad_freeze": 5,
    "iekf_max_upd": 10.0,
    "iekf_max_rot": 10.0,
    "iekf_max_vel": 0.0,
}


def as_node_params():
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
