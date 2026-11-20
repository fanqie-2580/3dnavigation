# Single Odin1 PV-LIO: pose Z := /odin1/odometry (init-aligned).
TUNE = {
    "odin_s2_enable": False,
    "odin_joint_only_en": False,
    "residual_z_en": False,
    "odin_z_factor_en": False,
    "odin_z_replace_en": True,
    "odin_z_flat_hold_en": False,
    "odin_att_replace_en": False,
    "odin_pose_hold_on_bad": True,
    "odin_odom_topic": "/odin1/odometry",
    "consec_bad_freeze": 5,
    "iekf_max_upd": 10.0,
    "iekf_max_rot": 10.0,
    "iekf_max_vel": 0.0,
}


def as_node_params():
    return {f"dual_tune.{k}": v for k, v in TUNE.items()}
