#!/usr/bin/env python3
"""Mid360 + Odin1 FAST_LIO_MULTI bringup.

B2 default:
  - System /Odometry + map→body TF from /odin1/odometry (odin_system_odom_bridge)
  - Mid IEKF remapped to /Odometry_mid (L2 restamp / internal only; no odom TF)
  - Odin ground densify with Odin odom (odin_independent_accum)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value='adaptive',
        description='bundle | async | adaptive',
    )
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='mid360_odin1.yaml',
        description='Config YAML under fast_lio_multi/config',
    )
    enable_nav_bridge_arg = DeclareLaunchArgument(
        'enable_nav_bridge',
        default_value='true',
        description='Republish system /Odometry and /cloud_registered to nav names',
    )
    enable_odin_system_odom_arg = DeclareLaunchArgument(
        'enable_odin_system_odom',
        default_value='false',
        description='B2 optional: system pose/TF from /odin1/odometry (off = Mid tight-coupled /Odometry)',
    )
    enable_color_viz_arg = DeclareLaunchArgument(
        'enable_color_viz',
        default_value='true',
        description='Mid360-only accum (/viz/accum_mid360); Odin accum is in frame_fuse',
    )
    enable_ct_lio_arg = DeclareLaunchArgument(
        'enable_ct_lio',
        default_value='false',
        description='Odin mode0 CT-LIO in odin_map (same-clock domain)',
    )
    enable_odin_align_arg = DeclareLaunchArgument(
        'enable_odin_align',
        default_value='true',
        description='B1: densify Odin ground with /odin1/odometry into Mid map',
    )
    enable_odin_timed_arg = DeclareLaunchArgument(
        'enable_odin_timed',
        default_value='false',
        description='Project Odin into Mid map via Mid /Odometry (legacy; off for B1)',
    )
    enable_frame_fuse_arg = DeclareLaunchArgument(
        'enable_frame_fuse',
        default_value='true',
        description='Fuse Mid + Odin-in-map into /cloud_fused and /viz/accum_fused',
    )
    odin_voxel_arg = DeclareLaunchArgument(
        'odin_voxel',
        default_value='0.08',
        description='Voxel size for Odin bridge/accum densify',
    )

    def choose_nodes(context, *args, **kwargs):
        method = context.launch_configurations['update_method']
        exe = {
            'bundle': 'laserMapping_bundle',
            'async': 'laserMapping_async',
            'adaptive': 'laserMapping_adaptive',
        }.get(method)
        if exe is None:
            raise ValueError(f'Unknown update_method: {method}')

        config_file = PathJoinSubstitution([
            FindPackageShare('fast_lio_multi'),
            'config',
            LaunchConfiguration('config'),
        ])

        use_odin_sys = (
            context.launch_configurations.get('enable_odin_system_odom', 'false') == 'true'
        )
        nodes = [
            Node(
                package='fast_lio_multi',
                executable=exe,
                name='laserMapping_multi',
                output='screen',
                parameters=[config_file],
                # B2: keep Mid IEKF pose off the system /Odometry name.
                remappings=[
                    ('/Odometry', '/Odometry_mid'),
                    ('/path', '/path_mid'),
                ] if use_odin_sys else [],
            )
        ]

        if use_odin_sys:
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='odin_system_odom_bridge.py',
                    name='odin_system_odom_bridge',
                    output='screen',
                    parameters=[{
                        'odin_odom_topic': '/odin1/odometry',
                        'mid_odom_topic': '/Odometry_mid',
                        'out_odom_topic': '/Odometry',
                        'out_path_topic': '/path',
                        'map_frame': 'map',
                        'body_frame': 'body',
                        'publish_tf': True,
                        'align_delay_sec': 3.0,
                        'imu_to_lidar_xyz': [-0.02663, 0.03447, 0.02174],
                        'imu_to_lidar_rpy': [0.0, 0.0, 0.0],
                        'base_to_odin_imu_xyz': [0.313654, -0.028974, 0.104418],
                        'base_to_odin_imu_rpy': [0.0, 0.0, 0.0],
                        'body_to_base_xyz': [-0.140709, -0.02329, -0.111671],
                        'body_to_base_rpy': [0.0, -0.226893, 0.0],
                        'extrinsic_T': [-0.011, -0.02329, 0.04412],
                        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        'extrinsic_T_L2_wrt_L1': [0.12157888, 0.00549610, 0.03169932],
                        'extrinsic_R_L2_wrt_L1': [
                            0.97437006, 0.0, -0.22495105,
                            0.0, 1.0, 0.0,
                            0.22495105, 0.0, 0.97437006,
                        ],
                    }],
                )
            )

        # USB host_sdk: /odin1/cloud_raw is already lidar-frame. Do NOT feed
        # cloud_slam into L2 — unproject was wrong (r_p50 ~4m vs raw ~1m) and
        # painted a ghost densify map (turn smear / offset).
        nodes.append(
            Node(
                package='fast_lio_multi',
                executable='odin_raw_downsample_bridge.py',
                name='odin_raw_downsample_bridge',
                output='screen',
                parameters=[{
                    'input_topic': '/odin1/cloud_raw',
                    # Full Odin cloud for L2 + densify (ground-only backup:
                    # launch/mid360_odin1_multi_ground_only.launch.py).
                    'output_topic': '/odin1/cloud_raw_ds',
                    'voxel_size': float(
                        context.launch_configurations.get('odin_voxel', '0.12')
                    ),
                    # Tight latency: unpack ≤4k from 49k raw (skip heavy voxel unique).
                    'max_points': 4000,
                    'pre_decimate_max': 4000,
                    'min_confidence': 30.0,
                    'restamp_from_odom': True,
                    # False: do not wait for Mid odom tick (was adding ~100ms+
                    # and stacking behind slow Python pack). Still restamp to a
                    # nearby Mid stamp when one exists within mid_restamp_max_dt.
                    'one_cloud_per_odom': False,
                    # Restamp against Mid IEKF timebase (not system Odin /Odometry).
                    'odom_topic': '/Odometry_mid' if use_odin_sys else '/Odometry',
                    'odin_odom_topic': '/odin1/odometry',
                    'odin_odom_max_dt': 1.0,
                    # Must match mapping.lidar2_max_sync_dt: never restamp L2
                    # outside this window (lying stamps twist Mid IEKF).
                    # Causal Mid densify pose window (interp/before only; never future).
                    # Mid odom ~10Hz → allow up to one period so we rarely skip.
                    'mid_restamp_max_dt': 0.12,
                    'max_cloud_age_sec': 0.25,
                    # 0: no artificial rate floor (latest-only already drops extras).
                    'fallback_period': 0.0,
                    'ground_only': False,
                    # raw is lidar-frame: no odom unproject on main or L2.
                    'unproject_odom_clouds': False,
                    'l2_body_topic': '/odin1/cloud_l2_body',
                    'l2_mid_pose_topic': '/odin1/l2_mid_pose',
                    'ground_topic': '/odin1/cloud_ground',
                    # Skip RANSAC entirely when not keeping ground-only.
                    'publish_ground_topic': False,
                    'publish_full_topic': '',
                    'ground_distance': 0.08,
                    'ground_max_angle_deg': 25.0,
                    'ground_ransac_iters': 20,
                    'ground_min_inliers': 60,
                    'ground_fit_z_span': 1.5,
                    'ground_z_band': 0.20,
                }],
            )
        )

        if context.launch_configurations.get('enable_nav_bridge', 'true') == 'true':
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='multi_lio_nav_bridge.py',
                    name='multi_lio_nav_bridge',
                    output='screen',
                    parameters=[{
                        # System pose (Odin B2) → dddmr
                        'odom_in': '/Odometry',
                        'odom_out': '/baselink2map',
                        'cloud_in': '/cloud_registered',
                        'cloud_out': '/cloud_registered_1',
                        'cloud_body_in': '/cloud_registered_body',
                        'cloud_body_out': '/cloud_registered_body_1',
                    }],
                )
            )

        if context.launch_configurations.get('enable_color_viz', 'true') == 'true':
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='dual_lidar_color_viz.py',
                    name='dual_lidar_color_viz',
                    output='screen',
                    parameters=[{
                        'mid_topic': '/cloud_registered',
                        'odin_mode': 'off',
                        'map_frame': 'map',
                        'mid_stride': 1,
                        'voxel_size': 0.08,
                        'max_accum_points': 400000,
                        'accum_publish_period': 0.5,
                        'accum_skip_sec': 3.0,
                        # Kill Mid360 ghost/flyers that otherwise pollute room-scale maps.
                        'max_range': 25.0,
                    }],
                )
            )

        use_ct = context.launch_configurations.get('enable_ct_lio', 'true') == 'true'
        if use_ct:
            ct_share = None
            try:
                from ament_index_python.packages import get_package_share_directory
                ct_share = get_package_share_directory('ct_lio')
            except Exception:
                ct_share = None
            src_yaml = '/home/ll/ct_lio_to_ros2-master/config/mapping_odin1.yaml'
            share_yaml = (
                os.path.join(ct_share, 'config', 'mapping_odin1.yaml')
                if ct_share else src_yaml
            )
            ct_yaml = src_yaml if os.path.isfile(src_yaml) else share_yaml
            nodes.append(
                Node(
                    package='ct_lio',
                    executable='ct_lio_eskf',
                    name='ct_lio',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        ParameterFile(
                            RewrittenYaml(
                                source_file=ct_yaml,
                                root_key='',
                                param_rewrites={},
                                convert_types=True,
                            ),
                            allow_substs=True,
                        )
                    ],
                    remappings=[
                        # Avoid colliding with Mid FAST-LIO topics
                        ('/cloud_registered', '/odin1/cloud_in_odom'),
                        ('/Laser_map', '/viz/accum_odin'),
                        ('/odom', '/odin1/ct_lio/odom'),
                        ('/odometry_path', '/odin1/ct_lio/path'),
                    ],
                )
            )

        use_timed = context.launch_configurations.get('enable_odin_timed', 'true') == 'true'
        if use_timed:
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='odin_mid_timed_project.py',
                    name='odin_mid_timed_project',
                    output='screen',
                    parameters=[{
                        # Full Odin cloud (lidar frame from downsample bridge).
                        'cloud_topic': '/odin1/cloud_raw_ds',
                        'out_topic': '/odin1/cloud_in_map',
                        'mid_odom_topic': '/Odometry_mid' if use_odin_sys else '/Odometry',
                        'odin_odom_topic': '/odin1/odometry',
                        'map_frame': 'map',
                        'body_frame': 'body',
                        # Ground cloud is lidar-frame from downsample bridge (not cloud_slam).
                        'cloud_in_odom': 'false',
                        # Exact Mid pairing. 0.15s: host_receive Odin stamps vs Mid capture lag.
                        'mid_pair_max_dt': 0.15,
                        'odin_pair_max_dt': 0.15,
                        # Allow EXACT viz under residual dock lag; fuse accum uses 0.25s.
                        'max_cloud_age_sec': 3.0,
                        'cloud_buffer_sec': 8.0,
                        'odom_buffer_sec': 10.0,
                        # Ground already voxel-filtered; keep all remaining pts.
                        'stride': 1,
                        'blind': 0.2,
                        # Ground cloud has no confidence field; keep gate inert.
                        'min_confidence': 0.0,
                        'extrinsic_T': [-0.011, -0.02329, 0.04412],
                        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        'use_yaw_only_body': False,
                        'extrinsic_T_L2_wrt_L1': [0.12157888, 0.00549610, 0.03169932],
                        'extrinsic_R_L2_wrt_L1': [
                            0.97437006, 0.0, -0.22495105,
                            0.0, 1.0, 0.0,
                            0.22495105, 0.0, 0.97437006,
                        ],
                    }],
                )
            )

        use_align = context.launch_configurations.get('enable_odin_align', 'true') == 'true'
        if use_align:
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='odin_independent_accum.py',
                    name='odin_independent_accum',
                    output='screen',
                    parameters=[{
                        # CT-LIO: align TF only. Else: densify lidar-frame Odin with Mid pose
                        # (no Odin cloud_slam / own map for RViz accum).
                        'align_only': use_ct,
                        'cloud_topic': (
                            '/odin1/cloud_in_odom' if use_ct else '/odin1/cloud_l2_body'
                        ),
                        'odom_topic': '/odin1/ct_lio/odom' if use_ct else '/odin1/odometry',
                        'mid_odom_topic': '/Odometry_mid' if use_odin_sys else '/Odometry',
                        'map_frame': 'odin_map',
                        'parent_frame': 'map',
                        'out_scan_topic': '/odin1/cloud_in_map',
                        'out_accum_topic': '/viz/accum_odin',
                        'cloud_already_in_odom': bool(use_ct),
                        'project_with_mid': (not use_ct),
                        'publish_in_parent_frame': False,
                        'wait_align_for_parent': False,
                        'align_mode': 'off' if not use_ct else 'yaw',
                        'align_delay_sec': 2.0,
                        'odom_pair_max_dt': 0.50,
                        # Warp stamp-projected Odin to latest Mid (L2 geom ~0.25s old).
                        'motion_sync_to_mid': True,
                        'motion_sync_min_trans': 0.015,
                        'motion_sync_min_rot_deg': 0.35,
                        'mid_wall_pair_max_dt': 0.20,
                        # Accum always stamp-locked; latest Mid only when moving (see gates).
                        'mid_pose_lookback_sec': 0.0,
                        'mid_pose_ema': 1.0,
                        'prefer_latest_mid_pose': False,
                        'latest_mid_min_trans_vel': 0.08,
                        'latest_mid_min_yaw_rate_deg': 8.0,
                        'still_trans_vel': 0.05,
                        'still_yaw_rate_deg': 3.0,
                        # Hold Mid IEKF noise; soft-track only (no 0.65 snap on slight move).
                        'densify_hold_trans': 0.025,
                        'densify_hold_rot_deg': 1.0,
                        'densify_track_alpha': 0.08,
                        'deskew_odin': True,
                        'restamp_scan_to_now': True,
                        'accum_max_yaw_rate_deg': 25.0,
                        'l2_mid_pose_topic': '/odin1/l2_mid_pose',
                        'imu_to_lidar_xyz': [-0.02663, 0.03447, 0.02174],
                        'imu_to_lidar_rpy_deg': [0.0, 0.0, 0.0],
                        'extrinsic_T': [-0.011, -0.02329, 0.04412],
                        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        'extrinsic_T_L2_wrt_L1': [0.12157888, 0.00549610, 0.03169932],
                        'extrinsic_R_L2_wrt_L1': [
                            0.97437006, 0.0, -0.22495105,
                            0.0, 1.0, 0.0,
                            0.22495105, 0.0, 0.97437006,
                        ],
                        'stride': 1,
                        'blind': 0.2,
                        'min_confidence': 0,
                        'voxel_size': float(
                            context.launch_configurations.get('odin_voxel', '0.08')
                        ),
                        'max_accum_points': 400000,
                        'accum_publish_period': 0.5,
                        'accum_skip_sec': 2.0,
                    }],
                )
            )

        if context.launch_configurations.get('enable_frame_fuse', 'true') == 'true':
            # B1: independent_accum owns Odin densify (/odin1/cloud_in_map + /viz/accum_odin).
            # Timed Mid-pose project is legacy (enable_odin_timed:=true).
            if use_timed:
                odin_scan = '/odin1/cloud_in_map'
                odin_in_map = True
                accum_from_scan = True
            elif use_align and not use_ct:
                odin_scan = '/odin1/cloud_in_map'
                odin_in_map = True
                accum_from_scan = False  # independent_accum owns /viz/accum_odin
            else:
                odin_scan = '/odin1/cloud_in_odom'
                odin_in_map = False
                accum_from_scan = True
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='mid_odin_frame_fuse.py',
                    name='mid_odin_frame_fuse',
                    output='screen',
                    parameters=[{
                        'mid_scan_topic': '/cloud_registered',
                        'odin_scan_topic': odin_scan,
                        'mid_accum_topic': '/viz/accum_mid360',
                        'odin_accum_topic': '/viz/accum_odin',
                        'map_frame': 'map',
                        'odin_frame': 'odin_map',
                        'mid_odom_topic': '/Odometry_mid' if use_odin_sys else '/Odometry',
                        'odin_already_in_map': odin_in_map,
                        'accumulate_odin_from_scan': accum_from_scan,
                        'voxel_size': float(
                            context.launch_configurations.get('odin_voxel', '0.08')
                        ),
                        'max_accum_points': 400000,
                        'accum_skip_sec': 2.0,
                        'accum_gyr_gate': 0.35,
                        'accum_gyr_cooldown_sec': 1.0,
                        # Allow residual dock TCP lag so fuse does not starve.
                        'max_scan_age_sec': 1.50,
                        'max_range': 25.0,
                        'stride': 1,
                        'accum_period': 0.5,
                    }],
                )
            )

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='multi_lio_body_to_base',
                # MidIMU(body)→Go2 base_link from official Mid@Go2IMU + URDF + Mid board extrinsic
                arguments=[
                    '-0.140709', '-0.02329', '-0.111671',
                    '0', '-0.226893', '0',
                    'body', 'base_link',
                ],
            )
        )
        return nodes

    return LaunchDescription([
        update_method_arg,
        config_arg,
        enable_nav_bridge_arg,
        enable_odin_system_odom_arg,
        enable_color_viz_arg,
        enable_ct_lio_arg,
        enable_odin_align_arg,
        enable_odin_timed_arg,
        enable_frame_fuse_arg,
        odin_voxel_arg,
        OpaqueFunction(function=choose_nodes),
    ])
