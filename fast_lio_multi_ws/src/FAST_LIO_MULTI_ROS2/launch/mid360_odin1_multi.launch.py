#!/usr/bin/env python3
"""Mid360 + Odin1 FAST_LIO_MULTI (tight coupling) + optional timed viz densify.

Default: Mid Livox + Mid IMU IEKF; Odin /cloud_raw → downsample bridge → L2 update
with CAD/Go2 extrinsic. Optional odin_mid_timed_project for RViz overlay.
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
        description='Republish Mid /Odometry and /cloud_registered to nav names',
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
        default_value='false',
        description='Legacy independent Odin odom accum + map→odin_map TF',
    )
    enable_odin_timed_arg = DeclareLaunchArgument(
        'enable_odin_timed',
        default_value='true',
        description='Project Odin raw into Mid map via Mid TF at Odin stamp',
    )
    enable_frame_fuse_arg = DeclareLaunchArgument(
        'enable_frame_fuse',
        default_value='true',
        description='Fuse Mid + Odin-in-map into /cloud_fused and /viz/accum_fused',
    )
    odin_voxel_arg = DeclareLaunchArgument(
        'odin_voxel',
        default_value='0.08',
        description='Voxel size for Odin accum inside mid_odin_frame_fuse',
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

        nodes = [
            Node(
                package='fast_lio_multi',
                executable=exe,
                name='laserMapping_multi',
                output='screen',
                parameters=[config_file],
            )
        ]

        # Odin1 → /odin1/cloud_raw_ds for FAST_LIO_MULTI L2 (host-restamp + voxel).
        nodes.append(
            Node(
                package='fast_lio_multi',
                executable='odin_raw_downsample_bridge.py',
                name='odin_raw_downsample_bridge',
                output='screen',
                parameters=[{
                    'input_topic': '/odin1/cloud_raw',
                    'output_topic': '/odin1/cloud_raw_ds',
                    'voxel_size': float(
                        context.launch_configurations.get('odin_voxel', '0.08')
                    ),
                    'max_points': 20000,
                    'min_confidence': 30.0,
                    'restamp_from_odom': True,
                    'one_cloud_per_odom': True,
                    'odom_topic': '/Odometry',
                    'odin_odom_topic': '/odin1/odometry',
                    'odin_odom_max_dt': 1.0,
                    'mid_restamp_max_dt': 0.15,
                    'max_cloud_age_sec': 0.35,
                    'fallback_period': 0.1,
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
                        'cloud_topic': '/odin1/cloud_raw',
                        'out_topic': '/odin1/cloud_in_map',
                        'mid_odom_topic': '/Odometry',
                        'odin_odom_topic': '/odin1/odometry',
                        'map_frame': 'map',
                        'body_frame': 'body',
                        # Exact Mid pairing. 0.15s: host_receive Odin stamps vs Mid capture lag.
                        'mid_pair_max_dt': 0.15,
                        'odin_pair_max_dt': 0.15,
                        # Allow EXACT viz under residual dock lag; fuse accum uses 0.25s.
                        'max_cloud_age_sec': 3.0,
                        'cloud_buffer_sec': 8.0,
                        'odom_buffer_sec': 10.0,
                        # Dock already strides TCP; keep all remaining pts for dense yellow live FOV.
                        'stride': 1,
                        'blind': 0.2,
                        # Lower than driver default 35 so live yellow FOV denser in RViz.
                        'min_confidence': 15.0,
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

        if context.launch_configurations.get('enable_odin_align', 'false') == 'true':
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='odin_independent_accum.py',
                    name='odin_independent_accum',
                    output='screen',
                    parameters=[{
                        'align_only': use_ct,
                        'cloud_topic': '/odin1/cloud_in_odom' if use_ct else '/odin1/cloud_raw',
                        'odom_topic': '/odin1/ct_lio/odom' if use_ct else '/odin1/odometry',
                        'mid_odom_topic': '/Odometry',
                        'map_frame': 'odin_map',
                        'parent_frame': 'map',
                        'cloud_already_in_odom': True if use_ct else False,
                        'align_mode': 'extrinsic',
                        'align_delay_sec': 4.0,
                        'extrinsic_T': [-0.011, -0.02329, 0.04412],
                        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        'extrinsic_T_L2_wrt_L1': [0.12157888, 0.00549610, 0.03169932],
                        'extrinsic_R_L2_wrt_L1': [
                            0.97437006, 0.0, -0.22495105,
                            0.0, 1.0, 0.0,
                            0.22495105, 0.0, 0.97437006,
                        ],
                        'stride': 2,
                        'blind': 0.3,
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
            # Mid360+Odin1: timed project → /odin1/cloud_in_map; color_viz is Mid-only
            # (odin_mode=off), so frame_fuse must voxel-accum Odin itself.
            odin_scan = '/odin1/cloud_in_map' if use_timed else '/odin1/cloud_in_odom'
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
                        'odin_already_in_map': use_timed,
                        'accumulate_odin_from_scan': True,
                        'voxel_size': float(
                            context.launch_configurations.get('odin_voxel', '0.08')
                        ),
                        'max_accum_points': 400000,
                        'accum_skip_sec': 2.0,
                        'accum_gyr_gate': 0.35,
                        'accum_gyr_cooldown_sec': 1.0,
                        # Residual dock lag ~1–2s ⇒ Odin voxel accum stays off until lag drops.
                        'max_scan_age_sec': 0.25,
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
        enable_color_viz_arg,
        enable_ct_lio_arg,
        enable_odin_align_arg,
        enable_odin_timed_arg,
        enable_frame_fuse_arg,
        odin_voxel_arg,
        OpaqueFunction(function=choose_nodes),
    ])
