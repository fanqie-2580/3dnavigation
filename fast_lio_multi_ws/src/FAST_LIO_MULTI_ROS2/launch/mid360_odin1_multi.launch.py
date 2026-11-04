#!/usr/bin/env python3
"""Mid360 LIO + independent Odin1 accum (no cross association)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        description='Mid360 independent accum (/viz/accum_mid360)',
    )
    enable_odin_accum_arg = DeclareLaunchArgument(
        'enable_odin_accum',
        default_value='true',
        description='Odin1 independent accum via /odin1/odometry (no Mid pose)',
    )
    odin_voxel_arg = DeclareLaunchArgument(
        'odin_voxel',
        default_value='0.08',
        description='Voxel size (m) for independent Odin accum',
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
            # Mid360-only FAST-LIO (yaml multi_lidar:=false)
            Node(
                package='fast_lio_multi',
                executable=exe,
                name='laserMapping_multi',
                output='screen',
                parameters=[config_file],
            )
        ]

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

        # Mid accum only (no Odin from Mid LIO)
        if context.launch_configurations.get('enable_color_viz', 'true') == 'true':
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='dual_lidar_color_viz.py',
                    name='dual_lidar_color_viz',
                    output='screen',
                    parameters=[{
                        'mid_topic': '/cloud_registered',
                        # Mid-only: Odin owns its own accum node (no association).
                        'odin_mode': 'off',
                        'map_frame': 'map',
                        'mid_stride': 1,
                        'voxel_size': 0.08,
                        'max_accum_points': 400000,
                        'accum_publish_period': 0.5,
                        'accum_skip_sec': 3.0,
                    }],
                )
            )

        # Independent Odin: prefer /odin1/cloud_slam (mode1, already in odom).
        # Node owns map→odin_map TF (one-shot visual lock to Mid; no pose fusion).
        if context.launch_configurations.get('enable_odin_accum', 'true') == 'true':
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='odin_independent_accum.py',
                    name='odin_independent_accum',
                    output='screen',
                    parameters=[{
                        'cloud_topic': '/odin1/cloud_slam',
                        'odom_topic': '/odin1/odometry',
                        'mid_odom_topic': '/Odometry',
                        'map_frame': 'odin_map',
                        'parent_frame': 'map',
                        'cloud_already_in_odom': True,
                        # Keep mode1 cloud_slam; lock TF with mode0 Mid↔Odin extrinsic.
                        'align_mode': 'extrinsic',
                        'align_delay_sec': 4.0,
                        'extrinsic_T': [-0.011, -0.02329, 0.04412],
                        'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        'extrinsic_T_L2_wrt_L1': [0.150, 0.0, 0.035],
                        'extrinsic_R_L2_wrt_L1': [
                            0.97317887, 0.0, -0.23004974,
                            0.0, 1.0, 0.0,
                            0.23004974, 0.0, 0.97317887,
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

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='multi_lio_body_to_base',
                arguments=[
                    '0', '0', '0',
                    '0', '-0.227', '0',
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
        enable_odin_accum_arg,
        odin_voxel_arg,
        OpaqueFunction(function=choose_nodes),
    ])
