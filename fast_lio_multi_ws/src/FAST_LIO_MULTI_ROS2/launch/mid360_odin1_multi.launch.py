#!/usr/bin/env python3
"""Mid360 + Odin1 multi-LiDAR LIO bringup (no pc_utils / wheelchair deps)."""
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
    enable_odin_ds_arg = DeclareLaunchArgument(
        'enable_odin_downsample',
        default_value='true',
        description='Run voxel downsample bridge /odin1/cloud_raw -> /odin1/cloud_raw_ds',
    )
    enable_nav_bridge_arg = DeclareLaunchArgument(
        'enable_nav_bridge',
        default_value='true',
        description='Republish /Odometry and /cloud_registered to nav stack topic names',
    )
    odin_voxel_arg = DeclareLaunchArgument(
        'odin_voxel',
        default_value='0.08',
        description='Voxel size (m) for Odin raw cloud downsample bridge',
    )
    enable_color_viz_arg = DeclareLaunchArgument(
        'enable_color_viz',
        default_value='true',
        description='Publish /viz/cloud_mid360 and /viz/cloud_odin for RViz color compare',
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

        if context.launch_configurations.get('enable_odin_downsample', 'true') == 'true':
            nodes.append(
                Node(
                    package='fast_lio_multi',
                    executable='odin_raw_downsample_bridge.py',
                    name='odin_raw_downsample_bridge',
                    output='screen',
                    parameters=[{
                        'input_topic': '/odin1/cloud_raw',
                        'output_topic': '/odin1/cloud_raw_ds',
                        'voxel_size': float(context.launch_configurations.get('odin_voxel', '0.08')),
                        # Match Mid ~10Hz for official FAST_LIO_MULTI bundle; restamp Odin device clock.
                        'restamp_from_odom': True,
                        'one_cloud_per_odom': True,
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
                        # Mid + Odin both from LIO map clouds (no re-pose).
                        # Raw×odom path fans on yaw: Odin 30Hz vs Mid odom 10Hz.
                        'mid_topic': '/cloud_registered',
                        'odin_mode': 'registered',
                        'odin_registered_topic': '/cloud_odin_registered',
                        'odin_raw_topic': '/odin1/cloud_raw_ds',
                        'odom_topic': '/Odometry',
                        'map_frame': 'map',
                        'mid_stride': 1,
                        'odin_stride': 1,
                        'odin_blind': 0.3,
                        'voxel_size': 0.08,
                        'max_accum_points': 400000,
                        'accum_publish_period': 0.5,
                        'accum_skip_sec': 3.0,
                    }],
                )
            )

        # RViz TF axes: map -> mid360_lidar / odin1_lidar (same extrinsics as yaml)
        nodes.append(
            Node(
                package='fast_lio_multi',
                executable='dual_lidar_frames.py',
                name='dual_lidar_frames',
                output='screen',
                parameters=[{
                    'map_frame': 'map',
                    'odom_topic': '/Odometry',
                    'mid_frame': 'mid360_lidar',
                    'odin_frame': 'odin1_lidar',
                    'gravity_frame': True,
                    'extrinsic_T': [-0.011, -0.02329, 0.04412],
                    'extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    # keep in sync with config/mid360_odin1.yaml (Odin front+higher)
                    'extrinsic_T_L2_wrt_L1': [0.150, 0.0, 0.035],
                    'extrinsic_R_L2_wrt_L1': [
                        0.99965732, 0.0, -0.02617695,
                        0.0, 1.0, 0.0,
                        0.02617695, 0.0, 0.99965732,
                    ],
                }],
            )
        )

        # body -> base_link using existing Go2 Mid360 extrinsic (qy/qw form as rpy approx 0)
        # Keep identity-ish; localization stack already has body_to_base static TF when using open3d_loc.
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='multi_lio_body_to_base',
                arguments=[
                    '0', '0', '0',
                    '0', '-0.227', '0',  # pitch ~ asin(2*qy*qw) with qy=-0.1132 qw=0.9936
                    'body', 'base_link',
                ],
            )
        )
        return nodes

    return LaunchDescription([
        update_method_arg,
        config_arg,
        enable_odin_ds_arg,
        enable_nav_bridge_arg,
        odin_voxel_arg,
        enable_color_viz_arg,
        OpaqueFunction(function=choose_nodes),
    ])
