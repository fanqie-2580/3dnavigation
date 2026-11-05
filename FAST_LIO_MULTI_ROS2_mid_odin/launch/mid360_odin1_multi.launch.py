#!/usr/bin/env python3
"""Mid360 + Odin1 on clean FAST_LIO_MULTI (tight fusion path).

Prereq:
  - Odin mode0, use_host_ros_time: 2 (host-aligned stamps)
  - Mid /livox/lidar (CustomMsg) + /livox/imu
  - Do NOT feed /odin1/cloud_slam into MULTI
"""
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
        'enable_odin_ds',
        default_value='true',
        description='Start odin_raw_downsample_bridge',
    )
    odin_voxel_arg = DeclareLaunchArgument(
        'odin_voxel',
        default_value='0.08',
        description='Voxel size for Odin downsample bridge',
    )
    odin_conf_min_arg = DeclareLaunchArgument(
        'odin_conf_min',
        default_value='30.0',
        description='Confidence filter in downsample bridge',
    )
    stamp_offset_arg = DeclareLaunchArgument(
        'stamp_offset_sec',
        default_value='0.0',
        description='Constant Odin stamp offset (sec) after host align',
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

        if context.launch_configurations.get('enable_odin_ds', 'true') == 'true':
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
                        'max_points': 20000,
                        'conf_min': float(context.launch_configurations.get('odin_conf_min', '30.0')),
                        'stamp_offset_sec': float(
                            context.launch_configurations.get('stamp_offset_sec', '0.0')
                        ),
                        'restamp_from_odom': False,
                    }],
                )
            )
        return nodes

    return LaunchDescription([
        update_method_arg,
        config_arg,
        enable_odin_ds_arg,
        odin_voxel_arg,
        odin_conf_min_arg,
        stamp_offset_arg,
        OpaqueFunction(function=choose_nodes),
    ])
