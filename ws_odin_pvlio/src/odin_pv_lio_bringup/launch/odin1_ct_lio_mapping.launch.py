#!/usr/bin/env python3
"""Odin1 host_sdk_sample + CT-LIO mapping (ROS 2 Humble)."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    odin = get_package_share_directory('odin_ros_driver')
    ct = get_package_share_directory('ct_lio')

    odin_cfg = os.path.join(odin, 'config', 'control_command.yaml')
    ct_launch = os.path.join(ct, 'launch', 'run_eskf_odin1.launch.py')
    # Prefer source tree if share is stale during symlink-install edits
    src_ct_launch = '/home/ll/ct_lio_to_ros2-master/launch/run_eskf_odin1.launch.py'
    if os.path.isfile(src_ct_launch):
        ct_launch = src_ct_launch

    return LaunchDescription([
        DeclareLaunchArgument('start_odin_driver', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'odin_config_file',
            default_value=odin_cfg,
            description='Odin control_command.yaml (custom_map_mode should be 0)',
        ),

        # Minimal driver: cloud_raw + imu (skip depth/reproject/odin-rviz)
        Node(
            package='odin_ros_driver',
            executable='host_sdk_sample',
            name='host_sdk_sample',
            output='screen',
            parameters=[{'config_file': LaunchConfiguration('odin_config_file')}],
            condition=IfCondition(LaunchConfiguration('start_odin_driver')),
        ),

        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ct_launch),
                    launch_arguments={
                        'rviz_use': LaunchConfiguration('rviz'),
                    }.items(),
                ),
            ],
        ),
    ])
