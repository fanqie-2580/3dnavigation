#!/usr/bin/env python3
"""Odin1 driver (optional) + cloud converter + PV-LIO mapping."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('odin_pv_lio_bringup')
    pv = get_package_share_directory('pv_lio')
    odin = get_package_share_directory('odin_ros_driver')

    cfg = os.path.join(bringup, 'config', 'odin1_pvlio.yaml')
    if not os.path.isfile(cfg):
        cfg = os.path.join(pv, 'config', 'odin1.yaml')

    rviz_cfg = os.path.join(pv, 'config', 'rviz_cfg', 'voxel_mapping.rviz')
    odin_launch = os.path.join(odin, 'launch', 'odin1_ros2.launch.py')
    if not os.path.isfile(odin_launch):
        # some installs flatten launch
        odin_launch = os.path.join(odin, 'launch', 'odin1_ros2.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('start_odin_driver', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odin_launch),
            condition=IfCondition(LaunchConfiguration('start_odin_driver')),
        ),

        Node(
            package='odin_pv_lio_bringup',
            executable='odin_cloud_to_pvlio',
            name='odin_cloud_to_pvlio',
            output='screen',
            parameters=[{
                'input_topic': '/odin1/cloud_raw',
                'output_topic': '/odin1/points_pvlio',
                'min_confidence': 30,
                'blind': 0.2,
            }],
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='pv_lio',
                    executable='pv_lio_node',
                    name='laserMapping',
                    output='screen',
                    parameters=[cfg],
                ),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
