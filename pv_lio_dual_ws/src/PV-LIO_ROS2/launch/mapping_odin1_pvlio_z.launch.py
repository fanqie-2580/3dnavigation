#!/usr/bin/env python3
"""Single Odin1 PV-LIO for bag replay; pose Z from /odin1/odometry."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pv_lio')
    src_cfg = '/home/ll/pv_lio_dual_ws/src/PV-LIO_ROS2/config/odin1_pvlio_z.yaml'
    cfg = src_cfg if os.path.isfile(src_cfg) else os.path.join(pkg, 'config', 'odin1_pvlio_z.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'rviz_cfg', 'voxel_mapping.rviz')

    src_tune = '/home/ll/pv_lio_dual_ws/src/PV-LIO_ROS2/config/odin1_pvlio_z_tune.py'
    tune_dir = os.path.dirname(src_tune if os.path.isfile(src_tune)
                               else os.path.join(pkg, 'config', 'odin1_pvlio_z_tune.py'))
    sys.path.insert(0, tune_dir)
    from odin1_pvlio_z_tune import as_node_params  # noqa: E402

    converter = '/home/ll/ws_odin_pvlio/src/odin_pv_lio_bringup/scripts/odin_cloud_to_pvlio'
    use_sim = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        ExecuteProcess(
            cmd=[
                'python3', converter, '--ros-args',
                '-p', 'use_sim_time:=true',
                '-p', 'input_topic:=/odin1/cloud_raw',
                '-p', 'output_topic:=/odin1/points_pvlio',
                '-p', 'min_confidence:=30',
                '-p', 'blind:=0.2',
            ],
            output='screen',
        ),

        Node(
            package='pv_lio',
            executable='pv_lio_node',
            name='pv_lio_odin1',
            output='screen',
            parameters=[
                cfg,
                {'use_sim_time': use_sim},
                as_node_params(),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_odin1_pvlio',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
