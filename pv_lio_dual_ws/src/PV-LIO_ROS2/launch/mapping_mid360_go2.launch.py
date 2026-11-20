#!/usr/bin/env python3
"""Single Mid360 PV-LIO from go2-3d-mapping-nav pv_lio_official (residual_z)."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pv_lio')
    src_cfg = '/home/ll/pv_lio_dual_ws/src/PV-LIO_ROS2/config/mid360_go2.yaml'
    cfg = src_cfg if os.path.isfile(src_cfg) else os.path.join(pkg, 'config', 'mid360_go2.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'rviz_cfg', 'voxel_mapping.rviz')

    src_tune = '/home/ll/pv_lio_dual_ws/src/PV-LIO_ROS2/config/mid360_go2_tune.py'
    tune_dir = os.path.dirname(src_tune if os.path.isfile(src_tune)
                               else os.path.join(pkg, 'config', 'mid360_go2_tune.py'))
    sys.path.insert(0, tune_dir)
    from mid360_go2_tune import as_node_params  # noqa: E402

    use_sim = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='pv_lio',
            executable='pv_lio_node',
            name='pv_lio_mid360',
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
            name='rviz2_mid360_go2',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
