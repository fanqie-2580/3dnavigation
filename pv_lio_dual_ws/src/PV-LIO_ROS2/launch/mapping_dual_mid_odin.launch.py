from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('pv_lio')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='pv_lio',
            executable='pv_lio_node',
            name='pv_lio_dual',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'dual_mid_odin.yaml')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_dual',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_cfg', 'voxel_mapping.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
