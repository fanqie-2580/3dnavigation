from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('fastlio_multi_pvlio')
    yaml_path = os.path.join(pkg_dir, 'config', 'mid360_odin1_multi.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='fastlio_multi_pvlio',
            executable='multi_pv_lio_node',
            name='fastlio_multi_pvlio',
            output='screen',
            parameters=[
                yaml_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_multi_pvlio',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_cfg', 'voxel_mapping.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
