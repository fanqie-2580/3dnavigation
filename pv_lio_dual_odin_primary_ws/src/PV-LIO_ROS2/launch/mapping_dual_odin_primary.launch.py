from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import importlib.util
import os


def _load_tune_params(pkg_dir: str) -> dict:
    path = os.path.join(pkg_dir, 'config', 'dual_odin_mid_tune.py')
    spec = importlib.util.spec_from_file_location('dual_odin_mid_tune', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.as_node_params()


def generate_launch_description():
    pkg_dir = get_package_share_directory('pv_lio_odin_pri')
    yaml_path = os.path.join(pkg_dir, 'config', 'dual_odin_mid.yaml')
    tune_params = _load_tune_params(pkg_dir)
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='pv_lio_odin_pri',
            executable='pv_lio_node',
            name='pv_lio_odin_primary',
            output='screen',
            parameters=[
                yaml_path,
                tune_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_odin_pri',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_cfg', 'voxel_mapping.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
