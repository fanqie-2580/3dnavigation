from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import importlib.util
import os


def _load_tune_params(pkg_dir: str) -> dict:
    """Load dual_mid_odin_tune.py from share/config (symlink-install → source)."""
    path = os.path.join(pkg_dir, 'config', 'dual_mid_odin_tune.py')
    spec = importlib.util.spec_from_file_location('dual_mid_odin_tune', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.as_node_params()


def generate_launch_description():
    pkg_dir = get_package_share_directory('pv_lio')
    yaml_path = os.path.join(pkg_dir, 'config', 'dual_mid_odin.yaml')
    tune_params = _load_tune_params(pkg_dir)
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='pv_lio',
            executable='pv_lio_node',
            name='pv_lio_dual',
            output='screen',
            parameters=[yaml_path, tune_params],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_dual',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_cfg', 'voxel_mapping.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
