import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    publish_livox_pointcloud2 = LaunchConfiguration('publish_livox_pointcloud2')
    livox_custom_topic = LaunchConfiguration('livox_custom_topic')
    livox_pointcloud2_topic = LaunchConfiguration('livox_pointcloud2_topic')
    livox_pointcloud2_frame_id = LaunchConfiguration('livox_pointcloud2_frame_id')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        # 'config_file', default_value='avia.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_publish_livox_pointcloud2_cmd = DeclareLaunchArgument(
        'publish_livox_pointcloud2', default_value='true',
        description='Publish a standard PointCloud2 topic from Livox CustomMsg for downstream consumers such as DDDMR'
    )
    declare_livox_custom_topic_cmd = DeclareLaunchArgument(
        'livox_custom_topic', default_value='/livox/lidar',
        description='Livox CustomMsg topic to convert; keep this aligned with common.lid_topic in the selected FAST-LIO config'
    )
    declare_livox_pointcloud2_topic_cmd = DeclareLaunchArgument(
        'livox_pointcloud2_topic', default_value='/livox/lidar_pointcloud2',
        description='Output PointCloud2 topic published from the Livox CustomMsg input'
    )
    declare_livox_pointcloud2_frame_id_cmd = DeclareLaunchArgument(
        'livox_pointcloud2_frame_id', default_value='',
        description='Optional frame_id override for the converted PointCloud2; leave empty to preserve the Livox message header frame_id'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    livox_pointcloud2_bridge_node = Node(
        package='fast_lio',
        executable='livox_custom_to_pointcloud2_node',
        parameters=[
            {'input_topic': livox_custom_topic},
            {'output_topic': livox_pointcloud2_topic},
            {'frame_id_override': livox_pointcloud2_frame_id},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
        condition=IfCondition(publish_livox_pointcloud2)
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_publish_livox_pointcloud2_cmd)
    ld.add_action(declare_livox_custom_topic_cmd)
    ld.add_action(declare_livox_pointcloud2_topic_cmd)
    ld.add_action(declare_livox_pointcloud2_frame_id_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(livox_pointcloud2_bridge_node)
    # ld.add_action(rviz_node)

    return ld
