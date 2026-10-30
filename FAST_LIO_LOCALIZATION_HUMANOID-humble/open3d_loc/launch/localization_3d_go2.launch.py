from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fast_lio_share = FindPackageShare('fast_lio')
    open3d_loc_share = FindPackageShare('open3d_loc')

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([fast_lio_share, 'launch', 'mapping.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('fastlio_config_path'),
            'config_file': LaunchConfiguration('fastlio_config_file'),
            'rviz': LaunchConfiguration('fastlio_rviz'),
            'publish_livox_pointcloud2': LaunchConfiguration('publish_livox_pointcloud2'),
            'livox_custom_topic': LaunchConfiguration('livox_custom_topic'),
            'livox_pointcloud2_topic': LaunchConfiguration('livox_pointcloud2_topic'),
            'livox_pointcloud2_frame_id': LaunchConfiguration('livox_pointcloud2_frame_id'),
        }.items(),
    )

    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([open3d_loc_share, 'launch', 'open3d_loc_go2.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_file': LaunchConfiguration('map_file'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'fastlio_world_frame': LaunchConfiguration('fastlio_world_frame'),
            'fastlio_body_frame': LaunchConfiguration('fastlio_body_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'motion_frame': LaunchConfiguration('motion_frame'),
            'publish_map_to_base_tf': LaunchConfiguration('publish_map_to_base_tf'),
            'publish_motion_frame_topic': LaunchConfiguration('publish_motion_frame_topic'),
            'publish_map_to_motion_tf': LaunchConfiguration('publish_map_to_motion_tf'),
            'publish_odom_to_fastlio_world_tf': LaunchConfiguration('publish_odom_to_fastlio_world_tf'),
            'publish_body_to_base_tf': LaunchConfiguration('publish_body_to_base_tf'),
            'body_to_base_x': LaunchConfiguration('body_to_base_x'),
            'body_to_base_y': LaunchConfiguration('body_to_base_y'),
            'body_to_base_z': LaunchConfiguration('body_to_base_z'),
            'body_to_base_qx': LaunchConfiguration('body_to_base_qx'),
            'body_to_base_qy': LaunchConfiguration('body_to_base_qy'),
            'body_to_base_qz': LaunchConfiguration('body_to_base_qz'),
            'body_to_base_qw': LaunchConfiguration('body_to_base_qw'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_map_cur',
        arguments=['-d', PathJoinSubstitution([open3d_loc_share, 'rviz_cfg', 'fastlio.rviz'])],
        output='screen',
        condition=IfCondition(LaunchConfiguration('localization_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd',
            description='Pointcloud map file used by open3d_loc. Pass an absolute .pcd or .ply path.'),
        DeclareLaunchArgument(
            'fastlio_config_path',
            default_value=PathJoinSubstitution([fast_lio_share, 'config'])),
        DeclareLaunchArgument('fastlio_config_file', default_value='mid360.yaml'),
        DeclareLaunchArgument('fastlio_rviz', default_value='false'),
        DeclareLaunchArgument('publish_livox_pointcloud2', default_value='true'),
        DeclareLaunchArgument('livox_custom_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('livox_pointcloud2_topic', default_value='/livox/lidar_pointcloud2'),
        DeclareLaunchArgument('livox_pointcloud2_frame_id', default_value=''),
        DeclareLaunchArgument('localization_rviz', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('fastlio_world_frame', default_value='camera_init'),
        DeclareLaunchArgument('fastlio_body_frame', default_value='body'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('motion_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_map_to_base_tf', default_value='false'),
        DeclareLaunchArgument('publish_motion_frame_topic', default_value='false'),
        DeclareLaunchArgument('publish_map_to_motion_tf', default_value='false'),
        DeclareLaunchArgument('publish_odom_to_fastlio_world_tf', default_value='true'),
        DeclareLaunchArgument('publish_body_to_base_tf', default_value='true'),
        DeclareLaunchArgument('body_to_base_x', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_y', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_z', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_qx', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_qy', default_value='-0.1132032138'),
        DeclareLaunchArgument('body_to_base_qz', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_qw', default_value='0.9935718557'),
        fast_lio_launch,
        open3d_loc_launch,
        rviz_node,
    ])
