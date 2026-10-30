import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ego_share = get_package_share_directory('ego_planner')
    wrapped_launch = os.path.join(ego_share, 'launch', 'dddmr_reference_tracking.launch.py')

    odom_topic = LaunchConfiguration('odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    reference_path_topic = LaunchConfiguration('reference_path_topic')
    frame_id = LaunchConfiguration('frame_id')
    base_frame = LaunchConfiguration('base_frame')
    cmd_topic = LaunchConfiguration('cmd_topic')
    odom_z_offset = LaunchConfiguration('odom_z_offset')
    grid_map_odom_z_offset = LaunchConfiguration('grid_map_odom_z_offset')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/baselink2map',
            description='Map-frame base_link odometry from open3d_loc. Use /Odometry_loc only if the FAST-LIO body frame is the planning body.'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/cloud_registered_1',
            description='Complete FAST-LIO registered scan; obstacle-only segmentation clouds cannot provide SCAN-style miss rays.'),
        DeclareLaunchArgument(
            'reference_path_topic',
            default_value='/global_path',
            description='Reference nav_msgs/Path topic, for example the DDDMR global planner output.'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Global planning frame shared by the localization and global-path stack.'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot body frame used when converting world velocity to /cmd_vel. Override to body until base_link is TF-connected if needed.'),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/cmd_vel',
            description='Velocity topic published by EGO.'),
        DeclareLaunchArgument(
            'odom_z_offset',
            default_value='0.0',
            description='Extra z offset applied to the planner odom state; keep 0.0 unless the odom pose is not at the planning body center.'),
        DeclareLaunchArgument(
            'grid_map_odom_z_offset',
            default_value='0.0',
            description='Extra z offset applied to the local occupancy-map odom state.'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch the ego planner RViz config.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(wrapped_launch),
            launch_arguments={
                'odom_topic': odom_topic,
                'cloud_topic': cloud_topic,
                'reference_path_topic': reference_path_topic,
                'frame_id': frame_id,
                'base_frame': base_frame,
                'cmd_topic': cmd_topic,
                'odom_z_offset': odom_z_offset,
                'grid_map_odom_z_offset': grid_map_odom_z_offset,
                'sensor_frame': 'body',
                'sensor_origin_offset_x': '-0.011',
                'sensor_origin_offset_y': '-0.02329',
                'sensor_origin_offset_z': '0.04412',
                'use_rviz': use_rviz,
                'enable_depth_fusion_input': 'false',
                'pose_type': '2',
                'project_pointcloud_to_ground': 'false',
            }.items(),
        ),
    ])
