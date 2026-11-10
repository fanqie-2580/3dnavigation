"""Odin1 localization: open3d_loc only (no Mid360 / FAST-LIO).

Requires Jetson-side topics (via TCP bridge + odin1_loc_topic_adapter):
  /Odometry_loc, /cloud_registered_1
Map default: data/odin1_odom_map.pcd
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    open3d_loc_launch = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'path_map': LaunchConfiguration('map_file'),
                'input_odom_topic': LaunchConfiguration('input_odom_topic'),
                'input_scan_topic': LaunchConfiguration('input_scan_topic'),
                'initialpose_topic': LaunchConfiguration('initialpose_topic'),
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'imu_frame': LaunchConfiguration('body_frame'),
                'motion_frame': LaunchConfiguration('base_frame'),
                'publish_map_to_base_tf': False,
                'publish_motion_frame_topic': False,
                'publish_map_to_motion_tf': False,
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.05,
                'voxelsize_fine': 0.2,
                # Odin map was built in the same continuous odom frame as live
                # cloud_slam. Identity already overlaps well; ICP often slides
                # ~1m/20deg in corridors and makes reloc look wrong. Keep map≡odom
                # after init by requiring near-perfect fitness to update.
                'threshold_fitness': 0.995,
                'threshold_fitness_init': 0.85,
                'loc_frequence': 2.0,
                'save_scan': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'confidence_loc_th': 0.5,
                'dis_updatemap': 3.5,
            },
        ],
    )

    # Odin: treat odom world == camera_init, body == imu
    static_tf_odom_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init_odin1',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init'],
    )
    static_tf_body_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_odin1',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'base_link'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_odin1_loc',
        arguments=['-d', PathJoinSubstitution([open3d_loc_share, 'rviz_cfg', 'fastlio.rviz'])],
        output='screen',
        condition=IfCondition(LaunchConfiguration('localization_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/odin1_odom_map.pcd',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/ll/ws_odin_pvlio/odin1_nav/config/loc_param_odin1.yaml',
        ),
        DeclareLaunchArgument('input_odom_topic', default_value='/Odometry_loc'),
        DeclareLaunchArgument('input_scan_topic', default_value='/cloud_registered_1'),
        DeclareLaunchArgument('initialpose_topic', default_value='/initialpose'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('body_frame', default_value='body'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('localization_rviz', default_value='true'),
        static_tf_odom_world,
        static_tf_body_base,
        open3d_loc_launch,
        rviz_node,
    ])
