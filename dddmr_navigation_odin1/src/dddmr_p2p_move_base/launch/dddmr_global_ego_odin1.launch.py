"""DDDMR global planner for Odin1 (lives only in dddmr_navigation_odin1)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('p2p_move_base')
    global_planner_share = get_package_share_directory('global_planner')
    global_planner_frame_override = os.path.join(
        pkg_share, 'config', 'dddmr_global_ego_frames_odin1.yaml')

    # Prefer source-tree odin1 maps inside the docker mount
    default_obstacle = '/root/dddmr_ws/src/dddmr_global_planner/data/odin1/segmentation_obstacle_map.pcd'
    default_ground = '/root/dddmr_ws/src/dddmr_global_planner/data/odin1/segmentation_ground_map.pcd'

    config_file = LaunchConfiguration('config_file')
    map_dir = LaunchConfiguration('map_dir')
    ground_dir = LaunchConfiguration('ground_dir')
    odom_topic = LaunchConfiguration('odom_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_sensor_tf = LaunchConfiguration('publish_sensor_tf')
    sensor_frame = LaunchConfiguration('sensor_frame')
    sensor_x = LaunchConfiguration('sensor_x')
    sensor_y = LaunchConfiguration('sensor_y')
    sensor_z = LaunchConfiguration('sensor_z')
    sensor_yaw = LaunchConfiguration('sensor_yaw')
    sensor_pitch = LaunchConfiguration('sensor_pitch')
    sensor_roll = LaunchConfiguration('sensor_roll')
    publish_base_footprint_tf = LaunchConfiguration('publish_base_footprint_tf')
    base_footprint_z = LaunchConfiguration('base_footprint_z')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pcl_publisher = Node(
        package='mcl_3dl',
        executable='pcl_publisher',
        output='screen',
        parameters=[
            config_file,
            {'map_dir': map_dir},
            {'ground_dir': ground_dir},
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    mcl_feature = Node(
        package='lego_loam_bor',
        executable='mcl_feature',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'laser.baselink_frame': 'base_link'},
            {'featureAssociation.publish_registered_cloud': False},
        ],
        remappings=[
            ('lslidar_point_cloud', lidar_topic),
            ('/lslidar_point_cloud', lidar_topic),
            ('odom', odom_topic),
            ('/odom', odom_topic),
        ],
    )

    sensor_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor2baselink_odin1',
        arguments=[
            sensor_x, sensor_y, sensor_z,
            sensor_yaw, sensor_pitch, sensor_roll,
            'base_link', sensor_frame,
        ],
        condition=IfCondition(publish_sensor_tf),
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink2footprint_odin1',
        arguments=[
            '0.0', '0.0', base_footprint_z,
            '0.0', '0.0', '0.0',
            'base_link', 'base_footprint',
        ],
        condition=IfCondition(publish_base_footprint_tf),
    )

    global_planner = Node(
        package='global_planner',
        executable='global_planner_node',
        output='screen',
        parameters=[
            config_file,
            global_planner_frame_override,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_odin1_nav',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, 'config', 'p2p_move_base_localization_odin1.yaml'),
        ),
        DeclareLaunchArgument('map_dir', default_value=default_obstacle),
        DeclareLaunchArgument('ground_dir', default_value=default_ground),
        DeclareLaunchArgument('odom_topic', default_value='/baselink2map'),
        DeclareLaunchArgument('lidar_topic', default_value='/odin1/cloud_raw'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_sensor_tf', default_value='true'),
        DeclareLaunchArgument('sensor_frame', default_value='lidar'),
        # Odin imu≈lidar near base; near-identity TF (meters / radians)
        DeclareLaunchArgument('sensor_x', default_value='0.0'),
        DeclareLaunchArgument('sensor_y', default_value='0.0'),
        DeclareLaunchArgument('sensor_z', default_value='0.0'),
        DeclareLaunchArgument('sensor_yaw', default_value='0.0'),
        DeclareLaunchArgument('sensor_pitch', default_value='0.0'),
        DeclareLaunchArgument('sensor_roll', default_value='0.0'),
        DeclareLaunchArgument('publish_base_footprint_tf', default_value='true'),
        DeclareLaunchArgument('base_footprint_z', default_value='-0.24'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'p2p_move_base_localization_odin1.rviz'),
        ),
        pcl_publisher,
        mcl_feature,
        sensor_tf,
        base_footprint_tf,
        global_planner,
        rviz,
    ])
