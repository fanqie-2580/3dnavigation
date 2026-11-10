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
        pkg_share, 'config', 'dddmr_global_ego_frames.yaml')

    config_file = LaunchConfiguration('config_file')
    map_dir = LaunchConfiguration('map_dir')
    ground_dir = LaunchConfiguration('ground_dir')
    odom_topic = LaunchConfiguration('odom_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_internal_registered_cloud = LaunchConfiguration('publish_internal_registered_cloud')

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

    goal_pose_topic = LaunchConfiguration('goal_pose_topic')
    goal_pose_3d_topic = LaunchConfiguration('goal_pose_3d_topic')
    clicked_point_topic = LaunchConfiguration('clicked_point_topic')
    enable_goal_pose_relay = LaunchConfiguration('enable_goal_pose_relay')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    mcl_feature_parameters = [
        config_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        {'laser.baselink_frame': 'base_link'},
        {'featureAssociation.publish_registered_cloud': ParameterValue(publish_internal_registered_cloud, value_type=bool)},
    ]

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
        parameters=mcl_feature_parameters,
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
        name='sensor2baselink',
        arguments=[
            '0.1870', '0.0', '0.0803',
            '0.0', '0.2268928', '0.0',
            'base_link', 'livox_frame',
        ],
        condition=IfCondition(publish_sensor_tf),
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink2footprint',
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

    goal_pose_relay = Node(
        package='p2p_move_base',
        executable='goal_pose_to_clicked_point.py',
        output='screen',
        parameters=[
            {'goal_pose_topic': goal_pose_topic},
            {'goal_pose_3d_topic': goal_pose_3d_topic},
            {'clicked_point_topic': clicked_point_topic},
            {'default_frame_id': 'map'},
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(enable_goal_pose_relay),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_share, 'config', 'p2p_move_base_localization.yaml'),
            description='DDDMR global-planner yaml config; only the mcl_fa, perception_3d_global, global_planner, dynamic_window_aware_global_planner, and ego_reference_path_provider sections are used'),
        DeclareLaunchArgument(
            'map_dir',
            default_value=os.path.join(global_planner_share, 'data', 'segmentation_obstacle_map.pcd'),
            description='Static map PCD published as mapcloud for the DDDMR global planner'),
        DeclareLaunchArgument(
            'ground_dir',
            default_value=os.path.join(global_planner_share, 'data', 'segmentation_ground_map.pcd'),
            description='Traversable-ground PCD published as mapground for the DDDMR global planner'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/baselink2map',
            description='External odometry topic used by DDDMR lidar preprocessing; this should come from the active localization stack'),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/livox/lidar_pointcloud2',
            description='Standard sensor_msgs/PointCloud2 topic used by DDDMR feature extraction'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time when replaying bags'),
        DeclareLaunchArgument(
            'publish_internal_registered_cloud',
            default_value='false',
            description='Optionally let mcl_feature publish the internal de-skewed /cloud_registered topic; keep this false when FAST-LIO already provides that topic'),
        DeclareLaunchArgument(
            'publish_sensor_tf',
            default_value='true',
            description='Publish a static TF from base_link to sensor_frame when the robot does not provide it'),
        DeclareLaunchArgument(
            'sensor_frame',
            default_value='livox_frame',
            description='Lidar frame name for the optional static TF'),
        DeclareLaunchArgument(
            'sensor_x',
            default_value='0.1870',
            description='Optional static TF x from base_link to sensor_frame'),
        DeclareLaunchArgument(
            'sensor_y',
            default_value='0.0',
            description='Optional static TF y from base_link to sensor_frame'),
        DeclareLaunchArgument(
            'sensor_z',
            default_value='0.0803',
            description='Optional static TF z from base_link to sensor_frame'),
        DeclareLaunchArgument(
            'sensor_yaw',
            default_value='0',
            description='Optional static TF yaw from base_link to sensor_frame'),
        DeclareLaunchArgument(
            'sensor_pitch',
            default_value='0.2268928',
            description='Optional static TF pitch from base_link to sensor_frame'),
        DeclareLaunchArgument(
            'sensor_roll',
            default_value='0.0',
            description='Optional static TF roll from base_link to sensor_frame'),
        DeclareLaunchArgument(
            'publish_base_footprint_tf',
            default_value='true',
            description='Publish a static TF from base_link to base_footprint'),
        DeclareLaunchArgument(
            'base_footprint_z',
            default_value='-0.24',
            description='Optional static TF z from base_link to base_footprint'),
        DeclareLaunchArgument(
            'goal_pose_topic',
            default_value='/goal_pose',
            description='Standard RViz goal pose topic to relay into /clicked_point'),
        DeclareLaunchArgument(
            'goal_pose_3d_topic',
            default_value='/goal_pose_3d',
            description='DDDMR RViz 3D goal pose topic to relay into /clicked_point'),
        DeclareLaunchArgument(
            'clicked_point_topic',
            default_value='/clicked_point',
            description='Global planner input topic for goal points'),
        DeclareLaunchArgument(
            'enable_goal_pose_relay',
            default_value='false',
            description='Relay RViz goal_pose and goal_pose_3d into /clicked_point; keep this false when you use RViz Publish Point directly'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch the DDDMR global-planning RViz'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'p2p_move_base_localization.rviz'),
            description='RViz config used by the DDDMR global-only launch'),
        pcl_publisher,
        mcl_feature,
        sensor_tf,
        base_footprint_tf,
        global_planner,
        goal_pose_relay,
        rviz,
    ])
