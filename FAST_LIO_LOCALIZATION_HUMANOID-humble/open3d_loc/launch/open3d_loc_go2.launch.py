from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open3d_loc_share = FindPackageShare('open3d_loc')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    map_file = LaunchConfiguration('map_file')
    input_odom_topic = LaunchConfiguration('input_odom_topic')
    input_scan_topic = LaunchConfiguration('input_scan_topic')
    initialpose_topic = LaunchConfiguration('initialpose_topic')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    fastlio_world_frame = LaunchConfiguration('fastlio_world_frame')
    fastlio_body_frame = LaunchConfiguration('fastlio_body_frame')
    base_frame = LaunchConfiguration('base_frame')
    motion_frame = LaunchConfiguration('motion_frame')
    publish_map_to_base_tf = LaunchConfiguration('publish_map_to_base_tf')
    publish_motion_frame_topic = LaunchConfiguration('publish_motion_frame_topic')
    publish_map_to_motion_tf = LaunchConfiguration('publish_map_to_motion_tf')

    static_tf_odom_to_fastlio_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_fastlio_world_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', '1', odom_frame, fastlio_world_frame],
        condition=IfCondition(LaunchConfiguration('publish_odom_to_fastlio_world_tf')),
    )

    static_tf_fastlio_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fastlio_body_to_base_broadcaster',
        arguments=[
            LaunchConfiguration('body_to_base_x'),
            LaunchConfiguration('body_to_base_y'),
            LaunchConfiguration('body_to_base_z'),
            LaunchConfiguration('body_to_base_qx'),
            LaunchConfiguration('body_to_base_qy'),
            LaunchConfiguration('body_to_base_qz'),
            LaunchConfiguration('body_to_base_qw'),
            fastlio_body_frame,
            base_frame,
        ],
        condition=IfCondition(LaunchConfiguration('publish_body_to_base_tf')),
    )

    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='screen',
        parameters=[
            config_file,
            {
                'path_map': map_file,
                'input_odom_topic': input_odom_topic,
                'input_scan_topic': input_scan_topic,
                'initialpose_topic': initialpose_topic,
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'imu_frame': fastlio_body_frame,
                'motion_frame': motion_frame,
                'publish_map_to_base_tf': ParameterValue(publish_map_to_base_tf, value_type=bool),
                'publish_motion_frame_topic': ParameterValue(publish_motion_frame_topic, value_type=bool),
                'publish_map_to_motion_tf': ParameterValue(publish_map_to_motion_tf, value_type=bool),
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.01,
                'voxelsize_fine': 0.2,
                'threshold_fitness': 0.5,
                'threshold_fitness_init': 0.5,
                'loc_frequence': 2.5,
                'save_scan': False,
                'maxpoints_source': 80000,
                'maxpoints_target': 400000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 3.5,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([open3d_loc_share, 'config', 'loc_param_go2.yaml']),
            description='open3d_loc localization parameter file.'),
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd',
            description='Pointcloud map file used by open3d_loc. Pass an absolute .pcd or .ply path.'),
        DeclareLaunchArgument('input_odom_topic', default_value='/Odometry_loc'),
        DeclareLaunchArgument('input_scan_topic', default_value='/cloud_registered_1'),
        DeclareLaunchArgument('initialpose_topic', default_value='/initialpose'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('fastlio_world_frame', default_value='camera_init'),
        DeclareLaunchArgument('fastlio_body_frame', default_value='body'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('motion_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'publish_map_to_base_tf',
            default_value='false',
            description='Keep false when body->base_link static TF is published to avoid a TF multiple-parent conflict.'),
        DeclareLaunchArgument('publish_motion_frame_topic', default_value='false'),
        DeclareLaunchArgument('publish_map_to_motion_tf', default_value='false'),
        DeclareLaunchArgument('publish_odom_to_fastlio_world_tf', default_value='true'),
        DeclareLaunchArgument('publish_body_to_base_tf', default_value='true'),
        DeclareLaunchArgument('body_to_base_x', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_y', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_z', default_value='0.0'),
        # GO2 MID360 is documented as approximately 6 degrees about Y from the robot body frame.
        # Flip the sign of qy if your actual mount uses the opposite pitch direction.
        DeclareLaunchArgument('body_to_base_qx', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_qy', default_value='0.0523359562'),
        DeclareLaunchArgument('body_to_base_qz', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_qw', default_value='0.9986295348'),
        static_tf_odom_to_fastlio_world,
        static_tf_fastlio_body_to_base,
        global_localization_node,
    ])
