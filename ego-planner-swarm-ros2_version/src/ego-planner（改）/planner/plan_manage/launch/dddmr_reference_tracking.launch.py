import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ego_share = get_package_share_directory('ego_planner')

    drone_id = LaunchConfiguration('drone_id')
    odom_topic = LaunchConfiguration('odom_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    camera_pose_topic = LaunchConfiguration('camera_pose_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    reference_path_topic = LaunchConfiguration('reference_path_topic')
    reference_path_z_offset = LaunchConfiguration('reference_path_z_offset')
    reference_path_start_skip_dist = LaunchConfiguration('reference_path_start_skip_dist')
    emergency_time = LaunchConfiguration('emergency_time')
    continuous_replan = LaunchConfiguration('continuous_replan')
    exec_interval_ms = LaunchConfiguration('exec_interval_ms')
    safety_interval_ms = LaunchConfiguration('safety_interval_ms')
    execution_state_timeout = LaunchConfiguration('execution_state_timeout')
    odom_z_offset = LaunchConfiguration('odom_z_offset')
    grid_map_odom_z_offset = LaunchConfiguration('grid_map_odom_z_offset')
    frame_id = LaunchConfiguration('frame_id')
    base_frame = LaunchConfiguration('base_frame')
    cmd_topic = LaunchConfiguration('cmd_topic')
    kp_pos_x = LaunchConfiguration('kp_pos_x')
    kp_pos_y = LaunchConfiguration('kp_pos_y')
    kp_yaw = LaunchConfiguration('kp_yaw')
    finish_distance = LaunchConfiguration('finish_distance')
    yaw_lookahead_time = LaunchConfiguration('yaw_lookahead_time')
    heading_error_threshold = LaunchConfiguration('heading_error_threshold')
    controller_max_vx = LaunchConfiguration('controller_max_vx')
    max_vy = LaunchConfiguration('max_vy')
    max_wz = LaunchConfiguration('max_wz')
    cmd_interval_ms = LaunchConfiguration('cmd_interval_ms')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_depth_fusion_input = LaunchConfiguration('enable_depth_fusion_input')
    pose_type = LaunchConfiguration('pose_type')

    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')
    max_vel = LaunchConfiguration('max_vel')
    max_acc = LaunchConfiguration('max_acc')
    planning_horizon = LaunchConfiguration('planning_horizon')
    obj_num = LaunchConfiguration('obj_num')
    use_distinctive_trajs = LaunchConfiguration('use_distinctive_trajs')
    resolution = LaunchConfiguration('resolution')
    local_update_range_x = LaunchConfiguration('local_update_range_x')
    local_update_range_y = LaunchConfiguration('local_update_range_y')
    local_update_range_z = LaunchConfiguration('local_update_range_z')
    obstacles_inflation = LaunchConfiguration('obstacles_inflation')
    footprint_radius = LaunchConfiguration('footprint_radius')
    footprint_offset = LaunchConfiguration('footprint_offset')
    footprint_z_down = LaunchConfiguration('footprint_z_down')
    footprint_z_up = LaunchConfiguration('footprint_z_up')
    local_map_margin = LaunchConfiguration('local_map_margin')
    ground_height = LaunchConfiguration('ground_height')
    min_ray_length = LaunchConfiguration('min_ray_length')
    max_ray_length = LaunchConfiguration('max_ray_length')
    map_sliding_en = LaunchConfiguration('map_sliding_en')
    map_sliding_thresh_vox = LaunchConfiguration('map_sliding_thresh_vox')
    sensor_frame = LaunchConfiguration('sensor_frame')
    sensor_origin_offset_x = LaunchConfiguration('sensor_origin_offset_x')
    sensor_origin_offset_y = LaunchConfiguration('sensor_origin_offset_y')
    sensor_origin_offset_z = LaunchConfiguration('sensor_origin_offset_z')
    virtual_ceil_height = LaunchConfiguration('virtual_ceil_height')
    vis_height = LaunchConfiguration('vis_height')
    pointcloud_vertical_inflation = LaunchConfiguration('pointcloud_vertical_inflation')
    pointcloud_vertical_inflation_down = LaunchConfiguration('pointcloud_vertical_inflation_down')
    project_pointcloud_to_ground = LaunchConfiguration('project_pointcloud_to_ground')
    dist0 = LaunchConfiguration('dist0')

    ego_planner_node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', drone_id, '_ego_planner_node'],
        output='screen',
        remappings=[
            ('odom_world', odom_topic),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/trajectory_execution_state', ['drone_', drone_id, '_planning/trajectory_execution_state']),
            ('planning/data_display', ['drone_', drone_id, '_planning/data_display']),
            ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
            ('goal_point', ['drone_', drone_id, '_plan_vis/goal_point']),
            ('global_list', ['drone_', drone_id, '_plan_vis/global_list']),
            ('init_list', ['drone_', drone_id, '_plan_vis/init_list']),
            ('optimal_list', ['drone_', drone_id, '_plan_vis/optimal_list']),
            ('a_star_list', ['drone_', drone_id, '_plan_vis/a_star_list']),
            ('grid_map/odom', odom_topic),
            ('grid_map/cloud', cloud_topic),
            ('grid_map/pose', camera_pose_topic),
            ('grid_map/depth', depth_topic),
            ('grid_map/occupancy_inflate', ['drone_', drone_id, '_grid/grid_map/occupancy_inflate'])
        ],
        parameters=[
            {'fsm/flight_type': 3},
            {'fsm/thresh_replan_time': 1.0},
            {'fsm/thresh_no_replan_meter': 0.1},
            {'fsm/planning_horizon': ParameterValue(planning_horizon, value_type=float)},
            {'fsm/planning_horizen_time': 3.0},
            {'fsm/emergency_time': ParameterValue(emergency_time, value_type=float)},
            {'fsm/realworld_experiment': False},
            {'fsm/fail_safe': True},
            {'fsm/continuous_replan': ParameterValue(continuous_replan, value_type=bool)},
            {'fsm/reference_path_topic': reference_path_topic},
            {'fsm/reference_path_z_offset': ParameterValue(reference_path_z_offset, value_type=float)},
            {'fsm/reference_path_start_skip_dist': ParameterValue(reference_path_start_skip_dist, value_type=float)},
            {'fsm/reference_path_sample_spacing': 1.0},
            {'fsm/odom_world_frame': frame_id},
            {'fsm/odom_z_offset': ParameterValue(odom_z_offset, value_type=float)},
            {'fsm/replan_distance': 1.0},
            {'fsm/exec_interval_ms': ParameterValue(exec_interval_ms, value_type=int)},
            {'fsm/safety_interval_ms': ParameterValue(safety_interval_ms, value_type=int)},
            {'fsm/max_replan_failures': 20},
            {'fsm/execution_state_timeout': ParameterValue(execution_state_timeout, value_type=float)},
            {'fsm/waypoint_num': 0},
            {'grid_map/resolution': ParameterValue(resolution, value_type=float)},
            {'grid_map/map_size_x': ParameterValue(map_size_x, value_type=float)},
            {'grid_map/map_size_y': ParameterValue(map_size_y, value_type=float)},
            {'grid_map/map_size_z': ParameterValue(map_size_z, value_type=float)},
            {'grid_map/local_update_range_x': ParameterValue(local_update_range_x, value_type=float)},
            {'grid_map/local_update_range_y': ParameterValue(local_update_range_y, value_type=float)},
            {'grid_map/local_update_range_z': ParameterValue(local_update_range_z, value_type=float)},
            {'grid_map/obstacles_inflation': ParameterValue(obstacles_inflation, value_type=float)},
            {'grid_map/footprint_radius': ParameterValue(footprint_radius, value_type=float)},
            {'grid_map/footprint_offset': ParameterValue(footprint_offset, value_type=float)},
            {'grid_map/footprint_z_down': ParameterValue(footprint_z_down, value_type=float)},
            {'grid_map/footprint_z_up': ParameterValue(footprint_z_up, value_type=float)},
            {'grid_map/local_map_margin': ParameterValue(local_map_margin, value_type=int)},
            {'grid_map/ground_height': ParameterValue(ground_height, value_type=float)},
            {'grid_map/cx': ParameterValue(cx, value_type=float)},
            {'grid_map/cy': ParameterValue(cy, value_type=float)},
            {'grid_map/fx': ParameterValue(fx, value_type=float)},
            {'grid_map/fy': ParameterValue(fy, value_type=float)},
            {'grid_map/use_depth_filter': True},
            {'grid_map/depth_filter_tolerance': 0.15},
            {'grid_map/depth_filter_maxdist': 3.0},
            {'grid_map/depth_filter_mindist': 0.3},
            {'grid_map/depth_filter_margin': 1},
            {'grid_map/k_depth_scaling_factor': 1000.0},
            {'grid_map/skip_pixel': 2},
            {'grid_map/p_hit': 0.85},
            {'grid_map/p_miss': 0.30},
            {'grid_map/p_min': 0.12},
            {'grid_map/p_max': 0.98},
            {'grid_map/p_occ': 0.80},
            {'grid_map/min_ray_length': ParameterValue(min_ray_length, value_type=float)},
            {'grid_map/max_ray_length': ParameterValue(max_ray_length, value_type=float)},
            {'grid_map/odom_z_offset': ParameterValue(grid_map_odom_z_offset, value_type=float)},
            {'grid_map/pointcloud_vertical_inflation': ParameterValue(pointcloud_vertical_inflation, value_type=float)},
            {'grid_map/pointcloud_vertical_inflation_down': ParameterValue(pointcloud_vertical_inflation_down, value_type=float)},
            {'grid_map/map_sliding_en': ParameterValue(map_sliding_en, value_type=bool)},
            {'grid_map/map_sliding_thresh_vox': ParameterValue(map_sliding_thresh_vox, value_type=int)},
            {'grid_map/sensor_frame': sensor_frame},
            {'grid_map/sensor_origin_offset_x': ParameterValue(sensor_origin_offset_x, value_type=float)},
            {'grid_map/sensor_origin_offset_y': ParameterValue(sensor_origin_offset_y, value_type=float)},
            {'grid_map/sensor_origin_offset_z': ParameterValue(sensor_origin_offset_z, value_type=float)},
            {'grid_map/project_pointcloud_to_ground': ParameterValue(project_pointcloud_to_ground, value_type=bool)},
            {'grid_map/virtual_ceil_height': ParameterValue(virtual_ceil_height, value_type=float)},
            {'grid_map/vis_height': ParameterValue(vis_height, value_type=float)},
            {'grid_map/show_occ_time': False},
            {'grid_map/pose_type': ParameterValue(pose_type, value_type=int)},
            {'grid_map/enable_depth_fusion_input': ParameterValue(enable_depth_fusion_input, value_type=bool)},
            {'grid_map/frame_id': frame_id},
            {'manager/max_vel': ParameterValue(max_vel, value_type=float)},
            {'manager/max_acc': ParameterValue(max_acc, value_type=float)},
            {'manager/max_jerk': 4.0},
            {'manager/control_points_distance': 0.2},
            {'manager/feasibility_tolerance': 0.5},
            {'manager/planning_horizon': ParameterValue(planning_horizon, value_type=float)},
            {'manager/use_distinctive_trajs': ParameterValue(use_distinctive_trajs, value_type=bool)},
            {'manager/drone_id': ParameterValue(drone_id, value_type=int)},
            {'manager/verbose_replan_log': False},
            {'optimization/lambda_smooth': 1.0},
            {'optimization/lambda_collision': 1.0},
            {'optimization/lambda_feasibility': 0.1},
            {'optimization/lambda_fitness': 1.0},
            {'optimization/dist0': ParameterValue(dist0, value_type=float)},
            {'optimization/swarm_clearance': 0.5},
            {'optimization/max_vel': ParameterValue(max_vel, value_type=float)},
            {'optimization/vel_tolerance': 1.0},
            {'optimization/max_acc': ParameterValue(max_acc, value_type=float)},
            {'optimization/acc_tolerance': 1.0},
            {'optimization/verbose_rebound_log': False},
            {'bspline/limit_vel': ParameterValue(max_vel, value_type=float)},
            {'bspline/limit_acc': ParameterValue(max_acc, value_type=float)},
            {'bspline/limit_ratio': 1.1},
            {'prediction/obj_num': ParameterValue(obj_num, value_type=int)},
            {'prediction/lambda': 1.0},
            {'prediction/predict_rate': 1.0}
        ]
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=['drone_', drone_id, '_traj_server'],
        output='screen',
        remappings=[
            ('position_cmd', ['drone_', drone_id, '_planning/pos_cmd']),
            ('planning/bspline', ['drone_', drone_id, '_planning/bspline']),
            ('planning/trajectory_execution_state', ['drone_', drone_id, '_planning/trajectory_execution_state'])
        ],
        parameters=[
            {'traj_server/world_frame': frame_id},
            {'traj_server/base_frame': base_frame},
            {'traj_server/odometry_topic': odom_topic},
            {'traj_server/cmd_topic': cmd_topic},
            {'traj_server/control_point_topic': '/control_point_state_'},
            {'traj_server/execution_state_topic': 'planning/trajectory_execution_state'},
            {'traj_server/kp_pos_x': ParameterValue(kp_pos_x, value_type=float)},
            {'traj_server/kp_pos_y': ParameterValue(kp_pos_y, value_type=float)},
            {'traj_server/kp_yaw': ParameterValue(kp_yaw, value_type=float)},
            {'traj_server/finish_distance': ParameterValue(finish_distance, value_type=float)},
            {'traj_server/yaw_lookahead_time': ParameterValue(yaw_lookahead_time, value_type=float)},
            {'traj_server/heading_error_threshold': ParameterValue(heading_error_threshold, value_type=float)},
            {'traj_server/max_vx': ParameterValue(controller_max_vx, value_type=float)},
            {'traj_server/max_vy': ParameterValue(max_vy, value_type=float)},
            {'traj_server/max_wz': ParameterValue(max_wz, value_type=float)},
            {'traj_server/cmd_interval_ms': ParameterValue(cmd_interval_ms, value_type=int)}
        ]
    )

    # Go2 real-robot layout: Fixed Frame=map, /baselink2map, /cloud_registered_1,
    # /drone_0_grid/...  (default.rviz is for sim and uses Fixed Frame=world).
    rviz_config_path = os.path.join(ego_share, 'launch', 'go2_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='0', description='Planner instance id'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/baselink2map',
            description='Odometry topic published by the DDDMR localization stack'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/cloud_registered_1',
            description='Complete FAST-LIO registered scan used for SCAN-style hit/miss raycasting'),
        DeclareLaunchArgument(
            'camera_pose_topic',
            default_value='/camera_pose',
            description='Unused in pointcloud-only mode; kept for compatibility'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/depth_image',
            description='Unused in pointcloud-only mode; kept for compatibility'),
        DeclareLaunchArgument(
            'reference_path_topic',
            default_value='/ego_repaired_global_path',
            description='DDDMR repaired global planner output path topic for EGO tracking'),
        DeclareLaunchArgument(
            'reference_path_z_offset',
            default_value='0.4',
            description='SCAN body_height equivalent: lift DDDMR ground-path z above Mid360 floor hits so inflate/collision checks run at body height'),
        DeclareLaunchArgument(
            'reference_path_start_skip_dist',
            default_value='0.2',
            description='Ignore near-duplicate reference poses very close to odom before fitting the EGO global guide'),
        DeclareLaunchArgument(
            'emergency_time',
            default_value='1.0',
            description='Emergency-stop trigger horizon when a collision remains unresolved after replanning'),
        DeclareLaunchArgument(
            'continuous_replan',
            default_value='true',
            description='SCAN-style: after leaving the traj start by replan_distance, keep replanning'),
        DeclareLaunchArgument(
            'exec_interval_ms',
            default_value='10',
            description='Main FSM execution period in milliseconds; matches SCAN'),
        DeclareLaunchArgument(
            'safety_interval_ms',
            default_value='50',
            description='Safety collision-check period in milliseconds; matches SCAN'),
        DeclareLaunchArgument(
            'execution_state_timeout',
            default_value='0.30',
            description='Maximum accepted age of controller execution-state feedback'),
        DeclareLaunchArgument(
            'odom_z_offset',
            default_value='0.0',
            description='Extra offset added to the EGO planning state z. Keep 0.0 when /baselink2map already reports the robot planning frame height'),
        DeclareLaunchArgument(
            'grid_map_odom_z_offset',
            default_value='0.0',
            description='Local map odom height offset. Keep 0.0 to match the original 3d-navi grid_map semantics for pointcloud integration'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Shared global planning frame'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot body frame used when converting world velocity to /cmd_vel'),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/cmd_vel',
            description='Velocity topic published by EGO to the robot'),
        DeclareLaunchArgument('kp_pos_x', default_value='0.8', description='World-frame X position feedback gain'),
        DeclareLaunchArgument('kp_pos_y', default_value='0.8', description='World-frame Y position feedback gain; matches SCAN'),
        DeclareLaunchArgument('kp_yaw', default_value='1.5', description='Yaw proportional feedback gain'),
        DeclareLaunchArgument('finish_distance', default_value='0.15', description='Planar error required before holding the end of a local trajectory; matches SCAN'),
        DeclareLaunchArgument('yaw_lookahead_time', default_value='0.8', description='Trajectory lookahead used to derive desired planar heading; matches SCAN'),
        DeclareLaunchArgument('heading_error_threshold', default_value='0.8', description='Yaw error above which trajectory time freezes while rotating; matches SCAN'),
        DeclareLaunchArgument('controller_max_vx', default_value='0.75', description='Maximum forward body-frame command; independent of the EGO planning velocity limit'),
        DeclareLaunchArgument('max_vy', default_value='0.20', description='Maximum lateral body-frame speed'),
        DeclareLaunchArgument('max_wz', default_value='1.0', description='Maximum yaw-rate command; matches SCAN'),
        DeclareLaunchArgument(
            'cmd_interval_ms',
            default_value='10',
            description='Command and execution-state publish period in milliseconds; matches SCAN'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch the ego planner RViz config'),
        DeclareLaunchArgument(
            'enable_depth_fusion_input',
            default_value='false',
            description='Keep false for DDDMR pointcloud-only integration'),
        DeclareLaunchArgument(
            'pose_type',
            default_value='2',
            description='Pose type kept for compatibility; odom is used in pointcloud-only mode'),
        DeclareLaunchArgument('map_size_x', default_value='10.0', description='SCAN-style rolling local map size along X'),
        DeclareLaunchArgument('map_size_y', default_value='10.0', description='SCAN-style rolling local map size along Y'),
        DeclareLaunchArgument('map_size_z', default_value='5.0', description='Local map size along Z; matches SCAN'),
        DeclareLaunchArgument('resolution', default_value='0.05', description='Occupancy map resolution'),
        DeclareLaunchArgument('local_update_range_x', default_value='5.0', description='SCAN local map update half-range along X'),
        DeclareLaunchArgument('local_update_range_y', default_value='5.0', description='SCAN local map update half-range along Y'),
        DeclareLaunchArgument('local_update_range_z', default_value='2.5', description='SCAN local map update half-range along Z'),
        DeclareLaunchArgument('obstacles_inflation', default_value='0.18', description='XY inflation; slightly under footprint so local A* can still find detours'),
        DeclareLaunchArgument('footprint_radius', default_value='0.25', description='Go2 twin-cylinder radius; matches SCAN double_cylinder_radius'),
        DeclareLaunchArgument('footprint_offset', default_value='0.18', description='Forward and rearward offset of the two Go2 footprint cylinders'),
        DeclareLaunchArgument('footprint_z_down', default_value='0.18', description='Collision volume below the body-frame trajectory height; excludes the leg support region'),
        DeclareLaunchArgument('footprint_z_up', default_value='0.22', description='Collision volume above the body-frame trajectory height'),
        DeclareLaunchArgument('local_map_margin', default_value='10', description='Voxel margin used when clearing the rolling local map'),
        DeclareLaunchArgument('ground_height', default_value='0.0', description='Ground height of the occupancy map; matches SCAN'),
        DeclareLaunchArgument('min_ray_length', default_value='0.0', description='SCAN does not discard valid near-range LiDAR returns'),
        DeclareLaunchArgument('max_ray_length', default_value='5.0', description='Ignore pointcloud returns farther than this distance from the robot'),
        DeclareLaunchArgument('map_sliding_en', default_value='true', description='Enable SCAN-style circular sliding occupancy map'),
        DeclareLaunchArgument('map_sliding_thresh_vox', default_value='4', description='SCAN 0.2 m circular-map shift threshold at 0.05 m resolution'),
        DeclareLaunchArgument('sensor_frame', default_value='body', description='FAST-LIO pose frame used with the calibrated MID360 origin offset'),
        DeclareLaunchArgument('sensor_origin_offset_x', default_value='-0.011', description='MID360 LiDAR origin X in FAST-LIO body frame'),
        DeclareLaunchArgument('sensor_origin_offset_y', default_value='-0.02329', description='MID360 LiDAR origin Y in FAST-LIO body frame'),
        DeclareLaunchArgument('sensor_origin_offset_z', default_value='0.04412', description='MID360 LiDAR origin Z in FAST-LIO body frame'),
        DeclareLaunchArgument('pointcloud_vertical_inflation', default_value='0.10', description='SCAN obstacles_inflation_z_up=0.1'),
        DeclareLaunchArgument('pointcloud_vertical_inflation_down', default_value='0.10', description='SCAN obstacles_inflation_z_down=0.1'),
        DeclareLaunchArgument('project_pointcloud_to_ground', default_value='false', description='Keep false so stairs remain traversable in the 3D local map'),
        DeclareLaunchArgument('dist0', default_value='0.2', description='Soft collision clearance; matches SCAN optimization/dist0'),
        DeclareLaunchArgument('virtual_ceil_height', default_value='-1.0', description='Disable the virtual ceiling to match SCAN map behavior'),
        DeclareLaunchArgument('vis_height', default_value='0.3', description='Display occupancy up to this height above the current sensor pose; matches SCAN'),
        DeclareLaunchArgument('cx', default_value='321.04638671875', description='Camera intrinsic cx'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094', description='Camera intrinsic cy'),
        DeclareLaunchArgument('fx', default_value='387.229248046875', description='Camera intrinsic fx'),
        DeclareLaunchArgument('fy', default_value='387.229248046875', description='Camera intrinsic fy'),
        DeclareLaunchArgument('max_vel', default_value='0.50', description='Maximum tracking velocity'),
        DeclareLaunchArgument('max_acc', default_value='0.5', description='Maximum tracking acceleration; matches SCAN'),
        DeclareLaunchArgument('planning_horizon', default_value='3.5', description='Local planning horizon; latest SCAN advanced_param.xml uses 3.5 (was 7.5)'),
        DeclareLaunchArgument('obj_num', default_value='10', description='Predicted dynamic object count'),
        DeclareLaunchArgument('use_distinctive_trajs', default_value='false', description='Enable distinctive trajectory generation'),
        ego_planner_node,
        traj_server_node,
        rviz_node,
    ])
