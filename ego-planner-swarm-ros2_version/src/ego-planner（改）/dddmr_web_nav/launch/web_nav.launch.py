import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("dddmr_web_nav")
    web_root = os.path.join(pkg_share, "web")

    http_port = LaunchConfiguration("http_port")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    launch_rosbridge = LaunchConfiguration("launch_rosbridge")
    use_sim_time = LaunchConfiguration("use_sim_time")
    frame_id = LaunchConfiguration("frame_id")

    mapground_topic = LaunchConfiguration("mapground_topic")
    web_map_topic = LaunchConfiguration("web_map_topic")
    ego_obstacle_topic = LaunchConfiguration("ego_obstacle_topic")
    web_ego_obstacle_topic = LaunchConfiguration("web_ego_obstacle_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    path_topic = LaunchConfiguration("path_topic")
    clicked_point_topic = LaunchConfiguration("clicked_point_topic")
    estop_topic = LaunchConfiguration("estop_topic")

    web_map_downsampler = Node(
        package="dddmr_web_nav",
        executable="web_map_downsampler.py",
        name="web_map_downsampler",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": mapground_topic},
            {"output_topic": web_map_topic},
            {"target_frame": frame_id},
            {"publish_period_sec": 1.0},
            {"sample_stride": 2},
            {"max_points": 25000},
            {"voxel_size": 0.12},
            {"point_scale": 0.08},
            {"marker_ns": "dddmr_web_mapground"},
            {"input_transient_local": True},
            {"color_r": 0.18},
            {"color_g": 0.95},
            {"color_b": 0.48},
            {"color_a": 0.85},
        ],
    )

    web_ego_obstacle_downsampler = Node(
        package="dddmr_web_nav",
        executable="web_map_downsampler.py",
        name="web_ego_obstacle_downsampler",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": ego_obstacle_topic},
            {"output_topic": web_ego_obstacle_topic},
            {"target_frame": frame_id},
            {"publish_period_sec": 0.5},
            {"sample_stride": 1},
            {"max_points": 40000},
            {"voxel_size": 0.08},
            {"point_scale": 0.10},
            {"marker_ns": "dddmr_web_ego_obstacles"},
            {"input_transient_local": False},
            {"color_r": 1.0},
            {"color_g": 0.34},
            {"color_b": 0.12},
            {"color_a": 0.92},
        ],
    )

    web_status_node = Node(
        package="dddmr_web_nav",
        executable="web_status_node.py",
        name="web_status_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"map_topic": web_map_topic},
            {"ego_obstacle_topic": web_ego_obstacle_topic},
            {"odom_topic": odom_topic},
            {"path_topic": path_topic},
            {"clicked_point_topic": clicked_point_topic},
            {"status_topic": "/web_nav/status"},
            {"alive_timeout_sec": 2.0},
            {"publish_period_sec": 1.0},
        ],
    )

    http_server = ExecuteProcess(
        cmd=[
            "python3",
            "-m",
            "http.server",
            http_port,
            "--bind",
            "0.0.0.0",
            "--directory",
            web_root,
        ],
        output="screen",
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            {"port": ParameterValue(rosbridge_port, value_type=int)},
        ],
        condition=IfCondition(launch_rosbridge),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("http_port", default_value="8080"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("launch_rosbridge", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("mapground_topic", default_value="/mapground"),
            DeclareLaunchArgument("web_map_topic", default_value="/web/mapground"),
            DeclareLaunchArgument(
                "ego_obstacle_topic",
                default_value="/drone_0_grid/grid_map/occupancy_inflate",
            ),
            DeclareLaunchArgument("web_ego_obstacle_topic", default_value="/web/ego_obstacles"),
            DeclareLaunchArgument("odom_topic", default_value="/baselink2map"),
            DeclareLaunchArgument("path_topic", default_value="/global_path"),
            DeclareLaunchArgument("clicked_point_topic", default_value="/clicked_point"),
            DeclareLaunchArgument("estop_topic", default_value="/web_estop"),
            web_map_downsampler,
            web_ego_obstacle_downsampler,
            web_status_node,
            http_server,
            rosbridge_node,
        ]
    )
