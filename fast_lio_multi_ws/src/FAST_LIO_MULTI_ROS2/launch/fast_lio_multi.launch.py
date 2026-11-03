#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value='bundle',
        description='Update method: bundle, async, adaptive'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='multi.yaml',
        description='Config YAML file (multi.yaml, ntu_viral.yaml, hilti2021.yaml, ...)'
    )

    update_method = LaunchConfiguration('update_method')
    config = LaunchConfiguration('config')

    config_file = PathJoinSubstitution([
        FindPackageShare('fast_lio_multi'),
        'config',
        config,
    ])

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link_tf",
        arguments=[
            "-0.6", "0.228", "-0.35",        # xyz (meters)
            "0.0015495", "-0.0005015", "-0.005389",        # rpy (radians)
            "body",               # parent frame
            "base_link"           # child frame
        ]
    )

    def choose_node(context, *args, **kwargs):
        method = context.launch_configurations['update_method']

        if method == 'bundle':
            exe = 'laserMapping_bundle'
        elif method == 'async':
            exe = 'laserMapping_async'
        elif method == 'adaptive':
            exe = 'laserMapping_adaptive'
        else:
            raise ValueError(f"Unknown update_method: {method}")

        lidar_node = Node(
            package='fast_lio_multi',
            executable=exe,
            name='laserMapping_multi',
            output='screen',
            parameters=[config_file]
        )

        return [lidar_node]

    lidar_node_action = OpaqueFunction(function=choose_node)

    fast_lio_to_wheelchair_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('pc_utils'),
                'launch',
                'fast_lio_to_wheelchair_odom.launch.py',
            ])
        )
    )   

    return LaunchDescription([
        update_method_arg,
        config_arg,
        static_tf,
        lidar_node_action,
        fast_lio_to_wheelchair_launch
    ])
