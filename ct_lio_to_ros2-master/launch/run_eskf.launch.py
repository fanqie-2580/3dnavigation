from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Prefer package share config so `ros2 launch ct_lio ...` always picks up the
    # rebuilt mapping.yaml (Ry(-13°) + auto ground leveling). Fall back to the
    # source-tree config next to this launch file when share is unavailable.
    ct_lio_dir = get_package_share_directory("ct_lio")
    share_config = os.path.join(ct_lio_dir, 'config', 'mapping.yaml')
    source_config = os.path.abspath(os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'mapping.yaml'))
    config_path = share_config if os.path.isfile(share_config) else source_config

    project = LaunchConfiguration('project')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration("params_file")
    rviz_use = LaunchConfiguration('rviz_use')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    
    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=config_path,
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz_use", default_value="True", description="Flag to launch RViz."
    )
    
    declare_project_cmd = DeclareLaunchArgument(
        'project',
        default_value='ct_lio',
        description='Project name'
    )
    
    # 主要的ESKF节点
    eskf_node = Node(
        package=project,
        executable=[project, '_eskf'],
        name=[project],
        output='screen',
        emulate_tty=True,
        respawn=False,
        respawn_delay=0.0,
        parameters=[configured_params]
    )

    start_rviz_node = Node(
        condition=IfCondition(rviz_use),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=namespace,
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([ct_lio_dir, "rviz_cfg", "loam_livox.rviz"]),
        ],
    )
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_project_cmd)
    
    # 添加节点
    ld.add_action(eskf_node)
    ld.add_action(start_rviz_node)
    
    return ld
