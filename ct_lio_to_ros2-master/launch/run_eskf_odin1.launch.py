from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ct_lio_dir = get_package_share_directory("ct_lio")
    share_config = os.path.join(ct_lio_dir, "config", "mapping_odin1.yaml")
    source_config = os.path.abspath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "mapping_odin1.yaml")
    )
    # Prefer source-tree yaml so edits take effect even when install/ is a stale copy.
    config_path = source_config if os.path.isfile(source_config) else share_config

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    rviz_use = LaunchConfiguration("rviz_use")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if true"
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=config_path,
        description="Full path to the ROS2 parameters file",
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz_use", default_value="True", description="Flag to launch RViz"
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    eskf_node = Node(
        package="ct_lio",
        executable="ct_lio_eskf",
        name="ct_lio",
        output="screen",
        emulate_tty=True,
        parameters=[configured_params],
    )

    start_rviz_node = Node(
        condition=IfCondition(rviz_use),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=namespace,
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        arguments=["-d", PathJoinSubstitution([ct_lio_dir, "rviz_cfg", "loam_livox.rviz"])],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(eskf_node)
    ld.add_action(start_rviz_node)
    return ld
