from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LOCAL_LOG_ROOT = "/tmp/inspection"


def _launch_nodes(context: LaunchContext) -> list[Node]:
    """Run Master/Control/Log on the chosen profile config, Vision always simulated."""

    profile = LaunchConfiguration("profile").perform(context)
    config_path = (
        Path(get_package_share_directory("inspection_bringup"))
        / "config"
        / f"{profile}.yaml"
    )
    parameters = [str(config_path)]
    namespace = "inspection"

    master_parameters = [
        *parameters,
        {"master.log_spool_path": f"{_LOCAL_LOG_ROOT}/spool/master.sqlite3"},
    ]
    log_parameters = [
        *parameters,
        {
            "log.database_path": f"{_LOCAL_LOG_ROOT}/log/inspection.sqlite3",
            "log.data_root": _LOCAL_LOG_ROOT,
            "log.report_root": f"{_LOCAL_LOG_ROOT}/reports",
            "log.timeout_tuning.generated_path": f"{_LOCAL_LOG_ROOT}/config/vision_timeout_tuning.json",
        },
    ]
    simulator_parameters = [
        *parameters,
        {"vision_sim.ng_threshold": LaunchConfiguration("ng_threshold")},
        {"vision_sim.failure_mode": LaunchConfiguration("failure_mode")},
        {"vision_sim.result_delay_ms": LaunchConfiguration("result_delay_ms")},
    ]

    return [
        Node(
            package="inspection_master",
            executable="master_node",
            name="master_node",
            namespace=namespace,
            output="screen",
            parameters=master_parameters,
        ),
        Node(
            package="inspection_control",
            executable="control_node",
            name="control_node",
            namespace=namespace,
            output="screen",
            parameters=parameters,
        ),
        Node(
            package="node_inspection_simulator",
            executable="vision_simulator_node",
            name="vision_node",
            namespace=namespace,
            output="screen",
            parameters=simulator_parameters,
        ),
        Node(
            package="inspection_log",
            executable="log_node",
            name="log_node",
            namespace=namespace,
            output="screen",
            parameters=log_parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim",
                choices=["sim", "hardware"],
                description=(
                    "sim: Master/Control/Log/Vision all fake. "
                    "hardware: real Control (conveyor/Mega), Vision still simulated."
                ),
            ),
            DeclareLaunchArgument("ng_threshold", default_value="0.5"),
            DeclareLaunchArgument("failure_mode", default_value="none"),
            DeclareLaunchArgument("result_delay_ms", default_value="100"),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
