from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")
    config_path = Path(
        get_package_share_directory("inspection_bringup")
    ) / "config" / "sim.yaml"
    parameters = [str(config_path)]
    namespace = "inspection"
    simulator_parameters = [
        *parameters,
        {"vision_sim.station_a_verdict": LaunchConfiguration("station_a_verdict")},
        {"vision_sim.station_b_verdict": LaunchConfiguration("station_b_verdict")},
        {"vision_sim.station_a_score": LaunchConfiguration("station_a_score")},
        {"vision_sim.station_b_score": LaunchConfiguration("station_b_score")},
        {"vision_sim.failure_mode": LaunchConfiguration("failure_mode")},
        {"vision_sim.result_delay_ms": LaunchConfiguration("result_delay_ms")},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="sim",
                choices=["sim"],
                description="Run the inspection stack with the Vision simulator",
            ),
            DeclareLaunchArgument("station_a_verdict", default_value="PASS"),
            DeclareLaunchArgument("station_b_verdict", default_value="PASS"),
            DeclareLaunchArgument("station_a_score", default_value="0.10"),
            DeclareLaunchArgument("station_b_score", default_value="0.10"),
            DeclareLaunchArgument("failure_mode", default_value="none"),
            DeclareLaunchArgument("result_delay_ms", default_value="100"),
            Node(
                package="inspection_master",
                executable="master_node",
                name="master_node",
                namespace=namespace,
                output="screen",
                parameters=simulator_parameters,
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
                parameters=parameters,
            ),
            Node(
                package="inspection_log",
                executable="log_node",
                name="log_node",
                namespace=namespace,
                output="screen",
                parameters=parameters,
            ),
        ]
    )
