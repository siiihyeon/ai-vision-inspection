"""네 검사 노드를 공통 프로필로 실행하는 Launch 파일입니다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context: LaunchContext) -> list[Node]:
    """선택한 프로필과 같은 이름의 YAML을 네 노드에 전달합니다."""

    profile = LaunchConfiguration("profile").perform(context)
    config_path = (
        Path(get_package_share_directory("inspection_bringup"))
        / "config"
        / f"{profile}.yaml"
    )
    if not config_path.is_file():
        raise RuntimeError(f"unknown inspection profile: {profile}")

    parameters = [str(config_path)]
    vision_fragments = [
        Path(get_package_share_directory("inspection_bringup"))
        / "config"
        / f"vision_{section}.{profile}.yaml"
        for section in ("capture", "model", "runtime")
    ]
    if any(not path.is_file() for path in vision_fragments):
        raise RuntimeError(f"Vision profile fragments are incomplete: {profile}")
    vision_parameters = [str(config_path), *map(str, vision_fragments)]
    return [
        Node(
            package="inspection_master",
            executable="master_node",
            name="master_node",
            namespace="inspection",
            output="screen",
            parameters=parameters,
        ),
        Node(
            package="inspection_control",
            executable="control_node",
            name="control_node",
            namespace="inspection",
            output="screen",
            parameters=parameters,
        ),
        Node(
            package="inspection_vision",
            executable="vision_node",
            name="vision_node",
            namespace="inspection",
            output="screen",
            parameters=vision_parameters,
        ),
        Node(
            package="inspection_log",
            executable="log_node",
            name="log_node",
            namespace="inspection",
            output="screen",
            parameters=parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """실행 프로필을 선언하고 네 노드를 구성합니다."""

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="hardware",
                choices=["sim", "hardware"],
                description="장비 없는 개발용 sim 또는 실제 장비용 hardware",
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
