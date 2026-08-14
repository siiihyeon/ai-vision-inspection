"""ControlNode의 통신 실행 골격입니다.

아직 Mega, 센서, 컨베이어, 액추에이터 제어는 구현하지 않습니다.
"""

import rclpy

from inspection_common import (
    InspectionNodeBase,
    NodeId,
    NodeInitializationOutcome,
    spin_node,
)


class ControlNode(InspectionNodeBase):
    """물리 장비 제어 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.CONTROL, provides_initialize_action=True)
        self.get_logger().info("ControlNode skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        """실제 Control 장비에 필요한 ROS 파라미터 키를 반환합니다."""

        # TODO(HARDWARE_REQUIRED): Mega·모터·센서·액추에이터 설정 키를 확정합니다.
        return tuple(super().required_hardware_parameters())

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        """Mega 연결과 안전 출력 검증을 구현할 전용 확장 지점입니다."""

        # TODO(IMPLEMENTATION): Mega 연결·통신 확인·안전 기본 출력을 검증합니다.
        return await super().initialize_node_resources()


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 ControlNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = ControlNode()
    spin_node(node)


if __name__ == "__main__":
    main()
