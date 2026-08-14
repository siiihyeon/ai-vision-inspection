"""LogNode의 통신 실행 골격입니다.

아직 SQLite, 이미지 경로, spool 저장은 구현하지 않습니다.
"""

import rclpy

from inspection_common import (
    InspectionNodeBase,
    NodeId,
    NodeInitializationOutcome,
    spin_node,
)


class LogNode(InspectionNodeBase):
    """영구 로그 저장 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.LOG, provides_initialize_action=True)
        self.get_logger().info("LogNode skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        """실제 저장 환경에 필요한 ROS 파라미터 키를 반환합니다."""

        # TODO(HARDWARE_REQUIRED): DB·이미지·spool·용량 설정 키를 확정합니다.
        return tuple(super().required_hardware_parameters())

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        """SQLite와 저장소 초기화를 구현할 전용 확장 지점입니다."""

        # TODO(IMPLEMENTATION): DB 연결·스키마·저장 경로·용량을 검증합니다.
        return await super().initialize_node_resources()


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 LogNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = LogNode()
    spin_node(node)


if __name__ == "__main__":
    main()
