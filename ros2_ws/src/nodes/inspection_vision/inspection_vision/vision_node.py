"""VisionNode의 통신 실행 골격입니다.

아직 카메라 SDK, 이미지 처리, 추론 작업은 구현하지 않습니다.
"""

import rclpy

from inspection_common import (
    InspectionNodeBase,
    NodeId,
    NodeInitializationOutcome,
    spin_node,
)


class VisionNode(InspectionNodeBase):
    """이미지 취득과 추론 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.VISION, provides_initialize_action=True)
        self.get_logger().info("VisionNode skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        """실제 Vision 장비에 필요한 ROS 파라미터 키를 반환합니다."""

        # TODO(HARDWARE_REQUIRED): 카메라·조명·모델·GPU 설정 키를 확정합니다.
        return tuple(super().required_hardware_parameters())

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        """카메라·모델·추론 워커 초기화를 구현할 전용 확장 지점입니다."""

        # TODO(IMPLEMENTATION): 카메라 연결·모델 로드·추론 큐 준비를 검증합니다.
        return await super().initialize_node_resources()


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 VisionNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = VisionNode()
    spin_node(node)


if __name__ == "__main__":
    main()
