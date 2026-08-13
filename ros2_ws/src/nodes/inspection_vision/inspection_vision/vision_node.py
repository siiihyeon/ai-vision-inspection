"""VisionNode의 통신 실행 골격입니다.

아직 카메라 SDK, 이미지 처리, 추론 작업은 구현하지 않습니다.
"""

import rclpy

from inspection_common import InspectionNodeBase, NodeId, spin_node


class VisionNode(InspectionNodeBase):
    """이미지 취득과 추론 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.VISION, provides_initialize_action=True)
        self.get_logger().info("VisionNode skeleton started")


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 VisionNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = VisionNode()
    spin_node(node)


if __name__ == "__main__":
    main()
