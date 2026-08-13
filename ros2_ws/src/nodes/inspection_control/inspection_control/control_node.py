"""ControlNode의 통신 실행 골격입니다.

아직 Mega, 센서, 컨베이어, 액추에이터 제어는 구현하지 않습니다.
"""

import rclpy

from inspection_common import InspectionNodeBase, NodeId, spin_node


class ControlNode(InspectionNodeBase):
    """물리 장비 제어 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.CONTROL, provides_initialize_action=True)
        self.get_logger().info("ControlNode skeleton started")


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 ControlNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = ControlNode()
    spin_node(node)


if __name__ == "__main__":
    main()
