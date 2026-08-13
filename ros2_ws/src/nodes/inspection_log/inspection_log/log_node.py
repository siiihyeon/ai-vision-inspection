"""LogNode의 통신 실행 골격입니다.

아직 SQLite, 이미지 경로, spool 저장은 구현하지 않습니다.
"""

import rclpy

from inspection_common import InspectionNodeBase, NodeId, spin_node


class LogNode(InspectionNodeBase):
    """영구 로그 저장 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.LOG, provides_initialize_action=True)
        self.get_logger().info("LogNode skeleton started")


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 LogNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = LogNode()
    spin_node(node)


if __name__ == "__main__":
    main()
