"""MasterNode의 통신 실행 골격입니다.

아직 시스템 FSM, 제품 FIFO, 초기화 순서와 업무 통신은 구현하지 않습니다.
"""

import rclpy
from inspection_interfaces.action import InitializeNode
from inspection_interfaces.msg import NodeHeartbeat
from rclpy.action import ActionClient

from inspection_common import InspectionNodeBase, NodeId, heartbeat_qos, spin_node


class MasterNode(InspectionNodeBase):
    """전체 공정 조율 기능이 들어갈 ROS 2 노드입니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.MASTER, provides_initialize_action=False)
        self.worker_heartbeats: dict[NodeId, NodeHeartbeat] = {}
        self._worker_heartbeat_subscriptions = [
            self.create_subscription(
                NodeHeartbeat,
                f"/inspection/{worker.value}/heartbeat",
                lambda message, worker_name=worker: self._handle_worker_heartbeat(
                    worker_name,
                    message,
                ),
                heartbeat_qos(),
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        ]
        self.initialize_clients = {
            worker: ActionClient(
                self,
                InitializeNode,
                f"/inspection/{worker.value}/initialize",
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        }
        self.get_logger().info("MasterNode skeleton started")

    def _handle_worker_heartbeat(
        self,
        worker_name: NodeId,
        message: NodeHeartbeat,
    ) -> None:
        """가장 최근 작업 노드 Heartbeat를 보관합니다."""

        self.worker_heartbeats[worker_name] = message


def main(args: list[str] | None = None) -> None:
    """ROS 2에서 MasterNode를 실행하는 진입점입니다."""

    rclpy.init(args=args)
    node = MasterNode()
    spin_node(node)


if __name__ == "__main__":
    main()
