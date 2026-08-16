"""MasterNode v2 통신 골격과 결과 소유권 경계."""

from __future__ import annotations

import time

import rclpy
from inspection_common import (
    NodeHealthState,
    NodeId,
    StationId,
    SystemState,
    Verdict,
    new_uuid,
)
from inspection_common.digest import payload_digest
from inspection_common.node_base import (
    InspectionNodeBase,
    heartbeat_qos,
    reliable_event_qos,
    spin_node,
    state_qos,
)
from inspection_interfaces.action import (
    ActuateProduct,
    CaptureProduct,
    InitializeNode,
    PositionProduct,
)
from inspection_interfaces.msg import (
    NodeHeartbeat,
    ProductResultLocked,
    SensorEvent,
    StationInferenceFailed,
    StationResult,
    SystemCommand,
    VisionQueueState,
)
from rclpy.action import ActionClient

from .product_flow import (
    LockedProduct,
    ProductLedger,
    ProductResultReorderBuffer,
    StationDecision,
)


class MasterNode(InspectionNodeBase):
    """제품 추적, A/B 결합, FIFO 결과 순서를 단독 소유합니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.MASTER, provides_initialize_action=False)
        self.session_id = new_uuid()
        self.ledger = ProductLedger()
        self.result_reorder = ProductResultReorderBuffer()
        self._paused_by_queue = False
        self._queue_recovered_pending = False
        self.worker_heartbeats: dict[NodeId, NodeHeartbeat] = {}
        self.worker_received_ns: dict[NodeId, int] = {}

        self._worker_heartbeat_subscriptions = [
            self.create_subscription(
                NodeHeartbeat,
                f"/inspection/{worker.value}/heartbeat",
                lambda message, worker_id=worker: self._handle_worker_heartbeat(
                    worker_id, message
                ),
                heartbeat_qos(),
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        ]
        self.initialize_clients = {
            worker: ActionClient(
                self, InitializeNode, f"/inspection/{worker.value}/initialize"
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        }
        self.position_client = ActionClient(
            self, PositionProduct, "/inspection/control/position_product"
        )
        self.capture_client = ActionClient(
            self, CaptureProduct, "/inspection/vision/capture_product"
        )
        self.actuate_client = ActionClient(
            self, ActuateProduct, "/inspection/control/actuate_product"
        )
        self._station_result_subscription = self.create_subscription(
            StationResult,
            "/inspection/vision/station_result",
            self._handle_station_result,
            reliable_event_qos(),
        )
        self._station_failure_subscription = self.create_subscription(
            StationInferenceFailed,
            "/inspection/vision/station_inference_failed",
            self._handle_station_failure,
            reliable_event_qos(),
        )
        self._sensor_subscription = self.create_subscription(
            SensorEvent,
            "/inspection/control/sensor_event",
            self._handle_sensor_event,
            reliable_event_qos(),
        )
        self._queue_subscription = self.create_subscription(
            VisionQueueState,
            "/inspection/vision/queue_state",
            self._handle_queue_state,
            state_qos(),
        )
        self._locked_publisher = self.create_publisher(
            ProductResultLocked,
            "master/product_result_locked",
            reliable_event_qos(),
        )
        self._system_command_publisher = self.create_publisher(
            SystemCommand,
            "master/system_command",
            reliable_event_qos(),
        )
        self._worker_watchdog = self.create_timer(0.5, self._check_worker_heartbeats)
        self.get_logger().info("MasterNode v2 communication skeleton started")

    def register_product(self, product_id: str, fifo_sequence: int) -> None:
        """Sensor1/제품 ID 할당 상태머신이 호출할 안정된 확장점."""

        self.ledger.register(product_id, fifo_sequence)

    def _handle_worker_heartbeat(
        self, worker_id: NodeId, message: NodeHeartbeat
    ) -> None:
        if message.node_id != worker_id.value:
            self.get_logger().error(f"heartbeat node_id mismatch for {worker_id.value}")
            return
        if message.interface_version != self.interface_version:
            self._pause(f"{worker_id.value} interface version mismatch")
            return
        previous = self.worker_heartbeats.get(worker_id)
        if (
            previous is not None
            and previous.node_instance_id
            and previous.node_instance_id != message.node_instance_id
        ):
            self.command_epoch += 1
            self._pause(f"{worker_id.value} process restarted")
        self.worker_heartbeats[worker_id] = message
        self.worker_received_ns[worker_id] = time.monotonic_ns()

    def _check_worker_heartbeats(self) -> None:
        if self.system_state in {
            SystemState.BOOT,
            SystemState.PAUSED,
            SystemState.FAULT_STOP,
        }:
            return
        now = time.monotonic_ns()
        timeout_ns = self.master_heartbeat_timeout_ms * 1_000_000
        missing = [
            worker.value
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
            if worker not in self.worker_received_ns
            or now - self.worker_received_ns[worker] > timeout_ns
        ]
        if missing:
            self._pause("worker heartbeat timeout: " + ", ".join(missing))

    def _handle_station_result(self, message: StationResult) -> None:
        if message.header.session_id != self.session_id:
            return
        context = self.ledger.get(message.product_id, message.fifo_sequence)
        if context is None:
            self.get_logger().error("station result references unknown product identity")
            return
        try:
            decision = StationDecision(
                station_id=StationId(message.station_id),
                verdict=Verdict(message.verdict),
                revision=message.result_revision,
                capture_id=message.capture_id,
                inference_job_id=message.inference_job_id,
            )
        except ValueError:
            self.get_logger().error("station result contains an invalid enum value")
            return
        try:
            if not context.apply_station_result(decision):
                return
        except ValueError as exc:
            self._fault_stop(f"station result identity conflict: {exc}")
            return
        locked = context.lock_if_complete()
        if locked is not None:
            self._accept_locked(locked)

    def _handle_station_failure(self, message: StationInferenceFailed) -> None:
        if message.header.session_id != self.session_id:
            return
        context = self.ledger.get(message.product_id, message.fifo_sequence)
        if context is None:
            self.get_logger().error("station failure references unknown product identity")
            return
        try:
            station_id = StationId(message.station_id)
        except ValueError:
            self.get_logger().error("station failure contains an invalid station_id")
            return
        self._accept_locked(context.lock_explicit_failure(station_id, message.reason))

    def lock_product_at_sensor3(
        self, product_id: str, fifo_sequence: int, sensor3_event_id: str
    ) -> None:
        """물리 FIFO 추적기가 Sensor3 이벤트를 제품에 매핑한 뒤 호출합니다."""

        context = self.ledger.get(product_id, fifo_sequence)
        if context is None:
            raise KeyError("Sensor3 mapping references unknown product")
        self._accept_locked(context.lock_at_sensor3(sensor3_event_id))

    def _handle_sensor_event(self, message: SensorEvent) -> None:
        # TODO(DECISION_REQUIRED): sensor_id 명명과 물리 FIFO 매핑 규칙 확정 후
        # Sensor3에 대해 lock_product_at_sensor3()를 호출합니다.
        del message

    def _handle_queue_state(self, message: VisionQueueState) -> None:
        if message.header.session_id != self.session_id:
            return
        if message.state == VisionQueueState.ENQUEUE_BLOCKED:
            if not self._paused_by_queue:
                self._paused_by_queue = True
                self._queue_recovered_pending = False
                self._pause(
                    f"Vision queue full; preserving FrameBatch {message.blocked_frame_batch_id}"
                )
        elif message.state == VisionQueueState.ACCEPTING and self._paused_by_queue:
            self._queue_recovered_pending = True
            if self.system_state == SystemState.PAUSED:
                self._resume_after_queue_recovery()

    def _pause(self, reason: str) -> None:
        if self.system_state in {SystemState.PAUSING, SystemState.PAUSED}:
            return
        self.system_state = SystemState.PAUSING
        self.set_health_state(NodeHealthState.DEGRADED)
        self._publish_system_command(SystemCommand.PAUSE, reason)
        self.get_logger().error(reason)

    def _fault_stop(self, reason: str) -> None:
        """추적·결과 무결성 충돌을 자동 재개 불가 상태로 전환합니다."""

        self.system_state = SystemState.FAULT_STOP
        self.set_health_state(NodeHealthState.FAULT)
        self._publish_system_command(SystemCommand.PAUSE, reason)
        self.get_logger().error(reason)

    def confirm_all_conveyors_stopped(self) -> None:
        """Control의 실제 정지 완료를 받은 뒤 PAUSING을 확정합니다.

        실제 Sensor/Conveyor 상태 매핑이 확정되면 PositionSettled가 아닌 별도의
        typed 장비 상태 이벤트에서 이 확장점을 호출해야 합니다.
        """

        if self.system_state == SystemState.PAUSING:
            self.system_state = SystemState.PAUSED
            if self._paused_by_queue and self._queue_recovered_pending:
                self._resume_after_queue_recovery()

    def _resume_after_queue_recovery(self) -> None:
        """실제 정지 확인과 동일 FrameBatch enqueue 성공 뒤 자동 재개합니다."""

        if self.system_state != SystemState.PAUSED:
            return
        self._paused_by_queue = False
        self._queue_recovered_pending = False
        self.system_state = SystemState.RUN_SYS
        self.set_health_state(NodeHealthState.READY)
        self._publish_system_command(SystemCommand.RESUME, "Vision queue recovered")

    def _publish_system_command(self, command_type: int, reason: str) -> None:
        message = SystemCommand()
        message.command.header.stamp = self.get_clock().now().to_msg()
        message.command.header.session_id = self.session_id
        message.command.header.message_id = new_uuid()
        message.command.header.correlation_id = ""
        message.command.command_epoch = self.command_epoch
        message.command.command_id = new_uuid()
        message.command.payload_digest = payload_digest(
            {"command_type": command_type, "reason": reason, "epoch": self.command_epoch}
        )
        message.command.issued_at = message.command.header.stamp
        message.command_type = command_type
        message.reason = reason
        self._system_command_publisher.publish(message)

    def _accept_locked(self, locked: LockedProduct) -> None:
        for ready in self.result_reorder.add(locked):
            message = ProductResultLocked()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.session_id = self.session_id
            message.header.message_id = new_uuid()
            message.header.correlation_id = ready.product_id
            message.product_id = ready.product_id
            message.fifo_sequence = ready.fifo_sequence
            message.final_verdict = int(ready.verdict)
            message.station_a_completed = ready.station_a_completed
            message.station_b_completed = ready.station_b_completed
            message.lock_reason = ready.reason
            message.sensor3_event_id = ready.sensor3_event_id
            message.locked_at = message.header.stamp
            self._locked_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    spin_node(MasterNode())


if __name__ == "__main__":
    main()
