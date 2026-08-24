"""LogNode: SQLite commit 이후 ACK를 보장하는 통신 골격."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from inspection_common import ErrorCode, NodeId, new_uuid
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
)
from inspection_interfaces.msg import LogEvent, LogPersistedAck
from inspection_interfaces.msg import StationInferenceFailed, StationResult
from inspection_interfaces.srv import ReplayStationResults

from .reporting import generate_session_report
from .service import LogEventService
from .storage import LogRepository, StoredLogEvent


class LogNode(InspectionNodeBase):
    """원본 이벤트 commit과 조회용 projection 저장소를 소유합니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.LOG, provides_initialize_action=True)
        self.declare_parameter(
            "log.database_path", "/tmp/inspection/log/inspection.sqlite3"
        )
        self.declare_parameter("log.producer_spool_root", "/tmp/inspection/spool")
        self.declare_parameter("log.data_root", "/tmp/inspection")
        self.declare_parameter("log.retention_policy", "")
        self.declare_parameter("log.image_retention.max_completed_images", 10000)
        self.declare_parameter("log.report_root", "/var/lib/inspection/reports")
        self.declare_parameter("log.timeout_tuning.auto_apply", False)
        self.declare_parameter(
            "log.timeout_tuning.generated_path",
            "/var/lib/inspection/config/vision_timeout_tuning.json",
        )
        self.declare_parameter("log.timeout_tuning.minimum_samples", 10000)
        self.declare_parameter("log.timeout_tuning.safety_factor", 1.2)
        self._shutdown_requested = False
        self._shutdown_ready = False
        self.repository: LogRepository | None = None
        self.log_service: LogEventService | None = None
        self._event_subscription = self.create_subscription(
            LogEvent,
            "log/event",
            self._handle_log_event,
            reliable_event_qos(depth=1000),
        )
        self._ack_publisher = self.create_publisher(
            LogPersistedAck,
            "log/persisted_ack",
            reliable_event_qos(depth=1000),
        )
        self._replay_service = self.create_service(
            ReplayStationResults,
            "log/replay_station_results",
            self._handle_replay_station_results,
        )
        self.get_logger().info("LogNode v2 communication skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        return (
            "log.database_path",
            "log.producer_spool_root",
            "log.data_root",
            "log.retention_policy",
            "log.report_root",
            "log.timeout_tuning.generated_path",
        )

    def validate_hardware_profile(self) -> list[str]:
        missing = super().validate_hardware_profile()
        if self.profile == "hardware":
            for key in (
                "log.database_path",
                "log.producer_spool_root",
                "log.data_root",
                "log.report_root",
                "log.timeout_tuning.generated_path",
            ):
                value = str(self.get_parameter(key).value)
                if value and not Path(value).is_absolute():
                    missing.append(f"{key} must be absolute")
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        if self.profile == "hardware":
            # 저장소 자체는 구현되어 있으나 최종 경로/보존정책 확정 전 hardware READY 금지.
            missing = self.validate_hardware_profile()
            if missing:
                return NodeInitializationOutcome(
                    success=False,
                    error_code=int(ErrorCode.NODE_INIT_FAILED),
                    reason="Log hardware configuration is incomplete",
                    retryable=True,
                )
        try:
            database_path = Path(str(self.get_parameter("log.database_path").value))
            replacement_repository = LogRepository(database_path)
        except Exception as exc:
            return NodeInitializationOutcome(
                success=False,
                error_code=int(ErrorCode.LOG_COMMIT_FAILED),
                reason=f"SQLite initialization failed: {type(exc).__name__}",
                retryable=True,
            )
        previous_repository = self.repository
        self.repository = replacement_repository
        self.log_service = LogEventService(replacement_repository)
        if previous_repository is not None:
            try:
                previous_repository.close()
            except Exception as exc:
                self.get_logger().warning(
                    "previous SQLite repository close failed after safe replacement: "
                    f"{type(exc).__name__}"
                )
        return NodeInitializationOutcome(
            success=True,
            reason="SQLite schema and WAL initialized",
            status_details={"database_path": str(database_path)},
        )

    def _handle_log_event(self, message: LogEvent) -> None:
        """유효성 검사 → transaction commit → ACK 순서를 바꾸지 않습니다."""

        if message.header.session_id != self.session_id:
            return
        if self.repository is None or self.log_service is None:
            self.get_logger().error("LogEvent received before SQLite initialization")
            return
        try:
            event = StoredLogEvent(
                log_id=message.log_id,
                revision=message.revision,
                severity=message.severity,
                event_type=message.event_type,
                source_node=message.source_node,
                producer_instance_id=message.producer_instance_id,
                product_id=message.product_id,
                payload_json=message.payload_json,
                payload_digest=message.payload_digest,
                occurred_at_ns=(
                    message.occurred_at.sec * 1_000_000_000
                    + message.occurred_at.nanosec
                ),
            )
            persisted = self.log_service.persist(event)
        except Exception as exc:
            self.get_logger().error(
                f"LogEvent commit rejected for {message.log_id}: {type(exc).__name__}"
            )
            return

        ack = LogPersistedAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.header.session_id = self.session_id
        ack.header.message_id = new_uuid()
        ack.header.correlation_id = message.header.message_id
        ack.producer_node = message.source_node
        ack.producer_instance_id = message.producer_instance_id
        ack.acked_log_ids = [persisted.log_id]
        ack.acked_revisions = [persisted.revision]
        ack.committed_at = ack.header.stamp
        self._ack_publisher.publish(ack)
        self._enforce_image_retention()

    def _handle_replay_station_results(
        self,
        request: ReplayStationResults.Request,
        response: ReplayStationResults.Response,
    ) -> ReplayStationResults.Response:
        """현재 session의 Vision 선기록 terminal만 Master에 재전송합니다."""

        if self.repository is None:
            response.success = False
            response.reason = "Log repository is not initialized"
            return response
        if request.session_id != self.session_id:
            response.success = False
            response.reason = "only the active session can be replayed"
            return response
        limit = max(1, min(int(request.max_results) or 100, 1000))
        try:
            terminals, has_more = self.repository.replay_station_terminals(
                request.session_id, limit, int(request.offset)
            )
        except Exception as exc:
            response.success = False
            response.reason = f"replay query failed: {type(exc).__name__}"
            return response
        stamp = self.get_clock().now().to_msg()
        for terminal in terminals:
            if terminal.terminal_kind == "RESULT":
                message = StationResult()
                message.header.stamp = stamp
                message.header.session_id = self.session_id
                message.header.message_id = new_uuid()
                message.header.correlation_id = terminal.capture_id
                message.product_id = terminal.product_id
                message.fifo_sequence = terminal.fifo_sequence
                message.station_id = terminal.station_id
                message.capture_id = terminal.capture_id
                message.frame_batch_id = terminal.frame_batch_id
                message.inference_job_id = terminal.inference_job_id
                message.result_revision = terminal.revision
                message.verdict = int(terminal.verdict or 0)
                message.score = float(terminal.score or 0.0)
                message.model_version = terminal.model_version
                message.completed_at = stamp
                response.results.append(message)
            else:
                message = StationInferenceFailed()
                message.header.stamp = stamp
                message.header.session_id = self.session_id
                message.header.message_id = new_uuid()
                message.header.correlation_id = terminal.capture_id
                message.product_id = terminal.product_id
                message.fifo_sequence = terminal.fifo_sequence
                message.station_id = terminal.station_id
                message.capture_id = terminal.capture_id
                message.frame_batch_id = terminal.frame_batch_id
                message.inference_job_id = terminal.inference_job_id
                message.result_revision = terminal.revision
                message.error_code = int(terminal.error_code or 0)
                message.reason = terminal.reason
                message.failed_at = stamp
                response.failures.append(message)
        response.success = True
        response.next_offset = int(request.offset) + len(terminals)
        response.has_more = has_more
        response.reason = f"replayed {len(terminals)} durable Vision terminals"
        return response

    def _enforce_image_retention(self) -> None:
        """LogNode만 완성된 canonical 파일을 최근 10,000장 기준으로 삭제합니다."""

        if self.repository is None:
            return
        keep_latest = int(
            self.get_parameter("log.image_retention.max_completed_images").value
        )
        if keep_latest < 1:
            self.get_logger().error("image retention must keep at least one image")
            return
        data_root = Path(str(self.get_parameter("log.data_root").value)).resolve()
        removed_records: list[str] = []
        for raw_path in self.repository.retention_candidates(keep_latest):
            path = Path(raw_path).resolve()
            if not path.is_relative_to(data_root):
                self.get_logger().error(
                    f"retention refused path outside log.data_root: {path}"
                )
                removed_records.append(raw_path)
                continue
            try:
                if path.exists():
                    path.unlink()
                removed_records.append(raw_path)
            except OSError as exc:
                self.get_logger().warning(
                    f"canonical image retention deletion failed: {type(exc).__name__}"
                )
        self.repository.remove_image_records(removed_records)

    def request_shutdown(self, reason: str = "program termination") -> bool:
        if self._shutdown_requested:
            return False
        self._shutdown_requested = True
        if self.repository is not None and self.session_id:
            try:
                result = generate_session_report(
                    self.repository,
                    session_id=self.session_id,
                    report_root=Path(
                        str(self.get_parameter("log.report_root").value)
                    ),
                    timeout_tuning_path=Path(
                        str(
                            self.get_parameter(
                                "log.timeout_tuning.generated_path"
                            ).value
                        )
                    ),
                    auto_apply_timeouts=bool(
                        self.get_parameter("log.timeout_tuning.auto_apply").value
                    ),
                    minimum_samples=int(
                        self.get_parameter(
                            "log.timeout_tuning.minimum_samples"
                        ).value
                    ),
                    safety_factor=float(
                        self.get_parameter("log.timeout_tuning.safety_factor").value
                    ),
                )
                self.get_logger().info(
                    f"normal session report written: {result.summary_path}"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"normal session report generation failed: {type(exc).__name__}"
                )
        self._shutdown_ready = True
        return True

    @property
    def shutdown_ready(self) -> bool:
        return self._shutdown_ready

    def destroy_node(self) -> bool:
        if self.repository is not None:
            self.repository.close()
            self.repository = None
            self.log_service = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    spin_node(LogNode())


if __name__ == "__main__":
    main()
