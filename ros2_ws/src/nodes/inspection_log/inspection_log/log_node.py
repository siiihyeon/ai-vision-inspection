"""LogNode: SQLite commit 이후 ACK를 보장하는 통신 골격."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from inspection_common import ErrorCode, NodeId, new_uuid, sha256_text
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
)
from inspection_interfaces.msg import LogEvent, LogPersistedAck

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
        self.repository: LogRepository | None = None
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
        self.get_logger().info("LogNode v2 communication skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        return (
            "log.database_path",
            "log.producer_spool_root",
            "log.data_root",
            "log.retention_policy",
        )

    def validate_hardware_profile(self) -> list[str]:
        missing = super().validate_hardware_profile()
        if self.profile == "hardware":
            for key in (
                "log.database_path",
                "log.producer_spool_root",
                "log.data_root",
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
        if self.repository is None:
            self.get_logger().error("LogEvent received before SQLite initialization")
            return
        try:
            json.loads(message.payload_json)
            if sha256_text(message.payload_json) != message.payload_digest:
                raise ValueError("payload_digest mismatch")
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
            self.repository.append_event(event)
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
        ack.acked_log_ids = [message.log_id]
        ack.acked_revisions = [message.revision]
        ack.committed_at = ack.header.stamp
        self._ack_publisher.publish(ack)

    def destroy_node(self) -> None:
        if self.repository is not None:
            self.repository.close()
            self.repository = None
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    spin_node(LogNode())


if __name__ == "__main__":
    main()
