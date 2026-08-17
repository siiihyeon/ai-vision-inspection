"""ROS와 분리된 LogEvent 검증/저장 응용 로직."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from inspection_common.digest import sha256_text

from .storage import LogRepository, StoredLogEvent


@dataclass(frozen=True, slots=True)
class PersistResult:
    """한 LogEvent 저장 시도의 결과."""

    log_id: str
    revision: int


class LogEventService:
    """LogEvent의 내용 검증과 SQLite 저장 경계를 소유합니다.

    이 클래스는 ROS 메시지 타입을 알지 못하므로 일반 Python 단위시험에서
    그대로 사용할 수 있습니다. ACK 발행은 commit 성공 이후 ROS 계층에서
    수행합니다.
    """

    def __init__(self, repository: LogRepository) -> None:
        self._repository = repository

    def persist(self, event: StoredLogEvent) -> PersistResult:
        envelope = self._validate_event(event)
        # 원본 이벤트 commit이 이 메서드의 durability boundary입니다.
        self._repository.append_event(event)
        return PersistResult(log_id=event.log_id, revision=event.revision)

    @staticmethod
    def _validate_event(event: StoredLogEvent) -> dict[str, Any]:
        if not event.log_id:
            raise ValueError("log_id is required")
        if event.revision < 1:
            raise ValueError("revision must be positive")
        if not event.event_type:
            raise ValueError("event_type is required")
        if not event.source_node:
            raise ValueError("source_node is required")
        if not event.producer_instance_id:
            raise ValueError("producer_instance_id is required")
        if event.occurred_at_ns < 0:
            raise ValueError("occurred_at_ns must not be negative")
        if sha256_text(event.payload_json) != event.payload_digest:
            raise ValueError("payload_digest mismatch")

        try:
            decoded = json.loads(event.payload_json)
        except (TypeError, ValueError) as exc:
            raise ValueError("payload_json must be valid JSON") from exc
        if not isinstance(decoded, dict):
            raise ValueError("payload_json root must be an object")

        # Master의 v2 producer envelope와 ROS 중복 필드가 서로 다른 경우,
        # 무엇을 신뢰해야 하는지 모호해지므로 원본 저장 전에 차단합니다.
        schema_version = decoded.get("schema_version")
        if schema_version is not None and schema_version != 2:
            raise ValueError("unsupported log schema_version")
        LogEventService._require_matching_field(decoded, "event_type", event.event_type)
        LogEventService._require_matching_field(decoded, "source_node", event.source_node)
        LogEventService._require_matching_field(
            decoded, "producer_instance_id", event.producer_instance_id
        )
        LogEventService._require_matching_field(decoded, "product_id", event.product_id)
        if "severity" in decoded and int(decoded["severity"]) != int(event.severity):
            raise ValueError("severity differs between envelope and LogEvent")
        if "payload" in decoded and not isinstance(decoded["payload"], dict):
            raise ValueError("payload field must be an object")
        return decoded

    @staticmethod
    def _require_matching_field(
        envelope: dict[str, Any], field: str, expected: str
    ) -> None:
        if field in envelope and str(envelope[field]) != expected:
            raise ValueError(f"{field} differs between envelope and LogEvent")
