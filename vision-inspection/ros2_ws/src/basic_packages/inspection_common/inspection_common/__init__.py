"""ROS 없이도 검사 가능한 순수 공통 계약만 package root에서 노출합니다.

ROS 노드는 ``inspection_common.node_base``를 명시적으로 import합니다.
"""

from .constants import (
    ConveyorId,
    ErrorCode,
    NodeHealthState,
    NodeId,
    StationId,
    SystemState,
    Verdict,
)
from .digest import canonical_json, is_sha256_hex, payload_digest, sha256_text
from .identifiers import is_uuid4, new_uuid
from .idempotency import IdempotencyStore, ReplayDecision, ReplayKind
from .log_spool import DurableLogSpool, SpoolRecord

__all__ = [
    "ConveyorId",
    "DurableLogSpool",
    "ErrorCode",
    "IdempotencyStore",
    "NodeHealthState",
    "NodeId",
    "ReplayDecision",
    "ReplayKind",
    "StationId",
    "SpoolRecord",
    "SystemState",
    "Verdict",
    "canonical_json",
    "is_sha256_hex",
    "is_uuid4",
    "new_uuid",
    "payload_digest",
    "sha256_text",
]
