"""노드 간 공통으로 사용하는 식별자와 상태 상수입니다."""

from enum import StrEnum


class NodeId(StrEnum):
    """실행 노드의 고정 식별자입니다."""

    MASTER = "master"
    CONTROL = "control"
    VISION = "vision"
    LOG = "log"


class NodeHealthState(StrEnum):
    """노드 자체의 준비·복구 상태입니다."""

    STARTING = "STARTING"
    READY = "READY"
    DEGRADED = "DEGRADED"
    RECOVERING = "RECOVERING"
    INIT_BLOCKED = "INIT_BLOCKED"
    FAULT = "FAULT"


class ErrorCode(StrEnum):
    """현재 공통 골격에서 실제 사용하는 오류 코드입니다."""

    COMMAND_CONFLICT = "COMMAND_CONFLICT"
    INTERFACE_VERSION_MISMATCH = "INTERFACE_VERSION_MISMATCH"
    NODE_INIT_FAILED = "NODE_INIT_FAILED"


NODE_EXECUTABLE_NAME = {
    NodeId.MASTER: "master_node",
    NodeId.CONTROL: "control_node",
    NodeId.VISION: "vision_node",
    NodeId.LOG: "log_node",
}

NODE_PACKAGE_NAME = {
    NodeId.MASTER: "inspection_master",
    NodeId.CONTROL: "inspection_control",
    NodeId.VISION: "inspection_vision",
    NodeId.LOG: "inspection_log",
}
