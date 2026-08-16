"""Master 전체 시스템 상태 전이표의 순수 Python 규칙.

ROS 통신과 장비 guard는 ``MasterNode``가 담당하고, 이 모듈은
``현재 상태 + 입력 이벤트 -> 다음 상태``가 허용되는지만 결정합니다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from inspection_common import SystemState


class SystemEvent(StrEnum):
    """전체 시스템 전이표에 들어오는 논리 이벤트."""

    APP_STARTED = "APP_STARTED"
    INIT_DONE = "INIT_DONE"
    INIT_FAILED = "INIT_FAILED"
    INIT_TIMEOUT = "INIT_TIMEOUT"
    START_REQUEST = "START_REQUEST"
    START_FAILED = "START_FAILED"
    ALL_CONVEYORS_RUNNING = "ALL_CONVEYORS_RUNNING"
    PAUSE_REQUEST = "PAUSE_REQUEST"
    RECOVERABLE_DEVICE_FAULT = "RECOVERABLE_DEVICE_FAULT"
    ALL_CONVEYORS_STOPPED = "ALL_CONVEYORS_STOPPED"
    PAUSE_FAILED = "PAUSE_FAILED"
    PAUSE_TIMEOUT = "PAUSE_TIMEOUT"
    RESUME_REQUEST = "RESUME_REQUEST"
    DEVICE_RECOVERED = "DEVICE_RECOVERED"
    RESUME_FAILED = "RESUME_FAILED"
    CRITICAL_FAULT = "CRITICAL_FAULT"
    RESET_REQUEST = "RESET_REQUEST"
    RESET_SUCCEEDED_EMPTY_LINE = "RESET_SUCCEEDED_EMPTY_LINE"
    RESET_SUCCEEDED_IN_PLACE = "RESET_SUCCEEDED_IN_PLACE"
    RESET_FAILED = "RESET_FAILED"
    RESET_TIMEOUT = "RESET_TIMEOUT"
    ESTOP_ASSERTED = "ESTOP_ASSERTED"


class RecoveryPolicy(StrEnum):
    """FAULT_STOP 이후 필요한 복구 절차."""

    NONE = "NONE"
    LINE_CLEAR_REQUIRED = "LINE_CLEAR_REQUIRED"
    EQUIPMENT_CHECK_REQUIRED = "EQUIPMENT_CHECK_REQUIRED"
    RETRY_IN_PLACE = "RETRY_IN_PLACE"


@dataclass(frozen=True, slots=True)
class SystemTransition:
    """전이표 조회 결과."""

    rule_id: str
    previous: SystemState
    event: SystemEvent
    current: SystemState
    reason: str

    @property
    def changed(self) -> bool:
        return self.previous != self.current


class InvalidSystemTransition(ValueError):
    """현재 상태에서 허용되지 않는 이벤트가 들어왔음을 나타냅니다."""


def _rule(
    rule_id: str,
    target: SystemState,
) -> tuple[str, SystemState]:
    return rule_id, target


# 같은 상태로 돌아가는 규칙은 실패를 무시한다는 뜻이 아닙니다.
# 안전 정지 상태를 유지하면서 로그와 재시도를 계속한다는 뜻입니다.
_TRANSITIONS: dict[
    tuple[SystemState, SystemEvent], tuple[str, SystemState]
] = {
    (SystemState.BOOT, SystemEvent.APP_STARTED): _rule(
        "SYS-01", SystemState.INITIALIZING
    ),
    (SystemState.INITIALIZING, SystemEvent.INIT_DONE): _rule(
        "SYS-02", SystemState.READY
    ),
    (SystemState.INITIALIZING, SystemEvent.INIT_FAILED): _rule(
        "SYS-03", SystemState.INITIALIZING
    ),
    (SystemState.INITIALIZING, SystemEvent.INIT_TIMEOUT): _rule(
        "SYS-03", SystemState.INITIALIZING
    ),
    # START_REQUEST 시점에는 READY를 유지합니다. 실제 양쪽 RUN 확인 뒤 RUN_SYS입니다.
    (SystemState.READY, SystemEvent.START_REQUEST): _rule(
        "SYS-04", SystemState.READY
    ),
    (SystemState.READY, SystemEvent.START_FAILED): _rule(
        "SYS-04", SystemState.READY
    ),
    (SystemState.READY, SystemEvent.ALL_CONVEYORS_RUNNING): _rule(
        "SYS-04", SystemState.RUN_SYS
    ),
    (SystemState.RUN_SYS, SystemEvent.PAUSE_REQUEST): _rule(
        "SYS-05", SystemState.PAUSING
    ),
    (SystemState.RUN_SYS, SystemEvent.RECOVERABLE_DEVICE_FAULT): _rule(
        "SYS-05A", SystemState.PAUSING
    ),
    (SystemState.PAUSING, SystemEvent.ALL_CONVEYORS_STOPPED): _rule(
        "SYS-06", SystemState.PAUSED
    ),
    (SystemState.PAUSING, SystemEvent.PAUSE_FAILED): _rule(
        "SYS-07", SystemState.FAULT_STOP
    ),
    (SystemState.PAUSING, SystemEvent.PAUSE_TIMEOUT): _rule(
        "SYS-07", SystemState.FAULT_STOP
    ),
    # RESUME도 실제 양쪽 RUN 확인 전까지 PAUSED를 유지합니다.
    (SystemState.PAUSED, SystemEvent.RESUME_REQUEST): _rule(
        "SYS-08", SystemState.PAUSED
    ),
    (SystemState.PAUSED, SystemEvent.DEVICE_RECOVERED): _rule(
        "SYS-08", SystemState.PAUSED
    ),
    (SystemState.PAUSED, SystemEvent.RESUME_FAILED): _rule(
        "SYS-08", SystemState.PAUSED
    ),
    (SystemState.PAUSED, SystemEvent.ALL_CONVEYORS_RUNNING): _rule(
        "SYS-08", SystemState.RUN_SYS
    ),
    (SystemState.FAULT_STOP, SystemEvent.RESET_REQUEST): _rule(
        "SYS-11", SystemState.RESETTING
    ),
    (SystemState.RESETTING, SystemEvent.RESET_SUCCEEDED_EMPTY_LINE): _rule(
        "SYS-12", SystemState.READY
    ),
    (SystemState.RESETTING, SystemEvent.RESET_SUCCEEDED_IN_PLACE): _rule(
        "SYS-12A", SystemState.PAUSED
    ),
    (SystemState.RESETTING, SystemEvent.RESET_FAILED): _rule(
        "SYS-13", SystemState.RESETTING
    ),
    (SystemState.RESETTING, SystemEvent.RESET_TIMEOUT): _rule(
        "SYS-13", SystemState.RESETTING
    ),
}


_FAULT_EVENTS = {
    SystemEvent.CRITICAL_FAULT: "SYS-10",
    SystemEvent.ESTOP_ASSERTED: "SYS-15",
}


def decide_system_transition(
    current: SystemState,
    event: SystemEvent,
    reason: str,
) -> SystemTransition:
    """전이표에 따라 다음 상태를 결정합니다.

    Guard가 필요한 요청은 MasterNode가 먼저 guard를 확인한 뒤 이 함수를
    호출합니다. 등록되지 않은 조합은 추정하지 않고 예외로 거부합니다.
    """

    fault_rule = _FAULT_EVENTS.get(event)
    if fault_rule is not None:
        return SystemTransition(
            rule_id=fault_rule,
            previous=current,
            event=event,
            current=SystemState.FAULT_STOP,
            reason=reason,
        )

    rule = _TRANSITIONS.get((current, event))
    if rule is None:
        raise InvalidSystemTransition(
            f"{current.name} does not accept {event.value}"
        )
    rule_id, target = rule
    return SystemTransition(
        rule_id=rule_id,
        previous=current,
        event=event,
        current=target,
        reason=reason,
    )


def allowed_system_events(current: SystemState) -> frozenset[SystemEvent]:
    """현재 상태에서 전이표가 허용하는 이벤트 집합을 반환합니다."""

    local = {
        event
        for state, event in _TRANSITIONS
        if state == current
    }
    local.update(_FAULT_EVENTS)
    return frozenset(local)
