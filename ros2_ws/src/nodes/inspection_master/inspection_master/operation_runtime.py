"""Master의 진행 중 ROS 작업과 복구 guard를 보관하는 순수 상태 객체."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from inspection_common import ConveyorId, StationId


class StationCyclePhase(StrEnum):
    POSITION_GOAL = "POSITION_GOAL"
    WAITING_POSITION = "WAITING_POSITION"
    CAPTURE_GOAL = "CAPTURE_GOAL"
    WAITING_CAPTURE_RESULT = "WAITING_CAPTURE_RESULT"
    RESUME_PENDING = "RESUME_PENDING"


class ActuationPhase(StrEnum):
    GOAL_PENDING = "GOAL_PENDING"
    WAITING_RESULT = "WAITING_RESULT"
    COMPLETED = "COMPLETED"


class ShutdownPhase(StrEnum):
    IDLE = "IDLE"
    WAITING_STOP = "WAITING_STOP"
    FINALIZING = "FINALIZING"
    READY_TO_EXIT = "READY_TO_EXIT"
    BLOCKED = "BLOCKED"


def finite_float_or_none(value: float) -> float | None:
    """유한한 실수만 로그·결과 계약에서 사용할 수 있게 정규화합니다."""

    normalized = float(value)
    return normalized if math.isfinite(normalized) else None


def build_late_operation_diagnostic(
    operation: str,
    correlation_id: str = "",
    details: dict[str, object] | None = None,
) -> dict[str, object]:
    """적용하지 않은 비동기 결과를 원본 정보와 함께 진단 로그로 보존합니다."""

    payload = dict(details or {})
    # 호출자가 공통 식별자를 덮어쓰지 못하도록 마지막에 확정합니다.
    payload["operation"] = operation
    payload["correlation_id"] = correlation_id
    return payload


@dataclass(slots=True)
class StationCycle:
    product_id: str
    fifo_sequence: int
    station_id: StationId
    conveyor_id: ConveyorId
    position_command_id: str
    capture_id: str
    target_step: int
    phase: StationCyclePhase = StationCyclePhase.POSITION_GOAL
    deadline_ns: int = 0
    position_goal_handle: object | None = None
    capture_goal_handle: object | None = None


@dataclass(slots=True)
class ActuationCycle:
    product_id: str
    fifo_sequence: int
    actuator_job_id: str
    command_id: str
    phase: ActuationPhase = ActuationPhase.GOAL_PENDING
    deadline_ns: int = 0
    goal_handle: object | None = None


@dataclass(slots=True)
class EquipmentSnapshot:
    """Control의 실제 상태를 Master가 안전 guard로 사용하는 최신 mirror."""

    conveyor_running: dict[ConveyorId, bool | None] = field(
        default_factory=lambda: {
            ConveyorId.UPPER: None,
            ConveyorId.LOWER: None,
        }
    )
    conveyor_stopped: dict[ConveyorId, bool | None] = field(
        default_factory=lambda: {
            ConveyorId.UPPER: None,
            ConveyorId.LOWER: None,
        }
    )
    sensor_clear: dict[int, bool | None] = field(
        default_factory=lambda: {1: None, 2: None, 3: None}
    )
    actuator_safe: bool | None = None
    line_clear_confirmed: bool = False
    estop_asserted: bool = False
    operator_id: str = ""

    @classmethod
    def sim_safe(cls) -> "EquipmentSnapshot":
        snapshot = cls()
        snapshot.mark_all_stopped()
        snapshot.sensor_clear = {1: True, 2: True, 3: True}
        snapshot.actuator_safe = True
        snapshot.line_clear_confirmed = True
        snapshot.operator_id = "sim"
        return snapshot

    def mark_all_stopped(self) -> None:
        for conveyor_id in self.conveyor_stopped:
            self.conveyor_stopped[conveyor_id] = True
            self.conveyor_running[conveyor_id] = False

    def mark_all_running(self) -> None:
        for conveyor_id in self.conveyor_running:
            self.conveyor_running[conveyor_id] = True
            self.conveyor_stopped[conveyor_id] = False

    def all_conveyors_stopped(self) -> bool:
        return all(value is True for value in self.conveyor_stopped.values())

    def all_conveyors_running(self) -> bool:
        return all(value is True for value in self.conveyor_running.values())

    def all_sensors_clear(self) -> bool:
        return all(value is True for value in self.sensor_clear.values())

    def line_clear_guards_satisfied(self) -> bool:
        return (
            self.line_clear_confirmed
            and self.all_conveyors_stopped()
            and self.all_sensors_clear()
            and self.actuator_safe is True
            and not self.estop_asserted
        )

    def in_place_guards_satisfied(self) -> bool:
        return (
            self.all_conveyors_stopped()
            and self.actuator_safe is True
            and not self.estop_asserted
        )

    def clear_operator_confirmation(self) -> None:
        self.line_clear_confirmed = False
        self.operator_id = ""
