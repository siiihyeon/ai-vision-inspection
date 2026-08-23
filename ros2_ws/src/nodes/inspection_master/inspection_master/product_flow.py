"""Master가 단독 소유하는 제품 물리 흐름, FIFO와 최종 판정 규칙."""

from __future__ import annotations

import time
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from enum import StrEnum

from inspection_common.constants import (
    PhysicalZone,
    ProductPhysicalState,
    StationId,
    Verdict,
)


class ProductFlowError(ValueError):
    """제품 물리 상태나 FIFO 순서를 신뢰할 수 없을 때 발생합니다."""


class ProductIdentityConflict(ProductFlowError):
    """제품·촬영·센서 식별자가 기존 원장과 충돌했습니다."""


class StationResultConflict(ProductFlowError):
    """동일 station result revision에 서로 다른 내용이 들어왔습니다."""


class StationProcessState(StrEnum):
    """Master가 바라보는 station 한 곳의 진행 단계입니다."""

    PENDING = "PENDING"
    POSITIONING = "POSITIONING"
    POSITION_SETTLED = "POSITION_SETTLED"
    CAPTURE_REQUESTED = "CAPTURE_REQUESTED"
    CAPTURE_COMPLETED = "CAPTURE_COMPLETED"
    SKIPPED = "SKIPPED"
    FAILED = "FAILED"


class SensorEventOutcome(StrEnum):
    """센서 event ID·sequence를 검사한 결과입니다."""

    ACCEPTED = "ACCEPTED"
    DUPLICATE = "DUPLICATE"
    CONFLICT = "CONFLICT"
    OUT_OF_ORDER = "OUT_OF_ORDER"
    SEQUENCE_GAP = "SEQUENCE_GAP"


@dataclass(frozen=True, slots=True)
class StationDecision:
    station_id: StationId
    verdict: Verdict
    revision: int
    capture_id: str
    inference_job_id: str
    frame_batch_id: str = ""


@dataclass(frozen=True, slots=True)
class LockedProduct:
    product_id: str
    fifo_sequence: int
    verdict: Verdict
    station_a_completed: bool
    station_b_completed: bool
    reason: str
    sensor3_event_id: str = ""


@dataclass(slots=True)
class StationInspection:
    """제품 한 개의 station별 촬영·추론 mirror입니다."""

    station_id: StationId
    process_state: StationProcessState = StationProcessState.PENDING
    position_command_id: str = ""
    position_target_step: int = 0
    position_action_succeeded: bool = False
    position_settled: bool = False
    capture_command_id: str = ""
    capture_id: str = ""
    frame_batch_id: str = ""
    inference_job_id: str = ""
    capture_succeeded: bool = False
    decision: StationDecision | None = None
    failed: bool = False
    conflicted: bool = False
    failure_reason: str = ""
    skipped_reason: str = ""
    last_failure_revision: int = 0

    @property
    def result_completed(self) -> bool:
        return self.decision is not None or self.failed or self.conflicted


def _station_map() -> dict[StationId, StationInspection]:
    return {
        StationId.A: StationInspection(StationId.A),
        StationId.B: StationInspection(StationId.B),
    }


@dataclass(slots=True)
class ProductContext:
    """활성 제품의 물리 위치, station 결과와 분류 작업 원장입니다."""

    product_id: str
    fifo_sequence: int
    physical_state: ProductPhysicalState = ProductPhysicalState.STATION_A_WAIT
    physical_zone: PhysicalZone = PhysicalZone.UPPER_INSPECTION
    stations: dict[StationId, StationInspection] = field(default_factory=_station_map)
    sensor_event_ids: dict[int, str] = field(default_factory=dict)
    sensor_steps: dict[int, int] = field(default_factory=dict)
    locked: LockedProduct | None = None
    actuator_job_id: str = ""
    actuator_command_id: str = ""
    actuation_id: str = ""
    completed: bool = False
    removed: bool = False
    removed_monotonic_ns: int = 0
    station_b_skip_requested: bool = False
    station_b_skip_reason: str = ""
    revision: int = 1

    def _touch(self) -> None:
        self.revision += 1

    def station(self, station_id: StationId) -> StationInspection:
        return self.stations[station_id]

    def record_sensor(self, sensor_index: int, event_id: str, step: int) -> None:
        previous = self.sensor_event_ids.get(sensor_index)
        if previous is not None:
            if previous != event_id:
                raise ProductIdentityConflict(
                    f"Sensor{sensor_index} already belongs to another event"
                )
            return
        self.sensor_event_ids[sensor_index] = event_id
        self.sensor_steps[sensor_index] = step
        self._touch()

    def begin_station_cycle(
        self,
        station_id: StationId,
        *,
        position_command_id: str,
        target_step: int,
        capture_id: str,
    ) -> None:
        expected = (
            ProductPhysicalState.STATION_A_WAIT
            if station_id == StationId.A
            else ProductPhysicalState.STATION_B_WAIT
        )
        if self.physical_state != expected:
            raise ProductFlowError(
                f"{self.product_id} cannot start station {station_id.name} "
                f"from {self.physical_state.name}"
            )
        station = self.station(station_id)
        if station.process_state != StationProcessState.PENDING:
            raise ProductFlowError(
                f"station {station_id.name} already has an active or completed cycle"
            )
        station.process_state = StationProcessState.POSITIONING
        station.position_command_id = position_command_id
        station.position_target_step = target_step
        station.capture_id = capture_id
        self._touch()

    def mark_position_action_succeeded(
        self, station_id: StationId, position_command_id: str
    ) -> None:
        station = self.station(station_id)
        if station.position_command_id != position_command_id:
            raise ProductIdentityConflict("position command identity mismatch")
        station.position_action_succeeded = True
        self._touch()

    def mark_position_settled(
        self, station_id: StationId, position_command_id: str
    ) -> None:
        station = self.station(station_id)
        if station.position_command_id != position_command_id:
            raise ProductIdentityConflict("PositionSettled command identity mismatch")
        # RELIABLE 재전송이나 Control 재발행으로 같은 정지 이벤트가 다시
        # 도착해도 CAPTURE_REQUESTED 이후 상태를 뒤로 되돌리지 않습니다.
        if station.position_settled:
            return
        if station.process_state != StationProcessState.POSITIONING:
            raise ProductFlowError(
                f"PositionSettled is not allowed from {station.process_state.value}"
            )
        station.position_settled = True
        station.process_state = StationProcessState.POSITION_SETTLED
        self._touch()

    def can_request_capture(self, station_id: StationId) -> bool:
        station = self.station(station_id)
        return (
            station.position_action_succeeded
            and station.position_settled
            and station.process_state == StationProcessState.POSITION_SETTLED
        )

    def mark_capture_requested(
        self, station_id: StationId, *, capture_id: str, command_id: str
    ) -> None:
        station = self.station(station_id)
        if station.capture_id != capture_id:
            raise ProductIdentityConflict("capture identity mismatch")
        if station.process_state == StationProcessState.CAPTURE_REQUESTED:
            if station.capture_command_id != command_id:
                raise ProductIdentityConflict("capture already has another command")
            return
        if not self.can_request_capture(station_id):
            raise ProductFlowError("capture requested before position was settled")
        station.capture_command_id = command_id
        station.process_state = StationProcessState.CAPTURE_REQUESTED
        self._touch()

    def reset_unfinished_station_cycle(self, station_id: StationId) -> None:
        """장비 복구 뒤 새 command ID로 위치 이동을 다시 시작하게 준비합니다."""

        station = self.station(station_id)
        if station.capture_succeeded or station.failed or station.decision is not None:
            raise ProductFlowError("completed station cycle cannot be reset")
        self.stations[station_id] = StationInspection(station_id)
        self._touch()

    def mark_capture_succeeded(
        self,
        station_id: StationId,
        *,
        capture_id: str,
        frame_batch_id: str,
        inference_job_id: str,
    ) -> None:
        station = self.station(station_id)
        if station.capture_id != capture_id:
            raise ProductIdentityConflict("CaptureProduct result identity mismatch")
        if station.frame_batch_id and station.frame_batch_id != frame_batch_id:
            raise ProductIdentityConflict("CaptureProduct frame_batch_id mismatch")
        if station.inference_job_id and station.inference_job_id != inference_job_id:
            raise ProductIdentityConflict("CaptureProduct inference_job_id mismatch")
        station.capture_succeeded = True
        station.frame_batch_id = frame_batch_id
        station.inference_job_id = inference_job_id
        station.process_state = StationProcessState.CAPTURE_COMPLETED
        self.physical_state = (
            ProductPhysicalState.STATION_A_DONE
            if station_id == StationId.A
            else ProductPhysicalState.STATION_B_DONE
        )
        self._touch()

    def mark_capture_failed(
        self, station_id: StationId, *, capture_id: str, reason: str
    ) -> None:
        station = self.station(station_id)
        if station.capture_id != capture_id:
            raise ProductIdentityConflict("CaptureProduct failure identity mismatch")
        station.failed = True
        station.failure_reason = reason
        station.process_state = StationProcessState.FAILED
        self.physical_state = (
            ProductPhysicalState.STATION_A_DONE
            if station_id == StationId.A
            else ProductPhysicalState.STATION_B_DONE
        )
        self._touch()

    def promote_capture_completion_from_vision(self, station_id: StationId) -> None:
        """Action Result보다 먼저 도착한 Vision 결과를 촬영 완료 증거로 사용합니다.

        Vision이 StationResult/StationInferenceFailed를 발행했다면 이미지
        저장과 inference queue 등록은 성공한 상태입니다. ROS Action
        Result가 통신 순서상 뒤늦게 오거나 유실되어도 같은 제품을
        다시 촬영하지 않도록 물리 완료 상태를 복원합니다.
        """

        station = self.station(station_id)
        if not station.result_completed:
            raise ProductFlowError("vision completion evidence is missing")
        station.capture_succeeded = True
        station.process_state = StationProcessState.CAPTURE_COMPLETED
        self.physical_state = (
            ProductPhysicalState.STATION_A_DONE
            if station_id == StationId.A
            else ProductPhysicalState.STATION_B_DONE
        )
        self._touch()

    def mark_conveyor_resumed_after_capture(self, station_id: StationId) -> None:
        expected = (
            ProductPhysicalState.STATION_A_DONE
            if station_id == StationId.A
            else ProductPhysicalState.STATION_B_DONE
        )
        if self.physical_state != expected:
            raise ProductFlowError(
                f"capture resume for {station_id.name} from {self.physical_state.name}"
            )
        if station_id == StationId.A:
            self.physical_state = ProductPhysicalState.FLIPPING
            self.physical_zone = PhysicalZone.FLIP_TRANSFER
        else:
            self.physical_state = ProductPhysicalState.SENSOR3_WAIT
            self.physical_zone = PhysicalZone.LOWER_TO_SENSOR3
        self._touch()

    def accept_sensor2(self, event_id: str, step: int) -> None:
        if self.physical_state != ProductPhysicalState.FLIPPING:
            raise ProductFlowError("Sensor2 product is not in FLIPPING")
        self.record_sensor(2, event_id, step)
        if self.station_b_skip_requested:
            station_b = self.station(StationId.B)
            station_b.process_state = StationProcessState.SKIPPED
            station_b.skipped_reason = self.station_b_skip_reason
            self.physical_state = ProductPhysicalState.SENSOR3_WAIT
            self.physical_zone = PhysicalZone.LOWER_TO_SENSOR3
        else:
            self.physical_state = ProductPhysicalState.STATION_B_WAIT
            self.physical_zone = PhysicalZone.LOWER_INSPECTION
        self._touch()

    def request_station_b_skip(self, reason: str) -> bool:
        """Station A terminal NG 뒤 B 촬영/추론을 영구 금지합니다."""

        if self.station_b_skip_requested:
            return False
        self.station_b_skip_requested = True
        self.station_b_skip_reason = reason or "Station A terminal NG"
        self._touch()
        return True

    def mark_station_b_skipped(self, reason: str = "") -> bool:
        """이미 B 구간에 진입한 제품을 촬영 없이 완료 상태로 전환합니다."""

        station_b = self.station(StationId.B)
        if station_b.process_state == StationProcessState.SKIPPED:
            return False
        if station_b.decision is not None or station_b.failed or station_b.conflicted:
            return False
        self.request_station_b_skip(reason)
        station_b.process_state = StationProcessState.SKIPPED
        station_b.skipped_reason = self.station_b_skip_reason
        self.physical_state = ProductPhysicalState.STATION_B_DONE
        self.physical_zone = PhysicalZone.LOWER_INSPECTION
        self._touch()
        return True

    def bypass_station_b(self, reason: str = "") -> bool:
        """B 위치 정지 명령 전에 lower conveyor를 계속 운전시킵니다."""

        if self.physical_state != ProductPhysicalState.STATION_B_WAIT:
            return False
        station_b = self.station(StationId.B)
        if station_b.process_state != StationProcessState.PENDING:
            return False
        self.request_station_b_skip(reason)
        station_b.process_state = StationProcessState.SKIPPED
        station_b.skipped_reason = self.station_b_skip_reason
        self.physical_state = ProductPhysicalState.SENSOR3_WAIT
        self.physical_zone = PhysicalZone.LOWER_TO_SENSOR3
        self._touch()
        return True

    def apply_station_result(self, decision: StationDecision) -> bool:
        """더 최신인 revision만 반영하며 Sensor3 잠금 뒤 결과는 거부합니다."""

        station = self.station(decision.station_id)
        if station.capture_id and station.capture_id != decision.capture_id:
            raise ProductIdentityConflict("station result capture_id mismatch")
        if (
            station.frame_batch_id
            and decision.frame_batch_id
            and station.frame_batch_id != decision.frame_batch_id
        ):
            raise ProductIdentityConflict("station result frame_batch_id mismatch")
        if (
            station.inference_job_id
            and station.inference_job_id != decision.inference_job_id
        ):
            raise ProductIdentityConflict("station result inference_job_id mismatch")
        if self.locked is not None:
            return False
        previous = station.decision
        if previous is not None:
            if (
                decision.station_id == StationId.A
                and previous.verdict == Verdict.NG
            ):
                if (
                    decision.verdict != Verdict.NG
                    or decision.capture_id != previous.capture_id
                    or decision.inference_job_id != previous.inference_job_id
                ):
                    station.conflicted = True
                    station.failure_reason = (
                        "Station A terminal NG cannot be revised or replaced"
                    )
                    self._touch()
                    raise StationResultConflict(station.failure_reason)
                return False
            if decision.revision < previous.revision:
                return False
            if decision.revision == previous.revision:
                if decision != previous:
                    station.conflicted = True
                    station.failure_reason = "same revision has conflicting station results"
                    self._touch()
                    raise StationResultConflict(station.failure_reason)
                return False
        station.decision = decision
        if decision.frame_batch_id:
            station.frame_batch_id = decision.frame_batch_id
        station.inference_job_id = decision.inference_job_id
        self._touch()
        return True

    def record_station_failure(
        self,
        station_id: StationId,
        *,
        capture_id: str,
        frame_batch_id: str,
        inference_job_id: str,
        revision: int,
        reason: str,
    ) -> bool:
        station = self.station(station_id)
        if station.capture_id and station.capture_id != capture_id:
            raise ProductIdentityConflict("station failure capture_id mismatch")
        if station.frame_batch_id and station.frame_batch_id != frame_batch_id:
            raise ProductIdentityConflict("station failure frame_batch_id mismatch")
        if station.inference_job_id and station.inference_job_id != inference_job_id:
            raise ProductIdentityConflict("station failure inference_job_id mismatch")
        if self.locked is not None or revision < station.last_failure_revision:
            return False
        if revision == station.last_failure_revision and station.failed:
            return False
        station.failed = True
        station.failure_reason = reason
        station.frame_batch_id = frame_batch_id
        station.inference_job_id = inference_job_id
        station.last_failure_revision = revision
        self._touch()
        return True

    def record_station_contract_failure(
        self, station_id: StationId, reason: str
    ) -> bool:
        """제품 식별은 유지되지만 Vision payload를 신뢰할 수 없는 경우입니다."""

        if self.locked is not None:
            return False
        station = self.station(station_id)
        station.conflicted = True
        station.failure_reason = reason
        self._touch()
        return True

    def _candidate_verdict(self) -> tuple[Verdict | None, str]:
        station_a = self.station(StationId.A)
        station_b = self.station(StationId.B)
        if station_a.failed or station_a.conflicted:
            return Verdict.FORCED_NG, station_a.failure_reason or "Station A failed"
        if (
            station_a.decision is not None
            and station_a.decision.verdict == Verdict.NG
        ):
            return Verdict.NG, "Station A terminal NG"
        if station_b.failed or station_b.conflicted:
            return Verdict.FORCED_NG, station_b.failure_reason or "Station B failed"
        if station_a.decision is None or station_b.decision is None:
            return None, "station result incomplete at Sensor3"
        verdict = (
            Verdict.PASS
            if station_a.decision.verdict == Verdict.PASS
            and station_b.decision.verdict == Verdict.PASS
            else Verdict.NG
        )
        return verdict, "both station results completed"

    def lock_at_sensor3(self, sensor3_event_id: str, step: int = 0) -> LockedProduct:
        """Sensor3 수락 순간의 결과만 최종 판정으로 잠급니다."""

        if self.locked is not None:
            if self.locked.sensor3_event_id != sensor3_event_id:
                raise ProductIdentityConflict("product already locked by another Sensor3 event")
            return self.locked
        if self.physical_state != ProductPhysicalState.SENSOR3_WAIT:
            raise ProductFlowError("Sensor3 product is not in SENSOR3_WAIT")
        self.record_sensor(3, sensor3_event_id, step)
        verdict, reason = self._candidate_verdict()
        if verdict is None:
            verdict = Verdict.FORCED_NG
        self.locked = LockedProduct(
            product_id=self.product_id,
            fifo_sequence=self.fifo_sequence,
            verdict=verdict,
            station_a_completed=self.station(StationId.A).result_completed,
            station_b_completed=self.station(StationId.B).result_completed,
            reason=reason,
            sensor3_event_id=sensor3_event_id,
        )
        self.physical_state = ProductPhysicalState.AT_SENSOR3
        self._touch()
        return self.locked

    def begin_actuation(self, *, actuator_job_id: str, command_id: str) -> None:
        if self.locked is None or self.physical_state != ProductPhysicalState.AT_SENSOR3:
            raise ProductFlowError("actuation requires a Sensor3-locked product")
        if self.actuator_job_id:
            if (
                self.actuator_job_id != actuator_job_id
                or self.actuator_command_id != command_id
            ):
                raise ProductIdentityConflict("product already has another actuation job")
            return
        self.actuator_job_id = actuator_job_id
        self.actuator_command_id = command_id
        self._touch()

    def mark_actuation_accepted(self, command_id: str) -> None:
        if self.actuator_command_id != command_id:
            raise ProductIdentityConflict("actuation command identity mismatch")
        self.physical_state = ProductPhysicalState.ACTUATING
        self._touch()

    def mark_actuation_completed(self, command_id: str, actuation_id: str) -> None:
        if self.actuator_command_id != command_id:
            raise ProductIdentityConflict("actuation completion identity mismatch")
        if self.physical_state not in {
            ProductPhysicalState.AT_SENSOR3,
            ProductPhysicalState.ACTUATING,
        }:
            raise ProductFlowError("actuation completed from an invalid physical state")
        self.actuation_id = actuation_id
        self.completed = True
        self.physical_state = ProductPhysicalState.DONE
        self._touch()

    def snapshot(self) -> dict[str, object]:
        return {
            "product_id": self.product_id,
            "fifo_sequence": self.fifo_sequence,
            "physical_state": self.physical_state.name,
            "physical_zone": self.physical_zone.name,
            "revision": self.revision,
            "sensor_event_ids": dict(self.sensor_event_ids),
            "capture_ids": {
                station.name: state.capture_id for station, state in self.stations.items()
            },
            "station_process_states": {
                station.name: state.process_state.value
                for station, state in self.stations.items()
            },
            "frame_batch_ids": {
                station.name: state.frame_batch_id
                for station, state in self.stations.items()
            },
            "inference_job_ids": {
                station.name: state.inference_job_id
                for station, state in self.stations.items()
            },
            "station_results": {
                station.name: (
                    state.decision.verdict.name if state.decision is not None else None
                )
                for station, state in self.stations.items()
            },
            "station_failure_reasons": {
                station.name: state.failure_reason
                for station, state in self.stations.items()
                if state.failure_reason
            },
            "station_skip_reasons": {
                station.name: state.skipped_reason
                for station, state in self.stations.items()
                if state.skipped_reason
            },
            "station_b_skip_requested": self.station_b_skip_requested,
            "final_verdict": self.locked.verdict.name if self.locked else "PENDING",
            "lock_reason": self.locked.reason if self.locked else "",
            "actuator_job_id": self.actuator_job_id,
            "actuation_id": self.actuation_id,
            "completed": self.completed,
            "removed": self.removed,
            "removed_monotonic_ns": self.removed_monotonic_ns,
        }


class SensorEventRegistry:
    """센서별 event_id와 sequence를 한 번만 수락합니다."""

    def __init__(self, *, event_capacity: int = 8192) -> None:
        if event_capacity < 1:
            raise ValueError("event_capacity must be positive")
        self._last_sequence: dict[str, int] = {}
        self._event_digests: OrderedDict[str, str] = OrderedDict()
        self._event_capacity = event_capacity

    @property
    def remembered_event_count(self) -> int:
        return len(self._event_digests)

    def accept(
        self, sensor_id: str, event_id: str, sequence: int, digest: str
    ) -> SensorEventOutcome:
        previous_digest = self._event_digests.get(event_id)
        if previous_digest is not None:
            return (
                SensorEventOutcome.DUPLICATE
                if previous_digest == digest
                else SensorEventOutcome.CONFLICT
            )
        if not event_id or sequence <= 0:
            return SensorEventOutcome.OUT_OF_ORDER
        previous_sequence = self._last_sequence.get(sensor_id)
        if previous_sequence is not None:
            if sequence <= previous_sequence:
                return SensorEventOutcome.OUT_OF_ORDER
            if sequence != previous_sequence + 1:
                return SensorEventOutcome.SEQUENCE_GAP
        self._event_digests[event_id] = digest
        while len(self._event_digests) > self._event_capacity:
            self._event_digests.popitem(last=False)
        self._last_sequence[sensor_id] = sequence
        return SensorEventOutcome.ACCEPTED

    def reset(self) -> None:
        self._last_sequence.clear()
        self._event_digests.clear()


class ProductLedger:
    """제품 원장과 활성 물리 FIFO를 하나의 단일 writer 구조로 관리합니다."""

    def __init__(self) -> None:
        self._by_id: dict[str, ProductContext] = {}
        self._id_by_sequence: dict[int, str] = {}
        self._capture_owner: dict[str, tuple[str, StationId]] = {}
        self._active: deque[str] = deque()
        self._last_sequence = 0
        self._retired_products: OrderedDict[tuple[str, int], None] = OrderedDict()
        self._retired_captures: OrderedDict[str, None] = OrderedDict()
        self._retired_capacity = 4096

    @property
    def active_size(self) -> int:
        return len(self._active)

    @property
    def next_sequence(self) -> int:
        return self._last_sequence + 1

    def register(self, product_id: str, fifo_sequence: int) -> ProductContext:
        if not product_id or fifo_sequence < 1:
            raise ValueError("product_id and positive fifo_sequence are required")
        existing = self._by_id.get(product_id)
        if existing is not None:
            if existing.fifo_sequence != fifo_sequence:
                raise ProductIdentityConflict(
                    "product_id already has another fifo_sequence"
                )
            return existing
        if fifo_sequence in self._id_by_sequence:
            raise ProductIdentityConflict(
                "fifo_sequence already belongs to another product"
            )
        if fifo_sequence <= self._last_sequence:
            raise ProductFlowError("fifo_sequence must increase monotonically")
        context = ProductContext(product_id=product_id, fifo_sequence=fifo_sequence)
        self._by_id[product_id] = context
        self._id_by_sequence[fifo_sequence] = product_id
        self._active.append(product_id)
        self._last_sequence = fifo_sequence
        return context

    def get(self, product_id: str, fifo_sequence: int) -> ProductContext | None:
        context = self._by_id.get(product_id)
        if context is None or context.fifo_sequence != fifo_sequence:
            return None
        return context

    def get_by_id(self, product_id: str) -> ProductContext | None:
        return self._by_id.get(product_id)

    def active_contexts(self) -> tuple[ProductContext, ...]:
        return tuple(self._by_id[product_id] for product_id in self._active)

    def register_capture(
        self, capture_id: str, product_id: str, station_id: StationId
    ) -> None:
        owner = self._capture_owner.get(capture_id)
        requested = (product_id, station_id)
        if owner is not None and owner != requested:
            raise ProductIdentityConflict("capture_id belongs to another product/station")
        self._capture_owner[capture_id] = requested

    def capture_owner(self, capture_id: str) -> tuple[str, StationId] | None:
        return self._capture_owner.get(capture_id)

    def oldest_in_state(self, state: ProductPhysicalState) -> ProductContext:
        active = self.active_contexts()
        for index, context in enumerate(active):
            if context.physical_state != state:
                continue
            for earlier in active[:index]:
                if earlier.physical_state.value < state.value:
                    raise ProductFlowError(
                        f"earlier product {earlier.product_id} is behind {state.name}"
                    )
            return context
        raise ProductFlowError(f"no active product is waiting in {state.name}")

    def validate_alignment(self) -> bool:
        contexts = self.active_contexts()
        sequences = [context.fifo_sequence for context in contexts]
        if sequences != sorted(sequences) or len(sequences) != len(set(sequences)):
            return False
        if any(context.removed for context in contexts):
            return False
        return all(
            earlier.physical_state.value >= later.physical_state.value
            for earlier, later in zip(contexts, contexts[1:])
        )

    def remove_completed_prefix(
        self, *, now_ns: int | None = None
    ) -> list[ProductContext]:
        removed_at = time.monotonic_ns() if now_ns is None else now_ns
        removed: list[ProductContext] = []
        while self._active:
            context = self._by_id[self._active[0]]
            if not context.completed or context.physical_state != ProductPhysicalState.DONE:
                break
            self._active.popleft()
            context.removed = True
            context.removed_monotonic_ns = removed_at
            context._touch()
            removed.append(context)
        return removed

    def clear_active(self, *, now_ns: int | None = None) -> list[ProductContext]:
        removed_at = time.monotonic_ns() if now_ns is None else now_ns
        cleared: list[ProductContext] = []
        while self._active:
            context = self._by_id[self._active.popleft()]
            context.removed = True
            context.removed_monotonic_ns = removed_at
            context._touch()
            cleared.append(context)
        return cleared

    def prune_removed(self, *, cutoff_ns: int) -> list[str]:
        """보존 기간이 끝난 비활성 Context를 제거하고 bounded tombstone을 남깁니다."""

        pruned: list[str] = []
        for product_id, context in tuple(self._by_id.items()):
            if (
                not context.removed
                or context.removed_monotonic_ns <= 0
                or context.removed_monotonic_ns > cutoff_ns
            ):
                continue
            self._by_id.pop(product_id, None)
            self._id_by_sequence.pop(context.fifo_sequence, None)
            product_key = (context.product_id, context.fifo_sequence)
            self._retired_products[product_key] = None
            self._retired_products.move_to_end(product_key)
            for capture_id, owner in tuple(self._capture_owner.items()):
                if owner[0] != context.product_id:
                    continue
                self._capture_owner.pop(capture_id, None)
                self._retired_captures[capture_id] = None
                self._retired_captures.move_to_end(capture_id)
            self._trim_retired()
            pruned.append(product_id)
        return pruned

    def is_retired_product(self, product_id: str, fifo_sequence: int) -> bool:
        return (product_id, fifo_sequence) in self._retired_products

    def is_retired_capture(self, capture_id: str) -> bool:
        return capture_id in self._retired_captures

    def _trim_retired(self) -> None:
        while len(self._retired_products) > self._retired_capacity:
            self._retired_products.popitem(last=False)
        while len(self._retired_captures) > self._retired_capacity * 2:
            self._retired_captures.popitem(last=False)


class ProductResultReorderBuffer:
    """병렬 결과를 fifo_sequence 순서로만 꺼냅니다."""

    def __init__(self, first_sequence: int = 1) -> None:
        if first_sequence < 1:
            raise ValueError("first_sequence must be positive")
        self._next_sequence = first_sequence
        self._pending: dict[int, LockedProduct] = {}

    def add(self, result: LockedProduct) -> list[LockedProduct]:
        if result.fifo_sequence < self._next_sequence:
            return []
        existing = self._pending.get(result.fifo_sequence)
        if existing is not None and existing != result:
            raise ValueError("conflicting lock for fifo_sequence")
        self._pending[result.fifo_sequence] = result
        ready: list[LockedProduct] = []
        while self._next_sequence in self._pending:
            ready.append(self._pending.pop(self._next_sequence))
            self._next_sequence += 1
        return ready

    def reset(self, first_sequence: int) -> None:
        if first_sequence < 1:
            raise ValueError("first_sequence must be positive")
        self._next_sequence = first_sequence
        self._pending.clear()


class StationMessageOutcome(StrEnum):
    """station result/failure를 원장에 반영하기 전 내린 판단입니다."""

    INVALID_STATION_ID = "INVALID_STATION_ID"
    EXPIRED_PRODUCT = "EXPIRED_PRODUCT"
    CAPTURE_OWNER_CONFLICT = "CAPTURE_OWNER_CONFLICT"
    UNKNOWN_PRODUCT = "UNKNOWN_PRODUCT"
    UNREGISTERED_CAPTURE = "UNREGISTERED_CAPTURE"
    REMOVED_PRODUCT = "REMOVED_PRODUCT"
    SUPERSEDED_CAPTURE = "SUPERSEDED_CAPTURE"
    LATE_AFTER_LOCK = "LATE_AFTER_LOCK"
    CONTRACT_INCOMPLETE = "CONTRACT_INCOMPLETE"
    ACCEPTED = "ACCEPTED"


class StationContractDefect(StrEnum):
    """CONTRACT_INCOMPLETE의 세부 사유입니다."""

    NONE = "NONE"
    SCORE_NOT_FINITE = "SCORE_NOT_FINITE"
    PAYLOAD_INCOMPLETE = "PAYLOAD_INCOMPLETE"


@dataclass(frozen=True, slots=True)
class StationMessageFacts:
    """ROS 메시지에서 판단에 필요한 값만 뽑아낸 순수 데이터입니다.

    score는 호출자가 finite 검사를 마친 값을 넘깁니다. NaN/inf는 None입니다.
    """

    product_id: str
    fifo_sequence: int
    station_id_raw: int
    capture_id: str
    frame_batch_id: str
    inference_job_id: str
    result_revision: int
    verdict_raw: int | None = None
    score: float | None = None


@dataclass(frozen=True, slots=True)
class StationMessageDecision:
    """판단 결과입니다. 부작용 실행은 호출한 ROS 콜백이 담당합니다."""

    outcome: StationMessageOutcome
    station_id: StationId | None = None
    context: ProductContext | None = None
    verdict: Verdict | None = None
    active_capture_id: str = ""
    defect: StationContractDefect = StationContractDefect.NONE


def classify_station_message(
    ledger: ProductLedger,
    facts: StationMessageFacts,
    *,
    expects_verdict: bool,
) -> StationMessageDecision:
    """원장을 읽어 이 메시지를 어떻게 처리할지만 결정합니다.

    원장을 변경하지 않고 로그도 남기지 않습니다. 실제 반영과 로그 발행,
    FAULT_STOP 전이는 호출한 ROS 콜백이 outcome을 보고 수행합니다.

    `expects_verdict`는 StationResult(True)와 StationInferenceFailed(False)를
    구분합니다. 실패 메시지에는 verdict/score가 없으므로 그 검사를 건너뜁니다.

    station_id 파싱을 원장 조회보다 먼저 수행합니다. 형식이 깨진 메시지는
    원장을 조회할 가치가 없고, "메시지가 오염됐다"가 "제품을 못 찾겠다"보다
    조치 가능한 진단 정보이기 때문입니다.
    """

    try:
        station_id = StationId(facts.station_id_raw)
    except ValueError:
        return StationMessageDecision(StationMessageOutcome.INVALID_STATION_ID)

    verdict: Verdict | None = None
    if expects_verdict and facts.verdict_raw is not None:
        try:
            verdict = Verdict(facts.verdict_raw)
        except ValueError:
            verdict = None

    context = ledger.get(facts.product_id, facts.fifo_sequence)
    if context is None:
        if ledger.is_retired_product(
            facts.product_id, facts.fifo_sequence
        ) or ledger.is_retired_capture(facts.capture_id):
            return StationMessageDecision(
                StationMessageOutcome.EXPIRED_PRODUCT, station_id=station_id
            )
        if ledger.capture_owner(facts.capture_id) is not None:
            return StationMessageDecision(
                StationMessageOutcome.CAPTURE_OWNER_CONFLICT, station_id=station_id
            )
        return StationMessageDecision(
            StationMessageOutcome.UNKNOWN_PRODUCT, station_id=station_id
        )

    if ledger.capture_owner(facts.capture_id) != (context.product_id, station_id):
        return StationMessageDecision(
            StationMessageOutcome.UNREGISTERED_CAPTURE,
            station_id=station_id,
            context=context,
        )
    if context.removed:
        return StationMessageDecision(
            StationMessageOutcome.REMOVED_PRODUCT,
            station_id=station_id,
            context=context,
        )
    active_capture_id = context.station(station_id).capture_id
    if active_capture_id != facts.capture_id:
        return StationMessageDecision(
            StationMessageOutcome.SUPERSEDED_CAPTURE,
            station_id=station_id,
            context=context,
            active_capture_id=active_capture_id,
        )
    if context.locked is not None:
        return StationMessageDecision(
            StationMessageOutcome.LATE_AFTER_LOCK,
            station_id=station_id,
            context=context,
        )

    defect = StationContractDefect.NONE
    if expects_verdict and facts.score is None:
        defect = StationContractDefect.SCORE_NOT_FINITE
    elif (
        (expects_verdict and verdict is None)
        or facts.result_revision < 1
        or not facts.frame_batch_id
        or not facts.inference_job_id
    ):
        defect = StationContractDefect.PAYLOAD_INCOMPLETE
    if defect is not StationContractDefect.NONE:
        return StationMessageDecision(
            StationMessageOutcome.CONTRACT_INCOMPLETE,
            station_id=station_id,
            context=context,
            defect=defect,
        )

    return StationMessageDecision(
        StationMessageOutcome.ACCEPTED,
        station_id=station_id,
        context=context,
        verdict=verdict,
    )
