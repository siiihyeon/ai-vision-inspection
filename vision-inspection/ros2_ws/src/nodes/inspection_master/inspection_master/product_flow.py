"""Master가 단독 소유하는 A/B 결과 결합과 FIFO 잠금 규칙."""

from __future__ import annotations

from dataclasses import dataclass, field

from inspection_common.constants import StationId, Verdict


@dataclass(frozen=True, slots=True)
class StationDecision:
    station_id: StationId
    verdict: Verdict
    revision: int
    capture_id: str
    inference_job_id: str


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
class ProductContext:
    product_id: str
    fifo_sequence: int
    stations: dict[StationId, StationDecision] = field(default_factory=dict)
    locked: LockedProduct | None = None

    def apply_station_result(self, decision: StationDecision) -> bool:
        """더 최신인 station revision만 반영하며 잠금 뒤 결과는 거부합니다."""

        if self.locked is not None:
            return False
        previous = self.stations.get(decision.station_id)
        if previous is not None and decision.revision <= previous.revision:
            return False
        self.stations[decision.station_id] = decision
        return True

    def lock_if_complete(self) -> LockedProduct | None:
        if self.locked is not None:
            return self.locked
        if StationId.A not in self.stations or StationId.B not in self.stations:
            return None
        verdict = (
            Verdict.PASS
            if all(item.verdict == Verdict.PASS for item in self.stations.values())
            else Verdict.NG
        )
        self.locked = LockedProduct(
            product_id=self.product_id,
            fifo_sequence=self.fifo_sequence,
            verdict=verdict,
            station_a_completed=True,
            station_b_completed=True,
            reason="both station results completed",
        )
        return self.locked

    def lock_explicit_failure(self, station_id: StationId, reason: str) -> LockedProduct:
        """명시적 station 실패는 즉시 FORCED_NG로 잠급니다."""

        if self.locked is None:
            self.locked = LockedProduct(
                product_id=self.product_id,
                fifo_sequence=self.fifo_sequence,
                verdict=Verdict.FORCED_NG,
                station_a_completed=StationId.A in self.stations,
                station_b_completed=StationId.B in self.stations,
                reason=f"station {station_id.name} failed: {reason}",
            )
        return self.locked

    def lock_at_sensor3(self, sensor3_event_id: str) -> LockedProduct:
        """Sensor3 도착 시 미완료 제품을 FORCED_NG로 확정합니다."""

        complete = self.lock_if_complete()
        if complete is not None:
            if complete.sensor3_event_id:
                return complete
            self.locked = LockedProduct(
                product_id=complete.product_id,
                fifo_sequence=complete.fifo_sequence,
                verdict=complete.verdict,
                station_a_completed=True,
                station_b_completed=True,
                reason=complete.reason,
                sensor3_event_id=sensor3_event_id,
            )
            return self.locked
        self.locked = LockedProduct(
            product_id=self.product_id,
            fifo_sequence=self.fifo_sequence,
            verdict=Verdict.FORCED_NG,
            station_a_completed=StationId.A in self.stations,
            station_b_completed=StationId.B in self.stations,
            reason="station result incomplete at Sensor3",
            sensor3_event_id=sensor3_event_id,
        )
        return self.locked


class ProductLedger:
    """product_id/fifo_sequence 유일성과 결과 적용을 관리합니다."""

    def __init__(self) -> None:
        self._by_id: dict[str, ProductContext] = {}
        self._id_by_sequence: dict[int, str] = {}

    def register(self, product_id: str, fifo_sequence: int) -> ProductContext:
        if not product_id or fifo_sequence < 1:
            raise ValueError("product_id and positive fifo_sequence are required")
        existing = self._by_id.get(product_id)
        if existing is not None:
            if existing.fifo_sequence != fifo_sequence:
                raise ValueError("product_id already has another fifo_sequence")
            return existing
        if fifo_sequence in self._id_by_sequence:
            raise ValueError("fifo_sequence already belongs to another product")
        context = ProductContext(product_id=product_id, fifo_sequence=fifo_sequence)
        self._by_id[product_id] = context
        self._id_by_sequence[fifo_sequence] = product_id
        return context

    def get(self, product_id: str, fifo_sequence: int) -> ProductContext | None:
        context = self._by_id.get(product_id)
        if context is None or context.fifo_sequence != fifo_sequence:
            return None
        return context


class ProductResultReorderBuffer:
    """병렬 추론 결과를 fifo_sequence 순서로만 꺼냅니다."""

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
