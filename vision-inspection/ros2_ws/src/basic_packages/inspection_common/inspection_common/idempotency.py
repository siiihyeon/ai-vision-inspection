"""같은 ID 재전송과 충돌을 구분하는 bounded 결과 저장소입니다."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from enum import StrEnum
from threading import RLock
from typing import Generic, TypeVar

T = TypeVar("T")


class ReplayKind(StrEnum):
    NEW = "NEW"
    REPLAY = "REPLAY"
    CONFLICT = "CONFLICT"


@dataclass(frozen=True, slots=True)
class ReplayDecision(Generic[T]):
    kind: ReplayKind
    result: T | None = None


class IdempotencyStore(Generic[T]):
    def __init__(self, capacity: int = 4096) -> None:
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self._capacity = capacity
        self._entries: OrderedDict[str, tuple[str, T]] = OrderedDict()
        self._lock = RLock()

    def inspect(self, command_id: str, digest: str) -> ReplayDecision[T]:
        with self._lock:
            entry = self._entries.get(command_id)
            if entry is None:
                return ReplayDecision(ReplayKind.NEW)
            stored_digest, result = entry
            self._entries.move_to_end(command_id)
            if stored_digest != digest:
                return ReplayDecision(ReplayKind.CONFLICT)
            return ReplayDecision(ReplayKind.REPLAY, result)

    def remember(self, command_id: str, digest: str, result: T) -> None:
        with self._lock:
            existing = self._entries.get(command_id)
            if existing is not None and existing[0] != digest:
                raise ValueError("same command_id cannot be stored with another digest")
            self._entries[command_id] = (digest, result)
            self._entries.move_to_end(command_id)
            while len(self._entries) > self._capacity:
                self._entries.popitem(last=False)
