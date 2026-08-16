"""교차 노드 식별자의 생성과 형식 검증입니다."""

from __future__ import annotations

from uuid import UUID, uuid4


def new_uuid() -> str:
    """소문자 하이픈 표기의 UUIDv4를 반환합니다."""

    return str(uuid4())


def is_uuid4(value: str) -> bool:
    try:
        parsed = UUID(value)
    except (AttributeError, TypeError, ValueError):
        return False
    return parsed.version == 4 and str(parsed) == value.lower()
