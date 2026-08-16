"""멱등 명령과 설정에 쓰는 canonical SHA-256 도구입니다."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from typing import Any

_SHA256_HEX = re.compile(r"^[0-9a-f]{64}$")


def canonical_json(value: Mapping[str, Any]) -> str:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    )


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def payload_digest(value: Mapping[str, Any]) -> str:
    return sha256_text(canonical_json(value))


def is_sha256_hex(value: str) -> bool:
    return bool(_SHA256_HEX.fullmatch(value))
