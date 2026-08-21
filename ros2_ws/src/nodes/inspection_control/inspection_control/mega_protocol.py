"""Small, deterministic serial protocol shared by ControlNode and Mega."""

from __future__ import annotations

from dataclasses import dataclass


def crc16_ccitt(value: str) -> int:
    crc = 0xFFFF
    for byte in value.encode("ascii"):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(*fields: object) -> bytes:
    body = "|".join(str(field) for field in fields)
    return f"{body}|{crc16_ccitt(body):04X}\n".encode("ascii")


def decode_frame(line: bytes) -> list[str] | None:
    try:
        parts = line.decode("ascii").strip().split("|")
        if len(parts) < 2:
            return None
        body = "|".join(parts[:-1])
        if int(parts[-1], 16) != crc16_ccitt(body):
            return None
        return parts[:-1]
    except (UnicodeDecodeError, ValueError):
        return None


@dataclass(frozen=True)
class MegaEvent:
    kind: str
    values: tuple[str, ...]


def parse_event(fields: list[str]) -> MegaEvent | None:
    if len(fields) < 2 or fields[0] != "E":
        return None
    return MegaEvent(fields[1], tuple(fields[2:]))