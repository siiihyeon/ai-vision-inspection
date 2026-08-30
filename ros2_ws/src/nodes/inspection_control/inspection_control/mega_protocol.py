"""Small, deterministic serial protocol shared by ControlNode and Mega."""

from __future__ import annotations

from dataclasses import dataclass
import math


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
    return decode_frame_diagnostic(line).fields


@dataclass(frozen=True)
class MegaEvent:
    kind: str
    values: tuple[str, ...]


@dataclass(frozen=True)
class FrameDecodeResult:
    fields: list[str] | None
    rejection_reason: str | None


@dataclass(frozen=True)
class Sensor3Telemetry:
    event: str
    firmware_millis: int
    firmware_micros: int
    distance_cm: float
    detection_armed: bool
    consecutive_detect_count: int
    consecutive_release_count: int


SENSOR3_TELEMETRY_EVENTS = frozenset(
    {
        "READ",
        "RELEASE_CHECK",
        "REARMED",
        "RELEASED",
        "TIMEOUT",
        "REARMED_TIMEOUT",
    }
)


def parse_event(fields: list[str]) -> MegaEvent | None:
    if len(fields) < 2 or fields[0] != "E":
        return None
    return MegaEvent(fields[1], tuple(fields[2:]))


def decode_frame_diagnostic(line: bytes) -> FrameDecodeResult:
    """Decode one Mega line while retaining a stable rejection category."""

    if not line:
        return FrameDecodeResult(None, "empty_read")
    try:
        text = line.decode("ascii").strip()
    except UnicodeDecodeError:
        return FrameDecodeResult(None, "ascii_decode_error")
    parts = text.split("|")
    if len(parts) < 2:
        return FrameDecodeResult(None, "missing_crc_field")
    body = "|".join(parts[:-1])
    try:
        received_crc = int(parts[-1], 16)
    except ValueError:
        return FrameDecodeResult(None, "invalid_crc_text")
    if received_crc != crc16_ccitt(body):
        return FrameDecodeResult(None, "crc_mismatch")
    return FrameDecodeResult(parts[:-1], None)


def parse_sensor3_telemetry(fields: list[str]) -> Sensor3Telemetry | None:
    """첨부 firmware의 CRC-framed Sensor3 진단 LOG를 엄격히 해석합니다."""

    if len(fields) != 9 or fields[:2] != ["LOG", "SENSOR3"]:
        return None
    event = fields[2]
    if event not in SENSOR3_TELEMETRY_EVENTS:
        return None
    expected_keys = (
        "millis",
        "micros",
        "distanceCm",
        "armed",
        "detectCount",
        "releaseCount",
    )
    values: dict[str, str] = {}
    for field, expected_key in zip(fields[3:], expected_keys, strict=True):
        key, separator, value = field.partition("=")
        if separator != "=" or key != expected_key or not value:
            return None
        values[key] = value
    try:
        firmware_millis = int(values["millis"])
        firmware_micros = int(values["micros"])
        distance_cm = float(values["distanceCm"])
        armed = int(values["armed"])
        detect_count = int(values["detectCount"])
        release_count = int(values["releaseCount"])
    except ValueError:
        return None
    if (
        firmware_millis < 0
        or firmware_micros < 0
        or not math.isfinite(distance_cm)
        or armed not in {0, 1}
        or detect_count < 0
        or release_count < 0
    ):
        return None
    return Sensor3Telemetry(
        event=event,
        firmware_millis=firmware_millis,
        firmware_micros=firmware_micros,
        distance_cm=distance_cm,
        detection_armed=bool(armed),
        consecutive_detect_count=detect_count,
        consecutive_release_count=release_count,
    )
