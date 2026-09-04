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


@dataclass(frozen=True)
class SensorDistanceEvent:
    sensor_id: str
    sensor_sequence: int
    distances_cm: tuple[float, float, float]


@dataclass(frozen=True)
class SensorDiagnosticEvent:
    sensor_id: str
    event: str
    sensor_sequence: int
    firmware_millis: int
    firmware_micros: int
    distance_cm: float
    detection_armed: bool
    consecutive_detect_count: int
    consecutive_release_count: int
    conveyor_state: int
    pending_detection: bool
    reason: str


SENSOR_DIAGNOSTIC_EVENTS = frozenset(
    {
        "CLOSE_SAMPLE",
        "DETECTION_DROPPED",
        "DETECTION_RESET_FAR",
        "DETECTION_RESET_HYSTERESIS",
        "DETECTION_RESET_INVALID",
        "DETECTION_RESET_TIMEOUT",
        "EVENT_SENT",
        "REARMED",
        "RELEASE_ECHO",
        "RELEASE_TIMEOUT",
    }
)


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


def parse_sensor_distance(fields: list[str]) -> SensorDistanceEvent | None:
    """Parse the three detection samples emitted for Sensor 1 or Sensor 2."""

    if len(fields) != 7 or fields[:2] != ["E", "SENSOR_DISTANCE"]:
        return None
    sensor_id = fields[2]
    if sensor_id not in {"SENSOR_1", "SENSOR_2"}:
        return None
    try:
        sensor_sequence = int(fields[3])
        distances = tuple(float(value) for value in fields[4:7])
    except ValueError:
        return None
    if sensor_sequence < 0 or any(
        not math.isfinite(distance) or distance < 0.0 for distance in distances
    ):
        return None
    return SensorDistanceEvent(
        sensor_id=sensor_id,
        sensor_sequence=sensor_sequence,
        distances_cm=distances,
    )


def parse_sensor_diagnostic(fields: list[str]) -> SensorDiagnosticEvent | None:
    """Parse one bounded Sensor 1/2 firmware diagnostic observation."""

    if len(fields) != 14 or fields[:2] != ["E", "SENSOR_DIAGNOSTIC"]:
        return None
    sensor_id, event = fields[2], fields[3]
    if sensor_id not in {"SENSOR_1", "SENSOR_2"}:
        return None
    if event not in SENSOR_DIAGNOSTIC_EVENTS:
        return None
    try:
        sensor_sequence = int(fields[4])
        firmware_millis = int(fields[5])
        firmware_micros = int(fields[6])
        distance_cm = float(fields[7])
        armed = int(fields[8])
        detect_count = int(fields[9])
        release_count = int(fields[10])
        conveyor_state = int(fields[11])
        pending = int(fields[12])
    except ValueError:
        return None
    reason = fields[13]
    reason_characters_valid = bool(reason) and all(
        character == "_" or character.isdigit() or "A" <= character <= "Z"
        for character in reason
    )
    if (
        sensor_sequence < 0
        or firmware_millis < 0
        or firmware_micros < 0
        or not math.isfinite(distance_cm)
        or distance_cm < -1.0
        or armed not in {0, 1}
        or not 0 <= detect_count <= 255
        or not 0 <= release_count <= 255
        or conveyor_state not in {0, 1, 2, 3}
        or pending not in {0, 1}
        or not reason_characters_valid
    ):
        return None
    return SensorDiagnosticEvent(
        sensor_id=sensor_id,
        event=event,
        sensor_sequence=sensor_sequence,
        firmware_millis=firmware_millis,
        firmware_micros=firmware_micros,
        distance_cm=distance_cm,
        detection_armed=bool(armed),
        consecutive_detect_count=detect_count,
        consecutive_release_count=release_count,
        conveyor_state=conveyor_state,
        pending_detection=bool(pending),
        reason=reason,
    )


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
