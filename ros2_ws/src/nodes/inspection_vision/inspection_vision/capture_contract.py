"""카메라 SDK, 파일 저장소와 ROS adapter 사이의 순수 Python 계약."""

from __future__ import annotations

import hashlib
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


class CaptureError(RuntimeError):
    """촬영 단계에서 분류 가능한 오류입니다."""

    def __init__(self, reason: str, *, retryable: bool = True) -> None:
        super().__init__(reason)
        self.reason = reason
        self.retryable = retryable


class CaptureSkewExceeded(CaptureError):
    """필수 카메라의 host callback 도착 skew가 허용치를 넘었습니다."""


class CaptureStorageError(CaptureError):
    """RGB PNG 또는 manifest 내구 저장이 완료되지 않았습니다."""


class CameraUnavailable(CaptureError):
    """필수 카메라가 준비되지 않았거나 프레임을 반환하지 않았습니다."""


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """SDK callback에서 즉시 복사한 RGB8 packed 프레임과 상관관계 정보."""

    camera_id: str
    width: int
    height: int
    rgb_bytes: bytes
    frame_number: int
    external_trigger_count: int
    camera_timestamp_raw: int
    camera_timestamp_domain: str
    camera_timestamp_ns: int
    camera_timestamp_synchronized: bool
    sdk_host_timestamp_raw: int
    host_arrival_monotonic_ns: int
    host_arrival_wall_time_ns: int

    def validate(self) -> None:
        if not self.camera_id:
            raise ValueError("camera_id is required")
        if self.width < 1 or self.height < 1:
            raise ValueError("frame dimensions must be positive")
        if len(self.rgb_bytes) != self.width * self.height * 3:
            raise ValueError("RGB8 packed byte length does not match dimensions")
        if self.frame_number < 0 or self.external_trigger_count < 0:
            raise ValueError("frame counters cannot be negative")
        if self.host_arrival_monotonic_ns <= 0 or self.host_arrival_wall_time_ns <= 0:
            raise ValueError("host arrival timestamps are required")


@dataclass(frozen=True, slots=True)
class RawCaptureBatch:
    """한 번의 station Action Command에서 연결된 메모리 프레임 묶음."""

    product_id: str
    station_id: int
    capture_id: str
    attempt: int
    trigger_requested_monotonic_ns: int
    trigger_returned_monotonic_ns: int
    trigger_requested_wall_time_ns: int
    trigger_returned_wall_time_ns: int
    frames: tuple[CameraFrame, ...]

    @property
    def frame_arrival_skew_us(self) -> int:
        arrivals = [frame.host_arrival_monotonic_ns for frame in self.frames]
        return 0 if len(arrivals) < 2 else (max(arrivals) - min(arrivals)) // 1_000

    def validate(self, required_camera_ids: tuple[str, ...]) -> None:
        if self.station_id not in {1, 2}:
            raise ValueError("station_id must be 1 or 2")
        if self.attempt < 1:
            raise ValueError("attempt must be positive")
        if self.trigger_requested_monotonic_ns <= 0:
            raise ValueError("trigger requested monotonic timestamp is missing")
        if self.trigger_returned_monotonic_ns < self.trigger_requested_monotonic_ns:
            raise ValueError("trigger returned before it was requested")
        if self.trigger_requested_wall_time_ns <= 0:
            raise ValueError("trigger requested wall timestamp is missing")
        if self.trigger_returned_wall_time_ns < self.trigger_requested_wall_time_ns:
            raise ValueError("trigger wall timestamps are not monotonic")
        actual = tuple(frame.camera_id for frame in self.frames)
        if len(actual) != len(set(actual)):
            raise ValueError("duplicate camera_id in raw capture")
        if set(actual) != set(required_camera_ids):
            raise ValueError("raw capture does not contain every required camera")
        for frame in self.frames:
            frame.validate()


@dataclass(frozen=True, slots=True)
class ImageArtifact:
    camera_id: str
    file_path: str
    sha256: str
    file_size_bytes: int
    width: int
    height: int
    pixel_format: str
    camera_timestamp_raw: int
    camera_timestamp_domain: str
    camera_timestamp_ns: int
    camera_timestamp_synchronized: bool
    host_arrival_monotonic_ns: int
    host_arrival_timestamp_ns: int
    frame_number: int = 0
    external_trigger_count: int = 0
    sdk_host_timestamp_raw: int = 0


@dataclass(frozen=True, slots=True)
class CaptureBatch:
    product_id: str
    station_id: int
    capture_id: str
    frame_batch_id: str
    attempt: int
    trigger_requested_monotonic_ns: int
    trigger_returned_monotonic_ns: int
    trigger_requested_wall_time_ns: int
    trigger_returned_wall_time_ns: int
    images: tuple[ImageArtifact, ...]
    manifest_path: str = ""

    @property
    def frame_arrival_skew_us(self) -> int:
        arrivals = [image.host_arrival_monotonic_ns for image in self.images]
        return 0 if len(arrivals) < 2 else (max(arrivals) - min(arrivals)) // 1_000

    def validate(self, required_camera_ids: tuple[str, ...]) -> None:
        if self.trigger_requested_monotonic_ns <= 0:
            raise ValueError("trigger requested monotonic timestamp is missing")
        if self.trigger_returned_monotonic_ns < self.trigger_requested_monotonic_ns:
            raise ValueError("trigger returned before it was requested")
        if self.trigger_requested_wall_time_ns <= 0:
            raise ValueError("trigger requested wall timestamp is missing")
        if self.trigger_returned_wall_time_ns < self.trigger_requested_wall_time_ns:
            raise ValueError("trigger wall timestamps are not monotonic")
        actual = tuple(image.camera_id for image in self.images)
        if len(set(actual)) != len(actual):
            raise ValueError("duplicate camera_id in CaptureBatch")
        if set(actual) != set(required_camera_ids):
            raise ValueError("CaptureBatch does not contain all required cameras")
        for image in self.images:
            path = Path(image.file_path)
            if image.pixel_format != "RGB8_PNG":
                raise ValueError("canonical image must be RGB8_PNG")
            if not path.is_absolute():
                raise ValueError("image file_path must be absolute")
            if len(image.sha256) != 64 or image.file_size_bytes < 1:
                raise ValueError("image digest/size metadata is incomplete")
            if not path.is_file() or path.stat().st_size != image.file_size_bytes:
                raise ValueError("image file is missing or size metadata mismatches")
            content = path.read_bytes()
            if hashlib.sha256(content).hexdigest() != image.sha256:
                raise ValueError("image SHA-256 metadata mismatches")
            _validate_rgb8_png(content, image.width, image.height)


def _validate_rgb8_png(content: bytes, expected_width: int, expected_height: int) -> None:
    """stdlib만으로 PNG CRC·압축 stream과 RGB8 IHDR를 검증합니다."""

    if not content.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("image file is not a PNG container")
    offset = 8
    width = height = bit_depth = color_type = interlace = None
    compressed = bytearray()
    saw_iend = False
    while offset + 12 <= len(content):
        length = struct.unpack(">I", content[offset : offset + 4])[0]
        chunk_type = content[offset + 4 : offset + 8]
        data_start = offset + 8
        data_end = data_start + length
        crc_end = data_end + 4
        if crc_end > len(content):
            raise ValueError("PNG chunk exceeds file boundary")
        chunk_data = content[data_start:data_end]
        expected_crc = struct.unpack(">I", content[data_end:crc_end])[0]
        actual_crc = zlib.crc32(chunk_type + chunk_data) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise ValueError("PNG chunk CRC mismatch")
        if chunk_type == b"IHDR":
            if length != 13 or width is not None:
                raise ValueError("PNG IHDR is invalid")
            width, height, bit_depth, color_type, compression, filtering, interlace = (
                struct.unpack(">IIBBBBB", chunk_data)
            )
            if compression != 0 or filtering != 0:
                raise ValueError("unsupported PNG compression/filter method")
        elif chunk_type == b"IDAT":
            compressed.extend(chunk_data)
        elif chunk_type == b"IEND":
            if length != 0:
                raise ValueError("PNG IEND is invalid")
            saw_iend = True
            offset = crc_end
            break
        offset = crc_end
    if not saw_iend or offset != len(content):
        raise ValueError("PNG does not end with a valid IEND chunk")
    if (width, height) != (expected_width, expected_height):
        raise ValueError("PNG dimensions do not match metadata")
    if bit_depth != 8 or color_type != 2 or interlace != 0:
        raise ValueError("canonical PNG must be non-interlaced 8-bit RGB")
    try:
        scanlines = zlib.decompress(bytes(compressed))
    except zlib.error as exc:
        raise ValueError("PNG image data cannot be decompressed") from exc
    expected_length = expected_height * (1 + expected_width * 3)
    if len(scanlines) != expected_length:
        raise ValueError("PNG RGB scanline length is invalid")
    stride = 1 + expected_width * 3
    if any(scanlines[row * stride] > 4 for row in range(expected_height)):
        raise ValueError("PNG scanline uses an invalid filter type")


class CaptureBackend(Protocol):
    async def initialize(self) -> dict[str, object]:
        """장치 열기, trigger 설정, callback 등록과 grabbing을 완료합니다."""

    async def capture_station(
        self,
        *,
        product_id: str,
        station_id: int,
        capture_id: str,
        attempt: int,
        required_camera_ids: tuple[str, ...],
    ) -> RawCaptureBatch:
        """GigE Action Command 1회에 대응하는 RGB 메모리 프레임을 반환합니다."""

    async def prepare_retry(
        self, *, station_id: int, required_camera_ids: tuple[str, ...]
    ) -> None:
        """고정 sleep 없이 SDK buffer clear와 trigger 재무장을 완료합니다."""

    async def recover_station(
        self, *, station_id: int, required_camera_ids: tuple[str, ...]
    ) -> bool:
        """재연결과 3회 시험 촬영을 수행하고 복구 성공 여부를 반환합니다."""

    async def close(self) -> None:
        """grabbing/handle/SDK 자원을 역순으로 안전하게 해제합니다."""
