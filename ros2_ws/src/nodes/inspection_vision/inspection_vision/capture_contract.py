"""HIKROBOT SDK adapter와 ROS Action 사이의 순수 Python 계약."""

from __future__ import annotations

import hashlib
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


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
    async def capture_station(
        self,
        *,
        product_id: str,
        station_id: int,
        capture_id: str,
        attempt: int,
        required_camera_ids: tuple[str, ...],
    ) -> CaptureBatch:
        """GigE Action Command 1회와 atomic RGB PNG 저장을 수행합니다."""


class UnimplementedCaptureBackend:
    async def capture_station(self, **_kwargs) -> CaptureBatch:
        raise NotImplementedError("HIKROBOT MVS GigE Action Command adapter is not implemented")
