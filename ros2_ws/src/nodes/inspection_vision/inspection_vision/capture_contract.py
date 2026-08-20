"""HIKROBOT SDK adapter와 ROS Action 사이의 순수 Python 계약."""

from __future__ import annotations

import hashlib
import struct
import time
import uuid
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


def _write_fake_rgb8_png(path: Path, width: int, height: int) -> None:
    """검증용 최소 크기의 단색 RGB8 PNG를 실제로 디스크에 씁니다."""

    path.parent.mkdir(parents=True, exist_ok=True)
    scanlines = bytearray()
    for _ in range(height):
        scanlines.append(0)  # PNG filter type: None
        scanlines.extend(bytes([200, 200, 200]) * width)
    compressed = zlib.compress(bytes(scanlines))

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + chunk_type
            + data
            + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    content = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", compressed)
        + chunk(b"IEND", b"")
    )
    path.write_bytes(content)


class FakeCaptureBackend:
    """실제 카메라 없이 파이프라인 전체를 검증하기 위한 sim 전용 백엔드."""

    def __init__(self, *, data_root: Path) -> None:
        self._data_root = data_root

    async def capture_station(
        self,
        *,
        product_id: str,
        station_id: int,
        capture_id: str,
        attempt: int,
        required_camera_ids: tuple[str, ...],
    ) -> CaptureBatch:
        now_ns = time.monotonic_ns()
        wall_ns = time.time_ns()
        images = []
        for camera_id in required_camera_ids:
            width, height = 64, 48
            path = self._data_root / f"{capture_id}_{camera_id}_{attempt}.png"
            _write_fake_rgb8_png(path, width, height)
            content = path.read_bytes()
            images.append(
                ImageArtifact(
                    camera_id=camera_id,
                    file_path=str(path),
                    sha256=hashlib.sha256(content).hexdigest(),
                    file_size_bytes=len(content),
                    width=width,
                    height=height,
                    pixel_format="RGB8_PNG",
                    camera_timestamp_raw=wall_ns,
                    camera_timestamp_domain="fake",
                    camera_timestamp_ns=wall_ns,
                    camera_timestamp_synchronized=False,
                    host_arrival_monotonic_ns=now_ns,
                    host_arrival_timestamp_ns=wall_ns,
                )
            )
        return CaptureBatch(
            product_id=product_id,
            station_id=station_id,
            capture_id=capture_id,
            frame_batch_id=uuid.uuid4().hex,
            attempt=attempt,
            trigger_requested_monotonic_ns=now_ns,
            trigger_returned_monotonic_ns=now_ns + 1,
            trigger_requested_wall_time_ns=wall_ns,
            trigger_returned_wall_time_ns=wall_ns + 1,
            images=tuple(images),
        )
