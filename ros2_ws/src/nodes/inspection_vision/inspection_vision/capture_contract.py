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


MONO8_PNG = "MONO8_PNG"
RGB8_PNG = "RGB8_PNG"
SUPPORTED_CANONICAL_PIXEL_FORMATS = frozenset({MONO8_PNG, RGB8_PNG})


class CapturePacketLossError(ValueError):
    """필수 frame에 복구되지 않은 GigE packet 손실이 남았습니다."""


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
    # 해당 frame 전송에서 끝내 복구하지 못한 packet 수입니다. SDK가 누적
    # counter만 제공하면 capture 직전/직후 counter의 delta를 기록해야 합니다.
    packet_loss_count: int
    # 재전송으로 정상 복구된 packet 수입니다. 0이 아니어도 frame은 유효할 수
    # 있지만 네트워크 품질 분석을 위해 packet loss와 분리해 보존합니다.
    packet_resend_count: int


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

    def validate(
        self,
        required_camera_ids: tuple[str, ...],
        *,
        expected_pixel_format: str | None = None,
    ) -> None:
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
        if (
            expected_pixel_format is not None
            and expected_pixel_format not in SUPPORTED_CANONICAL_PIXEL_FORMATS
        ):
            raise ValueError("expected canonical pixel format is unsupported")
        for image in self.images:
            path = Path(image.file_path)
            if image.pixel_format not in SUPPORTED_CANONICAL_PIXEL_FORMATS:
                raise ValueError("canonical image pixel format is unsupported")
            if (
                expected_pixel_format is not None
                and image.pixel_format != expected_pixel_format
            ):
                raise ValueError("canonical image pixel format differs from configuration")
            if image.packet_loss_count < 0 or image.packet_resend_count < 0:
                raise ValueError("packet counters must not be negative")
            if image.packet_loss_count != 0:
                raise CapturePacketLossError(
                    "capture contains unrecovered packet loss"
                )
            if not path.is_absolute():
                raise ValueError("image file_path must be absolute")
            if len(image.sha256) != 64 or image.file_size_bytes < 1:
                raise ValueError("image digest/size metadata is incomplete")
            if not path.is_file() or path.stat().st_size != image.file_size_bytes:
                raise ValueError("image file is missing or size metadata mismatches")
            content = path.read_bytes()
            if hashlib.sha256(content).hexdigest() != image.sha256:
                raise ValueError("image SHA-256 metadata mismatches")
            _validate_canonical_png(
                content,
                image.width,
                image.height,
                image.pixel_format,
            )


def _validate_canonical_png(
    content: bytes,
    expected_width: int,
    expected_height: int,
    pixel_format: str,
) -> None:
    """stdlib만으로 PNG CRC·압축 stream과 MONO8/RGB8 IHDR를 검증합니다."""

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
    expected_color_type = 0 if pixel_format == MONO8_PNG else 2
    channels = 1 if pixel_format == MONO8_PNG else 3
    if bit_depth != 8 or color_type != expected_color_type or interlace != 0:
        raise ValueError(
            "canonical PNG color type does not match declared pixel format"
        )
    try:
        scanlines = zlib.decompress(bytes(compressed))
    except zlib.error as exc:
        raise ValueError("PNG image data cannot be decompressed") from exc
    expected_length = expected_height * (1 + expected_width * channels)
    if len(scanlines) != expected_length:
        raise ValueError("PNG scanline length is invalid")
    stride = 1 + expected_width * channels
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
        """GigE Action Command 1회와 atomic canonical PNG 저장을 수행합니다."""


class UnimplementedCaptureBackend:
    async def capture_station(self, **_kwargs) -> CaptureBatch:
        raise NotImplementedError("HIKROBOT MVS GigE Action Command adapter is not implemented")


def _write_fake_png(
    path: Path,
    width: int,
    height: int,
    pixel_format: str,
) -> None:
    """검증용 최소 크기의 MONO8/RGB8 PNG를 실제로 디스크에 씁니다."""

    path.parent.mkdir(parents=True, exist_ok=True)
    scanlines = bytearray()
    if pixel_format == MONO8_PNG:
        color_type = 0
        row = bytes([200]) * width
    elif pixel_format == RGB8_PNG:
        color_type = 2
        row = bytes([200, 200, 200]) * width
    else:
        raise ValueError("unsupported fake canonical pixel format")
    for _ in range(height):
        scanlines.append(0)  # PNG filter type: None
        scanlines.extend(row)
    compressed = zlib.compress(bytes(scanlines))

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + chunk_type
            + data
            + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
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
            _write_fake_png(path, width, height, MONO8_PNG)
            content = path.read_bytes()
            images.append(
                ImageArtifact(
                    camera_id=camera_id,
                    file_path=str(path),
                    sha256=hashlib.sha256(content).hexdigest(),
                    file_size_bytes=len(content),
                    width=width,
                    height=height,
                    pixel_format=MONO8_PNG,
                    camera_timestamp_raw=wall_ns,
                    camera_timestamp_domain="fake",
                    camera_timestamp_ns=wall_ns,
                    camera_timestamp_synchronized=False,
                    host_arrival_monotonic_ns=now_ns,
                    host_arrival_timestamp_ns=wall_ns,
                    packet_loss_count=0,
                    packet_resend_count=0,
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
