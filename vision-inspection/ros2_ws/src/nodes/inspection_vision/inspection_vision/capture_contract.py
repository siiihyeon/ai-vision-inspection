"""HIKROBOT SDK adapter와 ROS Action 사이의 순수 Python 계약."""

from __future__ import annotations

import hashlib
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
    host_arrival_monotonic_ns: int
    host_arrival_timestamp_ns: int


@dataclass(frozen=True, slots=True)
class CaptureBatch:
    product_id: str
    station_id: int
    capture_id: str
    frame_batch_id: str
    attempt: int
    images: tuple[ImageArtifact, ...]

    @property
    def frame_arrival_skew_us(self) -> int:
        arrivals = [image.host_arrival_monotonic_ns for image in self.images]
        return 0 if len(arrivals) < 2 else (max(arrivals) - min(arrivals)) // 1_000

    def validate(self, required_camera_ids: tuple[str, ...]) -> None:
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
            if not content.startswith(b"\x89PNG\r\n\x1a\n"):
                raise ValueError("image file is not a readable PNG container")
            if hashlib.sha256(content).hexdigest() != image.sha256:
                raise ValueError("image SHA-256 metadata mismatches")


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
