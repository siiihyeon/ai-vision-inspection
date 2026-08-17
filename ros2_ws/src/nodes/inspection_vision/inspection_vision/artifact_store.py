"""Canonical RGB PNG와 FrameBatch manifest의 내구·원자 저장소."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import struct
import sys
import zlib
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from enum import StrEnum
from pathlib import Path

from inspection_common import new_uuid

from .capture_contract import (
    CaptureBatch,
    CaptureStorageError,
    ImageArtifact,
    RawCaptureBatch,
)


class DiskPressure(StrEnum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    PAUSE = "PAUSE"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True, slots=True)
class DiskPolicy:
    warning_used_percent: float = 80.0
    pause_used_percent: float = 90.0
    critical_used_percent: float = 95.0
    warning_free_bytes: int = 20 * 1024**3
    pause_free_bytes: int = 10 * 1024**3
    critical_free_bytes: int = 5 * 1024**3

    def __post_init__(self) -> None:
        percentages = (
            self.warning_used_percent,
            self.pause_used_percent,
            self.critical_used_percent,
        )
        if not 0 < percentages[0] < percentages[1] < percentages[2] <= 100:
            raise ValueError("disk used-percent thresholds must be strictly increasing")
        floors = (
            self.warning_free_bytes,
            self.pause_free_bytes,
            self.critical_free_bytes,
        )
        if not floors[0] > floors[1] > floors[2] >= 0:
            raise ValueError("disk free-byte thresholds must be strictly decreasing")

    def assess(self, *, total: int, used: int, free: int) -> DiskPressure:
        used_percent = 100.0 if total <= 0 else used * 100.0 / total
        if used_percent >= self.critical_used_percent or free <= self.critical_free_bytes:
            return DiskPressure.CRITICAL
        if used_percent >= self.pause_used_percent or free <= self.pause_free_bytes:
            return DiskPressure.PAUSE
        if used_percent >= self.warning_used_percent or free <= self.warning_free_bytes:
            return DiskPressure.WARNING
        return DiskPressure.NORMAL


def encode_rgb8_png(
    rgb_bytes: bytes, width: int, height: int, *, compression_level: int = 3
) -> bytes:
    """추가 image library 없이 non-interlaced RGB8 PNG를 생성합니다."""

    if width < 1 or height < 1:
        raise ValueError("PNG dimensions must be positive")
    if len(rgb_bytes) != width * height * 3:
        raise ValueError("RGB8 packed byte length does not match dimensions")
    if not 0 <= compression_level <= 9:
        raise ValueError("PNG compression level must be between 0 and 9")

    def chunk(kind: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(kind + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", crc)

    stride = width * 3
    scanlines = b"".join(
        b"\x00" + rgb_bytes[offset : offset + stride]
        for offset in range(0, len(rgb_bytes), stride)
    )
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(scanlines, compression_level))
        + chunk(b"IEND", b"")
    )


class ArtifactStore:
    """충돌 덮어쓰기를 금지하고 완료된 파일만 canonical 경로에 노출합니다."""

    def __init__(
        self,
        data_root: Path,
        *,
        compression_level: int = 3,
        disk_policy: DiskPolicy | None = None,
    ) -> None:
        if not data_root.is_absolute():
            raise ValueError("data_root must be absolute")
        self.data_root = data_root
        self.compression_level = compression_level
        self.disk_policy = disk_policy or DiskPolicy()
        self.data_root.mkdir(parents=True, exist_ok=True)
        self._apply_directory_mode(self.data_root)

    def disk_pressure(self) -> tuple[DiskPressure, dict[str, int | float]]:
        usage = shutil.disk_usage(self.data_root)
        pressure = self.disk_policy.assess(
            total=usage.total, used=usage.used, free=usage.free
        )
        details: dict[str, int | float] = {
            "total_bytes": usage.total,
            "used_bytes": usage.used,
            "free_bytes": usage.free,
            "used_percent": 100.0 * usage.used / max(usage.total, 1),
        }
        return pressure, details

    def cleanup_incomplete_files(self) -> tuple[str, ...]:
        """이 저장소가 생성한 `.vision-tmp` 파일만 재시작 시 제거합니다."""

        deleted: list[str] = []
        for path in self.data_root.rglob("*.vision-tmp"):
            if path.is_file():
                path.unlink()
                deleted.append(str(path))
        return tuple(deleted)

    def discover_capture_keys(self) -> frozenset[tuple[str, int, str]]:
        """현재 프로세스 시작 전에 완료된 manifest의 capture identity를 읽습니다."""

        keys: set[tuple[str, int, str]] = set()
        for path in self.data_root.glob("*/*/*/*/station_*/*/attempt_*/*/manifest.json"):
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                product_id = str(payload["product_id"])
                station_id = int(payload["station_id"])
                capture_id = str(payload["capture_id"])
                if station_id in {1, 2} and product_id and capture_id:
                    keys.add((product_id, station_id, capture_id))
            except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError):
                continue
        return frozenset(keys)

    def save_batch(
        self,
        raw: RawCaptureBatch,
        *,
        frame_batch_id: str,
        required_camera_ids: tuple[str, ...],
        capture_outcome: str,
        failure_reason: str = "",
    ) -> CaptureBatch:
        raw.validate(required_camera_ids)
        pressure, details = self.disk_pressure()
        if pressure in {DiskPressure.PAUSE, DiskPressure.CRITICAL}:
            raise CaptureStorageError(
                f"storage pressure {pressure}: free_bytes={details['free_bytes']}",
                retryable=pressure == DiskPressure.PAUSE,
            )
        for value, label in (
            (raw.product_id, "product_id"),
            (raw.capture_id, "capture_id"),
            (frame_batch_id, "frame_batch_id"),
        ):
            self._validate_path_component(value, label)
        for camera_id in required_camera_ids:
            self._validate_path_component(camera_id, "camera_id")

        date = datetime.fromtimestamp(
            raw.trigger_requested_wall_time_ns / 1_000_000_000, tz=UTC
        )
        batch_dir = (
            self.data_root
            / f"{date:%Y}"
            / f"{date:%m}"
            / f"{date:%d}"
            / raw.product_id
            / f"station_{raw.station_id}"
            / raw.capture_id
            / f"attempt_{raw.attempt}"
            / frame_batch_id
        )
        batch_dir.mkdir(parents=True, exist_ok=True)
        self._apply_directory_mode(batch_dir)

        by_camera = {frame.camera_id: frame for frame in raw.frames}
        artifacts: list[ImageArtifact] = []
        for camera_id in required_camera_ids:
            frame = by_camera[camera_id]
            content = encode_rgb8_png(
                frame.rgb_bytes,
                frame.width,
                frame.height,
                compression_level=self.compression_level,
            )
            target = batch_dir / f"{camera_id}.png"
            digest = self._atomic_write(target, content)
            artifacts.append(
                ImageArtifact(
                    camera_id=camera_id,
                    file_path=str(target.resolve()),
                    sha256=digest,
                    file_size_bytes=len(content),
                    width=frame.width,
                    height=frame.height,
                    pixel_format="RGB8_PNG",
                    camera_timestamp_raw=frame.camera_timestamp_raw,
                    camera_timestamp_domain=frame.camera_timestamp_domain,
                    camera_timestamp_ns=frame.camera_timestamp_ns,
                    camera_timestamp_synchronized=frame.camera_timestamp_synchronized,
                    host_arrival_monotonic_ns=frame.host_arrival_monotonic_ns,
                    host_arrival_timestamp_ns=frame.host_arrival_wall_time_ns,
                    frame_number=frame.frame_number,
                    external_trigger_count=frame.external_trigger_count,
                    sdk_host_timestamp_raw=frame.sdk_host_timestamp_raw,
                )
            )

        manifest_path = batch_dir / "manifest.json"
        manifest = {
            "schema_version": 2,
            "product_id": raw.product_id,
            "station_id": raw.station_id,
            "capture_id": raw.capture_id,
            "frame_batch_id": frame_batch_id,
            "attempt": raw.attempt,
            "capture_outcome": capture_outcome,
            "failure_reason": failure_reason,
            "trigger_requested_monotonic_ns": raw.trigger_requested_monotonic_ns,
            "trigger_returned_monotonic_ns": raw.trigger_returned_monotonic_ns,
            "trigger_requested_wall_time_ns": raw.trigger_requested_wall_time_ns,
            "trigger_returned_wall_time_ns": raw.trigger_returned_wall_time_ns,
            "frame_arrival_skew_us": raw.frame_arrival_skew_us,
            "required_camera_ids": list(required_camera_ids),
            "images": [asdict(artifact) for artifact in artifacts],
        }
        manifest_bytes = json.dumps(
            manifest,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        self._atomic_write(manifest_path, manifest_bytes)

        batch = CaptureBatch(
            product_id=raw.product_id,
            station_id=raw.station_id,
            capture_id=raw.capture_id,
            frame_batch_id=frame_batch_id,
            attempt=raw.attempt,
            trigger_requested_monotonic_ns=raw.trigger_requested_monotonic_ns,
            trigger_returned_monotonic_ns=raw.trigger_returned_monotonic_ns,
            trigger_requested_wall_time_ns=raw.trigger_requested_wall_time_ns,
            trigger_returned_wall_time_ns=raw.trigger_returned_wall_time_ns,
            images=tuple(artifacts),
            manifest_path=str(manifest_path.resolve()),
        )
        batch.validate(required_camera_ids)
        return batch

    @staticmethod
    def _validate_path_component(value: str, label: str) -> None:
        if (
            not value
            or value in {".", ".."}
            or "/" in value
            or "\\" in value
            or "\x00" in value
        ):
            raise CaptureStorageError(f"unsafe {label} path component", retryable=False)

    def _atomic_write(self, target: Path, content: bytes) -> str:
        digest = hashlib.sha256(content).hexdigest()
        if target.exists():
            if target.is_file() and self._sha256_file(target) == digest:
                return digest
            raise CaptureStorageError(
                f"refusing to overwrite conflicting artifact: {target}", retryable=False
            )
        temp = target.with_name(f".{target.name}.{new_uuid()}.vision-tmp")
        try:
            with temp.open("xb") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            self._apply_file_mode(temp)
            os.replace(temp, target)
            self._fsync_directory(target.parent)
        except Exception as exc:
            if temp.exists():
                temp.unlink()
            if isinstance(exc, CaptureStorageError):
                raise
            raise CaptureStorageError(
                f"atomic artifact write failed: {type(exc).__name__}"
            ) from exc
        if target.stat().st_size != len(content) or self._sha256_file(target) != digest:
            raise CaptureStorageError("artifact readback validation failed")
        return digest

    @staticmethod
    def _sha256_file(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()

    @staticmethod
    def _apply_directory_mode(path: Path) -> None:
        if os.name == "posix":
            path.chmod(0o2770)

    @staticmethod
    def _apply_file_mode(path: Path) -> None:
        if os.name == "posix":
            path.chmod(0o660)

    @staticmethod
    def _fsync_directory(path: Path) -> None:
        if os.name != "posix":
            return
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        descriptor = os.open(path, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def configure_restrictive_umask() -> int | None:
    """Linux 서비스 프로세스의 신규 파일 기본 권한을 0007로 제한합니다."""

    if sys.platform.startswith("linux"):
        return os.umask(0o007)
    return None
