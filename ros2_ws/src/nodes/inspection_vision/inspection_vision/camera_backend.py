"""카메라 구성값과 장비 없는 결정적 simulation capture backend."""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass

from .capture_contract import CameraFrame, CameraUnavailable, RawCaptureBatch


@dataclass(frozen=True, slots=True)
class CameraSpec:
    camera_id: str
    station_id: int
    serial_number: str
    expected_ip: str
    expected_mac: str = ""

    def validate(self, *, hardware: bool) -> None:
        if not self.camera_id:
            raise ValueError("camera_id is required")
        if self.station_id not in {1, 2}:
            raise ValueError("camera station_id must be 1 or 2")
        if hardware and (not self.serial_number or not self.expected_ip):
            raise ValueError(
                f"hardware camera {self.camera_id} requires serial_number and expected_ip"
            )


@dataclass(frozen=True, slots=True)
class ActionCommandConfig:
    device_key: int = 0x13572468
    station_a_group_key: int = 1
    station_b_group_key: int = 2
    group_mask: int = 0xFFFFFFFF
    broadcast_address: str = "255.255.255.255"
    ack_timeout_ms: int = 200
    nic_ip: str = "192.168.10.10"

    def group_key(self, station_id: int) -> int:
        if station_id == 1:
            return self.station_a_group_key
        if station_id == 2:
            return self.station_b_group_key
        raise ValueError("station_id must be 1 or 2")

    def validate(self) -> None:
        for value, name in (
            (self.device_key, "device_key"),
            (self.station_a_group_key, "station_a_group_key"),
            (self.station_b_group_key, "station_b_group_key"),
            (self.group_mask, "group_mask"),
        ):
            if not 0 <= value <= 0xFFFFFFFF:
                raise ValueError(f"{name} must fit uint32")
        if self.station_a_group_key == self.station_b_group_key:
            raise ValueError("station A/B group keys must differ")
        if self.ack_timeout_ms < 0:
            raise ValueError("ack_timeout_ms cannot be negative")


class SimulationCaptureBackend:
    """MVS 없이 sim launch와 ROS 통합시험에 사용하는 명시적 fake."""

    def __init__(
        self,
        camera_specs: tuple[CameraSpec, ...],
        *,
        width: int = 8,
        height: int = 6,
        callback_spacing_us: int = 100,
    ) -> None:
        if width < 1 or height < 1:
            raise ValueError("simulation frame dimensions must be positive")
        self._specs = {spec.camera_id: spec for spec in camera_specs}
        self._width = width
        self._height = height
        self._callback_spacing_us = callback_spacing_us
        self._initialized = False
        self._frame_numbers = {camera_id: 0 for camera_id in self._specs}

    async def initialize(self) -> dict[str, object]:
        for spec in self._specs.values():
            spec.validate(hardware=False)
        self._initialized = True
        return {
            "backend": "simulation",
            "camera_ids": sorted(self._specs),
            "canonical_pixel_format": "RGB8_PNG",
        }

    async def capture_station(
        self,
        *,
        product_id: str,
        station_id: int,
        capture_id: str,
        attempt: int,
        required_camera_ids: tuple[str, ...],
    ) -> RawCaptureBatch:
        if not self._initialized:
            raise CameraUnavailable("simulation camera backend is not initialized")
        unknown = set(required_camera_ids) - self._specs.keys()
        if unknown:
            raise CameraUnavailable(f"unknown simulation cameras: {sorted(unknown)}")
        if any(self._specs[camera_id].station_id != station_id for camera_id in required_camera_ids):
            raise CameraUnavailable("requested camera belongs to another station")
        requested_mono = time.monotonic_ns()
        requested_wall = time.time_ns()
        await asyncio.sleep(0)
        returned_mono = time.monotonic_ns()
        returned_wall = time.time_ns()
        frames: list[CameraFrame] = []
        base_arrival_mono = time.monotonic_ns()
        base_arrival_wall = time.time_ns()
        for index, camera_id in enumerate(required_camera_ids):
            self._frame_numbers[camera_id] += 1
            # 카메라별 색을 달리해 camera order/경로 혼동을 시험할 수 있습니다.
            color = bytes(
                (
                    (station_id * 40 + index * 30) % 256,
                    (attempt * 60 + index * 20) % 256,
                    (self._frame_numbers[camera_id] * 10) % 256,
                )
            )
            arrival_offset = index * self._callback_spacing_us * 1_000
            frames.append(
                CameraFrame(
                    camera_id=camera_id,
                    width=self._width,
                    height=self._height,
                    rgb_bytes=color * (self._width * self._height),
                    frame_number=self._frame_numbers[camera_id],
                    external_trigger_count=self._frame_numbers[camera_id],
                    camera_timestamp_raw=base_arrival_mono + arrival_offset,
                    camera_timestamp_domain="SIM_MONOTONIC_NS",
                    camera_timestamp_ns=base_arrival_mono + arrival_offset,
                    camera_timestamp_synchronized=True,
                    sdk_host_timestamp_raw=base_arrival_mono + arrival_offset,
                    host_arrival_monotonic_ns=base_arrival_mono + arrival_offset,
                    host_arrival_wall_time_ns=base_arrival_wall + arrival_offset,
                )
            )
        return RawCaptureBatch(
            product_id=product_id,
            station_id=station_id,
            capture_id=capture_id,
            attempt=attempt,
            trigger_requested_monotonic_ns=requested_mono,
            trigger_returned_monotonic_ns=returned_mono,
            trigger_requested_wall_time_ns=requested_wall,
            trigger_returned_wall_time_ns=returned_wall,
            frames=tuple(frames),
        )

    async def prepare_retry(
        self, *, station_id: int, required_camera_ids: tuple[str, ...]
    ) -> None:
        del station_id, required_camera_ids
        await asyncio.sleep(0)

    async def recover_station(
        self, *, station_id: int, required_camera_ids: tuple[str, ...]
    ) -> bool:
        del station_id, required_camera_ids
        return self._initialized

    async def close(self) -> None:
        self._initialized = False
