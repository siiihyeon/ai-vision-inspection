"""Ubuntu HIKROBOT MVS Linux SDK GigE Action1 capture backend.

공식 MVS Python wrapper는 시스템에 설치된 ``libMvCameraControl.so``를
``ctypes``로 로드합니다. 이 모듈은 sim/test import 시 SDK를 요구하지 않도록
하드웨어 초기화 시점에만 vendor module을 지연 로드합니다.
"""

from __future__ import annotations

import ctypes
import importlib
import ipaddress
import math
import os
import sys
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Callable, Mapping

from .capture_contract import (
    MONO8_PNG,
    CaptureBatch,
    ImageArtifact,
    write_mono8_png_atomic,
)


class MvsSdkError(RuntimeError):
    """MVS API 호출 또는 카메라 계약 검증 실패입니다."""


@dataclass(frozen=True, slots=True)
class ActionGroup:
    group_key: int
    group_mask: int

    def validate(self, label: str) -> None:
        for name, value in (
            ("group_key", self.group_key),
            ("group_mask", self.group_mask),
        ):
            if not 1 <= value <= 0xFFFFFFFF:
                raise ValueError(f"{label}.{name} must be a non-zero uint32")


@dataclass(frozen=True, slots=True)
class CameraInventoryEntry:
    serial: str
    station_id: int
    ip_address: str
    model_name: str
    firmware_version: str


@dataclass(frozen=True, slots=True)
class CameraAcquisitionSettings:
    exposure_time_us: float
    gain_db: float

    def validate(self, serial: str) -> None:
        if not math.isfinite(self.exposure_time_us) or self.exposure_time_us <= 0:
            raise ValueError(f"{serial}: exposure_time_us must be positive")
        if not math.isfinite(self.gain_db) or self.gain_db < 0:
            raise ValueError(f"{serial}: gain_db must not be negative")


@dataclass(frozen=True, slots=True)
class MvsBackendSettings:
    station_camera_ids: Mapping[int, tuple[str, ...]]
    camera_network_map: Mapping[str, str]
    camera_acquisition_settings: Mapping[str, CameraAcquisitionSettings]
    action_device_key: int
    action_groups: Mapping[int, ActionGroup]
    data_root: Path
    acquisition_timeout_ms: int
    packet_size: int = 1500
    packet_delay_ticks: int = 5000
    action_ack_timeout_ms: int = 100
    broadcast_address: str = "255.255.255.255"
    expected_model: str = "MV-CS050-10GC"
    expected_firmware_version: str = ""
    mvs_python_import_dir: Path = Path(
        "/opt/MVS/Samples/64/Python/MvImport"
    )
    mvs_runtime_root: Path = Path("/opt/MVS/lib")
    sdk_image_buffer_nodes: int = 8
    packet_resend_enabled: bool = True
    packet_resend_max_percent: int = 10
    packet_resend_timeout_ms: int = 50
    packet_resend_max_retry_times: int = 3
    packet_resend_interval_ms: int = 10
    reconnect_interval_ms: int = 1000
    reconnect_max_attempts: int = 5
    disconnect_pause_after_ms: int = 5000
    png_compression_level: int = 3

    def validate(self) -> None:
        station_a = tuple(self.station_camera_ids.get(1, ()))
        station_b = tuple(self.station_camera_ids.get(2, ()))
        if len(station_a) != 3 or len(station_b) != 1:
            raise ValueError("MVS camera groups must contain A=3 and B=1 cameras")
        all_serials = station_a + station_b
        if len(set(all_serials)) != 4:
            raise ValueError("MVS camera serials must be unique")
        if set(self.camera_network_map) != set(all_serials):
            raise ValueError("MVS camera network map must cover exactly four cameras")
        if set(self.camera_acquisition_settings) != set(all_serials):
            raise ValueError(
                "MVS acquisition settings must cover exactly four cameras"
            )
        for serial, acquisition in self.camera_acquisition_settings.items():
            acquisition.validate(serial)
        for address in self.camera_network_map.values():
            ipaddress.ip_address(str(address))
        if not 1 <= self.action_device_key <= 0xFFFFFFFF:
            raise ValueError("ActionDeviceKey must be a non-zero uint32")
        if set(self.action_groups) != {1, 2}:
            raise ValueError("station A/B Action1 groups are required")
        for station_id, group in self.action_groups.items():
            group.validate(f"station_{station_id}")
        if self.action_groups[1] == self.action_groups[2]:
            raise ValueError("station A/B Action1 groups must differ")
        if not self.data_root.is_absolute():
            raise ValueError("MVS data_root must be absolute")
        if self.acquisition_timeout_ms < 1:
            raise ValueError("MVS acquisition timeout must be positive")
        if self.packet_size != 1500:
            raise ValueError("MVS packet size is fixed at 1500")
        if self.packet_delay_ticks < 0:
            raise ValueError("MVS packet delay must not be negative")
        if self.action_ack_timeout_ms < 1:
            raise ValueError("Action Command ACK timeout must be positive")
        ipaddress.ip_address(self.broadcast_address)
        if not self.expected_model:
            raise ValueError("expected HIKROBOT model is required")
        if self.sdk_image_buffer_nodes < 1:
            raise ValueError("MVS SDK image buffer count must be positive")
        if (
            self.reconnect_interval_ms < 0
            or self.reconnect_max_attempts < 1
            or self.disconnect_pause_after_ms < 1
        ):
            raise ValueError("MVS reconnect policy is invalid")
        if not 0 <= self.png_compression_level <= 9:
            raise ValueError("PNG compression level must be between 0 and 9")


def validate_camera_inventory(
    entries: tuple[CameraInventoryEntry, ...],
    *,
    expected_serials: tuple[str, ...],
    expected_network_map: Mapping[str, str],
    expected_model: str,
    expected_firmware_version: str,
) -> str:
    """네 카메라의 identity와 firmware 일치를 검증하고 공통 버전을 반환합니다."""

    by_serial = {entry.serial: entry for entry in entries}
    if len(by_serial) != len(entries):
        raise MvsSdkError("duplicate HIKROBOT serial number was enumerated")
    missing = sorted(set(expected_serials) - set(by_serial))
    unexpected = sorted(set(by_serial) - set(expected_serials))
    if missing or unexpected:
        raise MvsSdkError(
            f"camera inventory mismatch: missing={missing}, unexpected={unexpected}"
        )
    for serial in expected_serials:
        entry = by_serial[serial]
        if entry.model_name != expected_model:
            raise MvsSdkError(
                f"{serial}: expected model {expected_model}, got {entry.model_name}"
            )
        expected_ip = str(expected_network_map[serial])
        if entry.ip_address != expected_ip:
            raise MvsSdkError(
                f"{serial}: expected IP {expected_ip}, got {entry.ip_address}"
            )
        if not entry.firmware_version:
            raise MvsSdkError(f"{serial}: firmware version is empty")
    firmware_versions = {entry.firmware_version for entry in entries}
    if len(firmware_versions) != 1:
        details = ", ".join(
            f"{entry.serial}={entry.firmware_version}" for entry in entries
        )
        raise MvsSdkError(f"camera firmware versions differ: {details}")
    common_firmware = next(iter(firmware_versions))
    if (
        expected_firmware_version
        and common_firmware != expected_firmware_version
    ):
        raise MvsSdkError(
            "camera firmware differs from configured expected version: "
            f"expected={expected_firmware_version}, actual={common_firmware}"
        )
    return common_firmware


@dataclass(frozen=True, slots=True)
class _MvsModules:
    camera: ModuleType
    params: ModuleType
    constants: ModuleType
    pixels: ModuleType
    errors: ModuleType


@dataclass(frozen=True, slots=True)
class _DeviceRecord:
    identity: CameraInventoryEntry
    raw_info: Any


@dataclass(frozen=True, slots=True)
class _NetworkCounters:
    lost_packets: int
    request_resend_packets: int
    resend_packets: int


_MVS_IMPORT_LOCK = threading.Lock()


def _load_mvs_modules(import_dir: Path, runtime_root: Path) -> _MvsModules:
    if sys.platform != "linux":
        raise MvsSdkError("HIKROBOT hardware backend requires Ubuntu/Linux")
    wrapper = import_dir / "MvCameraControl_class.py"
    runtime = runtime_root / "64" / "libMvCameraControl.so"
    if not wrapper.is_file():
        raise MvsSdkError(f"MVS Python wrapper not found: {wrapper}")
    if not runtime.is_file():
        raise MvsSdkError(f"MVS runtime library not found: {runtime}")

    with _MVS_IMPORT_LOCK:
        os.environ["MVCAM_COMMON_RUNENV"] = str(runtime_root)
        if str(import_dir) not in sys.path:
            sys.path.insert(0, str(import_dir))
        try:
            camera = importlib.import_module("MvCameraControl_class")
            params = importlib.import_module("CameraParams_header")
            constants = importlib.import_module("CameraParams_const")
            pixels = importlib.import_module("PixelType_header")
            errors = importlib.import_module("MvErrorDefine_const")
        except Exception as exc:
            raise MvsSdkError(
                f"MVS Python binding import failed: {type(exc).__name__}"
            ) from exc
        loaded_wrapper = Path(str(getattr(camera, "__file__", ""))).resolve()
        if import_dir.resolve() not in loaded_wrapper.parents:
            raise MvsSdkError(
                f"a different MVS Python binding is already loaded: {loaded_wrapper}"
            )
    required = (
        (camera, "MvCamera"),
        (params, "MV_ACTION_CMD_INFO"),
        (params, "MV_ACTION_CMD_RESULT_LIST"),
        (params, "MV_FRAME_OUT"),
        (constants, "MV_GIGE_DEVICE"),
        (pixels, "PixelType_Gvsp_Mono8"),
    )
    for module, symbol in required:
        if not hasattr(module, symbol):
            raise MvsSdkError(f"MVS binding is missing required symbol: {symbol}")
    return _MvsModules(camera, params, constants, pixels, errors)


def _check_mvs(return_code: int, operation: str) -> None:
    if int(return_code) != 0:
        raise MvsSdkError(
            f"{operation} failed: 0x{int(return_code) & 0xFFFFFFFF:08X}"
        )


def _decode_c_string(value: Any) -> str:
    return bytes(value).split(b"\0", 1)[0].decode("ascii", errors="replace")


def _ip_from_uint32(value: int) -> str:
    return ".".join(str((int(value) >> shift) & 0xFF) for shift in (24, 16, 8, 0))


def _enumerate_devices(
    modules: _MvsModules,
    station_by_serial: Mapping[str, int],
) -> tuple[_DeviceRecord, ...]:
    device_list = modules.params.MV_CC_DEVICE_INFO_LIST()
    ctypes.memset(ctypes.byref(device_list), 0, ctypes.sizeof(device_list))
    _check_mvs(
        modules.camera.MvCamera.MV_CC_EnumDevices(
            modules.constants.MV_GIGE_DEVICE, device_list
        ),
        "MV_CC_EnumDevices",
    )
    records: list[_DeviceRecord] = []
    for index in range(int(device_list.nDeviceNum)):
        source = ctypes.cast(
            device_list.pDeviceInfo[index],
            ctypes.POINTER(modules.params.MV_CC_DEVICE_INFO),
        ).contents
        copied = modules.params.MV_CC_DEVICE_INFO()
        ctypes.memmove(ctypes.byref(copied), ctypes.byref(source), ctypes.sizeof(copied))
        gige = copied.SpecialInfo.stGigEInfo
        serial = _decode_c_string(gige.chSerialNumber)
        if serial not in station_by_serial:
            continue
        records.append(
            _DeviceRecord(
                CameraInventoryEntry(
                    serial=serial,
                    station_id=station_by_serial[serial],
                    ip_address=_ip_from_uint32(gige.nCurrentIp),
                    model_name=_decode_c_string(gige.chModelName),
                    firmware_version=_decode_c_string(gige.chDeviceVersion),
                ),
                copied,
            )
        )
    return tuple(records)


class _CameraSession:
    def __init__(
        self,
        modules: _MvsModules,
        record: _DeviceRecord,
        settings: MvsBackendSettings,
    ) -> None:
        self.modules = modules
        self.record = record
        self.settings = settings
        self.camera = modules.camera.MvCamera()
        self.opened = False
        self.grabbing = False
        self.correction_status: dict[str, str] = {}
        self._lock = threading.Lock()

    @property
    def serial(self) -> str:
        return self.record.identity.serial

    def _enum(self, name: str, value: str) -> None:
        _check_mvs(
            self.camera.MV_CC_SetEnumValueByString(name, value),
            f"{self.serial}: {name}={value}",
        )

    def _integer(self, name: str, value: int) -> None:
        _check_mvs(
            self.camera.MV_CC_SetIntValueEx(name, int(value)),
            f"{self.serial}: {name}={value}",
        )

    def _float(self, name: str, value: float) -> None:
        _check_mvs(
            self.camera.MV_CC_SetFloatValue(name, float(value)),
            f"{self.serial}: {name}={value}",
        )

    def _boolean(self, name: str, value: bool) -> None:
        _check_mvs(
            self.camera.MV_CC_SetBoolValue(name, bool(value)),
            f"{self.serial}: {name}={value}",
        )

    def _disable_boolean_correction(self, name: str) -> None:
        """지원되는 보정은 OFF를 read-back하고, 비노출 node는 명시 기록합니다."""

        result = int(self.camera.MV_CC_SetBoolValue(name, False))
        gc_access = int(
            getattr(self.modules.errors, "MV_E_GC_ACCESS", 0x80000106)
        )
        if result == 0:
            current = ctypes.c_bool(True)
            _check_mvs(
                self.camera.MV_CC_GetBoolValue(name, current),
                f"{self.serial}: verify {name}=False",
            )
            if bool(current.value):
                raise MvsSdkError(f"{self.serial}: {name} remained enabled")
            self.correction_status[name] = "OFF_VERIFIED"
            return
        if (result & 0xFFFFFFFF) != (gc_access & 0xFFFFFFFF):
            _check_mvs(result, f"{self.serial}: {name}=False")

        # MV-CS050-10GC firmware는 Mono8 feature set에서 일부 color/gamma
        # node를 GenICam access-condition으로 숨깁니다. 읽을 수 있다면 OFF만
        # 허용하고, 읽기 자체도 같은 access-condition이면 비활성 feature로
        # 구분해 inventory에 남깁니다. 다른 SDK 오류는 절대 무시하지 않습니다.
        current = ctypes.c_bool(True)
        read_result = int(self.camera.MV_CC_GetBoolValue(name, current))
        if read_result == 0:
            if bool(current.value):
                raise MvsSdkError(
                    f"{self.serial}: {name} is enabled but not writable"
                )
            self.correction_status[name] = "OFF_READ_ONLY"
            return
        if (read_result & 0xFFFFFFFF) == (gc_access & 0xFFFFFFFF):
            self.correction_status[name] = "UNAVAILABLE_IN_MONO8_FEATURE_SET"
            return
        _check_mvs(read_result, f"{self.serial}: read {name}")

    def open(self) -> None:
        group = self.settings.action_groups[self.record.identity.station_id]
        _check_mvs(
            self.camera.MV_CC_CreateHandle(self.record.raw_info),
            f"{self.serial}: MV_CC_CreateHandle",
        )
        try:
            _check_mvs(
                self.camera.MV_CC_OpenDevice(
                    self.modules.constants.MV_ACCESS_Exclusive, 0
                ),
                f"{self.serial}: MV_CC_OpenDevice",
            )
            self.opened = True
            self._enum("TriggerMode", "Off")
            # BalanceWhiteAuto는 Mono8에서 GenICam access-condition으로
            # 숨겨집니다. 동일 color sensor의 Bayer feature set에서 먼저 OFF를
            # 확정한 뒤 운영 출력인 Mono8로 되돌립니다.
            self._enum("PixelFormat", "BayerRG8")
            self._enum("BalanceWhiteAuto", "Off")
            acquisition = self.settings.camera_acquisition_settings[self.serial]
            # 운영 계약: 자동 보정뿐 아니라 가능한 모든 영상 보정을 끕니다.
            self._enum("ExposureAuto", "Off")
            self._float("ExposureTime", acquisition.exposure_time_us)
            self._enum("GainAuto", "Off")
            self._float("Gain", acquisition.gain_db)
            self._integer("OffsetX", 0)
            self._integer("OffsetY", 0)
            self._integer("Width", 2448)
            self._integer("Height", 2048)
            self._enum("PixelFormat", "Mono8")
            for correction in (
                "GammaEnable",
                "SaturationEnable",
                "SharpnessEnable",
                "BlackLevelEnable",
            ):
                self._disable_boolean_correction(correction)
            self._integer("GevSCPSPacketSize", self.settings.packet_size)
            self._integer("GevSCPD", self.settings.packet_delay_ticks)
            if self.settings.packet_resend_enabled:
                _check_mvs(
                    self.camera.MV_GIGE_SetResend(
                        True,
                        self.settings.packet_resend_max_percent,
                        self.settings.packet_resend_timeout_ms,
                    ),
                    f"{self.serial}: MV_GIGE_SetResend",
                )
                _check_mvs(
                    self.camera.MV_GIGE_SetResendMaxRetryTimes(
                        self.settings.packet_resend_max_retry_times
                    ),
                    f"{self.serial}: MV_GIGE_SetResendMaxRetryTimes",
                )
                _check_mvs(
                    self.camera.MV_GIGE_SetResendTimeInterval(
                        self.settings.packet_resend_interval_ms
                    ),
                    f"{self.serial}: MV_GIGE_SetResendTimeInterval",
                )
            self._enum("TriggerSource", "Action1")
            self._integer("ActionDeviceKey", self.settings.action_device_key)
            self._integer("ActionGroupKey", group.group_key)
            self._integer("ActionGroupMask", group.group_mask)
            self._enum("TriggerMode", "On")
            _check_mvs(
                self.camera.MV_CC_SetImageNodeNum(
                    self.settings.sdk_image_buffer_nodes
                ),
                f"{self.serial}: MV_CC_SetImageNodeNum",
            )
            strategy_result = int(
                self.camera.MV_CC_SetGrabStrategy(
                    self.modules.params.MV_GrabStrategy_OneByOne
                )
            )
            # MVS Linux Python wrapper는 일부 GigE firmware에서 명시적
            # OneByOne 설정에 MV_E_SUPPORT를 반환합니다. 기본 전략도
            # OneByOne이므로 이 한 코드만 허용하고 나머지는 fail-closed합니다.
            not_supported = int(
                getattr(self.modules.errors, "MV_E_SUPPORT", 0x80000001)
            )
            if (strategy_result & 0xFFFFFFFF) != (not_supported & 0xFFFFFFFF):
                _check_mvs(
                    strategy_result,
                    f"{self.serial}: MV_CC_SetGrabStrategy",
                )
            _check_mvs(
                self.camera.MV_CC_StartGrabbing(),
                f"{self.serial}: MV_CC_StartGrabbing",
            )
            self.grabbing = True
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        with self._lock:
            if self.grabbing:
                try:
                    self.camera.MV_CC_StopGrabbing()
                except Exception:
                    pass
                finally:
                    self.grabbing = False
            if self.opened:
                try:
                    self.camera.MV_CC_CloseDevice()
                except Exception:
                    pass
                finally:
                    self.opened = False
            try:
                self.camera.MV_CC_DestroyHandle()
            except Exception:
                pass

    def network_counters(self) -> _NetworkCounters:
        net = self.modules.params.MV_MATCH_INFO_NET_DETECT()
        query = self.modules.params.MV_ALL_MATCH_INFO()
        ctypes.memset(ctypes.byref(net), 0, ctypes.sizeof(net))
        ctypes.memset(ctypes.byref(query), 0, ctypes.sizeof(query))
        query.nType = self.modules.constants.MV_MATCH_TYPE_NET_DETECT
        query.pInfo = ctypes.cast(ctypes.byref(net), ctypes.c_void_p)
        query.nInfoSize = ctypes.sizeof(net)
        _check_mvs(
            self.camera.MV_CC_GetAllMatchInfo(query),
            f"{self.serial}: MV_CC_GetAllMatchInfo",
        )
        return _NetworkCounters(
            lost_packets=int(net.nLostPacketCount),
            request_resend_packets=int(net.nRequestResendPacketCount),
            resend_packets=int(net.nResendPacketCount),
        )

    def drain_stale_frames(self, *, maximum_frames: int = 32) -> int:
        drained = 0
        no_data_codes = {
            int(getattr(self.modules.errors, "MV_E_NODATA", 0x80000007)),
            int(getattr(self.modules.errors, "MV_E_TIMEOUT", 0x80000007)),
        }
        with self._lock:
            for _ in range(maximum_frames):
                frame = self.modules.params.MV_FRAME_OUT()
                ctypes.memset(ctypes.byref(frame), 0, ctypes.sizeof(frame))
                result = int(self.camera.MV_CC_GetImageBuffer(frame, 1))
                if result != 0:
                    if (result & 0xFFFFFFFF) in {
                        value & 0xFFFFFFFF for value in no_data_codes
                    }:
                        break
                    _check_mvs(result, f"{self.serial}: drain stale frame")
                try:
                    drained += 1
                finally:
                    _check_mvs(
                        self.camera.MV_CC_FreeImageBuffer(frame),
                        f"{self.serial}: free stale frame",
                    )
            else:
                raise MvsSdkError(
                    f"{self.serial}: stale frame drain exceeded {maximum_frames}"
                )
        return drained

    def receive_and_save(
        self,
        path: Path,
        counters_before: _NetworkCounters,
        receiver_ready: threading.Event,
    ) -> ImageArtifact:
        frame = self.modules.params.MV_FRAME_OUT()
        ctypes.memset(ctypes.byref(frame), 0, ctypes.sizeof(frame))
        with self._lock:
            # Action을 보내기 전에 모든 camera thread가 blocking receive 직전까지
            # 도달하게 하여 thread scheduling 시간이 arrival skew에 섞이지 않게 합니다.
            receiver_ready.set()
            _check_mvs(
                self.camera.MV_CC_GetImageBuffer(
                    frame, self.settings.acquisition_timeout_ms
                ),
                f"{self.serial}: MV_CC_GetImageBuffer",
            )
            arrival_monotonic_ns = time.monotonic_ns()
            arrival_wall_ns = time.time_ns()
            try:
                info = frame.stFrameInfo
                width = int(info.nWidth or info.nExtendWidth)
                height = int(info.nHeight or info.nExtendHeight)
                if (width, height) != (2448, 2048):
                    raise MvsSdkError(
                        f"{self.serial}: unexpected frame size {width}x{height}"
                    )
                if int(info.enPixelType) != int(
                    self.modules.pixels.PixelType_Gvsp_Mono8
                ):
                    raise MvsSdkError(
                        f"{self.serial}: frame pixel type is not Mono8"
                    )
                required_bytes = width * height
                frame_length = int(info.nFrameLenEx or info.nFrameLen)
                if frame_length < required_bytes or not frame.pBufAddr:
                    raise MvsSdkError(
                        f"{self.serial}: incomplete Mono8 frame buffer"
                    )
                pixels = ctypes.string_at(frame.pBufAddr, required_bytes)
                lost_packets = int(info.nLostPacket)
                device_timestamp = (
                    int(info.nDevTimeStampHigh) << 32
                ) | int(info.nDevTimeStampLow)
            finally:
                _check_mvs(
                    self.camera.MV_CC_FreeImageBuffer(frame),
                    f"{self.serial}: MV_CC_FreeImageBuffer",
                )
            counters_after = self.network_counters()

        digest, size_bytes = write_mono8_png_atomic(
            path,
            pixels,
            width,
            height,
            compression_level=self.settings.png_compression_level,
        )
        return ImageArtifact(
            camera_id=self.serial,
            file_path=str(path),
            sha256=digest,
            file_size_bytes=size_bytes,
            width=width,
            height=height,
            pixel_format=MONO8_PNG,
            camera_timestamp_raw=device_timestamp,
            camera_timestamp_domain="DEVICE_TICKS_UNSYNCED",
            camera_timestamp_ns=0,
            camera_timestamp_synchronized=False,
            host_arrival_monotonic_ns=arrival_monotonic_ns,
            host_arrival_timestamp_ns=arrival_wall_ns,
            packet_loss_count=lost_packets,
            packet_resend_count=max(
                0, counters_after.resend_packets - counters_before.resend_packets
            ),
        )


class HikrobotMvsCaptureBackend:
    """Station별 즉시 GigE Action1 broadcast와 Mono8 원자 저장을 수행합니다."""

    def __init__(
        self,
        settings: MvsBackendSettings,
        *,
        sdk_loader: Callable[[Path, Path], _MvsModules] = _load_mvs_modules,
    ) -> None:
        self.settings = settings
        self._sdk_loader = sdk_loader
        self._modules: _MvsModules | None = None
        self._sessions: dict[str, _CameraSession] = {}
        self._inventory: tuple[CameraInventoryEntry, ...] = ()
        self._correction_status_by_serial: dict[str, dict[str, str]] = {}
        self._firmware_version = ""
        self._sdk_version = ""
        self._initialized = False
        self._closed = False
        self._lifecycle_lock = threading.RLock()
        self._action_lock = threading.Lock()
        self._frame_pool = ThreadPoolExecutor(
            max_workers=4, thread_name_prefix="mvs-frame"
        )

    @property
    def inventory(self) -> tuple[CameraInventoryEntry, ...]:
        return self._inventory

    @property
    def firmware_version(self) -> str:
        return self._firmware_version

    @property
    def sdk_version(self) -> str:
        return self._sdk_version

    @property
    def correction_status_by_serial(self) -> dict[str, dict[str, str]]:
        return {
            serial: dict(statuses)
            for serial, statuses in self._correction_status_by_serial.items()
        }

    def initialize(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                raise MvsSdkError("MVS backend is already closed")
            if self._initialized:
                return
            self.settings.validate()
            self._initialize_once()

    def initialize_with_reconnect(self) -> None:
        last_error: Exception | None = None
        started_ns = time.monotonic_ns()
        for attempt in range(1, self.settings.reconnect_max_attempts + 1):
            try:
                self.initialize()
                return
            except Exception as exc:
                last_error = exc
                self._close_sessions()
                if attempt < self.settings.reconnect_max_attempts:
                    time.sleep(self.settings.reconnect_interval_ms / 1000.0)
        remaining_seconds = max(
            0.0,
            self.settings.disconnect_pause_after_ms / 1000.0
            - (time.monotonic_ns() - started_ns) / 1_000_000_000,
        )
        if remaining_seconds:
            time.sleep(remaining_seconds)
        raise MvsSdkError(
            "MVS camera reconnect attempts exhausted: "
            f"{type(last_error).__name__ if last_error else 'unknown'}"
        ) from last_error

    def _initialize_once(self) -> None:
        if self._modules is None:
            self._modules = self._sdk_loader(
                self.settings.mvs_python_import_dir,
                self.settings.mvs_runtime_root,
            )
            version = int(self._modules.camera.MvCamera.MV_CC_GetSDKVersion())
            self._sdk_version = f"0x{version & 0xFFFFFFFF:08X}"

        station_by_serial = {
            serial: station_id
            for station_id, serials in self.settings.station_camera_ids.items()
            for serial in serials
        }
        records = _enumerate_devices(self._modules, station_by_serial)
        entries = tuple(record.identity for record in records)
        expected_serials = tuple(
            serial
            for station_id in (1, 2)
            for serial in self.settings.station_camera_ids[station_id]
        )
        self._firmware_version = validate_camera_inventory(
            entries,
            expected_serials=expected_serials,
            expected_network_map=self.settings.camera_network_map,
            expected_model=self.settings.expected_model,
            expected_firmware_version=self.settings.expected_firmware_version,
        )
        record_by_serial = {record.identity.serial: record for record in records}
        opened: dict[str, _CameraSession] = {}
        try:
            for serial in expected_serials:
                session = _CameraSession(
                    self._modules, record_by_serial[serial], self.settings
                )
                session.open()
                opened[serial] = session
        except Exception:
            for session in reversed(tuple(opened.values())):
                session.close()
            raise
        self._sessions = opened
        self._inventory = tuple(record_by_serial[serial].identity for serial in expected_serials)
        self._correction_status_by_serial = {
            serial: dict(opened[serial].correction_status)
            for serial in expected_serials
        }
        self._initialized = True

    def close(self) -> None:
        with self._lifecycle_lock:
            if self._closed:
                return
            self._close_sessions()
            self._frame_pool.shutdown(wait=True, cancel_futures=True)
            self._closed = True

    def deinitialize(self) -> None:
        """초기화 재시도를 허용한 채 현재 camera handle만 닫습니다."""

        with self._lifecycle_lock:
            self._close_sessions()

    def _close_sessions(self) -> None:
        for session in reversed(tuple(self._sessions.values())):
            session.close()
        self._sessions.clear()
        self._initialized = False

    def _mark_disconnected(self) -> None:
        with self._lifecycle_lock:
            self._close_sessions()

    def _issue_action(self, station_id: int) -> None:
        if self._modules is None:
            raise MvsSdkError("MVS modules are not loaded")
        group = self.settings.action_groups[station_id]
        command = self._modules.params.MV_ACTION_CMD_INFO()
        results = self._modules.params.MV_ACTION_CMD_RESULT_LIST()
        ctypes.memset(ctypes.byref(command), 0, ctypes.sizeof(command))
        ctypes.memset(ctypes.byref(results), 0, ctypes.sizeof(results))
        broadcast_bytes = self.settings.broadcast_address.encode("ascii")
        command.nDeviceKey = self.settings.action_device_key
        command.nGroupKey = group.group_key
        command.nGroupMask = group.group_mask
        command.bActionTimeEnable = 0  # PTP와 무관한 즉시 Action Command
        command.nActionTime = 0
        command.pBroadcastAddress = broadcast_bytes
        command.nTimeOut = self.settings.action_ack_timeout_ms
        command.bSpecialNetEnable = 0
        with self._action_lock:
            _check_mvs(
                self._modules.camera.MvCamera.MV_GIGE_IssueActionCommand(
                    command, results
                ),
                f"station {station_id}: MV_GIGE_IssueActionCommand",
            )
        received: dict[str, int] = {}
        for index in range(int(results.nNumResults)):
            result = results.pResults[index]
            received[_decode_c_string(result.strDeviceAddress)] = int(result.nStatus)
        expected = {
            str(self.settings.camera_network_map[serial])
            for serial in self.settings.station_camera_ids[station_id]
        }
        if set(received) != expected:
            raise MvsSdkError(
                f"station {station_id}: Action1 ACK set mismatch: "
                f"expected={sorted(expected)}, actual={sorted(received)}"
            )
        failed = {
            address: status
            for address, status in received.items()
            if status != 0
        }
        if failed:
            formatted = ", ".join(
                f"{address}=0x{status & 0xFFFFFFFF:08X}"
                for address, status in sorted(failed.items())
            )
            raise MvsSdkError(
                f"station {station_id}: Action1 device ACK failed: {formatted}"
            )

    def capture_station(
        self,
        *,
        product_id: str,
        fifo_sequence: int,
        station_id: int,
        capture_id: str,
        attempt: int,
        required_camera_ids: tuple[str, ...],
    ) -> CaptureBatch:
        if station_id not in {1, 2}:
            raise ValueError("station_id must be 1 or 2")
        expected_ids = tuple(self.settings.station_camera_ids[station_id])
        if len(required_camera_ids) != len(expected_ids) or set(
            required_camera_ids
        ) != set(expected_ids):
            raise ValueError("required cameras differ from configured station group")
        if not product_id or fifo_sequence < 1 or not capture_id or attempt < 1:
            raise ValueError("capture identity is incomplete")
        if not self._initialized:
            self.initialize_with_reconnect()

        frame_batch_id = uuid.uuid4().hex
        batch_dir = (
            self.settings.data_root
            / "raw"
            / f"station_{station_id}"
            / f"product_{fifo_sequence:06d}_{frame_batch_id}"
        )
        batch_dir.mkdir(parents=True, exist_ok=False)
        paths = {serial: batch_dir / f"{serial}.png" for serial in expected_ids}
        try:
            counters_before: dict[str, _NetworkCounters] = {}
            for serial in expected_ids:
                session = self._sessions[serial]
                session.drain_stale_frames()
                counters_before[serial] = session.network_counters()

            receiver_ready = {
                serial: threading.Event() for serial in expected_ids
            }
            futures = {
                self._frame_pool.submit(
                    self._sessions[serial].receive_and_save,
                    paths[serial],
                    counters_before[serial],
                    receiver_ready[serial],
                ): serial
                for serial in expected_ids
            }
            if not all(
                ready.wait(timeout=1.0) for ready in receiver_ready.values()
            ):
                # Action은 아직 전송하지 않았으므로 receiver가 timeout으로
                # 빠져나올 때까지 기다린 뒤 incomplete directory를 정리합니다.
                for future in futures:
                    try:
                        future.result()
                    except Exception:
                        pass
                raise MvsSdkError(
                    f"station {station_id}: frame receiver threads did not become ready"
                )

            trigger_requested_monotonic_ns = time.monotonic_ns()
            trigger_requested_wall_time_ns = time.time_ns()
            action_error: Exception | None = None
            try:
                self._issue_action(station_id)
            except Exception as exc:
                action_error = exc
            trigger_returned_monotonic_ns = time.monotonic_ns()
            trigger_returned_wall_time_ns = time.time_ns()

            artifacts: dict[str, ImageArtifact] = {}
            frame_errors: list[Exception] = []
            for future in as_completed(futures):
                serial = futures[future]
                try:
                    artifacts[serial] = future.result()
                except Exception as exc:
                    frame_errors.append(exc)
            if action_error is not None:
                raise action_error
            if frame_errors:
                raise frame_errors[0]
            return CaptureBatch(
                product_id=product_id,
                station_id=station_id,
                capture_id=capture_id,
                frame_batch_id=frame_batch_id,
                attempt=attempt,
                trigger_requested_monotonic_ns=trigger_requested_monotonic_ns,
                trigger_returned_monotonic_ns=trigger_returned_monotonic_ns,
                trigger_requested_wall_time_ns=trigger_requested_wall_time_ns,
                trigger_returned_wall_time_ns=trigger_returned_wall_time_ns,
                images=tuple(artifacts[serial] for serial in expected_ids),
            )
        except MvsSdkError:
            self._mark_disconnected()
            self._cleanup_incomplete_batch(batch_dir, paths.values())
            raise
        except Exception:
            self._cleanup_incomplete_batch(batch_dir, paths.values())
            raise

    @staticmethod
    def _cleanup_incomplete_batch(batch_dir: Path, paths) -> None:
        for path in paths:
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass
            for temporary in path.parent.glob(f".{path.name}.*.part"):
                try:
                    temporary.unlink()
                except OSError:
                    pass
        try:
            batch_dir.rmdir()
        except OSError:
            pass
