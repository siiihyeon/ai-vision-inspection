"""HIKROBOT MVS 5.0.2 GigE Action Command capture adapter.

SDK import는 hardware 초기화 시점까지 지연합니다. 따라서 ROS/SDK가 없는 PC에서도
순수 domain test가 가능하며, 실장비 프로필은 SDK 또는 필수 장치가 없으면 fail-closed
합니다.
"""

from __future__ import annotations

import asyncio
import ctypes
import importlib
import ipaddress
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType

from .camera_backend import ActionCommandConfig, CameraSpec
from .capture_contract import CameraFrame, CameraUnavailable, RawCaptureBatch


class MvsSdkError(CameraUnavailable):
    pass


@dataclass(frozen=True, slots=True)
class _SdkFrame:
    source_bytes: bytes
    width: int
    height: int
    pixel_type: int
    frame_number: int
    external_trigger_count: int
    device_timestamp_raw: int
    sdk_host_timestamp_raw: int
    host_arrival_monotonic_ns: int
    host_arrival_wall_time_ns: int


@dataclass(slots=True)
class _AttemptWindow:
    requested_monotonic_ns: int
    baseline_frame_number: int
    baseline_trigger_count: int


@dataclass(slots=True)
class _CameraRuntime:
    spec: CameraSpec
    handle: object
    callback: object | None = None
    condition: threading.Condition = field(default_factory=threading.Condition)
    frames: deque[_SdkFrame] = field(default_factory=deque)
    active_window: _AttemptWindow | None = None
    latest_frame_number: int = 0
    latest_trigger_count: int = 0
    ptp_supported: bool = False
    ptp_stable: bool = False


class MvsCaptureBackend:
    """MVS callback metadata로 요청/프레임을 연결하는 실장비 backend."""

    def __init__(
        self,
        camera_specs: tuple[CameraSpec, ...],
        *,
        action_config: ActionCommandConfig,
        python_module_path: Path,
        module_name: str = "MvCameraControl_class",
        frame_timeout_ms: int = 2000,
        sdk_buffer_count: int = 8,
        ptp_enable_if_supported: bool = True,
        camera_timestamp_tick_hz: int | None = None,
        expected_width: int = 2448,
        expected_height: int = 2048,
        bayer_conversion_quality: int = 1,
        expected_model_name: str = "MV-CS050-10GC",
        expected_firmware_version: str = "4.0.43",
        expected_sdk_version: str = "5.0.2",
        expected_sdk_version_raw: int | None = None,
        max_reconnect_attempts: int = 3,
        recovery_test_capture_count: int = 3,
    ) -> None:
        if not camera_specs:
            raise ValueError("at least one camera is required")
        if frame_timeout_ms < 1:
            raise ValueError("frame_timeout_ms must be positive")
        if sdk_buffer_count < 1:
            raise ValueError("sdk_buffer_count must be positive")
        if expected_width < 1 or expected_height < 1:
            raise ValueError("expected frame dimensions must be positive")
        if bayer_conversion_quality not in {0, 1, 2}:
            raise ValueError("bayer_conversion_quality must be 0, 1, or 2")
        if expected_sdk_version_raw is not None and expected_sdk_version_raw < 1:
            raise ValueError("enabled expected SDK raw version must be positive")
        if max_reconnect_attempts != 3 or recovery_test_capture_count != 3:
            raise ValueError("recovery policy is fixed at 3 reconnects/3 test captures")
        self._specs = {spec.camera_id: spec for spec in camera_specs}
        if len(self._specs) != len(camera_specs):
            raise ValueError("duplicate camera_id in camera specifications")
        for spec in camera_specs:
            spec.validate(hardware=True)
        action_config.validate()
        self._action_config = action_config
        self._python_module_path = python_module_path
        self._module_name = module_name
        self._frame_timeout_ms = frame_timeout_ms
        self._sdk_buffer_count = sdk_buffer_count
        self._ptp_enable_if_supported = ptp_enable_if_supported
        self._timestamp_tick_hz = camera_timestamp_tick_hz
        self._expected_width = expected_width
        self._expected_height = expected_height
        self._bayer_conversion_quality = bayer_conversion_quality
        self._expected_model_name = expected_model_name
        self._expected_firmware_version = expected_firmware_version
        self._expected_sdk_version = expected_sdk_version
        self._expected_sdk_version_raw = expected_sdk_version_raw
        self._max_reconnect_attempts = max_reconnect_attempts
        self._recovery_test_capture_count = recovery_test_capture_count
        self._sdk: ModuleType | None = None
        self._cameras: dict[str, _CameraRuntime] = {}
        self._sdk_initialized = False
        self._lifecycle_lock = asyncio.Lock()
        self._capture_locks = {1: asyncio.Lock(), 2: asyncio.Lock()}

    async def initialize(self) -> dict[str, object]:
        async with self._lifecycle_lock:
            await self._close_unlocked()
            try:
                self._sdk = self._load_sdk_module()
                self._check_static("MV_CC_Initialize")
                self._sdk_initialized = True
                actual_sdk_version_raw = self._sdk_version_raw()
                if (
                    self._expected_sdk_version_raw is not None
                    and actual_sdk_version_raw != self._expected_sdk_version_raw
                ):
                    raise MvsSdkError(
                        "MVS SDK version mismatch: "
                        f"expected=0x{self._expected_sdk_version_raw:08x}, "
                        f"actual=0x{actual_sdk_version_raw:08x}",
                        retryable=False,
                    )
                devices = await asyncio.to_thread(self._enumerate_devices)
                for spec in self._specs.values():
                    device = devices.get(spec.serial_number)
                    if device is None:
                        raise MvsSdkError(
                            f"required camera serial not found: {spec.camera_id}/"
                            f"{spec.serial_number}"
                        )
                    actual_ip = self._device_ip(device)
                    actual_mac = self._device_mac(device)
                    actual_model = self._device_model(device)
                    actual_firmware = self._device_firmware(device)
                    if actual_ip != spec.expected_ip:
                        raise MvsSdkError(
                            f"camera IP mismatch for {spec.camera_id}: "
                            f"expected={spec.expected_ip}, actual={actual_ip}"
                        )
                    if spec.expected_mac and actual_mac.lower() != spec.expected_mac.lower():
                        raise MvsSdkError(
                            f"camera MAC mismatch for {spec.camera_id}: "
                            f"expected={spec.expected_mac}, actual={actual_mac}"
                        )
                    if self._expected_model_name not in actual_model:
                        raise MvsSdkError(
                            f"camera model mismatch for {spec.camera_id}: "
                            f"expected={self._expected_model_name}, actual={actual_model}"
                        )
                    if self._expected_firmware_version not in actual_firmware:
                        raise MvsSdkError(
                            f"camera firmware mismatch for {spec.camera_id}: "
                            f"expected={self._expected_firmware_version}, actual={actual_firmware}"
                        )
                    runtime = await asyncio.to_thread(self._open_camera, spec, device)
                    self._cameras[spec.camera_id] = runtime
            except Exception:
                await self._close_unlocked()
                raise
            return {
                "backend": "HIKROBOT_MVS",
                "sdk_version": self._sdk_version(),
                "expected_sdk_version": self._expected_sdk_version,
                "expected_sdk_version_raw": (
                    f"0x{self._expected_sdk_version_raw:08x}"
                    if self._expected_sdk_version_raw is not None
                    else ""
                ),
                "camera_ids": sorted(self._cameras),
                "camera_serials": {
                    camera_id: runtime.spec.serial_number
                    for camera_id, runtime in self._cameras.items()
                },
                "ptp_supported": {
                    camera_id: runtime.ptp_supported
                    for camera_id, runtime in self._cameras.items()
                },
                "ptp_stable": {
                    camera_id: runtime.ptp_stable
                    for camera_id, runtime in self._cameras.items()
                },
                "trigger": "GIGE_ACTION_COMMAND_IMMEDIATE",
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
        if station_id not in self._capture_locks:
            raise ValueError("station_id must be 1 or 2")
        async with self._capture_locks[station_id]:
            runtimes = self._required_runtimes(station_id, required_camera_ids)
            await asyncio.to_thread(self._arm_attempt, runtimes)
            requested_mono = time.monotonic_ns()
            requested_wall = time.time_ns()
            for runtime in runtimes:
                with runtime.condition:
                    runtime.active_window = _AttemptWindow(
                        requested_monotonic_ns=requested_mono,
                        baseline_frame_number=runtime.latest_frame_number,
                        baseline_trigger_count=runtime.latest_trigger_count,
                    )
            try:
                await asyncio.to_thread(self._issue_action_command, station_id)
                returned_mono = time.monotonic_ns()
                returned_wall = time.time_ns()
                sdk_frames = await asyncio.gather(
                    *(
                        asyncio.to_thread(
                            self._wait_for_correlated_frame,
                            runtime,
                            self._frame_timeout_ms / 1000.0,
                        )
                        for runtime in runtimes
                    )
                )
                frames = await asyncio.gather(
                    *(
                        asyncio.to_thread(self._convert_frame, runtime, sdk_frame)
                        for runtime, sdk_frame in zip(runtimes, sdk_frames)
                    )
                )
            finally:
                for runtime in runtimes:
                    with runtime.condition:
                        runtime.active_window = None
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
        runtimes = self._required_runtimes(station_id, required_camera_ids)
        await asyncio.to_thread(self._arm_attempt, runtimes)

    async def recover_station(
        self, *, station_id: int, required_camera_ids: tuple[str, ...]
    ) -> bool:
        delays = (0.0, 0.5, 1.0)[: self._max_reconnect_attempts]
        for delay in delays:
            if delay:
                await asyncio.sleep(delay)
            try:
                async with self._lifecycle_lock:
                    devices = await asyncio.to_thread(self._enumerate_devices)
                    for camera_id in required_camera_ids:
                        previous = self._cameras.pop(camera_id, None)
                        if previous is not None:
                            await asyncio.to_thread(self._close_camera, previous)
                        spec = self._specs[camera_id]
                        device = devices.get(spec.serial_number)
                        if device is None:
                            raise MvsSdkError(f"camera not found during reconnect: {camera_id}")
                        self._cameras[camera_id] = await asyncio.to_thread(
                            self._open_camera, spec, device
                        )
                for index in range(self._recovery_test_capture_count):
                    await self.capture_station(
                        product_id="RECOVERY_TEST",
                        station_id=station_id,
                        capture_id=f"RECOVERY_TEST_{station_id}_{index}",
                        attempt=1,
                        required_camera_ids=required_camera_ids,
                    )
                return True
            except Exception:
                continue
        return False

    async def close(self) -> None:
        async with self._lifecycle_lock:
            await self._close_unlocked()

    async def _close_unlocked(self) -> None:
        for runtime in tuple(self._cameras.values())[::-1]:
            try:
                await asyncio.to_thread(self._close_camera, runtime)
            except Exception:
                pass
        self._cameras.clear()
        if self._sdk_initialized and self._sdk is not None:
            try:
                self._check_static("MV_CC_Finalize")
            except Exception:
                pass
        self._sdk_initialized = False

    def _load_sdk_module(self) -> ModuleType:
        if not self._python_module_path.is_absolute():
            raise MvsSdkError("MVS python_module_path must be absolute", retryable=False)
        if not self._python_module_path.is_dir():
            raise MvsSdkError(
                f"MVS Python wrapper directory does not exist: {self._python_module_path}"
            )
        wrapper = str(self._python_module_path)
        if wrapper not in sys.path:
            sys.path.insert(0, wrapper)
        try:
            return importlib.import_module(self._module_name)
        except Exception as exc:
            raise MvsSdkError(
                f"MVS Python wrapper import failed: {type(exc).__name__}"
            ) from exc

    def _enumerate_devices(self) -> dict[str, object]:
        sdk = self._require_sdk()
        device_list = sdk.MV_CC_DEVICE_INFO_LIST()
        self._check_return(
            sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_GIGE_DEVICE, device_list),
            "MV_CC_EnumDevices",
        )
        devices: dict[str, object] = {}
        for index in range(int(device_list.nDeviceNum)):
            device = ctypes.cast(
                device_list.pDeviceInfo[index], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)
            ).contents
            serial = self._decode_c_array(device.SpecialInfo.stGigEInfo.chSerialNumber)
            if serial in devices:
                raise MvsSdkError(f"duplicate enumerated camera serial: {serial}")
            devices[serial] = device
        return devices

    def _open_camera(self, spec: CameraSpec, device: object) -> _CameraRuntime:
        sdk = self._require_sdk()
        camera = sdk.MvCamera()
        try:
            self._check_return(camera.MV_CC_CreateHandle(device), "MV_CC_CreateHandle")
            self._check_return(
                camera.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0),
                "MV_CC_OpenDevice",
            )
            packet_size = int(camera.MV_CC_GetOptimalPacketSize())
            if packet_size > 0:
                self._check_return(
                    camera.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size),
                    "GevSCPSPacketSize",
                )
            self._check_return(
                camera.MV_CC_SetImageNodeNum(self._sdk_buffer_count),
                "MV_CC_SetImageNodeNum",
            )
            self._check_return(
                camera.MV_CC_SetGrabStrategy(sdk.MV_GrabStrategy_OneByOne),
                "MV_CC_SetGrabStrategy",
            )
            self._set_enum(camera, "PixelFormat", "BayerRG8")
            self._check_return(
                camera.MV_CC_SetBayerCvtQuality(self._bayer_conversion_quality),
                "MV_CC_SetBayerCvtQuality",
            )
            self._set_int(camera, "Width", self._expected_width)
            self._set_int(camera, "Height", self._expected_height)
            self._set_enum(camera, "TriggerSelector", "FrameBurstStart")
            self._set_int(camera, "AcquisitionBurstFrameCount", 1)
            self._set_enum(camera, "TriggerMode", "On")
            self._set_enum(camera, "TriggerSource", "Action1")
            self._set_int(camera, "ActionSelector", 1)
            self._set_int(camera, "ActionDeviceKey", self._action_config.device_key)
            self._set_int(
                camera,
                "ActionGroupKey",
                self._action_config.group_key(spec.station_id),
            )
            self._set_int(camera, "ActionGroupMask", self._action_config.group_mask)
            runtime = _CameraRuntime(spec=spec, handle=camera)
            runtime.ptp_supported = self._try_enable_ptp(camera)
            callback_type = getattr(ctypes, "WINFUNCTYPE", ctypes.CFUNCTYPE)(
                None,
                ctypes.POINTER(ctypes.c_ubyte),
                ctypes.POINTER(sdk.MV_FRAME_OUT_INFO_EX),
                ctypes.c_void_p,
            )

            def on_frame(data_pointer, info_pointer, _user) -> None:
                arrival_mono = time.monotonic_ns()
                arrival_wall = time.time_ns()
                try:
                    info = info_pointer.contents
                    frame = _SdkFrame(
                        source_bytes=ctypes.string_at(data_pointer, int(info.nFrameLen)),
                        width=int(info.nExtendWidth or info.nWidth),
                        height=int(info.nExtendHeight or info.nHeight),
                        pixel_type=int(info.enPixelType),
                        frame_number=int(info.nFrameNum),
                        external_trigger_count=int(info.nTriggerIndex),
                        device_timestamp_raw=(
                            int(info.nDevTimeStampHigh) << 32
                        )
                        | int(info.nDevTimeStampLow),
                        sdk_host_timestamp_raw=int(info.nHostTimeStamp),
                        host_arrival_monotonic_ns=arrival_mono,
                        host_arrival_wall_time_ns=arrival_wall,
                    )
                    with runtime.condition:
                        runtime.latest_frame_number = frame.frame_number
                        runtime.latest_trigger_count = frame.external_trigger_count
                        window = runtime.active_window
                        if window is not None and self._frame_matches(window, frame):
                            runtime.frames.append(frame)
                            runtime.condition.notify_all()
                except Exception:
                    # ctypes callback 밖으로 예외를 전파하지 않습니다. timeout 경로가
                    # 해당 attempt를 실패로 확정하고 telemetry가 원인을 남깁니다.
                    return

            runtime.callback = callback_type(on_frame)
            self._check_return(
                camera.MV_CC_RegisterImageCallBackEx(runtime.callback, None),
                "MV_CC_RegisterImageCallBackEx",
            )
            self._check_return(camera.MV_CC_StartGrabbing(), "MV_CC_StartGrabbing")
            return runtime
        except Exception:
            try:
                camera.MV_CC_CloseDevice()
            except Exception:
                pass
            try:
                camera.MV_CC_DestroyHandle()
            except Exception:
                pass
            raise

    def _close_camera(self, runtime: _CameraRuntime) -> None:
        camera = runtime.handle
        with runtime.condition:
            runtime.active_window = None
            runtime.frames.clear()
            runtime.condition.notify_all()
        stop_result = camera.MV_CC_StopGrabbing()
        close_result = camera.MV_CC_CloseDevice()
        destroy_result = camera.MV_CC_DestroyHandle()
        for result, operation in (
            (stop_result, "MV_CC_StopGrabbing"),
            (close_result, "MV_CC_CloseDevice"),
            (destroy_result, "MV_CC_DestroyHandle"),
        ):
            self._check_return(result, operation)

    def _arm_attempt(self, runtimes: tuple[_CameraRuntime, ...]) -> None:
        for runtime in runtimes:
            self._check_return(
                runtime.handle.MV_CC_ClearImageBuffer(), "MV_CC_ClearImageBuffer"
            )
            with runtime.condition:
                runtime.frames.clear()
                runtime.active_window = None

    def _issue_action_command(self, station_id: int) -> None:
        sdk = self._require_sdk()
        command = sdk.MV_ACTION_CMD_INFO()
        command.nDeviceKey = self._action_config.device_key
        command.nGroupKey = self._action_config.group_key(station_id)
        command.nGroupMask = self._action_config.group_mask
        command.bActionTimeEnable = 0
        command.nActionTime = 0
        command.pBroadcastAddress = self._action_config.broadcast_address.encode("ascii")
        command.nTimeOut = self._action_config.ack_timeout_ms
        command.bSpecialNetEnable = 1
        command.nSpecialNetIP = int(ipaddress.IPv4Address(self._action_config.nic_ip))
        results = sdk.MV_ACTION_CMD_RESULT_LIST()
        self._check_return(
            sdk.MvCamera.MV_GIGE_IssueActionCommand(command, results),
            "MV_GIGE_IssueActionCommand",
        )
        for index in range(int(results.nNumResults)):
            result = results.pResults[index]
            if int(result.nStatus) != 0:
                raise MvsSdkError(
                    f"Action Command device ACK failed: status=0x{int(result.nStatus):04x}"
                )

    def _wait_for_correlated_frame(
        self, runtime: _CameraRuntime, timeout_seconds: float
    ) -> _SdkFrame:
        deadline = time.monotonic() + timeout_seconds
        with runtime.condition:
            while not runtime.frames:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise MvsSdkError(
                        f"camera frame timeout: {runtime.spec.camera_id}"
                    )
                runtime.condition.wait(remaining)
            return runtime.frames.popleft()

    def _convert_frame(
        self, runtime: _CameraRuntime, frame: _SdkFrame
    ) -> CameraFrame:
        sdk = self._require_sdk()
        if frame.width != self._expected_width or frame.height != self._expected_height:
            raise MvsSdkError(
                f"unexpected dimensions from {runtime.spec.camera_id}: "
                f"{frame.width}x{frame.height}"
            )
        if frame.pixel_type != int(sdk.PixelType_Gvsp_BayerRG8):
            raise MvsSdkError(
                f"unexpected pixel type from {runtime.spec.camera_id}: {frame.pixel_type}"
            )
        source = (ctypes.c_ubyte * len(frame.source_bytes)).from_buffer_copy(
            frame.source_bytes
        )
        destination_size = frame.width * frame.height * 3
        destination = (ctypes.c_ubyte * destination_size)()
        parameters = sdk.MV_CC_PIXEL_CONVERT_PARAM_EX()
        parameters.nWidth = frame.width
        parameters.nHeight = frame.height
        parameters.pSrcData = source
        parameters.nSrcDataLen = len(frame.source_bytes)
        parameters.enSrcPixelType = frame.pixel_type
        parameters.enDstPixelType = sdk.PixelType_Gvsp_RGB8_Packed
        parameters.pDstBuffer = destination
        parameters.nDstBufferSize = destination_size
        self._check_return(
            runtime.handle.MV_CC_ConvertPixelTypeEx(parameters),
            "MV_CC_ConvertPixelTypeEx",
        )
        if int(parameters.nDstLen) != destination_size:
            raise MvsSdkError("MVS RGB conversion returned unexpected byte length")
        timestamp_ns = 0
        domain = "MVS_DEVICE_TICKS"
        if self._timestamp_tick_hz:
            timestamp_ns = (
                frame.device_timestamp_raw * 1_000_000_000 // self._timestamp_tick_hz
            )
            domain = f"MVS_DEVICE_TICKS_{self._timestamp_tick_hz}HZ"
        return CameraFrame(
            camera_id=runtime.spec.camera_id,
            width=frame.width,
            height=frame.height,
            rgb_bytes=bytes(destination),
            frame_number=frame.frame_number,
            external_trigger_count=frame.external_trigger_count,
            camera_timestamp_raw=frame.device_timestamp_raw,
            camera_timestamp_domain=domain,
            camera_timestamp_ns=timestamp_ns,
            camera_timestamp_synchronized=runtime.ptp_stable,
            sdk_host_timestamp_raw=frame.sdk_host_timestamp_raw,
            host_arrival_monotonic_ns=frame.host_arrival_monotonic_ns,
            host_arrival_wall_time_ns=frame.host_arrival_wall_time_ns,
        )

    def _try_enable_ptp(self, camera: object) -> bool:
        if not self._ptp_enable_if_supported:
            return False
        try:
            self._check_return(
                camera.MV_CC_SetBoolValue("GevIEEE1588", True), "GevIEEE1588"
            )
            # PTP lock 안정 시간/상태 판정은 아직 실측 미정입니다. immediate
            # Action은 lock에 의존하지 않으며 synchronized는 보수적으로 false입니다.
            return True
        except MvsSdkError:
            return False

    def _required_runtimes(
        self, station_id: int, camera_ids: tuple[str, ...]
    ) -> tuple[_CameraRuntime, ...]:
        if not camera_ids or len(camera_ids) != len(set(camera_ids)):
            raise MvsSdkError("required camera list is empty or duplicated", retryable=False)
        runtimes: list[_CameraRuntime] = []
        for camera_id in camera_ids:
            runtime = self._cameras.get(camera_id)
            if runtime is None:
                raise MvsSdkError(f"camera is not open: {camera_id}")
            if runtime.spec.station_id != station_id:
                raise MvsSdkError(
                    f"camera {camera_id} does not belong to station {station_id}",
                    retryable=False,
                )
            runtimes.append(runtime)
        return tuple(runtimes)

    @staticmethod
    def _frame_matches(window: _AttemptWindow, frame: _SdkFrame) -> bool:
        if frame.host_arrival_monotonic_ns < window.requested_monotonic_ns:
            return False
        if frame.external_trigger_count > 0 and window.baseline_trigger_count > 0:
            return MvsCaptureBackend._counter_after(
                frame.external_trigger_count, window.baseline_trigger_count
            )
        return MvsCaptureBackend._counter_after(
            frame.frame_number, window.baseline_frame_number
        )

    @staticmethod
    def _counter_after(value: int, baseline: int) -> bool:
        if value == baseline:
            return False
        return 0 < ((value - baseline) & 0xFFFFFFFF) < 0x80000000

    def _set_enum(self, camera: object, key: str, value: str) -> None:
        self._check_return(camera.MV_CC_SetEnumValueByString(key, value), key)

    def _set_int(self, camera: object, key: str, value: int) -> None:
        self._check_return(camera.MV_CC_SetIntValue(key, int(value)), key)

    def _check_static(self, operation: str) -> None:
        sdk = self._require_sdk()
        function = getattr(sdk.MvCamera, operation, None)
        if function is None:
            raise MvsSdkError(f"MVS wrapper is missing {operation}", retryable=False)
        self._check_return(function(), operation)

    @staticmethod
    def _check_return(result: int, operation: str) -> None:
        value = int(result)
        if value != 0:
            raise MvsSdkError(f"{operation} failed: 0x{value & 0xFFFFFFFF:08x}")

    def _require_sdk(self) -> ModuleType:
        if self._sdk is None:
            raise MvsSdkError("MVS SDK is not loaded")
        return self._sdk

    def _sdk_version(self) -> str:
        return f"0x{self._sdk_version_raw():08x}"

    def _sdk_version_raw(self) -> int:
        sdk = self._require_sdk()
        return int(sdk.MvCamera.MV_CC_GetSDKVersion())

    @staticmethod
    def _decode_c_array(value: object) -> str:
        raw = bytes(value)
        return raw.split(b"\x00", 1)[0].decode("ascii", errors="strict")

    @staticmethod
    def _device_ip(device: object) -> str:
        return str(ipaddress.IPv4Address(int(device.SpecialInfo.stGigEInfo.nCurrentIp)))

    @staticmethod
    def _device_mac(device: object) -> str:
        value = ((int(device.nMacAddrHigh) & 0xFFFF) << 32) | int(device.nMacAddrLow)
        return ":".join(f"{(value >> shift) & 0xFF:02X}" for shift in (40, 32, 24, 16, 8, 0))

    @staticmethod
    def _device_model(device: object) -> str:
        return MvsCaptureBackend._decode_c_array(
            device.SpecialInfo.stGigEInfo.chModelName
        )

    @staticmethod
    def _device_firmware(device: object) -> str:
        return MvsCaptureBackend._decode_c_array(
            device.SpecialInfo.stGigEInfo.chDeviceVersion
        )
