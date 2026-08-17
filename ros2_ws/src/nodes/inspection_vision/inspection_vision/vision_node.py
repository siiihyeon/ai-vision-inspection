"""VisionNode ROS adapter: 통신·수명주기와 순수 Python runtime의 경계."""

from __future__ import annotations

import asyncio
import json
import threading
from pathlib import Path

import rclpy
from inspection_common import (
    DurableLogSpool,
    ErrorCode,
    IdempotencyStore,
    NodeHealthState,
    NodeId,
    SpoolRecord,
    SystemState,
    canonical_json,
    new_uuid,
    payload_digest,
    sha256_text,
)
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
    state_qos,
)
from inspection_interfaces.action import CaptureProduct
from inspection_interfaces.msg import (
    ImageReference,
    LogEvent,
    LogPersistedAck,
    ProductResultLocked,
    StationInferenceFailed,
    StationResult,
    VisionQueueState,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter

from .artifact_store import ArtifactStore, DiskPolicy, configure_restrictive_umask
from .camera_backend import ActionCommandConfig, CameraSpec, SimulationCaptureBackend
from .capture_contract import CaptureBatch, ImageArtifact
from .capture_service import (
    CaptureCanceled,
    CapturePipelineFailed,
    CaptureProgress,
    CaptureProgressStage,
    CaptureRequest,
)
from .model_adapter import (
    SimulationModel,
    StationInference,
    load_plugin_model,
    load_rgb_png_with_opencv,
)
from .mvs_backend import MvsCaptureBackend
from .queue_journal import InferenceJournal
from .vision_runtime import (
    EnqueueCanceled,
    InferenceFailure,
    QueueRuntimeState,
    QueueStateEvent,
    VisionRuntime,
)


class VisionNode(InspectionNodeBase):
    """Capture 성공을 path-only FrameBatch의 Queue enqueue 완료로 정의합니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.VISION, provides_initialize_action=True)
        self._declare_vision_parameters()
        self._validate_fixed_contracts()
        self._runtime: VisionRuntime | None = None
        self._runtime_details: dict[str, object] = {}
        self._resource_guard = threading.RLock()
        self._active_capture_count = 0
        self._capture_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._capture_identities: dict[str, tuple[str, int]] = {}
        self._completed_captures: dict[
            tuple[str, int, str], dict[str, object]
        ] = {}
        self._identity_guard = threading.RLock()
        self._station_locks = {1: asyncio.Lock(), 2: asyncio.Lock()}
        configured_camera_ids = set(self._configured_camera_ids(1)) | set(
            self._configured_camera_ids(2)
        )
        self._camera_locks = {
            camera_id: asyncio.Lock() for camera_id in configured_camera_ids
        }
        self._last_queue_state: int | None = None
        self._deferred_queue_event: QueueStateEvent | None = None
        self._deferred_telemetry: list[
            tuple[str, int, str, dict[str, object]]
        ] = []
        self._log_spool: DurableLogSpool | None = None
        self._log_spool_open_failure_reported = False

        callback_group = ReentrantCallbackGroup()
        self._capture_server = ActionServer(
            self,
            CaptureProduct,
            "vision/capture_product",
            execute_callback=self._execute_capture,
            goal_callback=self._accept_capture_goal,
            cancel_callback=self._cancel_capture_goal,
            callback_group=callback_group,
        )
        self._queue_state_publisher = self.create_publisher(
            VisionQueueState, "vision/queue_state", state_qos()
        )
        self._station_result_publisher = self.create_publisher(
            StationResult, "vision/station_result", reliable_event_qos()
        )
        self._station_failure_publisher = self.create_publisher(
            StationInferenceFailed,
            "vision/station_inference_failed",
            reliable_event_qos(),
        )
        self._locked_subscription = self.create_subscription(
            ProductResultLocked,
            "/inspection/master/product_result_locked",
            self._handle_product_locked,
            reliable_event_qos(),
        )
        self._log_event_publisher = self.create_publisher(
            LogEvent, "log/event", reliable_event_qos(depth=1000)
        )
        self._log_ack_subscription = self.create_subscription(
            LogPersistedAck,
            "log/persisted_ack",
            self._handle_log_persisted_ack,
            reliable_event_qos(depth=1000),
        )
        flush_period_ms = int(self.get_parameter("vision.log.flush_period_ms").value)
        self._log_flush_timer = self.create_timer(
            flush_period_ms / 1000.0, self._flush_log_spool
        )
        self.get_logger().info(
            "VisionNode feature/vision-node adapter started; waiting for InitializeNode"
        )

    def _declare_vision_parameters(self) -> None:
        self.declare_parameter("vision.trigger.mode", "GIGE_ACTION_COMMAND")
        self.declare_parameter(
            "vision.camera_ids.station_a", Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter(
            "vision.camera_ids.station_b", Parameter.Type.STRING_ARRAY
        )
        for key in ("serials", "ips", "macs"):
            self.declare_parameter(
                f"vision.camera_{key}.station_a", Parameter.Type.STRING_ARRAY
            )
            self.declare_parameter(
                f"vision.camera_{key}.station_b", Parameter.Type.STRING_ARRAY
            )
        self.declare_parameter("vision.camera.width", 2448)
        self.declare_parameter("vision.camera.height", 2048)
        self.declare_parameter("vision.camera.sdk_buffer_count", 8)
        self.declare_parameter("vision.camera.frame_timeout_ms", 2000)
        self.declare_parameter("vision.camera.timestamp_tick_hz", 0)
        self.declare_parameter("vision.camera.ptp_enable_if_supported", True)
        self.declare_parameter("vision.camera.ptp_stability_ms", 0)
        self.declare_parameter("vision.camera.bayer_conversion_quality", 1)
        self.declare_parameter("vision.camera.expected_model_name", "MV-CS050-10GC")
        self.declare_parameter("vision.camera.expected_firmware_version", "4.0.43")
        self.declare_parameter("vision.mvs.python_module_path", "")
        self.declare_parameter("vision.mvs.module_name", "MvCameraControl_class")
        self.declare_parameter("vision.mvs.expected_sdk_version", "5.0.2")
        self.declare_parameter("vision.mvs.expected_sdk_version_raw", 0)
        self.declare_parameter("vision.gige_action.device_key", 0x13572468)
        self.declare_parameter("vision.gige_action.station_a.group_key", 1)
        self.declare_parameter("vision.gige_action.station_b.group_key", 2)
        self.declare_parameter("vision.gige_action.group_mask", 0xFFFFFFFF)
        self.declare_parameter(
            "vision.gige_action.broadcast_address", "255.255.255.255"
        )
        self.declare_parameter("vision.gige_action.ack_timeout_ms", 200)
        self.declare_parameter("vision.network.nic_ip", "192.168.10.10")
        self.declare_parameter("vision.frame_arrival_skew_limit_us", 0)
        self.declare_parameter("vision.capture.max_attempts", 2)
        self.declare_parameter("vision.queue.capacity", 16)
        self.declare_parameter("vision.queue.warning_ratio", 0.75)
        self.declare_parameter("vision.queue.resume_ratio", 0.50)
        self.declare_parameter("vision.worker_count", 2)
        self.declare_parameter("vision.worker.serialize_model_access", True)
        self.declare_parameter("vision.inference_queue_total_timeout_ms", 0)
        self.declare_parameter("vision.data_root", "/tmp/inspection/data")
        self.declare_parameter(
            "vision.queue_journal_path",
            "/tmp/inspection/spool/vision_queue.sqlite3",
        )
        self.declare_parameter("vision.png_compression_level", 3)
        self.declare_parameter("vision.storage.warning_used_percent", 80.0)
        self.declare_parameter("vision.storage.pause_used_percent", 90.0)
        self.declare_parameter("vision.storage.critical_used_percent", 95.0)
        self.declare_parameter("vision.storage.warning_free_gb", 20)
        self.declare_parameter("vision.storage.pause_free_gb", 10)
        self.declare_parameter("vision.storage.critical_free_gb", 5)
        self.declare_parameter("vision.model.path", "")
        self.declare_parameter("vision.model.sha256", "")
        self.declare_parameter("vision.model.factory", "")
        self.declare_parameter("vision.model.device", "cuda:0")
        self.declare_parameter(
            "vision.log.spool_path", "/tmp/inspection/spool/vision_log.sqlite3"
        )
        self.declare_parameter("vision.log.flush_period_ms", 1000)
        self.declare_parameter("vision.recovery.max_reconnect_attempts", 3)
        self.declare_parameter("vision.recovery.test_capture_count", 3)

    def _validate_fixed_contracts(self) -> None:
        fixed = {
            "vision.trigger.mode": "GIGE_ACTION_COMMAND",
            "vision.capture.max_attempts": 2,
            "vision.camera.sdk_buffer_count": 8,
            "vision.camera.bayer_conversion_quality": 1,
            "vision.camera.expected_model_name": "MV-CS050-10GC",
            "vision.camera.expected_firmware_version": "4.0.43",
            "vision.mvs.expected_sdk_version": "5.0.2",
            "vision.png_compression_level": 3,
            "vision.gige_action.device_key": 0x13572468,
            "vision.gige_action.station_a.group_key": 1,
            "vision.gige_action.station_b.group_key": 2,
            "vision.gige_action.group_mask": 0xFFFFFFFF,
            "vision.recovery.max_reconnect_attempts": 3,
            "vision.recovery.test_capture_count": 3,
        }
        for key, expected in fixed.items():
            actual = self.get_parameter(key).value
            if actual != expected:
                raise ValueError(f"{key} is fixed at {expected!r}, got {actual!r}")
        if float(self.get_parameter("vision.queue.warning_ratio").value) != 0.75:
            raise ValueError("vision.queue.warning_ratio is fixed at 0.75")
        if float(self.get_parameter("vision.queue.resume_ratio").value) != 0.50:
            raise ValueError("vision.queue.resume_ratio is fixed at 0.50")
        flush_period = int(self.get_parameter("vision.log.flush_period_ms").value)
        if not 100 <= flush_period <= 60_000:
            raise ValueError("vision.log.flush_period_ms must be 100..60000")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        return (
            "vision.camera_ids.station_a",
            "vision.camera_ids.station_b",
            "vision.camera_serials.station_a",
            "vision.camera_serials.station_b",
            "vision.camera_ips.station_a",
            "vision.camera_ips.station_b",
            "vision.mvs.python_module_path",
            "vision.mvs.expected_sdk_version_raw",
            "vision.frame_arrival_skew_limit_us",
            "vision.camera.frame_timeout_ms",
            "vision.queue.capacity",
            "vision.worker_count",
            "vision.inference_queue_total_timeout_ms",
            "vision.data_root",
            "vision.queue_journal_path",
            "vision.model.path",
            "vision.model.sha256",
            "vision.model.factory",
            "vision.log.spool_path",
        )

    def validate_hardware_profile(self) -> list[str]:
        missing = super().validate_hardware_profile()
        if self.profile != "hardware":
            return missing
        station_a = self._configured_camera_ids(1)
        station_b = self._configured_camera_ids(2)
        if len(station_a) != 3:
            missing.append("vision.camera_ids.station_a must contain exactly 3 IDs")
        if len(station_b) != 1:
            missing.append("vision.camera_ids.station_b must contain exactly 1 ID")
        if set(station_a) & set(station_b):
            missing.append("station camera IDs must be disjoint")
        for station_id, expected_count in ((1, 3), (2, 1)):
            for key in ("serials", "ips", "macs"):
                values = self._camera_values(key, station_id)
                if len(values) != expected_count:
                    missing.append(
                        f"vision.camera_{key}.station_{'a' if station_id == 1 else 'b'} "
                        f"must contain {expected_count} values"
                    )
                if key in {"serials", "ips"} and any(not value for value in values):
                    missing.append(f"all hardware camera {key} values must be nonempty")
        for key in (
            "vision.mvs.expected_sdk_version_raw",
            "vision.frame_arrival_skew_limit_us",
            "vision.camera.frame_timeout_ms",
            "vision.queue.capacity",
            "vision.worker_count",
            "vision.inference_queue_total_timeout_ms",
        ):
            if int(self.get_parameter(key).value) <= 0:
                missing.append(f"{key} must be positive")
        for key in (
            "vision.data_root",
            "vision.queue_journal_path",
            "vision.model.path",
            "vision.log.spool_path",
            "vision.mvs.python_module_path",
        ):
            value = str(self.get_parameter(key).value)
            if value and not Path(value).is_absolute():
                missing.append(f"{key} must be absolute")
        model_sha = str(self.get_parameter("vision.model.sha256").value)
        if len(model_sha) != 64:
            missing.append("vision.model.sha256 must be lowercase SHA-256 hex")
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        with self._resource_guard:
            if self._active_capture_count:
                return NodeInitializationOutcome(
                    success=False,
                    error_code=int(ErrorCode.NODE_INIT_FAILED),
                    reason="cannot reinitialize Vision resources during an active capture",
                    retryable=True,
                )
            previous = self._runtime
            self._runtime = None
        if previous is not None:
            try:
                await previous.close()
            except Exception as exc:
                self.get_logger().warning(
                    f"previous VisionRuntime close failed: {type(exc).__name__}"
                )
        replacement: VisionRuntime | None = None
        try:
            self._ensure_log_spool()
            replacement = await self._build_runtime()
            details = await replacement.start()
        except Exception as exc:
            if replacement is not None:
                try:
                    await replacement.close()
                except Exception:
                    pass
            self._emit_log_event(
                severity=LogEvent.ERROR,
                event_type="VISION_INITIALIZATION_FAILED",
                payload={"error_type": type(exc).__name__, "reason": str(exc)},
            )
            return NodeInitializationOutcome(
                success=False,
                error_code=int(ErrorCode.NODE_INIT_FAILED),
                reason=f"Vision resource initialization failed: {type(exc).__name__}: {exc}",
                retryable=True,
            )
        with self._resource_guard:
            self._runtime = replacement
            self._runtime_details = details
        return NodeInitializationOutcome(
            success=True,
            reason="camera, storage, journal, model warmup and workers initialized",
            status_details=details,
        )

    async def _build_runtime(self) -> VisionRuntime:
        specs = self._camera_specs()
        artifact_store = ArtifactStore(
            Path(str(self.get_parameter("vision.data_root").value)),
            compression_level=int(
                self.get_parameter("vision.png_compression_level").value
            ),
            disk_policy=DiskPolicy(
                warning_used_percent=float(
                    self.get_parameter("vision.storage.warning_used_percent").value
                ),
                pause_used_percent=float(
                    self.get_parameter("vision.storage.pause_used_percent").value
                ),
                critical_used_percent=float(
                    self.get_parameter("vision.storage.critical_used_percent").value
                ),
                warning_free_bytes=int(
                    self.get_parameter("vision.storage.warning_free_gb").value
                )
                * 1024**3,
                pause_free_bytes=int(
                    self.get_parameter("vision.storage.pause_free_gb").value
                )
                * 1024**3,
                critical_free_bytes=int(
                    self.get_parameter("vision.storage.critical_free_gb").value
                )
                * 1024**3,
            ),
        )
        action_config = ActionCommandConfig(
            device_key=int(self.get_parameter("vision.gige_action.device_key").value),
            station_a_group_key=int(
                self.get_parameter("vision.gige_action.station_a.group_key").value
            ),
            station_b_group_key=int(
                self.get_parameter("vision.gige_action.station_b.group_key").value
            ),
            group_mask=int(self.get_parameter("vision.gige_action.group_mask").value),
            broadcast_address=str(
                self.get_parameter("vision.gige_action.broadcast_address").value
            ),
            ack_timeout_ms=int(
                self.get_parameter("vision.gige_action.ack_timeout_ms").value
            ),
            nic_ip=str(self.get_parameter("vision.network.nic_ip").value),
        )
        if self.profile == "sim":
            backend = SimulationCaptureBackend(specs)
            model = SimulationModel()
            load_image = lambda path: path.read_bytes()
        else:
            tick_hz = int(self.get_parameter("vision.camera.timestamp_tick_hz").value)
            backend = MvsCaptureBackend(
                specs,
                action_config=action_config,
                python_module_path=Path(
                    str(self.get_parameter("vision.mvs.python_module_path").value)
                ),
                module_name=str(self.get_parameter("vision.mvs.module_name").value),
                frame_timeout_ms=int(
                    self.get_parameter("vision.camera.frame_timeout_ms").value
                ),
                sdk_buffer_count=int(
                    self.get_parameter("vision.camera.sdk_buffer_count").value
                ),
                ptp_enable_if_supported=bool(
                    self.get_parameter("vision.camera.ptp_enable_if_supported").value
                ),
                camera_timestamp_tick_hz=tick_hz if tick_hz > 0 else None,
                expected_width=int(self.get_parameter("vision.camera.width").value),
                expected_height=int(self.get_parameter("vision.camera.height").value),
                bayer_conversion_quality=int(
                    self.get_parameter("vision.camera.bayer_conversion_quality").value
                ),
                expected_model_name=str(
                    self.get_parameter("vision.camera.expected_model_name").value
                ),
                expected_firmware_version=str(
                    self.get_parameter("vision.camera.expected_firmware_version").value
                ),
                expected_sdk_version=str(
                    self.get_parameter("vision.mvs.expected_sdk_version").value
                ),
                expected_sdk_version_raw=(
                    int(self.get_parameter("vision.mvs.expected_sdk_version_raw").value)
                    or None
                ),
                max_reconnect_attempts=int(
                    self.get_parameter("vision.recovery.max_reconnect_attempts").value
                ),
                recovery_test_capture_count=int(
                    self.get_parameter("vision.recovery.test_capture_count").value
                ),
            )
            model = await asyncio.to_thread(
                load_plugin_model,
                factory_reference=str(self.get_parameter("vision.model.factory").value),
                model_path=Path(str(self.get_parameter("vision.model.path").value)),
                expected_sha256=str(self.get_parameter("vision.model.sha256").value),
                device=str(self.get_parameter("vision.model.device").value),
            )
            load_image = load_rgb_png_with_opencv
        journal = None
        try:
            journal = InferenceJournal(
                Path(str(self.get_parameter("vision.queue_journal_path").value))
            )
            skew_limit = int(
                self.get_parameter("vision.frame_arrival_skew_limit_us").value
            )
            queue_timeout = int(
                self.get_parameter("vision.inference_queue_total_timeout_ms").value
            )
            return VisionRuntime(
                backend=backend,
                artifact_store=artifact_store,
                journal=journal,
                model=model,
                load_image=load_image,
                queue_capacity=int(self.get_parameter("vision.queue.capacity").value),
                worker_count=int(self.get_parameter("vision.worker_count").value),
                queue_total_timeout_ms=queue_timeout if queue_timeout > 0 else None,
                frame_arrival_skew_limit_us=skew_limit if skew_limit > 0 else None,
                frame_timeout_ms=int(
                    self.get_parameter("vision.camera.frame_timeout_ms").value
                ),
                serialize_model_access=bool(
                    self.get_parameter("vision.worker.serialize_model_access").value
                ),
                on_queue_state=self._handle_runtime_queue_state,
                on_inference_success=self._publish_station_result,
                on_inference_failure=self._publish_station_failure,
                on_telemetry=self._runtime_telemetry,
                queue_warning_ratio=float(
                    self.get_parameter("vision.queue.warning_ratio").value
                ),
                queue_resume_ratio=float(
                    self.get_parameter("vision.queue.resume_ratio").value
                ),
            )
        except Exception:
            if journal is not None:
                try:
                    await asyncio.to_thread(journal.close)
                except Exception:
                    pass
            try:
                await asyncio.to_thread(model.close)
            except Exception:
                pass
            raise

    def _accept_capture_goal(self, goal_request) -> GoalResponse:
        cameras = tuple(goal_request.required_camera_ids)
        configured = (
            self._configured_camera_ids(int(goal_request.station_id))
            if int(goal_request.station_id) in {1, 2}
            else ()
        )
        valid = (
            bool(goal_request.product_id)
            and bool(goal_request.capture_id)
            and int(goal_request.station_id) in {1, 2}
            and cameras == configured
            and len(cameras) == len(set(cameras))
            and bool(goal_request.command.command_id)
        )
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def _cancel_capture_goal(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_capture(self, goal_handle) -> CaptureProduct.Result:
        request = goal_handle.request
        result = CaptureProduct.Result()
        self._publish_capture_feedback(
            goal_handle,
            CaptureProduct.Feedback.VALIDATING,
            0,
            (),
            tuple(request.required_camera_ids),
            "",
            0.0,
            "validating command envelope and capture identity",
        )
        valid, code, reason = self.validate_command_header(
            request.command,
            allowed_system_states={
                SystemState.RUN_SYS,
                SystemState.PAUSING,
                SystemState.PAUSED,
            },
        )
        if not valid:
            return self._terminal_capture(goal_handle, result, code, reason)
        expected_digest = payload_digest(
            {
                "product_id": request.product_id,
                "fifo_sequence": int(request.fifo_sequence),
                "station_id": int(request.station_id),
                "capture_id": request.capture_id,
                "required_camera_ids": list(request.required_camera_ids),
            }
        )
        if request.command.payload_digest != expected_digest:
            return self._terminal_capture(
                goal_handle,
                result,
                ErrorCode.COMMAND_CONFLICT,
                "CaptureProduct payload_digest mismatch",
            )
        replay = self._capture_results.inspect(
            request.command.command_id, request.command.payload_digest
        )
        if replay.kind.value == "CONFLICT":
            return self._terminal_capture(
                goal_handle,
                result,
                ErrorCode.COMMAND_CONFLICT,
                "same command_id received with another digest",
            )
        if replay.result is not None:
            self._apply_capture_values(result, replay.result)
            goal_handle.succeed() if result.success else goal_handle.abort()
            return result

        with self._resource_guard:
            runtime = self._runtime
            if runtime is None:
                return self._terminal_capture(
                    goal_handle,
                    result,
                    ErrorCode.NODE_INIT_FAILED,
                    "Vision resources are not initialized",
                )
            self._active_capture_count += 1
        try:
            return await self._execute_capture_with_runtime(goal_handle, result, runtime)
        finally:
            with self._resource_guard:
                self._active_capture_count -= 1

    async def _execute_capture_with_runtime(
        self, goal_handle, result: CaptureProduct.Result, runtime: VisionRuntime
    ) -> CaptureProduct.Result:
        request = goal_handle.request
        identity = (request.product_id, int(request.station_id))
        with self._identity_guard:
            previous_identity = self._capture_identities.get(request.capture_id)
            if previous_identity is not None and previous_identity != identity:
                return self._terminal_capture(
                    goal_handle,
                    result,
                    ErrorCode.COMMAND_CONFLICT,
                    "capture_id already belongs to another product/station",
                )
            self._capture_identities[request.capture_id] = identity
        capture_key = (request.product_id, int(request.station_id), request.capture_id)
        completed = self._completed_captures.get(capture_key)
        if completed is not None:
            self._capture_results.remember(
                request.command.command_id, request.command.payload_digest, completed
            )
            self._apply_capture_values(result, completed)
            goal_handle.succeed()
            return result
        if runtime.captured_before_runtime(*capture_key):
            values = self._failure_values(
                request,
                ErrorCode.CAPTURE_FAILED,
                "capture_id has artifacts from a previous Vision process; automatic recapture is prohibited",
            )
            return self._remember_terminal(goal_handle, result, values)

        async with self._station_locks[int(request.station_id)]:
            locks = [
                self._camera_locks[camera_id]
                for camera_id in sorted(request.required_camera_ids)
            ]
            for lock in locks:
                await lock.acquire()
            try:
                batch = await runtime.capture_service.capture(
                    CaptureRequest(
                        product_id=request.product_id,
                        fifo_sequence=int(request.fifo_sequence),
                        station_id=int(request.station_id),
                        capture_id=request.capture_id,
                        required_camera_ids=tuple(request.required_camera_ids),
                    ),
                    on_progress=lambda progress: self._publish_domain_progress(
                        goal_handle, progress
                    ),
                    is_cancel_requested=lambda: bool(goal_handle.is_cancel_requested),
                )
            except CaptureCanceled as exc:
                values = self._failure_values(
                    request, ErrorCode.CAPTURE_CANCELED, exc.failure.reason
                )
                return self._remember_terminal(
                    goal_handle, result, values, canceled=True
                )
            except CapturePipelineFailed as exc:
                if exc.failure.recovery_succeeded is False:
                    self.set_health_state(NodeHealthState.INIT_BLOCKED)
                    self._handle_runtime_queue_state(
                        QueueStateEvent(
                            QueueRuntimeState.FAULT,
                            runtime.queue.depth,
                            runtime.queue.capacity,
                            reason="camera recovery failed; all Vision stations blocked",
                        )
                    )
                values = self._failure_values(
                    request, exc.failure.error_code, exc.failure.reason
                )
                self._emit_log_event(
                    severity=LogEvent.ERROR,
                    event_type="CAPTURE_FINAL_FAILED",
                    product_id=request.product_id,
                    payload={
                        "station_id": int(request.station_id),
                        "capture_id": request.capture_id,
                        "attempt": exc.failure.attempt,
                        "error_code": exc.failure.error_code,
                        "reason": exc.failure.reason,
                        "camera_recovery_succeeded": exc.failure.recovery_succeeded,
                    },
                )
                return self._remember_terminal(goal_handle, result, values)
            finally:
                for lock in reversed(locks):
                    lock.release()

        self._publish_capture_feedback(
            goal_handle,
            CaptureProduct.Feedback.ENQUEUEING_INFERENCE,
            batch.attempt,
            tuple(image.camera_id for image in batch.images),
            (),
            batch.frame_batch_id,
            0.95,
            "committing and enqueueing path-only station FrameBatch",
        )
        try:
            job = await runtime.enqueue_saved_batch(
                batch=batch,
                fifo_sequence=int(request.fifo_sequence),
                is_cancel_requested=lambda: bool(goal_handle.is_cancel_requested),
                on_blocked=lambda blocked_job: self._publish_capture_feedback(
                    goal_handle,
                    CaptureProduct.Feedback.ENQUEUE_BLOCKED,
                    batch.attempt,
                    tuple(image.camera_id for image in batch.images),
                    (),
                    batch.frame_batch_id,
                    0.95,
                    f"queue full; preserving saved job {blocked_job.inference_job_id}",
                ),
                resume_after_block_allowed=lambda: self.system_state
                == SystemState.PAUSED,
            )
        except EnqueueCanceled as exc:
            values = self._failure_values(
                request, ErrorCode.CAPTURE_CANCELED, str(exc), batch=batch
            )
            return self._remember_terminal(goal_handle, result, values, canceled=True)
        except Exception as exc:
            values = self._failure_values(
                request,
                ErrorCode.INFERENCE_FAILED,
                f"inference enqueue failed: {type(exc).__name__}: {exc}",
                batch=batch,
            )
            return self._remember_terminal(goal_handle, result, values)

        values = self._capture_success_values(batch, job.inference_job_id)
        with self._identity_guard:
            self._completed_captures[capture_key] = values
        self._capture_results.remember(
            request.command.command_id, request.command.payload_digest, values
        )
        self._apply_capture_values(result, values)
        self._emit_capture_telemetry(request, batch, job.inference_job_id)
        goal_handle.succeed()
        return result

    def _publish_domain_progress(self, goal_handle, progress: CaptureProgress) -> None:
        mapping = {
            CaptureProgressStage.CAMERAS_READY: (
                CaptureProduct.Feedback.CAMERAS_READY,
                0.10,
            ),
            CaptureProgressStage.TRIGGERING: (
                CaptureProduct.Feedback.TRIGGERING,
                0.20,
            ),
            CaptureProgressStage.WAITING_FRAMES: (
                CaptureProduct.Feedback.WAITING_FRAMES,
                0.55,
            ),
            CaptureProgressStage.VALIDATING_SKEW: (
                CaptureProduct.Feedback.VALIDATING_SKEW,
                0.70,
            ),
            CaptureProgressStage.SAVING_FILES: (
                CaptureProduct.Feedback.SAVING_FILES,
                0.85,
            ),
            CaptureProgressStage.RETRYING: (CaptureProduct.Feedback.RETRYING, 0.0),
        }
        stage, fraction = mapping[progress.stage]
        self._publish_capture_feedback(
            goal_handle,
            stage,
            progress.attempt,
            progress.completed_camera_ids,
            progress.pending_camera_ids,
            progress.frame_batch_id,
            fraction,
            progress.reason,
        )

    @staticmethod
    def _capture_success_values(
        batch: CaptureBatch, inference_job_id: str
    ) -> dict[str, object]:
        return {
            "success": True,
            "product_id": batch.product_id,
            "station_id": batch.station_id,
            "capture_id": batch.capture_id,
            "frame_batch_id": batch.frame_batch_id,
            "attempt_count": batch.attempt,
            "images": batch.images,
            "frame_arrival_skew_us": batch.frame_arrival_skew_us,
            "inference_job_id": inference_job_id,
            "error_code": int(ErrorCode.NONE),
            "reason": "",
        }

    @staticmethod
    def _failure_values(
        request,
        code: ErrorCode | int,
        reason: str,
        *,
        batch: CaptureBatch | None = None,
    ) -> dict[str, object]:
        return {
            "success": False,
            "product_id": request.product_id,
            "station_id": int(request.station_id),
            "capture_id": request.capture_id,
            "frame_batch_id": batch.frame_batch_id if batch else "",
            "attempt_count": batch.attempt if batch else 0,
            "images": batch.images if batch else (),
            "frame_arrival_skew_us": batch.frame_arrival_skew_us if batch else 0,
            "inference_job_id": "",
            "error_code": int(code),
            "reason": reason,
        }

    def _remember_terminal(
        self,
        goal_handle,
        result: CaptureProduct.Result,
        values: dict[str, object],
        *,
        canceled: bool = False,
    ) -> CaptureProduct.Result:
        self._capture_results.remember(
            goal_handle.request.command.command_id,
            goal_handle.request.command.payload_digest,
            values,
        )
        self._apply_capture_values(result, values)
        goal_handle.canceled() if canceled else goal_handle.abort()
        return result

    def _terminal_capture(
        self,
        goal_handle,
        result: CaptureProduct.Result,
        code: ErrorCode | int,
        reason: str,
    ) -> CaptureProduct.Result:
        values = self._failure_values(goal_handle.request, code, reason)
        self._apply_capture_values(result, values)
        goal_handle.abort()
        return result

    @staticmethod
    def _apply_capture_values(
        result: CaptureProduct.Result, values: dict[str, object]
    ) -> None:
        result.success = bool(values["success"])
        result.product_id = str(values["product_id"])
        result.station_id = int(values["station_id"])
        result.capture_id = str(values["capture_id"])
        result.frame_batch_id = str(values["frame_batch_id"])
        result.attempt_count = int(values["attempt_count"])
        result.frame_arrival_skew_us = int(values["frame_arrival_skew_us"])
        result.inference_job_id = str(values["inference_job_id"])
        result.error_code = int(values["error_code"])
        result.reason = str(values["reason"])
        for artifact in values["images"]:
            result.images.append(VisionNode._image_reference(artifact))

    @staticmethod
    def _image_reference(artifact: ImageArtifact) -> ImageReference:
        image = ImageReference()
        image.camera_id = artifact.camera_id
        image.file_path = artifact.file_path
        image.sha256 = artifact.sha256
        image.file_size_bytes = artifact.file_size_bytes
        image.width = artifact.width
        image.height = artifact.height
        image.pixel_format = artifact.pixel_format
        image.camera_timestamp_raw = artifact.camera_timestamp_raw
        image.camera_timestamp_domain = artifact.camera_timestamp_domain
        image.camera_timestamp_ns = artifact.camera_timestamp_ns
        image.camera_timestamp_synchronized = artifact.camera_timestamp_synchronized
        image.host_arrival_monotonic_ns = artifact.host_arrival_monotonic_ns
        image.host_arrival_wall_time.sec = (
            artifact.host_arrival_timestamp_ns // 1_000_000_000
        )
        image.host_arrival_wall_time.nanosec = (
            artifact.host_arrival_timestamp_ns % 1_000_000_000
        )
        return image

    @staticmethod
    def _publish_capture_feedback(
        goal_handle,
        stage: int,
        attempt: int,
        completed_camera_ids: tuple[str, ...],
        pending_camera_ids: tuple[str, ...],
        frame_batch_id: str,
        progress: float,
        reason: str,
    ) -> None:
        feedback = CaptureProduct.Feedback()
        feedback.stage = stage
        feedback.attempt = attempt
        feedback.frame_batch_id = frame_batch_id
        feedback.completed_camera_ids = list(completed_camera_ids)
        feedback.pending_camera_ids = list(pending_camera_ids)
        feedback.progress = min(max(progress, 0.0), 1.0)
        feedback.reason = reason
        goal_handle.publish_feedback(feedback)

    def _handle_runtime_queue_state(self, event: QueueStateEvent) -> None:
        if not self.session_id:
            self._deferred_queue_event = event
            return
        mapping = {
            QueueRuntimeState.ACCEPTING: VisionQueueState.ACCEPTING,
            QueueRuntimeState.ENQUEUE_BLOCKED: VisionQueueState.ENQUEUE_BLOCKED,
            QueueRuntimeState.DRAINING: VisionQueueState.DRAINING,
            QueueRuntimeState.FAULT: VisionQueueState.FAULT,
        }
        state = mapping[event.state]
        if self._last_queue_state == state:
            return
        self._last_queue_state = state
        message = VisionQueueState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = event.blocked_frame_batch_id
        message.state = state
        message.depth = event.depth
        message.capacity = event.capacity
        message.blocked_frame_batch_id = event.blocked_frame_batch_id
        message.reason = event.reason
        self._queue_state_publisher.publish(message)

    def set_health_state(self, state: NodeHealthState) -> None:
        super().set_health_state(state)
        if state != NodeHealthState.READY:
            return
        runtime = getattr(self, "_runtime", None)
        if runtime is not None:
            runtime.activate_workers()
        deferred_telemetry = list(getattr(self, "_deferred_telemetry", ()))
        self._deferred_telemetry.clear()
        for event_type, severity, product_id, payload in deferred_telemetry:
            self._emit_log_event(
                severity=severity,
                event_type=event_type,
                product_id=product_id,
                payload=payload,
            )
        deferred = getattr(self, "_deferred_queue_event", None)
        if deferred is not None and hasattr(self, "_queue_state_publisher"):
            self._deferred_queue_event = None
            self._handle_runtime_queue_state(deferred)

    def _publish_station_result(self, job, inference: StationInference) -> None:
        inference.validate()
        message = StationResult()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = job.inference_job_id
        message.product_id = job.product_id
        message.fifo_sequence = job.fifo_sequence
        message.station_id = job.station_id
        message.capture_id = job.capture_id
        message.frame_batch_id = job.frame_batch_id
        message.inference_job_id = job.inference_job_id
        message.result_revision = job.result_revision
        message.verdict = inference.verdict
        message.score = inference.score
        message.model_version = inference.model_version
        message.completed_at = message.header.stamp
        self._station_result_publisher.publish(message)
        self._emit_log_event(
            severity=LogEvent.INFO if inference.verdict == 1 else LogEvent.WARNING,
            event_type="STATION_INFERENCE_RESULT",
            product_id=job.product_id,
            payload={
                "fifo_sequence": job.fifo_sequence,
                "station_id": job.station_id,
                "capture_id": job.capture_id,
                "frame_batch_id": job.frame_batch_id,
                "inference_job_id": job.inference_job_id,
                "result_revision": job.result_revision,
                "verdict": inference.verdict,
                "score": inference.score,
                "model_version": inference.model_version,
            },
        )

    def _publish_station_failure(self, failure: InferenceFailure) -> None:
        job = failure.job
        message = StationInferenceFailed()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = job.inference_job_id
        message.product_id = job.product_id
        message.fifo_sequence = job.fifo_sequence
        message.station_id = job.station_id
        message.capture_id = job.capture_id
        message.frame_batch_id = job.frame_batch_id
        message.inference_job_id = job.inference_job_id
        message.result_revision = job.result_revision
        message.error_code = failure.error_code
        message.reason = failure.reason
        message.failed_at = message.header.stamp
        self._station_failure_publisher.publish(message)
        self._emit_log_event(
            severity=LogEvent.ERROR,
            event_type="STATION_INFERENCE_FAILED",
            product_id=job.product_id,
            payload={
                "fifo_sequence": job.fifo_sequence,
                "station_id": job.station_id,
                "capture_id": job.capture_id,
                "frame_batch_id": job.frame_batch_id,
                "inference_job_id": job.inference_job_id,
                "result_revision": job.result_revision,
                "error_code": failure.error_code,
                "reason": failure.reason,
            },
        )

    def _handle_product_locked(self, message: ProductResultLocked) -> None:
        if message.header.session_id != self.session_id:
            return
        with self._resource_guard:
            runtime = self._runtime
        if runtime is not None:
            removed = runtime.lock_product(message.product_id)
            if removed:
                self.get_logger().warning(
                    f"removed {removed} queued jobs for locked product {message.product_id}"
                )

    def _emit_capture_telemetry(self, request, batch: CaptureBatch, job_id: str) -> None:
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="CAPTURE_ENQUEUED",
            product_id=request.product_id,
            payload={
                "fifo_sequence": int(request.fifo_sequence),
                "station_id": int(request.station_id),
                "capture_id": request.capture_id,
                "frame_batch_id": batch.frame_batch_id,
                "inference_job_id": job_id,
                "attempt": batch.attempt,
                "manifest_path": batch.manifest_path,
                "frame_arrival_skew_us": batch.frame_arrival_skew_us,
                "trigger_requested_monotonic_ns": batch.trigger_requested_monotonic_ns,
                "trigger_returned_monotonic_ns": batch.trigger_returned_monotonic_ns,
                "trigger_requested_wall_time_ns": batch.trigger_requested_wall_time_ns,
                "trigger_returned_wall_time_ns": batch.trigger_returned_wall_time_ns,
                "images": [
                    {
                        "camera_id": image.camera_id,
                        "file_path": image.file_path,
                        "sha256": image.sha256,
                        "file_size_bytes": image.file_size_bytes,
                        "frame_number": image.frame_number,
                        "external_trigger_count": image.external_trigger_count,
                        "camera_timestamp_raw": image.camera_timestamp_raw,
                        "camera_timestamp_domain": image.camera_timestamp_domain,
                        "camera_timestamp_ns": image.camera_timestamp_ns,
                        "camera_timestamp_synchronized": image.camera_timestamp_synchronized,
                        "sdk_host_timestamp_raw": image.sdk_host_timestamp_raw,
                        "host_arrival_monotonic_ns": image.host_arrival_monotonic_ns,
                        "host_arrival_wall_time_ns": image.host_arrival_timestamp_ns,
                    }
                    for image in batch.images
                ],
            },
        )

    def _runtime_telemetry(
        self,
        event_type: str,
        severity: int,
        product_id: str,
        payload: dict[str, object],
    ) -> None:
        if not self.session_id:
            self._deferred_telemetry.append(
                (event_type, severity, product_id, payload)
            )
            return
        self._emit_log_event(
            severity=severity,
            event_type=event_type,
            product_id=product_id,
            payload=payload,
        )

    def _ensure_log_spool(self) -> None:
        if self._log_spool is not None:
            return
        path = Path(str(self.get_parameter("vision.log.spool_path").value))
        if not path.is_absolute():
            raise ValueError("vision.log.spool_path must be absolute")
        self._log_spool = DurableLogSpool(path)
        self._log_spool_open_failure_reported = False

    def _emit_log_event(
        self,
        *,
        severity: int,
        event_type: str,
        payload: dict[str, object],
        product_id: str = "",
    ) -> None:
        log_id = new_uuid()
        envelope = {
            "schema_version": 2,
            "event_type": event_type,
            "severity": int(severity),
            "source_node": NodeId.VISION.value,
            "producer_instance_id": self.node_instance_id,
            "session_id": self.session_id,
            "product_id": product_id,
            "payload": payload,
        }
        try:
            payload_json = canonical_json(envelope)
            record = SpoolRecord(log_id, 1, payload_json, sha256_text(payload_json))
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Vision LogEvent serialization failed: {type(exc).__name__}"
            )
            return
        if self._log_spool is not None:
            try:
                self._log_spool.enqueue(record)
            except Exception as exc:
                self.set_health_state(NodeHealthState.DEGRADED)
                self.get_logger().error(
                    f"Vision local log spool enqueue failed: {type(exc).__name__}"
                )
        self._publish_spool_record(record)

    def _publish_spool_record(self, record: SpoolRecord) -> None:
        try:
            envelope = json.loads(record.payload_json)
        except (TypeError, ValueError):
            return
        message = LogEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = str(envelope.get("product_id", ""))
        message.log_id = record.log_id
        message.revision = record.revision
        message.severity = int(envelope.get("severity", LogEvent.INFO))
        message.event_type = str(envelope.get("event_type", "UNKNOWN"))
        message.source_node = NodeId.VISION.value
        message.producer_instance_id = self.node_instance_id
        message.product_id = str(envelope.get("product_id", ""))
        message.payload_json = record.payload_json
        message.payload_digest = record.payload_digest
        message.occurred_at = message.header.stamp
        self._log_event_publisher.publish(message)

    def _handle_log_persisted_ack(self, message: LogPersistedAck) -> None:
        if (
            message.header.session_id != self.session_id
            or message.producer_node != NodeId.VISION.value
            or message.producer_instance_id != self.node_instance_id
            or len(message.acked_log_ids) != len(message.acked_revisions)
            or self._log_spool is None
        ):
            return
        identities = [
            (log_id, int(revision))
            for log_id, revision in zip(
                message.acked_log_ids, message.acked_revisions
            )
            if log_id and int(revision) > 0
        ]
        if identities:
            self._log_spool.acknowledge(identities)

    def _flush_log_spool(self) -> None:
        if self._log_spool is None:
            try:
                self._ensure_log_spool()
            except Exception as exc:
                if not self._log_spool_open_failure_reported:
                    self._log_spool_open_failure_reported = True
                    self.get_logger().error(
                        f"Vision log spool unavailable: {type(exc).__name__}"
                    )
                return
        for record in self._log_spool.pending(limit=100):
            self._publish_spool_record(record)

    def _configured_camera_ids(self, station_id: int) -> tuple[str, ...]:
        suffix = "a" if station_id == 1 else "b"
        return tuple(self.get_parameter(f"vision.camera_ids.station_{suffix}").value)

    def _camera_values(self, key: str, station_id: int) -> tuple[str, ...]:
        suffix = "a" if station_id == 1 else "b"
        return tuple(self.get_parameter(f"vision.camera_{key}.station_{suffix}").value)

    def _camera_specs(self) -> tuple[CameraSpec, ...]:
        specs: list[CameraSpec] = []
        for station_id in (1, 2):
            ids = self._configured_camera_ids(station_id)
            serials = self._camera_values("serials", station_id)
            ips = self._camera_values("ips", station_id)
            macs = self._camera_values("macs", station_id)
            if self.profile == "sim":
                serials = serials if len(serials) == len(ids) else ("",) * len(ids)
                ips = ips if len(ips) == len(ids) else ("",) * len(ids)
                macs = macs if len(macs) == len(ids) else ("",) * len(ids)
            if not (len(ids) == len(serials) == len(ips) == len(macs)):
                raise ValueError(f"camera mapping lengths differ for station {station_id}")
            specs.extend(
                CameraSpec(camera_id, station_id, serial, ip, mac)
                for camera_id, serial, ip, mac in zip(ids, serials, ips, macs)
            )
        if len(specs) != 4:
            raise ValueError("Vision requires Station A 3 cameras and Station B 1 camera")
        return tuple(specs)

    def _status_snapshot(self, **extra: object) -> str:
        runtime_status: dict[str, object] = {}
        runtime = getattr(self, "_runtime", None)
        if runtime is not None:
            try:
                runtime_status = runtime.status_snapshot()
            except Exception as exc:
                runtime_status = {"status_error": type(exc).__name__}
        extra["vision_runtime"] = runtime_status
        return super()._status_snapshot(**extra)

    def destroy_node(self) -> None:
        runtime = self._runtime
        self._runtime = None
        if runtime is not None:
            try:
                asyncio.run(runtime.close())
            except Exception as exc:
                self.get_logger().error(
                    f"VisionRuntime shutdown failed: {type(exc).__name__}"
                )
        if self._log_spool is not None:
            self._log_spool.close()
            self._log_spool = None
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    configure_restrictive_umask()
    rclpy.init(args=args)
    spin_node(VisionNode())


if __name__ == "__main__":
    main()
