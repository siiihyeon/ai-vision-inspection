"""VisionNode: GigE Action capture와 파일경로 FIFO 추론의 연결 골격."""

from __future__ import annotations

import ipaddress
import json
import math
import shutil
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import AsyncExitStack
from pathlib import Path

import rclpy
from inspection_common import (
    ErrorCode,
    DurableLogSpool,
    IdempotencyStore,
    NodeId,
    NodeHealthState,
    SpoolRecord,
    SystemState,
    canonical_json,
    new_uuid,
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
    InferenceCancellation,
    InferenceCancellationAck,
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
from rclpy.task import Future

from .capture_contract import (
    CaptureBackend,
    CaptureBatch,
    CapturePacketLossError,
    FakeCaptureBackend,
    MONO8_PNG,
    UnimplementedCaptureBackend,
)
from .inference_queue import (
    InferenceFailure,
    InferenceFailureKind,
    InferenceJob,
    InferenceQueue,
    WorkerPool,
    InferenceTiming,
)
from .gpu_monitor import GpuMonitor
from .model_backend import FakeStationModel


def _fake_load_image(path: Path):
    """진짜 이미지 로드 전, 파이프라인 검증용 placeholder."""

    return path


def _infer_station(model, images: tuple):
    return model.infer(images)


class ExecutorLock:
    """rclpy executor 위에서 동작하는 async 락. `asyncio.Lock`을 대체합니다.

    asyncio.Lock을 쓸 수 없는 이유:
        경합이 없으면 fast path로 즉시 획득하므로 멀쩡히 동작하지만,
        경합이 생기면 대기용 Future를 만들려고 실행 중인 asyncio 이벤트
        루프를 찾습니다. 이 프로젝트는 MultiThreadedExecutor만 쓰고
        asyncio 루프를 띄우지 않으므로 조회 결과가 None이 되고,
        `'NoneType' object has no attribute 'create_future'`로 죽습니다.
        즉 부하가 걸려 촬영이 겹치는 순간에만 터지는 잠복 결함이었습니다.

    동작 방식:
        대기자에게는 rclpy Future를 발급합니다. rclpy Future는 executor가
        직접 깨우므로, await 하는 동안 executor 스레드가 정상 반납되어
        heartbeat 등 다른 콜백이 계속 처리됩니다.
        release()는 `_locked`를 유지한 채 대기열 선두에게 소유권을 그대로
        넘깁니다(직접 인계). 이렇게 하면 깨어난 대기자가 다시 경쟁하지
        않으므로 FIFO 순서가 보장되고 기아 상태가 생기지 않습니다.

    ⚠️ 이 변경으로 생길 수 있는 문제 — 실장비 최초 투입 시 중점 관찰 대상:
      1. **실측 검증 불가.** sim에서는 FakeCaptureBackend가 너무 빨라
         경합 자체가 발생하지 않습니다. 이 클래스의 경합 경로는 단위
         시험으로만 확인했고 실제 라인에서 검증된 적이 없습니다.
         촬영이 겹치기 시작하는 시점(제품 간격이 좁아질 때)에
         촬영 지연·순서 역전이 없는지 반드시 확인하십시오.
      2. **재진입 불가.** 같은 Task가 같은 락을 두 번 잡으면 영구 교착
         입니다. asyncio.Lock과 동일한 제약이므로 기존 호출 구조를
         바꾸지 않는 한 문제없지만, 락 구간 안에서 다른 락을 잡는
         코드를 추가할 때는 반드시 순서를 검토하십시오.
      3. **교착 방지는 호출부에 의존.** 카메라 락은 호출부가
         `sorted(required_camera_ids)`로 항상 같은 순서로 획득하기
         때문에 안전합니다. 이 정렬을 제거하면 교착이 발생합니다.
      4. **대기 중 Task가 폐기되면 락이 반납되지 않습니다.**
         소유권을 넘겨받은 뒤 재개되지 못한 Task가 있으면 그 락은
         영구 점유 상태가 됩니다. 현재는 executor 종료 시에만 가능한
         상황이라 프로세스가 어차피 끝나지만, 향후 Task를 직접
         cancel하는 코드를 넣는다면 이 경로를 다시 검토해야 합니다.
      5. **타임아웃이 없습니다.** asyncio.Lock과 마찬가지로 무한 대기
         합니다. 촬영이 멈추면 뒤따르는 요청이 조용히 쌓입니다.
         Master의 capture_timeout_ms가 상위에서 이를 끊어 줍니다.
    """

    def __init__(self) -> None:
        # _guard는 짧게만 잡습니다. 이 안에서 블로킹 호출을 하면 안 됩니다.
        self._guard = threading.Lock()
        self._locked = False
        self._waiters: deque[Future] = deque()

    async def __aenter__(self) -> ExecutorLock:
        with self._guard:
            if not self._locked:
                self._locked = True
                return self
            waiter: Future = Future()
            self._waiters.append(waiter)
        # 깨어난 시점에는 release()가 이미 소유권을 넘겨준 상태입니다.
        await waiter
        return self

    async def __aexit__(self, exc_type, exc, traceback) -> bool:
        self.release()
        return False

    def release(self) -> None:
        successor: Future | None = None
        with self._guard:
            while self._waiters:
                candidate = self._waiters.popleft()
                if not candidate.cancelled():
                    successor = candidate
                    break
            if successor is None:
                self._locked = False
        # set_result는 executor를 깨우므로 _guard 밖에서 호출합니다.
        if successor is not None:
            successor.set_result(True)


class VisionNode(InspectionNodeBase):
    """Capture Action 성공을 FrameBatch enqueue 완료 시점으로 정의합니다."""

    def __init__(self, capture_backend: CaptureBackend | None = None) -> None:
        super().__init__(NodeId.VISION, provides_initialize_action=True)
        self.declare_parameter("vision.trigger.mode", "GIGE_ACTION_COMMAND")
        self.declare_parameter(
            "vision.camera_ids.station_a", Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter(
            "vision.camera_ids.station_b", Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter("vision.gige_action.device_key", 0)
        self.declare_parameter("vision.gige_action.group_key", 0)
        self.declare_parameter("vision.gige_action.group_mask", 0)
        self.declare_parameter("vision.camera_network_map_json", "{}")
        self.declare_parameter("vision.frame_arrival_skew_limit_us", 0)
        self.declare_parameter("vision.capture.acquisition_timeout_ms", 0)
        self.declare_parameter("vision.capture.max_attempts", 2)
        self.declare_parameter("vision.gige.packet_size", 1500)
        self.declare_parameter("vision.gige.packet_delay_ticks", 5000)
        self.declare_parameter("vision.ptp.enabled", False)
        self.declare_parameter("vision.ptp.validation_completed", False)
        self.declare_parameter("vision.camera.disconnect_pause_after_ms", 5000)
        self.declare_parameter("vision.camera.reconnect_interval_ms", 1000)
        self.declare_parameter("vision.camera.reconnect_max_attempts", 5)
        self.declare_parameter("vision.queue.capacity", 16)
        self.declare_parameter("vision.worker_count", 1)
        self.declare_parameter("vision.model.serialize_access", True)
        self.declare_parameter("vision.inference_queue_total_timeout_ms", 0)
        self.declare_parameter("vision.inference.station_a.total_timeout_ms", 0)
        self.declare_parameter("vision.inference.station_b.total_timeout_ms", 0)
        self.declare_parameter("vision.data_root", "")
        self.declare_parameter("vision.model.path", "")
        self.declare_parameter("vision.model.version", "Model_v_1")
        self.declare_parameter("vision.model.sha256", "")
        self.declare_parameter("vision.model.runtime", "PYTORCH_TORCHSCRIPT")
        self.declare_parameter("vision.model.cuda_required", True)
        self.declare_parameter("vision.model.warmup_runs", 10)
        self.declare_parameter("vision.image.sensor_width", 2248)
        self.declare_parameter("vision.image.sensor_height", 2048)
        self.declare_parameter("vision.image.canonical_pixel_format", MONO8_PNG)
        self.declare_parameter(
            "vision.result_spool_path", "/tmp/inspection/spool/vision.sqlite3"
        )
        self.declare_parameter("vision.shutdown.queue_drain_timeout_ms", 3000)
        self.declare_parameter("vision.gpu.sample_interval_ms", 200)
        self.declare_parameter("vision.disk.warning_ratio", 0.90)
        self.declare_parameter("vision.disk.stop_ratio", 0.95)
        self.declare_parameter("vision.timeout_tuning.auto_apply", False)
        self.declare_parameter(
            "vision.timeout_tuning.generated_path",
            "/var/lib/inspection/config/vision_timeout_tuning.json",
        )
        self.declare_parameter("vision.timeout_tuning.minimum_samples", 10000)
        self.declare_parameter("vision.timeout_tuning.safety_factor", 1.2)

        self._apply_generated_timeout_tuning()

        if str(self.get_parameter("vision.trigger.mode").value) != "GIGE_ACTION_COMMAND":
            raise ValueError("vision.trigger.mode must be GIGE_ACTION_COMMAND")
        if int(self.get_parameter("vision.capture.max_attempts").value) != 2:
            raise ValueError("vision.capture.max_attempts is fixed at 2")
        canonical_pixel_format = str(
            self.get_parameter("vision.image.canonical_pixel_format").value
        )
        if canonical_pixel_format != MONO8_PNG:
            raise ValueError("vision.image.canonical_pixel_format must be MONO8_PNG")

        capacity = int(self.get_parameter("vision.queue.capacity").value)
        self.inference_queue = InferenceQueue(max(capacity, 1))
        self.worker_pool: WorkerPool | None = None
        self._worker_stopped = False
        self._session_metrics_emitted = False
        self._disk_warning_active = False
        self._shutdown_requested = False
        self._shutdown_ready = False
        self._state_guard = threading.RLock()
        self._canceled_station_scopes: set[tuple[str, int | None]] = set()
        self._inference_timings: dict[str, InferenceTiming] = {}
        self._result_spool: DurableLogSpool | None = None
        self._result_spool_path = str(
            self.get_parameter("vision.result_spool_path").value
        )
        self._gpu_monitor = GpuMonitor(
            interval_seconds=max(
                int(self.get_parameter("vision.gpu.sample_interval_ms").value), 1
            )
            / 1000.0
        )
        # 블로킹 호출을 executor 스레드 밖으로 넘기기 위한 전용 풀입니다.
        self._blocking_pool = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="vision-blocking"
        )
        self.capture_backend = capture_backend or (
            FakeCaptureBackend(
                data_root=Path(
                    str(self.get_parameter("vision.data_root").value)
                    or "/tmp/inspection/vision_fake"
                )
            )
            if self.profile == "sim"
            else UnimplementedCaptureBackend()
        )
        self._capture_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._capture_identities: dict[str, tuple[str, int]] = {}
        self._completed_captures: dict[
            tuple[str, int, str], dict[str, object]
        ] = {}
        self._station_locks = {1: ExecutorLock(), 2: ExecutorLock()}
        configured_camera_ids = set(
            self.get_parameter("vision.camera_ids.station_a").value
        ) | set(self.get_parameter("vision.camera_ids.station_b").value)
        self._camera_locks = {
            camera_id: ExecutorLock() for camera_id in configured_camera_ids
        }
        self._capture_action_group = ReentrantCallbackGroup()
        self._capture_server = ActionServer(
            self,
            CaptureProduct,
            "vision/capture_product",
            execute_callback=self._execute_capture,
            goal_callback=self._accept_capture_goal,
            cancel_callback=self._cancel_capture_goal,
            callback_group=self._capture_action_group,
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
        self._cancellation_subscription = self.create_subscription(
            InferenceCancellation,
            "/inspection/master/inference_cancellation",
            self._handle_inference_cancellation,
            reliable_event_qos(),
        )
        self._cancellation_ack_publisher = self.create_publisher(
            InferenceCancellationAck,
            "vision/inference_cancellation_ack",
            reliable_event_qos(),
        )
        self._log_event_publisher = self.create_publisher(
            LogEvent, "log/event", reliable_event_qos()
        )
        self._log_ack_subscription = self.create_subscription(
            LogPersistedAck,
            "/inspection/log/persisted_ack",
            self._handle_log_persisted_ack,
            reliable_event_qos(),
        )
        self._result_spool_timer = self.create_timer(1.0, self._flush_result_spool)
        self._gpu_snapshot_timer = self.create_timer(
            10.0, self._emit_gpu_metrics_snapshot
        )
        self._locked_subscription = self.create_subscription(
            ProductResultLocked,
            "/inspection/master/product_result_locked",
            self._handle_product_locked,
            reliable_event_qos(),
        )
        self._publish_queue_state(VisionQueueState.ACCEPTING, "")
        self.get_logger().info("VisionNode v2 communication skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        return (
            "vision.camera_ids.station_a",
            "vision.camera_ids.station_b",
            "vision.gige_action.device_key",
            "vision.gige_action.group_key",
            "vision.gige_action.group_mask",
            "vision.camera_network_map_json",
            "vision.frame_arrival_skew_limit_us",
            "vision.capture.acquisition_timeout_ms",
            "vision.gige.packet_delay_ticks",
            "vision.ptp.validation_completed",
            "vision.queue.capacity",
            "vision.worker_count",
            "vision.inference.station_a.total_timeout_ms",
            "vision.inference.station_b.total_timeout_ms",
            "vision.data_root",
            "vision.model.path",
            "vision.model.version",
            "vision.model.sha256",
            "vision.result_spool_path",
            "vision.image.canonical_pixel_format",
        )

    def validate_hardware_profile(self) -> list[str]:
        missing = super().validate_hardware_profile()
        if self.profile != "hardware":
            return missing
        positive_keys = (
            "vision.frame_arrival_skew_limit_us",
            "vision.queue.capacity",
            "vision.worker_count",
            "vision.inference.station_a.total_timeout_ms",
            "vision.inference.station_b.total_timeout_ms",
            "vision.capture.acquisition_timeout_ms",
            "vision.gige.packet_delay_ticks",
        )
        for key in positive_keys:
            if self.has_parameter(key) and int(self.get_parameter(key).value) <= 0:
                missing.append(key)
        station_a = tuple(self.get_parameter("vision.camera_ids.station_a").value)
        station_b = tuple(self.get_parameter("vision.camera_ids.station_b").value)
        if len(station_a) != 3:
            missing.append("vision.camera_ids.station_a must contain 3 cameras")
        if len(station_b) != 1:
            missing.append("vision.camera_ids.station_b must contain 1 camera")
        if set(station_a) & set(station_b):
            missing.append("station camera sets must be disjoint")
        try:
            network_map = json.loads(
                str(self.get_parameter("vision.camera_network_map_json").value)
            )
            if not isinstance(network_map, dict) or set(network_map) != set(
                station_a + station_b
            ):
                missing.append("vision.camera_network_map_json camera set mismatch")
            else:
                for address in network_map.values():
                    ipaddress.ip_address(str(address))
        except (TypeError, ValueError):
            missing.append("vision.camera_network_map_json is invalid")
        data_root = str(self.get_parameter("vision.data_root").value)
        if data_root and not Path(data_root).is_absolute():
            missing.append("vision.data_root must be absolute")
        spool_path = str(self.get_parameter("vision.result_spool_path").value)
        if spool_path and not Path(spool_path).is_absolute():
            missing.append("vision.result_spool_path must be absolute")
        if int(self.get_parameter("vision.image.sensor_width").value) != 2248:
            missing.append("vision.image.sensor_width must be 2248")
        if int(self.get_parameter("vision.image.sensor_height").value) != 2048:
            missing.append("vision.image.sensor_height must be 2048")
        if int(self.get_parameter("vision.gige.packet_size").value) != 1500:
            missing.append("vision.gige.packet_size must remain 1500")
        if not bool(self.get_parameter("vision.ptp.validation_completed").value):
            missing.append("vision.ptp.validation_completed must be confirmed")
        warning_ratio = float(self.get_parameter("vision.disk.warning_ratio").value)
        stop_ratio = float(self.get_parameter("vision.disk.stop_ratio").value)
        if warning_ratio != 0.90 or stop_ratio != 0.95:
            missing.append("vision disk ratios must remain warning=0.90/stop=0.95")
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        try:
            replacement_spool = DurableLogSpool(Path(self._result_spool_path))
        except Exception as exc:
            return NodeInitializationOutcome(
                success=False,
                error_code=int(ErrorCode.LOG_COMMIT_FAILED),
                reason=f"Vision result spool initialization failed: {type(exc).__name__}",
                retryable=True,
            )
        previous_spool = self._result_spool
        self._result_spool = replacement_spool
        if previous_spool is not None:
            previous_spool.close()
        if self.profile == "hardware":
            return NodeInitializationOutcome(
                success=False,
                error_code=int(ErrorCode.IMPLEMENTATION_PENDING),
                reason=(
                    "MVS Action1 adapter and TorchScript preprocessing/output "
                    "decoder are awaiting hardware/model injection"
                ),
                retryable=True,
            )
        if self.worker_pool is None:
            worker_count = int(self.get_parameter("vision.worker_count").value)
            self.worker_pool = WorkerPool(
                queue=self.inference_queue,
                model=FakeStationModel(),
                worker_count=max(worker_count, 1),
                load_image=_fake_load_image,
                infer=_infer_station,
                on_success=self._on_inference_success,
                on_failure=self._on_inference_failure,
                on_timing=self._on_inference_timing,
                on_canceled=self._on_inference_canceled,
                serialize_model_access=bool(
                    self.get_parameter("vision.model.serialize_access").value
                ),
            )
            self.worker_pool.start()
        self._gpu_monitor.start()
        return NodeInitializationOutcome(
            success=True,
            reason="sim Mono8 capture and batch model skeleton initialized",
        )

    def _run_blocking(self, fn, *args) -> Future:
        """블로킹 호출을 스레드 풀에 넘기고 rclpy Future로 결과를 받습니다.

        rclpy executor에는 asyncio 이벤트 루프가 없어 asyncio.to_thread를
        쓸 수 없습니다. rclpy Future는 executor가 직접 깨우므로 await 시
        executor 스레드가 정상 반납됩니다.
        """

        rclpy_future = Future()
        pool_future = self._blocking_pool.submit(fn, *args)

        def _relay(done_future) -> None:
            try:
                rclpy_future.set_result(done_future.result())
            except Exception as exc:  # 호출부 await에서 다시 발생시킵니다.
                rclpy_future.set_exception(exc)

        pool_future.add_done_callback(_relay)
        return rclpy_future

    def destroy_node(self) -> bool:
        """종료 시 워커와 블로킹 스레드 풀을 정리합니다.

        순서가 중요합니다. 추론 워커는 결과를 ROS 퍼블리셔로 내보내므로,
        super()가 퍼블리셔를 파괴하기 전에 워커를 먼저 멈춰야 합니다.
        (worker_pool.stop()은 워커당 최대 5초 join하므로 실제 모델이
        추론 중이면 종료가 그만큼 지연될 수 있습니다.)
        """

        self._stop_workers_and_flush()
        self._blocking_pool.shutdown(wait=False, cancel_futures=True)
        if self._result_spool is not None:
            self._result_spool.close()
            self._result_spool = None
        return super().destroy_node()

    def request_shutdown(self, reason: str = "program termination") -> bool:
        if self._shutdown_requested:
            return False
        self._shutdown_requested = True
        self._stop_workers_and_flush(reason)
        self._shutdown_ready = True
        return True

    @property
    def shutdown_ready(self) -> bool:
        return self._shutdown_ready

    def _stop_workers_and_flush(self, reason: str = "node destroy") -> None:
        if not self._worker_stopped and self.worker_pool is not None:
            timeout_ms = int(
                self.get_parameter("vision.shutdown.queue_drain_timeout_ms").value
            )
            self.worker_pool.stop(soft_timeout_seconds=max(timeout_ms, 0) / 1000.0)
            if self.worker_pool.soft_shutdown_timeout_exceeded:
                self.get_logger().critical(
                    "active forward exceeded the 3 second soft shutdown timeout"
                )
            self._worker_stopped = True
        snapshot = self._gpu_monitor.stop()
        if self.session_id and not self._session_metrics_emitted:
            self._session_metrics_emitted = True
            self._emit_durable_event(
                "VISION_SESSION_METRICS",
                {
                    "shutdown_reason": reason,
                    "normal_shutdown": True,
                    "gpu_available": snapshot.available,
                    "gpu_sample_count": snapshot.sample_count,
                    "mean_gpu_utilization_pct": snapshot.mean_utilization_pct,
                    "mean_vram_used_mib": snapshot.mean_vram_used_mib,
                    "peak_vram_used_mib": snapshot.peak_vram_used_mib,
                    "total_vram_mib": snapshot.total_vram_mib,
                },
            )
            self._flush_result_spool()

    def _on_inference_success(self, job: InferenceJob, result) -> None:
        """추론 성공 결과를 StationResult로 포장해 발행합니다."""

        expected_views = 3 if job.station_id == 1 else 1
        view_verdicts = tuple(int(value) for value in result.view_verdicts)
        derived_verdict = (
            int(StationResult.NG)
            if int(StationResult.NG) in view_verdicts
            else int(StationResult.PASS)
        )
        if (
            len(job.image_paths) != expected_views
            or len(view_verdicts) != expected_views
            or len(result.view_scores) != expected_views
            or any(
                verdict not in {int(StationResult.PASS), int(StationResult.NG)}
                for verdict in view_verdicts
            )
            or int(result.verdict) != derived_verdict
            or not math.isfinite(float(result.score))
            or any(not math.isfinite(float(score)) for score in result.view_scores)
        ):
            self._on_inference_failure(
                job,
                InferenceFailure(
                    kind=InferenceFailureKind.MODEL,
                    reason="model output contract is invalid",
                ),
            )
            return

        message = StationResult()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = job.capture_id
        message.product_id = job.product_id
        message.fifo_sequence = job.fifo_sequence
        message.station_id = job.station_id
        message.capture_id = job.capture_id
        message.frame_batch_id = job.frame_batch_id
        message.inference_job_id = job.inference_job_id
        message.result_revision = 1
        message.verdict = int(result.verdict)
        message.score = result.score
        message.model_version = result.model_version
        message.completed_at = message.header.stamp
        timing = self._inference_timings.pop(job.inference_job_id, None)
        self._emit_durable_event(
            "VISION_STATION_RESULT_DURABLE",
            {
                "product_id": job.product_id,
                "fifo_sequence": job.fifo_sequence,
                "station_id": job.station_id,
                "capture_id": job.capture_id,
                "frame_batch_id": job.frame_batch_id,
                "inference_job_id": job.inference_job_id,
                "result_revision": 1,
                "verdict": int(result.verdict),
                "score": float(result.score),
                "model_version": result.model_version,
                "model_sha256": result.model_sha256,
                "config_fingerprint": self._timeout_tuning_fingerprint(),
                "view_verdicts": list(result.view_verdicts),
                "view_scores": list(result.view_scores),
                "camera_ids": list(job.camera_ids),
                "image_paths": list(job.image_paths),
                "capture_completed_monotonic_ns": job.capture_completed_monotonic_ns,
                "completed_monotonic_ns": (
                    timing.completed_monotonic_ns if timing else time.monotonic_ns()
                ),
                "model_forward_ms": timing.model_forward_ms if timing else None,
                "enqueue_to_result_ms": (
                    timing.enqueue_to_terminal_ms if timing else None
                ),
            },
            product_id=job.product_id,
        )
        self._station_result_publisher.publish(message)

    def _emit_gpu_metrics_snapshot(self) -> None:
        snapshot = self._gpu_monitor.snapshot()
        if not self.session_id or snapshot.sample_count < 1:
            return
        self._emit_durable_event(
            "VISION_GPU_METRICS_SNAPSHOT",
            {
                "normal_shutdown": False,
                "gpu_sample_count": snapshot.sample_count,
                "mean_gpu_utilization_pct": snapshot.mean_utilization_pct,
                "mean_vram_used_mib": snapshot.mean_vram_used_mib,
                "peak_vram_used_mib": snapshot.peak_vram_used_mib,
                "total_vram_mib": snapshot.total_vram_mib,
            },
        )

    def _on_inference_timing(
        self, job: InferenceJob, timing: InferenceTiming
    ) -> None:
        self._inference_timings[job.inference_job_id] = timing

    def _on_inference_canceled(self, job: InferenceJob, stage: str) -> None:
        self._inference_timings.pop(job.inference_job_id, None)
        self._emit_durable_event(
            "VISION_INFERENCE_CANCELED",
            {
                "product_id": job.product_id,
                "fifo_sequence": job.fifo_sequence,
                "station_id": job.station_id,
                "capture_id": job.capture_id,
                "frame_batch_id": job.frame_batch_id,
                "inference_job_id": job.inference_job_id,
                "stage": stage,
                "image_paths": list(job.image_paths),
            },
            product_id=job.product_id,
        )

    def _on_inference_failure(
        self, job: InferenceJob, failure: InferenceFailure
    ) -> None:
        """추론 실패를 StationInferenceFailed로 포장해 발행합니다."""

        message = StationInferenceFailed()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = job.capture_id
        message.product_id = job.product_id
        message.fifo_sequence = job.fifo_sequence
        message.station_id = job.station_id
        message.capture_id = job.capture_id
        message.frame_batch_id = job.frame_batch_id
        message.inference_job_id = job.inference_job_id
        message.result_revision = 1
        error_codes = {
            InferenceFailureKind.TIMEOUT: ErrorCode.INFERENCE_TIMEOUT,
            InferenceFailureKind.FILE_READ: ErrorCode.INFERENCE_FILE_READ_FAILED,
            InferenceFailureKind.MODEL: ErrorCode.INFERENCE_FAILED,
            InferenceFailureKind.CUDA_OOM: ErrorCode.GPU_OUT_OF_MEMORY,
        }
        message.error_code = int(error_codes[failure.kind])
        message.reason = failure.reason
        message.failed_at = message.header.stamp
        self._inference_timings.pop(job.inference_job_id, None)
        self._emit_durable_event(
            "VISION_STATION_FAILURE_DURABLE",
            {
                "product_id": job.product_id,
                "fifo_sequence": job.fifo_sequence,
                "station_id": job.station_id,
                "capture_id": job.capture_id,
                "frame_batch_id": job.frame_batch_id,
                "inference_job_id": job.inference_job_id,
                "result_revision": 1,
                "error_code": int(message.error_code),
                "failure_kind": failure.kind.value,
                "reason": failure.reason,
                "camera_ids": list(job.camera_ids),
                "image_paths": list(job.image_paths),
            },
            product_id=job.product_id,
            severity=LogEvent.ERROR,
        )
        self._station_failure_publisher.publish(message)
        if failure.kind == InferenceFailureKind.CUDA_OOM:
            self.set_health_state(NodeHealthState.DEGRADED)

    def _accept_capture_goal(self, goal_request) -> GoalResponse:
        cameras = tuple(goal_request.required_camera_ids)
        parameter = (
            "vision.camera_ids.station_a"
            if goal_request.station_id == 1
            else "vision.camera_ids.station_b"
        )
        configured = (
            tuple(self.get_parameter(parameter).value)
            if goal_request.station_id in self._station_locks
            else ()
        )
        valid = (
            not self._shutdown_requested
            and
            bool(goal_request.product_id)
            and bool(goal_request.capture_id)
            and goal_request.station_id in self._station_locks
            and bool(cameras)
            and len(cameras) == len(set(cameras))
            and cameras == configured
            and bool(goal_request.command.command_id)
            and not self._is_scope_canceled(
                goal_request.product_id, int(goal_request.station_id)
            )
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
            "validating capture identity and command envelope",
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

        disk_stop_reason = self._check_disk_capacity()
        if disk_stop_reason:
            return self._terminal_capture(
                goal_handle,
                result,
                ErrorCode.VISION_DISK_STOP,
                disk_stop_reason,
            )

        async with self._station_locks[request.station_id]:
            if self._is_scope_canceled(request.product_id, request.station_id):
                return self._terminal_capture(
                    goal_handle,
                    result,
                    ErrorCode.CAPTURE_CANCELED,
                    "capture scope was canceled before camera reservation",
                    canceled=True,
                )
            identity = (request.product_id, request.station_id)
            previous_identity = self._capture_identities.get(request.capture_id)
            if previous_identity is not None and previous_identity != identity:
                return self._terminal_capture(
                    goal_handle,
                    result,
                    ErrorCode.COMMAND_CONFLICT,
                    "capture_id already belongs to another product/station",
                )
            self._capture_identities[request.capture_id] = identity
            capture_key = (request.product_id, request.station_id, request.capture_id)
            completed = self._completed_captures.get(capture_key)
            if completed is not None:
                self._capture_results.remember(
                    request.command.command_id,
                    request.command.payload_digest,
                    completed,
                )
                self._apply_capture_values(result, completed)
                goal_handle.succeed()
                return result
            async with AsyncExitStack() as camera_stack:
                for camera_id in sorted(request.required_camera_ids):
                    camera_lock = self._camera_locks.get(camera_id)
                    if camera_lock is None:
                        return self._terminal_capture(
                            goal_handle,
                            result,
                            ErrorCode.COMMAND_CONFLICT,
                            f"camera is not configured: {camera_id}",
                        )
                    await camera_stack.enter_async_context(camera_lock)
                batch, capture_error, capture_reason = await self._capture_required_batch(
                    goal_handle
                )
            if batch is None:
                if goal_handle.is_cancel_requested:
                    return self._terminal_capture(
                        goal_handle,
                        result,
                        ErrorCode.CAPTURE_CANCELED,
                        "capture canceled",
                        canceled=True,
                    )
                return self._terminal_capture(
                    goal_handle,
                    result,
                    capture_error,
                    capture_reason,
                )

            if (
                goal_handle.is_cancel_requested
                or self._is_scope_canceled(request.product_id, request.station_id)
            ):
                self._emit_capture_discarded(
                    request.product_id,
                    request.fifo_sequence,
                    request.station_id,
                    request.capture_id,
                    batch,
                    "capture completed after cancellation; canonical files retained for LogNode",
                )
                return self._terminal_capture(
                    goal_handle,
                    result,
                    ErrorCode.CAPTURE_CANCELED,
                    "capture completed after its inference scope was canceled",
                    canceled=True,
                )

            inference_job_id = new_uuid()
            station_timeout_parameter = (
                "vision.inference.station_a.total_timeout_ms"
                if request.station_id == 1
                else "vision.inference.station_b.total_timeout_ms"
            )
            timeout_ms = int(self.get_parameter(station_timeout_parameter).value)
            if timeout_ms <= 0:
                timeout_ms = int(
                    self.get_parameter(
                        "vision.inference_queue_total_timeout_ms"
                    ).value
                )
            job = InferenceJob(
                inference_job_id=inference_job_id,
                product_id=request.product_id,
                fifo_sequence=request.fifo_sequence,
                station_id=request.station_id,
                capture_id=request.capture_id,
                frame_batch_id=batch.frame_batch_id,
                image_paths=tuple(image.file_path for image in batch.images),
                # 실제 Queue 등록 성공 시 InferenceQueue가 monotonic 시각을 찍습니다.
                enqueued_monotonic_ns=0,
                queue_total_timeout_ms=timeout_ms if timeout_ms > 0 else None,
                camera_ids=tuple(image.camera_id for image in batch.images),
                capture_completed_monotonic_ns=max(
                    image.host_arrival_monotonic_ns for image in batch.images
                ),
            )
            self._publish_capture_feedback(
                goal_handle,
                CaptureProduct.Feedback.ENQUEUEING_INFERENCE,
                batch.attempt,
                tuple(image.camera_id for image in batch.images),
                (),
                batch.frame_batch_id,
                0.95,
                "enqueueing path-only FrameBatch inference job",
            )
            while not self.inference_queue.try_enqueue(job):
                if self.inference_queue.closed:
                    return self._terminal_capture(
                        goal_handle,
                        result,
                        ErrorCode.INFERENCE_FAILED,
                        "inference queue is closed",
                    )
                if self.inference_queue.is_product_locked(request.product_id):
                    return self._terminal_capture(
                        goal_handle,
                        result,
                        ErrorCode.CAPTURE_CANCELED,
                        "product was locked before FrameBatch enqueue",
                    )
                if self._is_scope_canceled(request.product_id, request.station_id):
                    self._emit_capture_discarded(
                        request.product_id,
                        request.fifo_sequence,
                        request.station_id,
                        request.capture_id,
                        batch,
                        "inference scope canceled while waiting for queue capacity",
                    )
                    return self._terminal_capture(
                        goal_handle,
                        result,
                        ErrorCode.CAPTURE_CANCELED,
                        "inference scope canceled before enqueue",
                        canceled=True,
                    )
                if goal_handle.is_cancel_requested:
                    return self._terminal_capture(
                        goal_handle,
                        result,
                        ErrorCode.CAPTURE_CANCELED,
                        "ENQUEUE_BLOCKED capture canceled",
                        canceled=True,
                    )
                self._publish_capture_feedback(
                    goal_handle,
                    CaptureProduct.Feedback.ENQUEUE_BLOCKED,
                    batch.attempt,
                    tuple(image.camera_id for image in batch.images),
                    (),
                    batch.frame_batch_id,
                    0.95,
                    "queue full; preserving saved FrameBatch",
                )
                self._publish_queue_state(
                    VisionQueueState.ENQUEUE_BLOCKED,
                    "queue full",
                    batch.frame_batch_id,
                )
                await self._run_blocking(self.inference_queue.wait_for_space, 0.1)

            self._publish_queue_state(VisionQueueState.ACCEPTING, "")
            values = self._capture_success_values(batch, inference_job_id)
            self._completed_captures[capture_key] = values
            self._capture_results.remember(
                request.command.command_id, request.command.payload_digest, values
            )
            self._apply_capture_values(result, values)
            goal_handle.succeed()
            return result

    async def _capture_required_batch(
        self, goal_handle
    ) -> tuple[CaptureBatch | None, ErrorCode, str]:
        request = goal_handle.request
        required = tuple(request.required_camera_ids)
        max_attempts = int(self.get_parameter("vision.capture.max_attempts").value)
        skew_limit = int(
            self.get_parameter("vision.frame_arrival_skew_limit_us").value
        )
        last_error = ErrorCode.CAPTURE_FAILED
        last_reason = "capture did not start"
        for attempt in range(1, max_attempts + 1):
            attempt_error = ErrorCode.CAPTURE_FAILED
            if goal_handle.is_cancel_requested:
                return None, ErrorCode.CAPTURE_CANCELED, "capture canceled"
            self._publish_capture_feedback(
                goal_handle,
                CaptureProduct.Feedback.CAMERAS_READY,
                attempt,
                (),
                required,
                "",
                0.1,
                "required cameras reserved and ready",
            )
            self._publish_capture_feedback(
                goal_handle,
                CaptureProduct.Feedback.TRIGGERING,
                attempt,
                (),
                required,
                "",
                0.2,
                "broadcasting GigE Vision Action Command",
            )
            try:
                capture_started_ns = time.monotonic_ns()
                batch = await self.capture_backend.capture_station(
                    product_id=request.product_id,
                    station_id=request.station_id,
                    capture_id=request.capture_id,
                    attempt=attempt,
                    required_camera_ids=required,
                )
                capture_completed_ns = time.monotonic_ns()
                if (
                    batch.product_id != request.product_id
                    or batch.station_id != request.station_id
                    or batch.capture_id != request.capture_id
                    or batch.attempt != attempt
                ):
                    raise ValueError("capture backend returned mismatched identity")
                self._publish_capture_feedback(
                    goal_handle,
                    CaptureProduct.Feedback.VALIDATING_SKEW,
                    attempt,
                    tuple(image.camera_id for image in batch.images),
                    tuple(
                        camera_id
                        for camera_id in required
                        if camera_id not in {image.camera_id for image in batch.images}
                    ),
                    batch.frame_batch_id,
                    0.75,
                    "validating host-arrival skew and saved canonical PNG files",
                )
                # rclpy executor에는 asyncio 이벤트 루프가 없어 to_thread를 쓸 수 없습니다.
                validation_started_ns = time.monotonic_ns()
                batch.validate(
                    required,
                    expected_pixel_format=str(
                        self.get_parameter(
                            "vision.image.canonical_pixel_format"
                        ).value
                    ),
                )
                if self.profile == "hardware" and any(
                    image.width
                    != int(self.get_parameter("vision.image.sensor_width").value)
                    or image.height
                    != int(self.get_parameter("vision.image.sensor_height").value)
                    for image in batch.images
                ):
                    raise ValueError("canonical image resolution is not 2248x2048")
                if skew_limit > 0 and batch.frame_arrival_skew_us > skew_limit:
                    attempt_error = ErrorCode.CAPTURE_SKEW_EXCEEDED
                    raise ValueError("frame_arrival_skew_us exceeded configured limit")
                validation_completed_ns = time.monotonic_ns()
                backend_ms = (capture_completed_ns - capture_started_ns) / 1_000_000
                validation_ms = (
                    validation_completed_ns - validation_started_ns
                ) / 1_000_000
                acquisition_ms = max(
                    0.0,
                    (
                        max(
                            image.host_arrival_monotonic_ns
                            for image in batch.images
                        )
                        - batch.trigger_requested_monotonic_ns
                    )
                    / 1_000_000,
                )
                save_overhead_ms = max(0.0, backend_ms - acquisition_ms)
                self._emit_durable_event(
                    "VISION_CAPTURE_TIMING",
                    {
                        "product_id": request.product_id,
                        "fifo_sequence": request.fifo_sequence,
                        "station_id": request.station_id,
                        "capture_id": request.capture_id,
                        "frame_batch_id": batch.frame_batch_id,
                        "attempt": attempt,
                        "trigger_round_trip_ms": (
                            batch.trigger_returned_monotonic_ns
                            - batch.trigger_requested_monotonic_ns
                        )
                        / 1_000_000,
                        "capture_and_save_ms": backend_ms,
                        "capture_acquisition_ms": acquisition_ms,
                        "save_overhead_ms": save_overhead_ms,
                        "validation_ms": validation_ms,
                        "capture_timeout_candidate_ms": acquisition_ms * 2.0
                        + save_overhead_ms
                        + validation_ms,
                        "packet_loss_by_camera": {
                            image.camera_id: image.packet_loss_count
                            for image in batch.images
                        },
                        "packet_resend_by_camera": {
                            image.camera_id: image.packet_resend_count
                            for image in batch.images
                        },
                    },
                    product_id=request.product_id,
                )
                return batch, ErrorCode.NONE, ""
            except Exception as exc:
                if attempt_error != ErrorCode.CAPTURE_SKEW_EXCEEDED:
                    attempt_error = (
                        ErrorCode.IMPLEMENTATION_PENDING
                        if isinstance(exc, NotImplementedError)
                        else ErrorCode.CAPTURE_FAILED
                        if isinstance(exc, CapturePacketLossError)
                        else ErrorCode.CAPTURE_SAVE_FAILED
                        if isinstance(exc, ValueError)
                        else ErrorCode.CAPTURE_FAILED
                    )
                last_error = attempt_error
                last_reason = f"attempt {attempt}/{max_attempts} failed: {exc}"
                self.get_logger().error(
                    f"capture_id={request.capture_id} attempt={attempt} failed: {exc}"
                )
                # 같은 capture_id로 station 필수 카메라 전체를 다음 attempt에 재촬영합니다.
                if attempt < max_attempts:
                    self._publish_capture_feedback(
                        goal_handle,
                        CaptureProduct.Feedback.RETRYING,
                        attempt,
                        (),
                        required,
                        "",
                        0.0,
                        "retrying all required cameras with the same capture_id",
                    )
        return None, last_error, last_reason

    def _capture_success_values(
        self, batch: CaptureBatch, inference_job_id: str
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

    def _terminal_capture(
        self,
        goal_handle,
        result,
        code: ErrorCode,
        reason: str,
        *,
        canceled: bool = False,
    ) -> CaptureProduct.Result:
        result.success = False
        result.product_id = goal_handle.request.product_id
        result.station_id = goal_handle.request.station_id
        result.capture_id = goal_handle.request.capture_id
        result.error_code = int(code)
        result.reason = reason
        goal_handle.canceled() if canceled else goal_handle.abort()
        return result

    @staticmethod
    def _apply_capture_values(result, values: dict[str, object]) -> None:
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
            result.images.append(image)

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

    def _publish_queue_state(
        self, state: int, reason: str, blocked_frame_batch_id: str = ""
    ) -> None:
        message = VisionQueueState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = blocked_frame_batch_id
        message.state = state
        message.depth = self.inference_queue.depth
        message.capacity = self.inference_queue.capacity
        message.blocked_frame_batch_id = blocked_frame_batch_id
        message.reason = reason
        self._queue_state_publisher.publish(message)

    def _handle_product_locked(self, message: ProductResultLocked) -> None:
        if message.header.session_id != self.session_id:
            return
        with self._state_guard:
            self._canceled_station_scopes.add((message.product_id, None))
        removed = self.inference_queue.lock_product(message.product_id)
        self._emit_durable_event(
            "VISION_PRODUCT_LOCKED_CANCELLATION",
            {
                "product_id": message.product_id,
                "fifo_sequence": message.fifo_sequence,
                "removed_queue_jobs": removed,
                "active_job_found": bool(
                    self.worker_pool
                    and self.worker_pool.has_active_job(message.product_id)
                ),
            },
            product_id=message.product_id,
        )

    def _apply_generated_timeout_tuning(self) -> None:
        """옵션이 True일 때만 직전 정상 session의 제안을 현재 실행에 적용합니다."""

        if not bool(self.get_parameter("vision.timeout_tuning.auto_apply").value):
            return
        path = Path(
            str(self.get_parameter("vision.timeout_tuning.generated_path").value)
        )
        if not path.is_file():
            return
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, TypeError, ValueError) as exc:
            raise ValueError(
                f"generated timeout tuning file is invalid: {type(exc).__name__}"
            ) from exc
        expected = {
            "model_version": str(self.get_parameter("vision.model.version").value),
            "model_sha256": str(self.get_parameter("vision.model.sha256").value),
            "config_fingerprint": self._timeout_tuning_fingerprint(),
        }
        if any(str(document.get(key, "")) != value for key, value in expected.items()):
            raise ValueError("generated timeout tuning model fingerprint mismatch")
        updates = []
        for station_name, parameter_name in (
            ("station_a", "vision.inference.station_a.total_timeout_ms"),
            ("station_b", "vision.inference.station_b.total_timeout_ms"),
        ):
            candidate = document.get("timeouts_ms", {}).get(station_name)
            if candidate is not None and int(candidate) > 0:
                updates.append(Parameter(parameter_name, value=int(candidate)))
        if updates:
            results = self.set_parameters(updates)
            if not all(result.successful for result in results):
                raise ValueError("generated timeout tuning parameters were rejected")

    def _timeout_tuning_fingerprint(self) -> str:
        return sha256_text(
            canonical_json(
                {
                    "camera_ids_station_a": list(
                        self.get_parameter("vision.camera_ids.station_a").value or ()
                    ),
                    "camera_ids_station_b": list(
                        self.get_parameter("vision.camera_ids.station_b").value or ()
                    ),
                    "canonical_pixel_format": str(
                        self.get_parameter(
                            "vision.image.canonical_pixel_format"
                        ).value
                    ),
                    "camera_network_map_json": str(
                        self.get_parameter("vision.camera_network_map_json").value
                    ),
                    "model_runtime": str(
                        self.get_parameter("vision.model.runtime").value
                    ),
                    "model_version": str(
                        self.get_parameter("vision.model.version").value
                    ),
                    "model_sha256": str(
                        self.get_parameter("vision.model.sha256").value
                    ),
                    "worker_count": int(
                        self.get_parameter("vision.worker_count").value
                    ),
                    "serialize_model_access": bool(
                        self.get_parameter("vision.model.serialize_access").value
                    ),
                }
            )
        )

    def _is_scope_canceled(self, product_id: str, station_id: int) -> bool:
        with self._state_guard:
            return (
                (product_id, None) in self._canceled_station_scopes
                or (product_id, station_id) in self._canceled_station_scopes
            )

    def _check_disk_capacity(self) -> str:
        root = Path(str(self.get_parameter("vision.data_root").value))
        probe = root
        while not probe.exists() and probe != probe.parent:
            probe = probe.parent
        try:
            usage = shutil.disk_usage(probe)
        except OSError as exc:
            return f"DISK_STOP_LIMIT: usage check failed: {type(exc).__name__}"
        used_ratio = usage.used / usage.total if usage.total else 1.0
        warning_ratio = float(self.get_parameter("vision.disk.warning_ratio").value)
        stop_ratio = float(self.get_parameter("vision.disk.stop_ratio").value)
        if used_ratio >= stop_ratio:
            self._emit_durable_event(
                "VISION_DISK_STOP_LIMIT",
                {
                    "data_root": str(root),
                    "used_ratio": used_ratio,
                    "stop_ratio": stop_ratio,
                },
                severity=LogEvent.CRITICAL,
            )
            self.set_health_state(NodeHealthState.DEGRADED)
            return (
                f"DISK_STOP_LIMIT: used={used_ratio:.4f}, "
                f"limit={stop_ratio:.2f}"
            )
        if used_ratio >= warning_ratio and not self._disk_warning_active:
            self._disk_warning_active = True
            self._emit_durable_event(
                "VISION_DISK_WARNING_LIMIT",
                {
                    "data_root": str(root),
                    "used_ratio": used_ratio,
                    "warning_ratio": warning_ratio,
                },
                severity=LogEvent.WARNING,
            )
        elif used_ratio < warning_ratio:
            self._disk_warning_active = False
        return ""

    def _handle_inference_cancellation(
        self, message: InferenceCancellation
    ) -> None:
        if message.header.session_id != self.session_id or not message.product_id:
            return
        station_id = int(message.station_id) or None
        with self._state_guard:
            self._canceled_station_scopes.add((message.product_id, station_id))
        removed = self.inference_queue.cancel_scope(message.product_id, station_id)
        active = bool(
            self.worker_pool
            and self.worker_pool.has_active_job(message.product_id, station_id)
        )
        self._emit_durable_event(
            "VISION_INFERENCE_CANCELLATION_APPLIED",
            {
                "cancellation_id": message.cancellation_id,
                "product_id": message.product_id,
                "fifo_sequence": message.fifo_sequence,
                "station_id": int(message.station_id),
                "reason_code": int(message.reason_code),
                "reason": message.reason,
                "removed_inference_job_ids": [
                    job.inference_job_id for job in removed
                ],
                "active_job_found": active,
                "active_result_will_be_discarded": active,
            },
            product_id=message.product_id,
        )
        ack = InferenceCancellationAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.header.session_id = self.session_id
        ack.header.message_id = new_uuid()
        ack.header.correlation_id = message.cancellation_id
        ack.cancellation_id = message.cancellation_id
        ack.product_id = message.product_id
        ack.fifo_sequence = message.fifo_sequence
        ack.station_id = int(message.station_id)
        ack.removed_queue_jobs = len(removed)
        ack.active_job_found = active
        ack.active_result_will_be_discarded = active
        ack.reason = "cancellation scope installed"
        ack.acknowledged_at = ack.header.stamp
        self._cancellation_ack_publisher.publish(ack)

    def _emit_capture_discarded(
        self,
        product_id: str,
        fifo_sequence: int,
        station_id: int,
        capture_id: str,
        batch: CaptureBatch,
        reason: str,
    ) -> None:
        self._emit_durable_event(
            "VISION_CAPTURE_DISCARDED",
            {
                "product_id": product_id,
                "fifo_sequence": fifo_sequence,
                "station_id": station_id,
                "capture_id": capture_id,
                "frame_batch_id": batch.frame_batch_id,
                "image_paths": [image.file_path for image in batch.images],
                "reason": reason,
                "deletion_owner": "LOG_NODE",
            },
            product_id=product_id,
            severity=LogEvent.WARNING,
        )

    def _emit_durable_event(
        self,
        event_type: str,
        payload: dict[str, object],
        *,
        product_id: str = "",
        severity: int = LogEvent.INFO,
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
            record = SpoolRecord(
                log_id=log_id,
                revision=1,
                payload_json=payload_json,
                payload_digest=sha256_text(payload_json),
            )
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Vision durable event serialization failed: {type(exc).__name__}"
            )
            return
        if self._result_spool is not None:
            try:
                self._result_spool.enqueue(record)
            except Exception as exc:
                self.get_logger().error(
                    f"Vision result spool enqueue failed: {type(exc).__name__}"
                )
        self._publish_spool_record(record)

    def _publish_spool_record(self, record: SpoolRecord) -> None:
        try:
            envelope = json.loads(record.payload_json)
        except (TypeError, ValueError):
            self.get_logger().error(f"invalid Vision spool JSON: {record.log_id}")
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
        if message.header.session_id != self.session_id:
            return
        if message.producer_node != NodeId.VISION.value:
            return
        if message.producer_instance_id != self.node_instance_id:
            return
        if len(message.acked_log_ids) != len(message.acked_revisions):
            self.get_logger().error("Vision LogPersistedAck arrays have unequal lengths")
            return
        if self._result_spool is None:
            return
        identities = [
            (log_id, int(revision))
            for log_id, revision in zip(
                message.acked_log_ids, message.acked_revisions
            )
            if log_id and int(revision) > 0
        ]
        if identities:
            self._result_spool.acknowledge(identities)

    def _flush_result_spool(self) -> None:
        if self._result_spool is None:
            return
        try:
            records = self._result_spool.pending(limit=100)
        except Exception as exc:
            self.get_logger().error(
                f"Vision result spool read failed: {type(exc).__name__}"
            )
            return
        for record in records:
            self._publish_spool_record(record)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    spin_node(VisionNode())


if __name__ == "__main__":
    main()
