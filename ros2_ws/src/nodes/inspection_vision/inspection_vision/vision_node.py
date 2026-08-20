"""VisionNode: GigE Action capture와 파일경로 FIFO 추론의 연결 골격."""

from __future__ import annotations

import asyncio
from contextlib import AsyncExitStack
from dataclasses import dataclass
from pathlib import Path

import rclpy
from inspection_common import (
    ErrorCode,
    IdempotencyStore,
    NodeId,
    SystemState,
    Verdict,
    new_uuid,
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
    ProductResultLocked,
    StationInferenceFailed,
    StationResult,
    VisionQueueState,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter

from .capture_contract import (
    CaptureBackend,
    CaptureBatch,
    FakeCaptureBackend,
    UnimplementedCaptureBackend,
)
from .inference_queue import InferenceJob, InferenceQueue, WorkerPool


@dataclass(frozen=True, slots=True)
class _FakeInferenceResult:
    """진짜 모델이 없을 때 파이프라인을 검증하기 위한 placeholder 결과."""

    verdict: int = Verdict.PASS
    score: float = 0.1
    model_version: str = "fake-v0"


def _fake_load_image(path: Path):
    """진짜 이미지 로드 전, 파이프라인 검증용 placeholder."""

    return path


def _fake_infer(model, images: tuple):
    """진짜 모델 추론 전, 파이프라인 검증용 placeholder."""

    return _FakeInferenceResult()


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
        self.declare_parameter("vision.frame_arrival_skew_limit_us", 0)
        self.declare_parameter("vision.capture.max_attempts", 2)
        self.declare_parameter("vision.queue.capacity", 16)
        self.declare_parameter("vision.worker_count", 1)
        self.declare_parameter("vision.inference_queue_total_timeout_ms", 0)
        self.declare_parameter("vision.data_root", "")
        self.declare_parameter("vision.model.path", "")

        if str(self.get_parameter("vision.trigger.mode").value) != "GIGE_ACTION_COMMAND":
            raise ValueError("vision.trigger.mode must be GIGE_ACTION_COMMAND")
        if int(self.get_parameter("vision.capture.max_attempts").value) != 2:
            raise ValueError("vision.capture.max_attempts is fixed at 2")

        capacity = int(self.get_parameter("vision.queue.capacity").value)
        self.inference_queue = InferenceQueue(max(capacity, 1))
        self.worker_pool: WorkerPool | None = None
        self.capture_backend = capture_backend or (
            FakeCaptureBackend(data_root=Path("/tmp/inspection/vision_fake"))
            if self.profile == "sim"
            else UnimplementedCaptureBackend()
        )
        self._capture_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._capture_identities: dict[str, tuple[str, int]] = {}
        self._completed_captures: dict[
            tuple[str, int, str], dict[str, object]
        ] = {}
        self._station_locks = {1: asyncio.Lock(), 2: asyncio.Lock()}
        configured_camera_ids = set(
            self.get_parameter("vision.camera_ids.station_a").value
        ) | set(self.get_parameter("vision.camera_ids.station_b").value)
        self._camera_locks = {
            camera_id: asyncio.Lock() for camera_id in configured_camera_ids
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
            "vision.frame_arrival_skew_limit_us",
            "vision.queue.capacity",
            "vision.worker_count",
            "vision.inference_queue_total_timeout_ms",
            "vision.data_root",
            "vision.model.path",
        )

    def validate_hardware_profile(self) -> list[str]:
        missing = super().validate_hardware_profile()
        if self.profile != "hardware":
            return missing
        positive_keys = (
            "vision.frame_arrival_skew_limit_us",
            "vision.queue.capacity",
            "vision.worker_count",
            "vision.inference_queue_total_timeout_ms",
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
        data_root = str(self.get_parameter("vision.data_root").value)
        if data_root and not Path(data_root).is_absolute():
            missing.append("vision.data_root must be absolute")
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        if self.worker_pool is None:
            worker_count = int(self.get_parameter("vision.worker_count").value)
            self.worker_pool = WorkerPool(
                queue=self.inference_queue,
                model=None,
                worker_count=max(worker_count, 1),
                load_image=_fake_load_image,
                infer=_fake_infer,
                on_success=self._on_inference_success,
                on_failure=self._on_inference_failure,
            )
            self.worker_pool.start()
        # TODO(IMPLEMENTATION): MVS enumeration/configuration, Action1/PTP capability,
        # RGB PNG atomic storage, shared model load.
        return await super().initialize_node_resources()

    def _on_inference_success(self, job: InferenceJob, result) -> None:
        """추론 성공 결과를 StationResult로 포장해 발행합니다."""

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
        self._station_result_publisher.publish(message)

    def _on_inference_failure(self, job: InferenceJob, reason: str) -> None:
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
        message.error_code = int(ErrorCode.INFERENCE_FAILED)
        message.reason = reason
        message.failed_at = message.header.stamp
        self._station_failure_publisher.publish(message)

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
            bool(goal_request.product_id)
            and bool(goal_request.capture_id)
            and goal_request.station_id in self._station_locks
            and bool(cameras)
            and len(cameras) == len(set(cameras))
            and cameras == configured
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

        async with self._station_locks[request.station_id]:
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

            inference_job_id = new_uuid()
            timeout_ms = int(
                self.get_parameter("vision.inference_queue_total_timeout_ms").value
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
                await asyncio.to_thread(self.inference_queue.wait_for_space, 0.1)

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
                batch = await self.capture_backend.capture_station(
                    product_id=request.product_id,
                    station_id=request.station_id,
                    capture_id=request.capture_id,
                    attempt=attempt,
                    required_camera_ids=required,
                )
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
                    "validating host-arrival skew and saved RGB PNG files",
                )
                # rclpy executor에는 asyncio 이벤트 루프가 없어 to_thread를 쓸 수 없습니다.
                batch.validate(required)
                if skew_limit > 0 and batch.frame_arrival_skew_us > skew_limit:
                    attempt_error = ErrorCode.CAPTURE_SKEW_EXCEEDED
                    raise ValueError("frame_arrival_skew_us exceeded configured limit")
                return batch, ErrorCode.NONE, ""
            except Exception as exc:
                if attempt_error != ErrorCode.CAPTURE_SKEW_EXCEEDED:
                    attempt_error = (
                        ErrorCode.IMPLEMENTATION_PENDING
                        if isinstance(exc, NotImplementedError)
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
        self.inference_queue.lock_product(message.product_id)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    spin_node(VisionNode())


if __name__ == "__main__":
    main()
