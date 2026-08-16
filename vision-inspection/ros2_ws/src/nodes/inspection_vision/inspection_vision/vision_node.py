"""VisionNode: GigE Action capture와 파일경로 FIFO 추론의 연결 골격."""

from __future__ import annotations

import asyncio

import rclpy
from inspection_common import ErrorCode, IdempotencyStore, NodeId, new_uuid
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
    UnimplementedCaptureBackend,
)
from .inference_queue import InferenceJob, InferenceQueue


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

        capacity = int(self.get_parameter("vision.queue.capacity").value)
        self.inference_queue = InferenceQueue(max(capacity, 1))
        self.capture_backend = capture_backend or UnimplementedCaptureBackend()
        self._capture_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._capture_identities: dict[str, tuple[str, int]] = {}
        self._completed_captures: dict[
            tuple[str, int, str], dict[str, object]
        ] = {}
        self._station_locks = {1: asyncio.Lock(), 2: asyncio.Lock()}
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
        )
        for key in positive_keys:
            if self.has_parameter(key) and int(self.get_parameter(key).value) <= 0:
                missing.append(key)
        if str(self.get_parameter("vision.trigger.mode").value) != "GIGE_ACTION_COMMAND":
            missing.append("vision.trigger.mode=GIGE_ACTION_COMMAND")
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        # TODO(IMPLEMENTATION): MVS enumeration/configuration, Action1/PTP capability,
        # RGB PNG atomic storage, shared model load and WorkerPool.start().
        return await super().initialize_node_resources()

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
            and set(cameras) == set(configured)
            and bool(goal_request.command.command_id)
        )
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def _cancel_capture_goal(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_capture(self, goal_handle) -> CaptureProduct.Result:
        request = goal_handle.request
        result = CaptureProduct.Result()
        valid, code, reason = self.validate_command_header(request.command)
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
            batch = await self._capture_required_batch(goal_handle)
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
                    ErrorCode.CAPTURE_FAILED,
                    "all station capture attempts failed; see LogEvent",
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
            while not self.inference_queue.try_enqueue(job):
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
                    len(batch.images),
                    len(request.required_camera_ids),
                    batch.frame_batch_id,
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

    async def _capture_required_batch(self, goal_handle) -> CaptureBatch | None:
        request = goal_handle.request
        required = tuple(request.required_camera_ids)
        max_attempts = int(self.get_parameter("vision.capture.max_attempts").value)
        skew_limit = int(
            self.get_parameter("vision.frame_arrival_skew_limit_us").value
        )
        for attempt in range(1, max_attempts + 1):
            if goal_handle.is_cancel_requested:
                return None
            self._publish_capture_feedback(
                goal_handle,
                CaptureProduct.Feedback.TRIGGERING,
                attempt,
                0,
                len(required),
                "",
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
                await asyncio.to_thread(batch.validate, required)
                if skew_limit > 0 and batch.frame_arrival_skew_us > skew_limit:
                    raise ValueError("frame_arrival_skew_us exceeded configured limit")
                return batch
            except Exception as exc:
                self.get_logger().error(
                    f"capture_id={request.capture_id} attempt={attempt} failed: {exc}"
                )
                # 같은 capture_id로 station 필수 카메라 전체를 다음 attempt에 재촬영합니다.
        return None

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
            "reason": "OK",
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
            image.camera_timestamp.sec = artifact.camera_timestamp_ns // 1_000_000_000
            image.camera_timestamp.nanosec = artifact.camera_timestamp_ns % 1_000_000_000
            image.host_arrival_monotonic_ns = artifact.host_arrival_monotonic_ns
            image.host_arrival_timestamp.sec = (
                artifact.host_arrival_timestamp_ns // 1_000_000_000
            )
            image.host_arrival_timestamp.nanosec = (
                artifact.host_arrival_timestamp_ns % 1_000_000_000
            )
            result.images.append(image)

    @staticmethod
    def _publish_capture_feedback(
        goal_handle,
        stage: int,
        attempt: int,
        received: int,
        required: int,
        frame_batch_id: str,
        reason: str,
    ) -> None:
        feedback = CaptureProduct.Feedback()
        feedback.stage = stage
        feedback.attempt = attempt
        feedback.received_camera_count = received
        feedback.required_camera_count = required
        feedback.frame_batch_id = frame_batch_id
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
