"""Vision contract simulator for running Master without the real VisionNode."""

from __future__ import annotations

import hashlib
import struct
import time
import zlib
from pathlib import Path

import rclpy
from inspection_common.constants import ErrorCode, NodeId
from inspection_common.identifiers import new_uuid
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
    state_qos,
)
from inspection_interfaces.action import CaptureProduct
from inspection_interfaces.msg import (
    CommonHeader,
    ImageReference,
    InferenceCancellation,
    InferenceCancellationAck,
    StationInferenceFailed,
    StationResult,
    VisionQueueState,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


class VisionSimulatorNode(InspectionNodeBase):
    """Deterministic replacement for the camera and inference node."""

    def __init__(self) -> None:
        super().__init__(NodeId.VISION, provides_initialize_action=True)
        self.declare_parameter("vision_sim.output_root", "/tmp/inspection/vision_sim")
        self.declare_parameter("vision_sim.result_delay_ms", 100)
        self.declare_parameter("vision_sim.station_a_verdict", "PASS")
        self.declare_parameter("vision_sim.station_b_verdict", "PASS")
        self.declare_parameter("vision_sim.station_a_score", 0.10)
        self.declare_parameter("vision_sim.station_b_score", 0.10)
        self.declare_parameter("vision_sim.failure_mode", "none")
        self.declare_parameter("vision_sim.model_version", "sim-model-v1")

        self._capture_server = ActionServer(
            self,
            CaptureProduct,
            "vision/capture_product",
            execute_callback=self._execute_capture,
            goal_callback=self._accept_capture_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self._queue_state_publisher = self.create_publisher(
            VisionQueueState, "vision/queue_state", state_qos()
        )
        self._result_publisher = self.create_publisher(
            StationResult, "vision/station_result", reliable_event_qos()
        )
        self._failure_publisher = self.create_publisher(
            StationInferenceFailed,
            "vision/station_inference_failed",
            reliable_event_qos(),
        )
        self._cancellation_ack_publisher = self.create_publisher(
            InferenceCancellationAck,
            "vision/inference_cancellation_ack",
            reliable_event_qos(),
        )
        self._cancellation_subscription = self.create_subscription(
            InferenceCancellation,
            "/inspection/master/inference_cancellation",
            self._handle_cancellation,
            reliable_event_qos(),
        )
        self._publish_queue_state(VisionQueueState.ACCEPTING, "")
        self.get_logger().info("Vision simulator started; real VisionNode is not required")

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        return NodeInitializationOutcome(
            success=True,
            reason="vision simulator resources initialized",
        )

    def _accept_capture_goal(self, request: CaptureProduct.Goal) -> GoalResponse:
        if (
            not request.product_id
            or not request.capture_id
            or int(request.station_id) not in (1, 2)
            or not request.required_camera_ids
        ):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _execute_capture(self, goal_handle) -> CaptureProduct.Result:
        request = goal_handle.request
        result = CaptureProduct.Result()
        result.product_id = request.product_id
        result.station_id = request.station_id
        result.capture_id = request.capture_id
        result.attempt_count = 1
        result.frame_arrival_skew_us = 0

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.error_code = int(ErrorCode.CAPTURE_CANCELED)
            result.reason = "capture canceled by Master"
            return result

        self._publish_feedback(
            goal_handle, CaptureProduct.Feedback.CAMERAS_READY, "sim cameras ready"
        )
        self._publish_feedback(goal_handle, CaptureProduct.Feedback.TRIGGERING, "sim trigger")
        delay_ms = max(0, int(self.get_parameter("vision_sim.result_delay_ms").value))
        if delay_ms:
            time.sleep(delay_ms / 1000.0)

        failure_mode = str(self.get_parameter("vision_sim.failure_mode").value).lower()
        if failure_mode == "capture":
            goal_handle.abort()
            result.error_code = int(ErrorCode.CAPTURE_FAILED)
            result.reason = "simulated capture failure"
            return result

        result.frame_batch_id = f"sim-frame-{request.capture_id}"
        result.inference_job_id = f"sim-job-{request.capture_id}"
        output_root = Path(str(self.get_parameter("vision_sim.output_root").value))
        result.images = [
            self._create_image_reference(output_root, request.capture_id, camera_id)
            for camera_id in request.required_camera_ids
        ]
        result.success = True
        goal_handle.succeed()
        self._schedule_station_outcome(request)
        return result

    def _create_image_reference(
        self, output_root: Path, capture_id: str, camera_id: str
    ) -> ImageReference:
        path = output_root / self.session_id / capture_id / f"{camera_id}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        width, height = 16, 12
        pixels = b"\x80" * (width * height)
        scanlines = b"".join(
            b"\x00" + pixels[row * width : (row + 1) * width]
            for row in range(height)
        )
        png = b"\x89PNG\r\n\x1a\n"
        png += self._png_chunk(
            b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
        )
        png += self._png_chunk(b"IDAT", zlib.compress(scanlines))
        png += self._png_chunk(b"IEND", b"")
        path.write_bytes(png)

        image = ImageReference()
        image.camera_id = camera_id
        image.file_path = str(path.resolve())
        image.sha256 = hashlib.sha256(png).hexdigest()
        image.file_size_bytes = len(png)
        image.width = width
        image.height = height
        image.pixel_format = "MONO8_PNG"
        image.camera_timestamp_domain = "sim"
        image.camera_timestamp_ns = time.time_ns()
        image.camera_timestamp_synchronized = True
        image.host_arrival_monotonic_ns = time.monotonic_ns()
        image.host_arrival_wall_time = self.get_clock().now().to_msg()
        return image

    @staticmethod
    def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + chunk_type
            + data
            + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    def _schedule_station_outcome(self, request: CaptureProduct.Goal) -> None:
        timer = self.create_timer(
            0.05, lambda: self._publish_scheduled_outcome(timer, request)
        )

    def _publish_scheduled_outcome(self, timer, request: CaptureProduct.Goal) -> None:
        timer.cancel()
        self.destroy_timer(timer)
        self._publish_station_outcome(request)

    def _publish_station_outcome(self, request: CaptureProduct.Goal) -> None:
        station_id = int(request.station_id)
        verdict_parameter = (
            "vision_sim.station_a_verdict"
            if station_id == 1
            else "vision_sim.station_b_verdict"
        )
        score_parameter = (
            "vision_sim.station_a_score"
            if station_id == 1
            else "vision_sim.station_b_score"
        )
        failure_mode = str(self.get_parameter("vision_sim.failure_mode").value).lower()
        if failure_mode in {"inference", "timeout"}:
            message = StationInferenceFailed()
            self._fill_header(message.header, request.capture_id)
            message.product_id = request.product_id
            message.fifo_sequence = request.fifo_sequence
            message.station_id = request.station_id
            message.capture_id = request.capture_id
            message.frame_batch_id = f"sim-frame-{request.capture_id}"
            message.inference_job_id = f"sim-job-{request.capture_id}"
            message.result_revision = 1
            message.error_code = int(
                ErrorCode.INFERENCE_TIMEOUT
                if failure_mode == "timeout"
                else ErrorCode.INFERENCE_FAILED
            )
            message.reason = f"simulated inference {failure_mode}"
            message.failed_at = self.get_clock().now().to_msg()
            self._failure_publisher.publish(message)
            return

        message = StationResult()
        self._fill_header(message.header, request.capture_id)
        message.product_id = request.product_id
        message.fifo_sequence = request.fifo_sequence
        message.station_id = request.station_id
        message.capture_id = request.capture_id
        message.frame_batch_id = f"sim-frame-{request.capture_id}"
        message.inference_job_id = f"sim-job-{request.capture_id}"
        message.result_revision = 1
        verdict = str(self.get_parameter(verdict_parameter).value).upper()
        message.verdict = StationResult.NG if verdict == "NG" else StationResult.PASS
        message.score = float(self.get_parameter(score_parameter).value)
        message.model_version = str(self.get_parameter("vision_sim.model_version").value)
        message.completed_at = self.get_clock().now().to_msg()
        self._result_publisher.publish(message)

    def _publish_feedback(self, goal_handle, stage: int, reason: str) -> None:
        feedback = CaptureProduct.Feedback()
        feedback.stage = stage
        feedback.attempt = 1
        feedback.progress = 0.5
        feedback.reason = reason
        goal_handle.publish_feedback(feedback)

    def _fill_header(self, header: CommonHeader, correlation_id: str) -> None:
        header.stamp = self.get_clock().now().to_msg()
        header.session_id = self.session_id
        header.message_id = new_uuid()
        header.correlation_id = correlation_id

    def _publish_queue_state(self, state: int, reason: str) -> None:
        message = VisionQueueState()
        self._fill_header(message.header, "")
        message.state = state
        message.depth = 0
        message.capacity = 1
        message.reason = reason
        self._queue_state_publisher.publish(message)

    def _handle_cancellation(self, request: InferenceCancellation) -> None:
        message = InferenceCancellationAck()
        self._fill_header(message.header, request.cancellation_id)
        message.cancellation_id = request.cancellation_id
        message.product_id = request.product_id
        message.fifo_sequence = request.fifo_sequence
        message.station_id = request.station_id
        message.reason = "simulator has no queued inference"
        message.acknowledged_at = self.get_clock().now().to_msg()
        self._cancellation_ack_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    spin_node(VisionSimulatorNode())
