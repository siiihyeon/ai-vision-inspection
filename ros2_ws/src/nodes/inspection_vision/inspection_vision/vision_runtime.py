"""ROS callback과 독립적인 Vision capture→journal→FIFO→worker runtime."""

from __future__ import annotations

import asyncio
import math
from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Generic, TypeVar

from inspection_common import ErrorCode, new_uuid

from .artifact_store import ArtifactStore
from .capture_contract import CaptureBackend, CaptureBatch
from .capture_service import CaptureService
from .inference_queue import InferenceJob, InferenceQueue, WorkerPool
from .model_adapter import ModelAdapter, StationInference
from .queue_journal import InferenceJournal

LoadedImageT = TypeVar("LoadedImageT")


class QueueRuntimeState(StrEnum):
    ACCEPTING = "ACCEPTING"
    ENQUEUE_BLOCKED = "ENQUEUE_BLOCKED"
    DRAINING = "DRAINING"
    FAULT = "FAULT"


@dataclass(frozen=True, slots=True)
class QueueStateEvent:
    state: QueueRuntimeState
    depth: int
    capacity: int
    blocked_frame_batch_id: str = ""
    reason: str = ""


@dataclass(frozen=True, slots=True)
class InferenceFailure:
    job: InferenceJob
    error_code: int
    reason: str


@dataclass(frozen=True, slots=True)
class EnqueueOutcome:
    batch: CaptureBatch
    job: InferenceJob


class EnqueueCanceled(RuntimeError):
    pass


QueueStateCallback = Callable[[QueueStateEvent], None]
SuccessCallback = Callable[[InferenceJob, StationInference], None]
FailureCallback = Callable[[InferenceFailure], None]
TelemetryCallback = Callable[[str, int, str, dict[str, object]], None]


class VisionRuntime(Generic[LoadedImageT]):
    """모든 장기 자원의 소유권과 shutdown 순서를 한곳에 둡니다."""

    def __init__(
        self,
        *,
        backend: CaptureBackend,
        artifact_store: ArtifactStore,
        journal: InferenceJournal,
        model: ModelAdapter,
        load_image: Callable[[Path], LoadedImageT],
        queue_capacity: int,
        worker_count: int,
        queue_total_timeout_ms: int | None,
        frame_arrival_skew_limit_us: int | None,
        frame_timeout_ms: int,
        serialize_model_access: bool,
        on_queue_state: QueueStateCallback,
        on_inference_success: SuccessCallback,
        on_inference_failure: FailureCallback,
        on_telemetry: TelemetryCallback,
        queue_warning_ratio: float = 0.75,
        queue_resume_ratio: float = 0.50,
    ) -> None:
        if queue_capacity < 1:
            raise ValueError("queue_capacity must be positive")
        if worker_count < 1:
            raise ValueError("worker_count must be positive")
        if queue_total_timeout_ms is not None and queue_total_timeout_ms < 1:
            raise ValueError("enabled queue timeout must be positive")
        if not 0.0 < queue_resume_ratio < queue_warning_ratio < 1.0:
            raise ValueError("queue ratios must satisfy 0 < resume < warning < 1")
        self.backend = backend
        self.artifact_store = artifact_store
        self.journal = journal
        self.model = model
        self.queue = InferenceQueue(queue_capacity)
        self.capture_service = CaptureService(
            backend=backend,
            artifact_store=artifact_store,
            max_attempts=2,
            frame_arrival_skew_limit_us=frame_arrival_skew_limit_us,
            frame_timeout_ms=frame_timeout_ms,
            on_attempt_failure=self._capture_attempt_failed,
        )
        self.queue_total_timeout_ms = queue_total_timeout_ms
        self._worker_count = worker_count
        self._resume_depth = math.floor(queue_capacity * queue_resume_ratio)
        self._warning_depth = max(1, math.ceil(queue_capacity * queue_warning_ratio))
        self._on_queue_state = on_queue_state
        self._on_inference_success = on_inference_success
        self._on_inference_failure = on_inference_failure
        self._on_telemetry = on_telemetry
        self._last_queue_state: QueueRuntimeState | None = None
        self._queue_warning_active = False
        self._enqueue_gate = asyncio.Lock()
        self._started = False
        self._workers_started = False
        self._closed = False
        self._preexisting_capture_keys: frozenset[tuple[str, int, str]] = frozenset()
        self._worker_pool = WorkerPool(
            queue=self.queue,
            model=model,
            worker_count=worker_count,
            load_image=load_image,
            infer=lambda loaded_model, images: loaded_model.infer(images),
            on_claim=self._on_worker_claim,
            on_success=self._on_worker_success,
            on_failure=self._on_worker_failure,
            serialize_model_access=serialize_model_access,
        )

    async def start(self) -> dict[str, object]:
        if self._started or self._closed:
            raise RuntimeError("VisionRuntime cannot be started twice")
        camera_details = await self.backend.initialize()
        removed_temp_files = await asyncio.to_thread(
            self.artifact_store.cleanup_incomplete_files
        )
        self._preexisting_capture_keys = await asyncio.to_thread(
            self.artifact_store.discover_capture_keys
        )
        await asyncio.to_thread(self.model.warmup)
        recovered = await asyncio.to_thread(self.journal.recoverable_jobs)
        self._started = True
        for job in recovered:
            if not self.queue.try_enqueue(job):
                raise RuntimeError(
                    "recoverable journal jobs exceed configured queue capacity"
                )
        self._emit_queue_transition(QueueRuntimeState.ACCEPTING)
        pressure, disk_details = self.artifact_store.disk_pressure()
        self._on_telemetry(
            "VISION_RUNTIME_INITIALIZED",
            20,
            "",
            {
                "camera": camera_details,
                "recovered_job_count": len(recovered),
                "deleted_incomplete_temp_files": list(removed_temp_files),
                "disk_pressure": pressure,
                "disk": disk_details,
                "queue_capacity": self.queue.capacity,
                "worker_count": self._worker_count,
            },
        )
        return {
            "camera": camera_details,
            "queue_capacity": self.queue.capacity,
            "worker_count": self._worker_count,
            "recovered_job_count": len(recovered),
            "journal": self.journal.snapshot(),
            "disk_pressure": pressure,
            "disk": disk_details,
        }

    def activate_workers(self) -> None:
        """NodeBase가 session_id를 적용하고 READY가 된 뒤에만 결과 처리를 시작합니다."""

        if not self._started or self._closed:
            raise RuntimeError("VisionRuntime is not ready for worker activation")
        if self._workers_started:
            return
        self._worker_pool.start()
        self._workers_started = True

    async def enqueue_saved_batch(
        self,
        *,
        batch: CaptureBatch,
        fifo_sequence: int,
        is_cancel_requested: Callable[[], bool],
        on_blocked: Callable[[InferenceJob], None],
        resume_after_block_allowed: Callable[[], bool] | None = None,
    ) -> InferenceJob:
        if not self._started or self._closed:
            raise RuntimeError("VisionRuntime is not active")
        job = InferenceJob(
            inference_job_id=new_uuid(),
            product_id=batch.product_id,
            fifo_sequence=fifo_sequence,
            station_id=batch.station_id,
            capture_id=batch.capture_id,
            frame_batch_id=batch.frame_batch_id,
            image_paths=tuple(image.file_path for image in batch.images),
            enqueued_monotonic_ns=0,
            enqueued_wall_time_ns=0,
            queue_total_timeout_ms=self.queue_total_timeout_ms,
            result_revision=1,
        )
        await asyncio.to_thread(self.journal.prepare, job)
        async with self._enqueue_gate:
            blocked_reported = False
            while True:
                if self.queue.is_product_locked(job.product_id) or self.journal.is_product_locked(
                    job.product_id
                ):
                    raise EnqueueCanceled("product result was locked before enqueue")
                if is_cancel_requested():
                    await asyncio.to_thread(
                        self.journal.mark_failed,
                        job.inference_job_id,
                        "capture Action canceled before enqueue",
                    )
                    raise EnqueueCanceled("capture Action canceled before enqueue")
                if (
                    blocked_reported
                    and resume_after_block_allowed is not None
                    and not resume_after_block_allowed()
                ):
                    await asyncio.sleep(0.05)
                    continue
                accepted_job: list[InferenceJob] = []
                try:
                    enqueued = self.queue.try_enqueue(
                        job,
                        on_enqueued=lambda accepted: (
                            self.journal.mark_enqueued(accepted),
                            accepted_job.append(accepted),
                        ),
                    )
                except Exception:
                    raise
                if enqueued:
                    if blocked_reported:
                        self._emit_queue_transition(
                            QueueRuntimeState.ACCEPTING,
                            reason="queue recovered to configured resume threshold",
                        )
                    self._check_queue_warning()
                    return accepted_job[0]
                if self.queue.closed:
                    await asyncio.to_thread(
                        self.journal.mark_failed,
                        job.inference_job_id,
                        "inference queue closed before enqueue",
                    )
                    raise RuntimeError("inference queue is closed")
                blocked_reported = True
                on_blocked(job)
                self._emit_queue_transition(
                    QueueRuntimeState.ENQUEUE_BLOCKED,
                    blocked_frame_batch_id=batch.frame_batch_id,
                    reason="queue full; saved FrameBatch retained in this process",
                )
                await asyncio.to_thread(
                    self.queue.wait_until_depth_at_most, self._resume_depth, 0.1
                )

    def lock_product(self, product_id: str) -> int:
        queued = self.queue.queued_jobs_for_product(product_id)
        removed = self.queue.lock_product(product_id)
        self.journal.lock_product(product_id)
        for job in queued:
            self._on_telemetry(
                "INFERENCE_JOB_LOCKED",
                30,
                product_id,
                {
                    "inference_job_id": job.inference_job_id,
                    "frame_batch_id": job.frame_batch_id,
                    "station_id": job.station_id,
                },
            )
        self._check_queue_warning()
        return removed

    def status_snapshot(self) -> dict[str, object]:
        pressure, disk = self.artifact_store.disk_pressure()
        return {
            "queue_depth": self.queue.depth,
            "queue_capacity": self.queue.capacity,
            "queue_state": (
                self._last_queue_state.value if self._last_queue_state else "UNKNOWN"
            ),
            "queue_warning_active": self._queue_warning_active,
            "journal": self.journal.snapshot(),
            "disk_pressure": pressure,
            "disk": disk,
            "started": self._started,
            "workers_started": self._workers_started,
            "closed": self._closed,
        }

    def captured_before_runtime(
        self, product_id: str, station_id: int, capture_id: str
    ) -> bool:
        return (product_id, station_id, capture_id) in self._preexisting_capture_keys

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._started:
            self._emit_queue_transition(
                QueueRuntimeState.DRAINING, reason="VisionRuntime shutdown"
            )
            if self._workers_started:
                await asyncio.to_thread(self._worker_pool.stop)
            else:
                self.queue.close()
        try:
            await self.backend.close()
        finally:
            try:
                await asyncio.to_thread(self.model.close)
            finally:
                await asyncio.to_thread(self.journal.close)

    def _on_worker_claim(self, job: InferenceJob) -> None:
        self.journal.mark_running(job.inference_job_id)
        self._check_queue_warning()

    def _on_worker_success(
        self, job: InferenceJob, inference: StationInference
    ) -> None:
        if self.queue.is_product_locked(job.product_id) or self.journal.is_product_locked(
            job.product_id
        ):
            return
        try:
            inference.validate()
        except Exception as exc:
            self._on_worker_failure(
                job, f"invalid model result: {type(exc).__name__}: {exc}"
            )
            return
        try:
            self._on_inference_success(job, inference)
        except Exception as exc:
            self._on_worker_failure(
                job, f"station result publication failed: {type(exc).__name__}"
            )
            return
        try:
            self.journal.mark_done(job.inference_job_id)
        except Exception as exc:
            self._on_telemetry(
                "INFERENCE_JOURNAL_FINALIZE_FAILED",
                40,
                job.product_id,
                {
                    "inference_job_id": job.inference_job_id,
                    "frame_batch_id": job.frame_batch_id,
                    "error": type(exc).__name__,
                },
            )

    def _on_worker_failure(self, job: InferenceJob, reason: str) -> None:
        if self.queue.is_product_locked(job.product_id) or self.journal.is_product_locked(
            job.product_id
        ):
            return
        error_code = int(ErrorCode.INFERENCE_FAILED)
        if "timeout" in reason.lower():
            error_code = int(ErrorCode.INFERENCE_TIMEOUT)
        elif "read" in reason.lower() or "decode" in reason.lower():
            error_code = int(ErrorCode.INFERENCE_FILE_READ_FAILED)
        try:
            self._on_inference_failure(InferenceFailure(job, error_code, reason))
        finally:
            try:
                self.journal.mark_failed(job.inference_job_id, reason)
            except ValueError:
                # ProductResultLocked와 worker callback이 경합한 경우 terminal
                # LOCKED 상태를 실패 상태로 되돌리지 않습니다.
                pass
            except Exception as exc:
                self._on_telemetry(
                    "INFERENCE_JOURNAL_FAILURE_FINALIZE_FAILED",
                    40,
                    job.product_id,
                    {
                        "inference_job_id": job.inference_job_id,
                        "frame_batch_id": job.frame_batch_id,
                        "error": type(exc).__name__,
                    },
                )

    def _capture_attempt_failed(self, failure) -> None:
        self._on_telemetry(
            "CAPTURE_ATTEMPT_FAILED",
            30,
            failure.product_id,
            {
                "product_id": failure.product_id,
                "station_id": failure.station_id,
                "capture_id": failure.capture_id,
                "attempt": failure.attempt,
                "error_code": failure.error_code,
                "reason": failure.reason,
                "retryable": failure.retryable,
                "recovery_succeeded": failure.recovery_succeeded,
            },
        )

    def _check_queue_warning(self) -> None:
        depth = self.queue.depth
        if depth >= self._warning_depth and not self._queue_warning_active:
            self._queue_warning_active = True
            self._on_telemetry(
                "INFERENCE_QUEUE_WARNING",
                30,
                "",
                {
                    "depth": depth,
                    "capacity": self.queue.capacity,
                    "warning_depth": self._warning_depth,
                },
            )
        elif depth < self._warning_depth and self._queue_warning_active:
            self._queue_warning_active = False
            self._on_telemetry(
                "INFERENCE_QUEUE_WARNING_CLEARED",
                20,
                "",
                {"depth": depth, "capacity": self.queue.capacity},
            )

    def _emit_queue_transition(
        self,
        state: QueueRuntimeState,
        *,
        blocked_frame_batch_id: str = "",
        reason: str = "",
    ) -> None:
        if self._last_queue_state == state:
            return
        self._last_queue_state = state
        self._on_queue_state(
            QueueStateEvent(
                state=state,
                depth=self.queue.depth,
                capacity=self.queue.capacity,
                blocked_frame_batch_id=blocked_frame_batch_id,
                reason=reason,
            )
        )
