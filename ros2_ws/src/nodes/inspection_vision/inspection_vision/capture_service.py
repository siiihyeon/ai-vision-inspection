"""ROS와 독립적인 station 촬영 재시도·검증·저장 orchestration."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import StrEnum

from inspection_common import ErrorCode, new_uuid

from .artifact_store import ArtifactStore
from .capture_contract import (
    CameraUnavailable,
    CaptureBackend,
    CaptureBatch,
    CaptureError,
    CaptureSkewExceeded,
    CaptureStorageError,
)


class CaptureProgressStage(StrEnum):
    CAMERAS_READY = "CAMERAS_READY"
    TRIGGERING = "TRIGGERING"
    WAITING_FRAMES = "WAITING_FRAMES"
    VALIDATING_SKEW = "VALIDATING_SKEW"
    SAVING_FILES = "SAVING_FILES"
    RETRYING = "RETRYING"


@dataclass(frozen=True, slots=True)
class CaptureRequest:
    product_id: str
    fifo_sequence: int
    station_id: int
    capture_id: str
    required_camera_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class CaptureProgress:
    stage: CaptureProgressStage
    attempt: int
    completed_camera_ids: tuple[str, ...] = ()
    pending_camera_ids: tuple[str, ...] = ()
    frame_batch_id: str = ""
    reason: str = ""


@dataclass(frozen=True, slots=True)
class CaptureAttemptFailure:
    attempt: int
    error_code: int
    reason: str
    retryable: bool
    recovery_succeeded: bool | None = None
    product_id: str = ""
    station_id: int = 0
    capture_id: str = ""


class CapturePipelineFailed(RuntimeError):
    def __init__(self, failure: CaptureAttemptFailure) -> None:
        super().__init__(failure.reason)
        self.failure = failure


class CaptureCanceled(CapturePipelineFailed):
    pass


ProgressCallback = Callable[[CaptureProgress], None]
CancelCheck = Callable[[], bool]
AttemptEventCallback = Callable[[CaptureAttemptFailure], None]


class CaptureService:
    """동일 capture_id로 필수 카메라 전체를 최대 두 번 촬영합니다."""

    def __init__(
        self,
        *,
        backend: CaptureBackend,
        artifact_store: ArtifactStore,
        max_attempts: int = 2,
        frame_arrival_skew_limit_us: int | None,
        frame_timeout_ms: int,
        on_attempt_failure: AttemptEventCallback | None = None,
    ) -> None:
        if max_attempts != 2:
            raise ValueError("capture max_attempts is fixed at 2")
        if frame_arrival_skew_limit_us is not None and frame_arrival_skew_limit_us < 1:
            raise ValueError("enabled skew limit must be positive")
        if frame_timeout_ms < 1:
            raise ValueError("frame_timeout_ms must be positive")
        self.backend = backend
        self.artifact_store = artifact_store
        self.max_attempts = max_attempts
        self.frame_arrival_skew_limit_us = frame_arrival_skew_limit_us
        self.frame_timeout_ms = frame_timeout_ms
        self.on_attempt_failure = on_attempt_failure

    async def capture(
        self,
        request: CaptureRequest,
        *,
        on_progress: ProgressCallback,
        is_cancel_requested: CancelCheck,
    ) -> CaptureBatch:
        self._validate_request(request)
        last_failure = CaptureAttemptFailure(
            attempt=0,
            error_code=int(ErrorCode.CAPTURE_FAILED),
            reason="capture did not start",
            retryable=True,
        )
        for attempt in range(1, self.max_attempts + 1):
            self._raise_if_canceled(is_cancel_requested, attempt)
            on_progress(
                CaptureProgress(
                    CaptureProgressStage.CAMERAS_READY,
                    attempt,
                    pending_camera_ids=request.required_camera_ids,
                    reason="required station cameras are armed",
                )
            )
            on_progress(
                CaptureProgress(
                    CaptureProgressStage.TRIGGERING,
                    attempt,
                    pending_camera_ids=request.required_camera_ids,
                    reason="issuing immediate GigE Action Command",
                )
            )
            try:
                raw = await asyncio.wait_for(
                    self.backend.capture_station(
                        product_id=request.product_id,
                        station_id=request.station_id,
                        capture_id=request.capture_id,
                        attempt=attempt,
                        required_camera_ids=request.required_camera_ids,
                    ),
                    timeout=self.frame_timeout_ms / 1000.0,
                )
                on_progress(
                    CaptureProgress(
                        CaptureProgressStage.WAITING_FRAMES,
                        attempt,
                        completed_camera_ids=tuple(
                            frame.camera_id for frame in raw.frames
                        ),
                        pending_camera_ids=tuple(
                            camera_id
                            for camera_id in request.required_camera_ids
                            if camera_id not in {frame.camera_id for frame in raw.frames}
                        ),
                        reason="received candidate frames from SDK callbacks",
                    )
                )
                self._validate_raw_identity(raw, request, attempt)
                raw.validate(request.required_camera_ids)
                on_progress(
                    CaptureProgress(
                        CaptureProgressStage.VALIDATING_SKEW,
                        attempt,
                        completed_camera_ids=request.required_camera_ids,
                        reason="calculating host callback arrival skew",
                    )
                )
                skew_failed = (
                    self.frame_arrival_skew_limit_us is not None
                    and raw.frame_arrival_skew_us
                    > self.frame_arrival_skew_limit_us
                )
                frame_batch_id = new_uuid()
                on_progress(
                    CaptureProgress(
                        CaptureProgressStage.SAVING_FILES,
                        attempt,
                        completed_camera_ids=request.required_camera_ids,
                        frame_batch_id=frame_batch_id,
                        reason="atomically saving canonical RGB PNG files and manifest",
                    )
                )
                batch = await asyncio.to_thread(
                    self.artifact_store.save_batch,
                    raw,
                    frame_batch_id=frame_batch_id,
                    required_camera_ids=request.required_camera_ids,
                    capture_outcome=("SKEW_FAILED" if skew_failed else "SUCCESS"),
                    failure_reason=(
                        "frame_arrival_skew_us exceeded configured limit"
                        if skew_failed
                        else ""
                    ),
                )
                if skew_failed:
                    raise CaptureSkewExceeded(
                        "frame_arrival_skew_us "
                        f"{batch.frame_arrival_skew_us} exceeded limit "
                        f"{self.frame_arrival_skew_limit_us}"
                    )
                return batch
            except asyncio.TimeoutError:
                failure = CaptureAttemptFailure(
                    attempt,
                    int(ErrorCode.CAPTURE_FAILED),
                    f"attempt {attempt}/{self.max_attempts}: frame timeout",
                    True,
                )
            except CaptureSkewExceeded as exc:
                failure = CaptureAttemptFailure(
                    attempt,
                    int(ErrorCode.CAPTURE_SKEW_EXCEEDED),
                    f"attempt {attempt}/{self.max_attempts}: {exc.reason}",
                    exc.retryable,
                )
            except CaptureStorageError as exc:
                failure = CaptureAttemptFailure(
                    attempt,
                    int(ErrorCode.CAPTURE_SAVE_FAILED),
                    f"attempt {attempt}/{self.max_attempts}: {exc.reason}",
                    exc.retryable,
                )
            except (CameraUnavailable, CaptureError) as exc:
                failure = CaptureAttemptFailure(
                    attempt,
                    int(ErrorCode.CAPTURE_FAILED),
                    f"attempt {attempt}/{self.max_attempts}: {exc.reason}",
                    exc.retryable,
                )
            except (ValueError, OSError) as exc:
                failure = CaptureAttemptFailure(
                    attempt,
                    int(ErrorCode.CAPTURE_FAILED),
                    f"attempt {attempt}/{self.max_attempts}: {type(exc).__name__}: {exc}",
                    True,
                )
            failure = replace(
                failure,
                product_id=request.product_id,
                station_id=request.station_id,
                capture_id=request.capture_id,
            )
            last_failure = failure
            if self.on_attempt_failure is not None:
                self.on_attempt_failure(failure)
            if not failure.retryable or attempt >= self.max_attempts:
                break
            self._raise_if_canceled(is_cancel_requested, attempt)
            on_progress(
                CaptureProgress(
                    CaptureProgressStage.RETRYING,
                    attempt,
                    pending_camera_ids=request.required_camera_ids,
                    reason=(
                        "clearing SDK buffers and re-arming all required cameras; "
                        "same capture_id"
                    ),
                )
            )
            try:
                await self.backend.prepare_retry(
                    station_id=request.station_id,
                    required_camera_ids=request.required_camera_ids,
                )
            except Exception as exc:
                last_failure = CaptureAttemptFailure(
                    attempt,
                    int(ErrorCode.CAPTURE_FAILED),
                    f"retry preparation failed: {type(exc).__name__}: {exc}",
                    False,
                )
                break

        recovery_succeeded: bool | None = None
        if last_failure.error_code == int(ErrorCode.CAPTURE_FAILED):
            try:
                recovery_succeeded = await self.backend.recover_station(
                    station_id=request.station_id,
                    required_camera_ids=request.required_camera_ids,
                )
            except Exception:
                recovery_succeeded = False
        last_failure = CaptureAttemptFailure(
            attempt=last_failure.attempt,
            error_code=last_failure.error_code,
            reason=last_failure.reason,
            retryable=last_failure.retryable,
            recovery_succeeded=recovery_succeeded,
            product_id=request.product_id,
            station_id=request.station_id,
            capture_id=request.capture_id,
        )
        raise CapturePipelineFailed(last_failure)

    @staticmethod
    def _validate_request(request: CaptureRequest) -> None:
        if not request.product_id or not request.capture_id:
            raise ValueError("product_id and capture_id are required")
        if request.station_id not in {1, 2}:
            raise ValueError("station_id must be 1 or 2")
        if not request.required_camera_ids:
            raise ValueError("required_camera_ids cannot be empty")
        if len(request.required_camera_ids) != len(set(request.required_camera_ids)):
            raise ValueError("required_camera_ids cannot contain duplicates")

    @staticmethod
    def _validate_raw_identity(raw, request: CaptureRequest, attempt: int) -> None:
        if (
            raw.product_id != request.product_id
            or raw.station_id != request.station_id
            or raw.capture_id != request.capture_id
            or raw.attempt != attempt
        ):
            raise CameraUnavailable("camera backend returned mismatched capture identity")

    @staticmethod
    def _raise_if_canceled(is_cancel_requested: CancelCheck, attempt: int) -> None:
        if is_cancel_requested():
            failure = CaptureAttemptFailure(
                attempt,
                int(ErrorCode.CAPTURE_CANCELED),
                "capture canceled before inference enqueue",
                False,
            )
            raise CaptureCanceled(failure)
