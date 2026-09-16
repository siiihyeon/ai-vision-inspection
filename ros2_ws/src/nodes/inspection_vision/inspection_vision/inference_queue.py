"""파일 경로 기반 bounded FIFO와 공유 모델 worker pool."""

from __future__ import annotations

import threading
import time
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Callable, Generic, TypeVar

ModelT = TypeVar("ModelT")
LoadedT = TypeVar("LoadedT")
ResultT = TypeVar("ResultT")


class InferenceDeadlineExceeded(RuntimeError):
    """enqueue부터 결과 확정까지의 Queue 총시간이 만료되었습니다."""


class InferenceFailureKind(str, Enum):
    """ROS error code로 손실 없이 변환할 수 있는 내부 실패 분류."""

    TIMEOUT = "TIMEOUT"
    FILE_READ = "FILE_READ"
    PREPROCESSING = "PREPROCESSING"
    MODEL = "MODEL"
    CUDA_OOM = "CUDA_OOM"


@dataclass(frozen=True, slots=True)
class InferenceFailure:
    kind: InferenceFailureKind
    reason: str


@dataclass(frozen=True, slots=True)
class InferenceTiming:
    load_ms: float
    model_forward_ms: float
    enqueue_to_terminal_ms: float
    completed_monotonic_ns: int


@dataclass(frozen=True, slots=True)
class CancellationOutcome:
    removed_jobs: tuple[InferenceJob, ...]
    active_job_found: bool


@dataclass(frozen=True, slots=True)
class InferenceJob:
    inference_job_id: str
    product_id: str
    fifo_sequence: int
    station_id: int
    capture_id: str
    frame_batch_id: str
    image_paths: tuple[str, ...]
    enqueued_monotonic_ns: int
    queue_total_timeout_ms: int | None = None
    camera_ids: tuple[str, ...] = ()
    capture_completed_monotonic_ns: int = 0

    def expired(self, now_ns: int | None = None) -> bool:
        if self.queue_total_timeout_ms is None:
            return False
        current = time.monotonic_ns() if now_ns is None else now_ns
        return current - self.enqueued_monotonic_ns > self.queue_total_timeout_ms * 1_000_000


class InferenceQueue:
    """get 순서까지 fifo_sequence/enqueue 순서를 보장하는 bounded queue."""

    def __init__(self, capacity: int) -> None:
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.capacity = capacity
        self._items: deque[InferenceJob] = deque()
        self._locked_products: set[str] = set()
        self._canceled_scopes: set[tuple[str, int | None]] = set()
        self._condition = threading.Condition()
        self._closed = False

    @property
    def depth(self) -> int:
        with self._condition:
            return len(self._items)

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed

    def try_enqueue(self, job: InferenceJob) -> bool:
        with self._condition:
            if self._closed or self._job_canceled_unlocked(job):
                return False
            if len(self._items) >= self.capacity:
                return False
            if job.enqueued_monotonic_ns <= 0:
                job = replace(job, enqueued_monotonic_ns=time.monotonic_ns())
            # fifo_sequence가 공식 순서키입니다. 같은 sequence 안에서는
            # 실제 enqueue monotonic 시각과 station_id로 안정적으로 정렬합니다.
            order = (job.fifo_sequence, job.enqueued_monotonic_ns, job.station_id)
            insert_at = len(self._items)
            for index, queued in enumerate(self._items):
                queued_order = (
                    queued.fifo_sequence,
                    queued.enqueued_monotonic_ns,
                    queued.station_id,
                )
                if order < queued_order:
                    insert_at = index
                    break
            self._items.insert(insert_at, job)
            self._condition.notify()
            return True

    def wait_for_space(self, timeout_seconds: float) -> bool:
        with self._condition:
            if self._closed:
                return False
            if len(self._items) < self.capacity:
                return True
            self._condition.wait(timeout_seconds)
            return not self._closed and len(self._items) < self.capacity

    def discard_expired(self, now_ns: int | None = None) -> tuple[InferenceJob, ...]:
        """deadline이 지난 대기 job을 FIFO에서 강제로 제거합니다."""

        current = time.monotonic_ns() if now_ns is None else now_ns
        with self._condition:
            expired = tuple(job for job in self._items if job.expired(current))
            if expired:
                expired_ids = {job.inference_job_id for job in expired}
                self._items = deque(
                    job
                    for job in self._items
                    if job.inference_job_id not in expired_ids
                )
                self._condition.notify_all()
            return expired

    def get(self) -> InferenceJob | None:
        with self._condition:
            while not self._items and not self._closed:
                self._condition.wait()
            if self._closed:
                return None
            job = self._items.popleft()
            self._condition.notify_all()
            return job

    def _job_canceled_unlocked(self, job: InferenceJob) -> bool:
        return (
            job.product_id in self._locked_products
            or (job.product_id, None) in self._canceled_scopes
            or (job.product_id, job.station_id) in self._canceled_scopes
        )

    def is_job_canceled(self, job: InferenceJob) -> bool:
        with self._condition:
            return self._job_canceled_unlocked(job)

    def cancel_scope(
        self, product_id: str, station_id: int | None = None
    ) -> tuple[InferenceJob, ...]:
        """대기 job을 제거하고 이후 enqueue/result publish를 막습니다."""

        with self._condition:
            self._canceled_scopes.add((product_id, station_id))
            removed = tuple(
                job
                for job in self._items
                if job.product_id == product_id
                and (station_id is None or job.station_id == station_id)
            )
            removed_ids = {job.inference_job_id for job in removed}
            if removed_ids:
                self._items = deque(
                    job
                    for job in self._items
                    if job.inference_job_id not in removed_ids
                )
                self._condition.notify_all()
            return removed

    def cancel_all_waiting(self) -> tuple[InferenceJob, ...]:
        with self._condition:
            removed = tuple(self._items)
            self._items.clear()
            self._condition.notify_all()
            return removed

    def lock_product(self, product_id: str) -> int:
        """Sensor3 ProductResultLocked 이후 대기 작업을 제거하고 late 결과를 막습니다."""

        with self._condition:
            self._locked_products.add(product_id)
            self._canceled_scopes.add((product_id, None))
            before = len(self._items)
            self._items = deque(job for job in self._items if job.product_id != product_id)
            removed = before - len(self._items)
            self._condition.notify_all()
            return removed

    def is_product_locked(self, product_id: str) -> bool:
        with self._condition:
            return product_id in self._locked_products

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()


class WorkerPool(Generic[ModelT, LoadedT, ResultT]):
    """하나의 모델 인스턴스를 N개 worker가 공유하는 실행 골격."""

    def __init__(
        self,
        *,
        queue: InferenceQueue,
        model: ModelT,
        worker_count: int,
        image_load_worker_count: int = 1,
        load_image: Callable[[Path], LoadedT],
        infer: Callable[[ModelT, tuple[LoadedT, ...]], ResultT],
        on_success: Callable[[InferenceJob, ResultT], None],
        on_failure: Callable[[InferenceJob, InferenceFailure], None],
        on_timing: Callable[[InferenceJob, InferenceTiming], None] | None = None,
        on_canceled: Callable[[InferenceJob, str], None] | None = None,
        serialize_model_access: bool = True,
    ) -> None:
        if worker_count < 1:
            raise ValueError("worker_count must be positive")
        if image_load_worker_count < 1:
            raise ValueError("image_load_worker_count must be positive")
        self._queue = queue
        self._model = model
        self._worker_count = worker_count
        self._image_load_worker_count = image_load_worker_count
        self._load_image = load_image
        self._infer = infer
        self._on_success = on_success
        self._on_failure = on_failure
        self._on_timing = on_timing or (lambda _job, _timing: None)
        self._on_canceled = on_canceled or (lambda _job, _stage: None)
        self._model_lock = threading.Lock() if serialize_model_access else None
        self._threads: list[threading.Thread] = []
        self._image_load_executor: ThreadPoolExecutor | None = None
        self._sweeper_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._active_guard = threading.Lock()
        self._active_jobs: dict[int, InferenceJob] = {}
        self.soft_shutdown_timeout_exceeded = False

    def start(self) -> None:
        if self._threads:
            raise RuntimeError("worker pool already started")
        self._stop_event.clear()
        if self._image_load_worker_count > 1:
            self._image_load_executor = ThreadPoolExecutor(
                max_workers=self._image_load_worker_count,
                thread_name_prefix="image-decode",
            )
        self._threads = [
            threading.Thread(target=self._run, name=f"inference-worker-{index}", daemon=True)
            for index in range(self._worker_count)
        ]
        for thread in self._threads:
            thread.start()
        self._sweeper_thread = threading.Thread(
            target=self._sweep_expired,
            name="inference-deadline-sweeper",
            daemon=True,
        )
        self._sweeper_thread.start()

    def stop(self, *, soft_timeout_seconds: float = 3.0) -> None:
        """대기 job은 취소하고 이미 시작된 forward는 끝까지 기다립니다."""

        if soft_timeout_seconds < 0:
            raise ValueError("soft_timeout_seconds must not be negative")
        self._stop_event.set()
        for job in self._queue.cancel_all_waiting():
            self._on_canceled(job, "SHUTDOWN_QUEUE")
        self._queue.close()
        soft_deadline = time.monotonic() + soft_timeout_seconds
        for thread in self._threads:
            thread.join(timeout=max(0.0, soft_deadline - time.monotonic()))
        alive = [thread for thread in self._threads if thread.is_alive()]
        self.soft_shutdown_timeout_exceeded = bool(alive)
        # 승인 정책: soft timeout 뒤에도 active PyTorch forward를 강제 종료하지 않습니다.
        for thread in alive:
            thread.join()
        self._threads.clear()
        if self._sweeper_thread is not None:
            self._sweeper_thread.join(timeout=5.0)
            self._sweeper_thread = None
        if self._image_load_executor is not None:
            self._image_load_executor.shutdown(wait=True, cancel_futures=True)
            self._image_load_executor = None

    def _sweep_expired(self) -> None:
        while not self._stop_event.wait(0.05):
            for job in self._queue.discard_expired():
                self._fail(
                    job,
                    InferenceFailureKind.TIMEOUT,
                    "queue total inference timeout",
                )

    def _fail(
        self,
        job: InferenceJob,
        kind: InferenceFailureKind,
        reason: str,
    ) -> None:
        self._on_failure(job, InferenceFailure(kind=kind, reason=reason))

    def _run(self) -> None:
        while True:
            job = self._queue.get()
            if job is None:
                return
            worker_id = threading.get_ident()
            with self._active_guard:
                self._active_jobs[worker_id] = job
            try:
                self._execute_job(job)
            finally:
                with self._active_guard:
                    self._active_jobs.pop(worker_id, None)

    def has_active_job(self, product_id: str, station_id: int | None = None) -> bool:
        with self._active_guard:
            return any(
                job.product_id == product_id
                and (station_id is None or job.station_id == station_id)
                for job in self._active_jobs.values()
            )

    def _execute_job(self, job: InferenceJob) -> None:
        if self._queue.is_job_canceled(job):
            self._on_canceled(job, "DEQUEUED_BEFORE_LOAD")
            return
        if job.expired():
            self._fail(
                job,
                InferenceFailureKind.TIMEOUT,
                "queue total inference timeout",
            )
            return
        load_started_ns = time.monotonic_ns()
        try:
            loaded = self._load_job_images(job)
        except InferenceDeadlineExceeded:
            self._fail(
                job,
                InferenceFailureKind.TIMEOUT,
                "queue total inference timeout",
            )
            return
        except Exception as exc:
            failure_kind = (
                InferenceFailureKind.PREPROCESSING
                if bool(getattr(exc, "is_preprocessing_failure", False))
                else InferenceFailureKind.FILE_READ
            )
            self._fail(
                job,
                failure_kind,
                (
                    f"preprocessing failed: {type(exc).__name__}"
                    if failure_kind == InferenceFailureKind.PREPROCESSING
                    else f"image read failed after retry: {type(exc).__name__}"
                ),
            )
            return
        load_completed_ns = time.monotonic_ns()
        if self._queue.is_job_canceled(job):
            self._on_canceled(job, "LOADED_BEFORE_FORWARD")
            return
        if job.expired():
            self._fail(
                job,
                InferenceFailureKind.TIMEOUT,
                "queue total inference timeout",
            )
            return
        forward_started_ns = time.monotonic_ns()
        try:
            result = self._infer_once(job, loaded)
        except InferenceDeadlineExceeded:
            self._fail(
                job,
                InferenceFailureKind.TIMEOUT,
                "queue total inference timeout",
            )
            return
        except Exception as exc:
            failure_kind = (
                InferenceFailureKind.CUDA_OOM
                if "outofmemory" in type(exc).__name__.lower()
                or "cuda out of memory" in str(exc).lower()
                else InferenceFailureKind.MODEL
            )
            self._fail(
                job,
                failure_kind,
                f"inference failed: {type(exc).__name__}",
            )
            return
        completed_ns = time.monotonic_ns()
        timing = InferenceTiming(
            load_ms=(load_completed_ns - load_started_ns) / 1_000_000,
            model_forward_ms=(completed_ns - forward_started_ns) / 1_000_000,
            enqueue_to_terminal_ms=(completed_ns - job.enqueued_monotonic_ns)
            / 1_000_000,
            completed_monotonic_ns=completed_ns,
        )
        self._on_timing(job, timing)
        if self._queue.is_job_canceled(job):
            self._on_canceled(job, "DISCARDED_AFTER_FORWARD")
        elif job.expired():
            self._fail(
                job,
                InferenceFailureKind.TIMEOUT,
                "queue total inference timeout",
            )
        else:
            self._on_success(job, result)

    def _load_job_images(self, job: InferenceJob) -> tuple[LoadedT, ...]:
        paths = tuple(Path(path) for path in job.image_paths)
        if len(paths) < 2 or self._image_load_executor is None:
            return tuple(self._load_with_one_retry(job, path) for path in paths)

        futures: list[Future[LoadedT]] = [
            self._image_load_executor.submit(self._load_with_one_retry, job, path)
            for path in paths
        ]
        try:
            # 제출 순서대로 result를 모아 artifact의 view 순서를 보존합니다.
            return tuple(future.result() for future in futures)
        except Exception:
            for future in futures:
                future.cancel()
            raise

    def _load_with_one_retry(self, job: InferenceJob, path: Path) -> LoadedT:
        try:
            return self._load_image(path)
        except Exception as exc:
            # 같은 canonical image를 다시 읽어도 바뀌지 않는 전처리 계약 실패는
            # retry 대상이 아닙니다. I/O의 일시 오류만 한 번 재시도합니다.
            if bool(getattr(exc, "is_preprocessing_failure", False)):
                raise
            if job.expired():
                raise InferenceDeadlineExceeded
            return self._load_image(path)

    def _infer_once(
        self, job: InferenceJob, images: tuple[LoadedT, ...]
    ) -> ResultT:
        def execute() -> ResultT:
            if self._model_lock is None:
                return self._infer(self._model, images)
            with self._model_lock:
                return self._infer(self._model, images)

        result = execute()
        if job.expired():
            raise InferenceDeadlineExceeded
        return result
