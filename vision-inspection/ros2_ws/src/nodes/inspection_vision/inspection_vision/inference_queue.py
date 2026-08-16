"""파일 경로 기반 bounded FIFO와 공유 모델 worker pool."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Generic, TypeVar

ModelT = TypeVar("ModelT")
LoadedT = TypeVar("LoadedT")
ResultT = TypeVar("ResultT")


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
        self._condition = threading.Condition()
        self._closed = False

    @property
    def depth(self) -> int:
        with self._condition:
            return len(self._items)

    def try_enqueue(self, job: InferenceJob) -> bool:
        with self._condition:
            if self._closed or job.product_id in self._locked_products:
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
            if len(self._items) < self.capacity:
                return True
            self._condition.wait(timeout_seconds)
            return len(self._items) < self.capacity

    def get(self) -> InferenceJob | None:
        with self._condition:
            while not self._items and not self._closed:
                self._condition.wait()
            if self._closed:
                return None
            job = self._items.popleft()
            self._condition.notify_all()
            return job

    def lock_product(self, product_id: str) -> int:
        """Sensor3 ProductResultLocked 이후 대기 작업을 제거하고 late 결과를 막습니다."""

        with self._condition:
            self._locked_products.add(product_id)
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
        load_image: Callable[[Path], LoadedT],
        infer: Callable[[ModelT, tuple[LoadedT, ...]], ResultT],
        on_success: Callable[[InferenceJob, ResultT], None],
        on_failure: Callable[[InferenceJob, str], None],
        serialize_model_access: bool = True,
    ) -> None:
        if worker_count < 1:
            raise ValueError("worker_count must be positive")
        self._queue = queue
        self._model = model
        self._worker_count = worker_count
        self._load_image = load_image
        self._infer = infer
        self._on_success = on_success
        self._on_failure = on_failure
        self._model_lock = threading.Lock() if serialize_model_access else None
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        if self._threads:
            raise RuntimeError("worker pool already started")
        self._threads = [
            threading.Thread(target=self._run, name=f"inference-worker-{index}", daemon=True)
            for index in range(self._worker_count)
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self._queue.close()
        for thread in self._threads:
            thread.join(timeout=5.0)
        self._threads.clear()

    def _run(self) -> None:
        while True:
            job = self._queue.get()
            if job is None:
                return
            if self._queue.is_product_locked(job.product_id):
                continue
            if job.expired():
                self._on_failure(job, "queue total inference timeout")
                continue
            try:
                loaded = tuple(self._load_with_one_retry(Path(path)) for path in job.image_paths)
            except Exception as exc:
                self._on_failure(job, f"image read failed after retry: {type(exc).__name__}")
                continue
            try:
                result = self._infer_with_one_retry(loaded)
            except Exception as exc:
                self._on_failure(job, f"inference failed after retry: {type(exc).__name__}")
                continue
            if not self._queue.is_product_locked(job.product_id):
                self._on_success(job, result)

    def _load_with_one_retry(self, path: Path) -> LoadedT:
        try:
            return self._load_image(path)
        except Exception:
            return self._load_image(path)

    def _infer_with_one_retry(self, images: tuple[LoadedT, ...]) -> ResultT:
        def execute() -> ResultT:
            if self._model_lock is None:
                return self._infer(self._model, images)
            with self._model_lock:
                return self._infer(self._model, images)

        try:
            return execute()
        except Exception:
            return execute()
