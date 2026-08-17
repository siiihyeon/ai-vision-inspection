"""추론 queue 작업의 enqueue 완료 여부와 재시작 복구를 보존하는 SQLite journal."""

from __future__ import annotations

import hashlib
import json
import sqlite3
import threading
import time
from dataclasses import asdict
from enum import StrEnum
from pathlib import Path

from .inference_queue import InferenceJob


class JournalStatus(StrEnum):
    PENDING = "pending"
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"
    LOCKED = "locked"


class InferenceJournal:
    """commit된 enqueue만 복원하며 ENQUEUE_BLOCKED 재시작 항목은 폐기합니다."""

    def __init__(self, database_path: Path) -> None:
        if not database_path.is_absolute():
            raise ValueError("queue journal path must be absolute")
        database_path.parent.mkdir(parents=True, exist_ok=True)
        self.database_path = database_path
        self._connection = sqlite3.connect(database_path, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        with self._connection:
            self._connection.execute("PRAGMA journal_mode=WAL")
            self._connection.execute("PRAGMA synchronous=FULL")
            self._connection.execute(
                """
                CREATE TABLE IF NOT EXISTS inference_queue_journal (
                    inference_job_id TEXT PRIMARY KEY,
                    frame_batch_id TEXT NOT NULL UNIQUE,
                    product_id TEXT NOT NULL,
                    payload_json TEXT NOT NULL,
                    payload_digest TEXT NOT NULL,
                    status TEXT NOT NULL,
                    enqueue_committed INTEGER NOT NULL DEFAULT 0,
                    failure_reason TEXT NOT NULL DEFAULT '',
                    created_wall_time_ns INTEGER NOT NULL,
                    updated_wall_time_ns INTEGER NOT NULL
                )
                """
            )
            self._connection.execute(
                """
                CREATE TABLE IF NOT EXISTS locked_products (
                    product_id TEXT PRIMARY KEY,
                    locked_wall_time_ns INTEGER NOT NULL
                )
                """
            )

    def prepare(self, job: InferenceJob) -> None:
        payload_json, digest = self._serialize_job(job)
        now = time.time_ns()
        with self._lock, self._connection:
            if self.is_product_locked(job.product_id):
                raise ValueError("cannot prepare inference for a locked product")
            existing = self._connection.execute(
                """
                SELECT payload_digest, status FROM inference_queue_journal
                WHERE inference_job_id=? OR frame_batch_id=?
                """,
                (job.inference_job_id, job.frame_batch_id),
            ).fetchone()
            if existing is not None:
                if existing["payload_digest"] != digest:
                    raise ValueError("inference identity has a conflicting payload")
                if existing["status"] in {
                    JournalStatus.DONE,
                    JournalStatus.FAILED,
                    JournalStatus.LOCKED,
                }:
                    raise ValueError(
                        f"terminal inference job cannot be prepared: {existing['status']}"
                    )
                return
            self._connection.execute(
                """
                INSERT INTO inference_queue_journal (
                    inference_job_id, frame_batch_id, product_id,
                    payload_json, payload_digest, status, enqueue_committed,
                    created_wall_time_ns, updated_wall_time_ns
                ) VALUES (?, ?, ?, ?, ?, ?, 0, ?, ?)
                """,
                (
                    job.inference_job_id,
                    job.frame_batch_id,
                    job.product_id,
                    payload_json,
                    digest,
                    JournalStatus.PENDING,
                    now,
                    now,
                ),
            )

    def mark_enqueued(self, job_or_id: InferenceJob | str) -> None:
        """Queue 삽입 순간의 timestamp와 enqueue commit을 한 transaction에 기록합니다."""

        inference_job_id = (
            job_or_id.inference_job_id
            if isinstance(job_or_id, InferenceJob)
            else job_or_id
        )
        with self._lock, self._connection:
            row = self._connection.execute(
                """
                SELECT payload_json, payload_digest, status, enqueue_committed
                FROM inference_queue_journal WHERE inference_job_id=?
                """,
                (inference_job_id,),
            ).fetchone()
            if row is None:
                raise KeyError(f"unknown inference job: {inference_job_id}")
            if JournalStatus(str(row["status"])) != JournalStatus.PENDING:
                raise ValueError("only a pending inference job can be enqueued")
            if int(row["enqueue_committed"]):
                return
            payload_json = str(row["payload_json"])
            payload_digest = hashlib.sha256(payload_json.encode("utf-8")).hexdigest()
            if payload_digest != str(row["payload_digest"]):
                raise ValueError("prepared inference journal payload digest mismatches")
            if isinstance(job_or_id, InferenceJob):
                prepared = self._deserialize_job(payload_json)
                comparable = InferenceJob(
                    inference_job_id=job_or_id.inference_job_id,
                    product_id=job_or_id.product_id,
                    fifo_sequence=job_or_id.fifo_sequence,
                    station_id=job_or_id.station_id,
                    capture_id=job_or_id.capture_id,
                    frame_batch_id=job_or_id.frame_batch_id,
                    image_paths=job_or_id.image_paths,
                    enqueued_monotonic_ns=prepared.enqueued_monotonic_ns,
                    enqueued_wall_time_ns=prepared.enqueued_wall_time_ns,
                    queue_total_timeout_ms=job_or_id.queue_total_timeout_ms,
                    result_revision=job_or_id.result_revision,
                )
                if comparable != prepared:
                    raise ValueError("enqueue timestamp update changed immutable job fields")
                payload_json, payload_digest = self._serialize_job(job_or_id)
            self._connection.execute(
                """
                UPDATE inference_queue_journal
                SET payload_json=?, payload_digest=?, enqueue_committed=1,
                    updated_wall_time_ns=? WHERE inference_job_id=?
                """,
                (payload_json, payload_digest, time.time_ns(), inference_job_id),
            )

    def mark_running(self, inference_job_id: str) -> None:
        self._transition(
            inference_job_id,
            allowed={JournalStatus.PENDING, JournalStatus.RUNNING},
            status=JournalStatus.RUNNING,
            require_committed=True,
        )

    def mark_done(self, inference_job_id: str) -> None:
        self._transition(
            inference_job_id,
            allowed={JournalStatus.RUNNING},
            status=JournalStatus.DONE,
            require_committed=True,
        )

    def mark_failed(self, inference_job_id: str, reason: str) -> None:
        self._transition(
            inference_job_id,
            allowed={JournalStatus.PENDING, JournalStatus.RUNNING},
            status=JournalStatus.FAILED,
            failure_reason=reason,
        )

    def lock_product(self, product_id: str) -> tuple[str, ...]:
        now = time.time_ns()
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT OR IGNORE INTO locked_products(product_id, locked_wall_time_ns)
                VALUES (?, ?)
                """,
                (product_id, now),
            )
            rows = self._connection.execute(
                """
                SELECT inference_job_id FROM inference_queue_journal
                WHERE product_id=? AND status IN (?, ?)
                """,
                (product_id, JournalStatus.PENDING, JournalStatus.RUNNING),
            ).fetchall()
            self._connection.execute(
                """
                UPDATE inference_queue_journal
                SET status=?, failure_reason='product result locked',
                    updated_wall_time_ns=?
                WHERE product_id=? AND status IN (?, ?)
                """,
                (
                    JournalStatus.LOCKED,
                    now,
                    product_id,
                    JournalStatus.PENDING,
                    JournalStatus.RUNNING,
                ),
            )
        return tuple(str(row["inference_job_id"]) for row in rows)

    def is_product_locked(self, product_id: str) -> bool:
        with self._lock:
            return (
                self._connection.execute(
                    "SELECT 1 FROM locked_products WHERE product_id=?", (product_id,)
                ).fetchone()
                is not None
            )

    def recoverable_jobs(self) -> tuple[InferenceJob, ...]:
        """enqueue commit된 pending/running만 반환하고 나머지는 명시적으로 실패 처리."""

        now = time.time_ns()
        recovered: list[InferenceJob] = []
        with self._lock, self._connection:
            self._connection.execute(
                """
                UPDATE inference_queue_journal
                SET status=?, failure_reason='process restarted before queue enqueue',
                    updated_wall_time_ns=?
                WHERE status=? AND enqueue_committed=0
                """,
                (JournalStatus.FAILED, now, JournalStatus.PENDING),
            )
            rows = self._connection.execute(
                """
                SELECT inference_job_id, payload_json, payload_digest
                FROM inference_queue_journal
                WHERE status IN (?, ?) AND enqueue_committed=1
                ORDER BY created_wall_time_ns, inference_job_id
                """,
                (JournalStatus.PENDING, JournalStatus.RUNNING),
            ).fetchall()
            for row in rows:
                payload_json = str(row["payload_json"])
                if hashlib.sha256(payload_json.encode("utf-8")).hexdigest() != str(
                    row["payload_digest"]
                ):
                    self._connection.execute(
                        """
                        UPDATE inference_queue_journal
                        SET status=?, failure_reason='journal payload digest mismatch',
                            updated_wall_time_ns=? WHERE inference_job_id=?
                        """,
                        (JournalStatus.FAILED, now, row["inference_job_id"]),
                    )
                    continue
                try:
                    job = self._deserialize_job(payload_json)
                except (TypeError, ValueError, KeyError, json.JSONDecodeError):
                    self._connection.execute(
                        """
                        UPDATE inference_queue_journal
                        SET status=?, failure_reason='journal payload is invalid',
                            updated_wall_time_ns=? WHERE inference_job_id=?
                        """,
                        (JournalStatus.FAILED, now, row["inference_job_id"]),
                    )
                    continue
                if self.is_product_locked(job.product_id):
                    self._connection.execute(
                        """
                        UPDATE inference_queue_journal
                        SET status=?, failure_reason='product result locked',
                            updated_wall_time_ns=? WHERE inference_job_id=?
                        """,
                        (JournalStatus.LOCKED, now, job.inference_job_id),
                    )
                elif job.expired():
                    self._connection.execute(
                        """
                        UPDATE inference_queue_journal
                        SET status=?, failure_reason='queue total inference timeout',
                            updated_wall_time_ns=? WHERE inference_job_id=?
                        """,
                        (JournalStatus.FAILED, now, job.inference_job_id),
                    )
                elif all(Path(path).is_file() for path in job.image_paths):
                    recovered.append(job)
                    self._connection.execute(
                        """
                        UPDATE inference_queue_journal
                        SET status=?, updated_wall_time_ns=? WHERE inference_job_id=?
                        """,
                        (JournalStatus.PENDING, now, job.inference_job_id),
                    )
                else:
                    self._connection.execute(
                        """
                        UPDATE inference_queue_journal
                        SET status=?, failure_reason='saved image path is missing on restart',
                            updated_wall_time_ns=? WHERE inference_job_id=?
                        """,
                        (JournalStatus.FAILED, now, job.inference_job_id),
                    )
        return tuple(recovered)

    def snapshot(self) -> dict[str, int]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT status, COUNT(*) AS count FROM inference_queue_journal
                GROUP BY status
                """
            ).fetchall()
        return {str(row["status"]): int(row["count"]) for row in rows}

    def close(self) -> None:
        with self._lock:
            self._connection.close()

    def _transition(
        self,
        inference_job_id: str,
        *,
        allowed: set[JournalStatus],
        status: JournalStatus,
        enqueue_committed: int | None = None,
        failure_reason: str = "",
        require_committed: bool = False,
    ) -> None:
        with self._lock, self._connection:
            row = self._connection.execute(
                """
                SELECT status, enqueue_committed FROM inference_queue_journal
                WHERE inference_job_id=?
                """,
                (inference_job_id,),
            ).fetchone()
            if row is None:
                raise KeyError(f"unknown inference job: {inference_job_id}")
            current = JournalStatus(str(row["status"]))
            if current == status and (
                enqueue_committed is None
                or int(row["enqueue_committed"]) == enqueue_committed
            ):
                return
            if current not in allowed:
                raise ValueError(f"invalid journal transition {current}->{status}")
            if require_committed and not int(row["enqueue_committed"]):
                raise ValueError("cannot run an inference job before enqueue commit")
            committed = (
                int(row["enqueue_committed"])
                if enqueue_committed is None
                else enqueue_committed
            )
            self._connection.execute(
                """
                UPDATE inference_queue_journal
                SET status=?, enqueue_committed=?, failure_reason=?,
                    updated_wall_time_ns=? WHERE inference_job_id=?
                """,
                (
                    status,
                    committed,
                    failure_reason,
                    time.time_ns(),
                    inference_job_id,
                ),
            )

    @staticmethod
    def _serialize_job(job: InferenceJob) -> tuple[str, str]:
        payload = asdict(job)
        payload["image_paths"] = list(job.image_paths)
        text = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return text, hashlib.sha256(text.encode("utf-8")).hexdigest()

    @staticmethod
    def _deserialize_job(payload_json: str) -> InferenceJob:
        payload = json.loads(payload_json)
        payload["image_paths"] = tuple(payload["image_paths"])
        return InferenceJob(**payload)
