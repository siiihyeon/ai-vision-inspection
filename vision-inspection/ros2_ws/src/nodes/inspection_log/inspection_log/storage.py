"""LogNode SQLite commit 경계와 v2 projection schema."""

from __future__ import annotations

import sqlite3
import threading
from dataclasses import dataclass
from pathlib import Path


SCHEMA_SQL = """
PRAGMA journal_mode=WAL;
PRAGMA foreign_keys=ON;

CREATE TABLE IF NOT EXISTS log_events (
    log_id TEXT NOT NULL,
    revision INTEGER NOT NULL,
    severity INTEGER NOT NULL,
    event_type TEXT NOT NULL,
    source_node TEXT NOT NULL,
    producer_instance_id TEXT NOT NULL,
    product_id TEXT NOT NULL,
    payload_json TEXT NOT NULL,
    payload_digest TEXT NOT NULL,
    occurred_at_ns INTEGER NOT NULL,
    committed_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    PRIMARY KEY (log_id, revision)
);

CREATE TABLE IF NOT EXISTS products_latest (
    product_id TEXT PRIMARY KEY,
    fifo_sequence INTEGER NOT NULL UNIQUE,
    final_verdict INTEGER,
    lock_reason TEXT,
    revision INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS frame_batch_attempts (
    capture_id TEXT NOT NULL,
    attempt INTEGER NOT NULL,
    product_id TEXT NOT NULL,
    station_id INTEGER NOT NULL,
    frame_batch_id TEXT,
    state TEXT NOT NULL,
    metadata_json TEXT NOT NULL,
    PRIMARY KEY (capture_id, attempt)
);

CREATE TABLE IF NOT EXISTS camera_result_revisions (
    frame_batch_id TEXT NOT NULL,
    camera_id TEXT NOT NULL,
    revision INTEGER NOT NULL,
    score REAL,
    verdict INTEGER,
    error_code INTEGER,
    payload_json TEXT NOT NULL,
    PRIMARY KEY (frame_batch_id, camera_id, revision)
);

CREATE TABLE IF NOT EXISTS inference_jobs_latest (
    inference_job_id TEXT PRIMARY KEY,
    frame_batch_id TEXT NOT NULL,
    product_id TEXT NOT NULL,
    fifo_sequence INTEGER NOT NULL,
    station_id INTEGER NOT NULL,
    state TEXT NOT NULL,
    revision INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS station_result_revisions (
    product_id TEXT NOT NULL,
    station_id INTEGER NOT NULL,
    revision INTEGER NOT NULL,
    verdict INTEGER,
    error_code INTEGER,
    payload_json TEXT NOT NULL,
    PRIMARY KEY (product_id, station_id, revision)
);

CREATE TABLE IF NOT EXISTS faults (
    fault_id TEXT NOT NULL,
    revision INTEGER NOT NULL,
    source_node TEXT NOT NULL,
    error_code INTEGER NOT NULL,
    reason TEXT NOT NULL,
    payload_json TEXT NOT NULL,
    PRIMARY KEY (fault_id, revision)
);

CREATE TABLE IF NOT EXISTS pending_projections (
    log_id TEXT NOT NULL,
    revision INTEGER NOT NULL,
    dependency_type TEXT NOT NULL,
    dependency_id TEXT NOT NULL,
    retry_count INTEGER NOT NULL DEFAULT 0,
    last_error TEXT NOT NULL,
    PRIMARY KEY (log_id, revision, dependency_type, dependency_id),
    FOREIGN KEY (log_id, revision) REFERENCES log_events(log_id, revision)
);
"""


@dataclass(frozen=True, slots=True)
class StoredLogEvent:
    log_id: str
    revision: int
    severity: int
    event_type: str
    source_node: str
    producer_instance_id: str
    product_id: str
    payload_json: str
    payload_digest: str
    occurred_at_ns: int


class LogRepository:
    """한 append 호출의 commit 성공 후에만 반환합니다."""

    def __init__(self, database_path: Path) -> None:
        database_path.parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(database_path, check_same_thread=False)
        self._lock = threading.Lock()
        with self._lock, self._connection:
            self._connection.executescript(SCHEMA_SQL)

    def append_event(self, event: StoredLogEvent) -> None:
        with self._lock, self._connection:
            existing = self._connection.execute(
                "SELECT payload_digest FROM log_events WHERE log_id=? AND revision=?",
                (event.log_id, event.revision),
            ).fetchone()
            if existing is not None:
                if existing[0] != event.payload_digest:
                    raise ValueError("same log_id/revision has a conflicting digest")
                return
            self._connection.execute(
                """
                INSERT INTO log_events (
                    log_id, revision, severity, event_type, source_node,
                    producer_instance_id, product_id, payload_json,
                    payload_digest, occurred_at_ns
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    event.log_id,
                    event.revision,
                    event.severity,
                    event.event_type,
                    event.source_node,
                    event.producer_instance_id,
                    event.product_id,
                    event.payload_json,
                    event.payload_digest,
                    event.occurred_at_ns,
                ),
            )

    def close(self) -> None:
        with self._lock:
            self._connection.close()
