"""LogNode SQLite commit 경계와 v2 projection schema."""

from __future__ import annotations

import json
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

CREATE TABLE IF NOT EXISTS vision_station_terminals (
    session_id TEXT NOT NULL,
    inference_job_id TEXT NOT NULL,
    revision INTEGER NOT NULL,
    terminal_kind TEXT NOT NULL,
    product_id TEXT NOT NULL,
    fifo_sequence INTEGER NOT NULL,
    station_id INTEGER NOT NULL,
    capture_id TEXT NOT NULL,
    frame_batch_id TEXT NOT NULL,
    verdict INTEGER,
    score REAL,
    model_version TEXT NOT NULL,
    model_sha256 TEXT NOT NULL,
    config_fingerprint TEXT NOT NULL,
    error_code INTEGER,
    reason TEXT NOT NULL,
    model_forward_ms REAL,
    enqueue_to_result_ms REAL,
    capture_completed_monotonic_ns INTEGER,
    completed_monotonic_ns INTEGER,
    payload_json TEXT NOT NULL,
    PRIMARY KEY (session_id, inference_job_id, revision, terminal_kind)
);

CREATE INDEX IF NOT EXISTS idx_vision_terminal_replay
ON vision_station_terminals(session_id, fifo_sequence, station_id);

CREATE TABLE IF NOT EXISTS product_terminal_results (
    session_id TEXT NOT NULL,
    product_id TEXT NOT NULL,
    fifo_sequence INTEGER NOT NULL,
    final_verdict TEXT NOT NULL,
    reason TEXT NOT NULL,
    PRIMARY KEY (session_id, product_id)
);

CREATE TABLE IF NOT EXISTS vision_capture_timings (
    session_id TEXT NOT NULL,
    capture_id TEXT NOT NULL,
    station_id INTEGER NOT NULL,
    capture_and_save_ms REAL NOT NULL,
    validation_ms REAL NOT NULL,
    timeout_candidate_ms REAL NOT NULL,
    PRIMARY KEY (session_id, capture_id)
);

CREATE TABLE IF NOT EXISTS vision_session_gpu_metrics (
    session_id TEXT PRIMARY KEY,
    normal_shutdown INTEGER NOT NULL,
    sample_count INTEGER NOT NULL,
    mean_gpu_utilization_pct REAL NOT NULL,
    mean_vram_used_mib REAL NOT NULL,
    peak_vram_used_mib REAL NOT NULL,
    total_vram_mib REAL NOT NULL,
    payload_json TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS canonical_image_artifacts (
    artifact_order INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL,
    product_id TEXT NOT NULL,
    station_id INTEGER NOT NULL,
    file_path TEXT NOT NULL UNIQUE
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


@dataclass(frozen=True, slots=True)
class ReplayStationTerminal:
    terminal_kind: str
    product_id: str
    fifo_sequence: int
    station_id: int
    capture_id: str
    frame_batch_id: str
    inference_job_id: str
    revision: int
    verdict: int | None
    score: float | None
    model_version: str
    error_code: int | None
    reason: str


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
            self._project_event(event)

    def _project_event(self, event: StoredLogEvent) -> None:
        """원본 event와 같은 transaction에서 승인된 조회 projection을 갱신합니다."""

        envelope = json.loads(event.payload_json)
        payload = envelope.get("payload", {})
        if not isinstance(payload, dict):
            return
        session_id = str(envelope.get("session_id", ""))
        if not session_id:
            return
        if event.event_type in {
            "VISION_STATION_RESULT_DURABLE",
            "VISION_STATION_FAILURE_DURABLE",
        }:
            terminal_kind = (
                "RESULT"
                if event.event_type == "VISION_STATION_RESULT_DURABLE"
                else "FAILURE"
            )
            self._connection.execute(
                """
                INSERT OR IGNORE INTO vision_station_terminals (
                    session_id, inference_job_id, revision, terminal_kind,
                    product_id, fifo_sequence, station_id, capture_id,
                    frame_batch_id, verdict, score, model_version, model_sha256,
                    config_fingerprint,
                    error_code, reason, model_forward_ms, enqueue_to_result_ms,
                    capture_completed_monotonic_ns, completed_monotonic_ns,
                    payload_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    session_id,
                    str(payload.get("inference_job_id", "")),
                    int(payload.get("result_revision", 1)),
                    terminal_kind,
                    str(payload.get("product_id", event.product_id)),
                    int(payload.get("fifo_sequence", 0)),
                    int(payload.get("station_id", 0)),
                    str(payload.get("capture_id", "")),
                    str(payload.get("frame_batch_id", "")),
                    self._optional_int(payload.get("verdict")),
                    self._optional_float(payload.get("score")),
                    str(payload.get("model_version", "")),
                    str(payload.get("model_sha256", "")),
                    str(payload.get("config_fingerprint", "")),
                    self._optional_int(payload.get("error_code")),
                    str(payload.get("reason", "")),
                    self._optional_float(payload.get("model_forward_ms")),
                    self._optional_float(payload.get("enqueue_to_result_ms")),
                    self._optional_int(
                        payload.get("capture_completed_monotonic_ns")
                    ),
                    self._optional_int(payload.get("completed_monotonic_ns")),
                    event.payload_json,
                ),
            )
            for image_path in payload.get("image_paths", []):
                if image_path:
                    self._connection.execute(
                        """
                        INSERT OR IGNORE INTO canonical_image_artifacts
                            (session_id, product_id, station_id, file_path)
                        VALUES (?, ?, ?, ?)
                        """,
                        (
                            session_id,
                            str(payload.get("product_id", event.product_id)),
                            int(payload.get("station_id", 0)),
                            str(image_path),
                        ),
                    )
        elif event.event_type == "VISION_CAPTURE_DISCARDED":
            for image_path in payload.get("image_paths", []):
                if image_path:
                    self._connection.execute(
                        """
                        INSERT OR IGNORE INTO canonical_image_artifacts
                            (session_id, product_id, station_id, file_path)
                        VALUES (?, ?, ?, ?)
                        """,
                        (
                            session_id,
                            str(payload.get("product_id", event.product_id)),
                            int(payload.get("station_id", 0)),
                            str(image_path),
                        ),
                    )
        elif event.event_type == "PRODUCT_RESULT_LOCKED":
            self._connection.execute(
                """
                INSERT INTO product_terminal_results
                    (session_id, product_id, fifo_sequence, final_verdict, reason)
                VALUES (?, ?, ?, ?, ?)
                ON CONFLICT(session_id, product_id) DO UPDATE SET
                    fifo_sequence=excluded.fifo_sequence,
                    final_verdict=excluded.final_verdict,
                    reason=excluded.reason
                """,
                (
                    session_id,
                    event.product_id,
                    int(payload.get("fifo_sequence", 0)),
                    str(payload.get("verdict", "")),
                    str(payload.get("reason", "")),
                ),
            )
        elif event.event_type == "VISION_CAPTURE_TIMING":
            self._connection.execute(
                """
                INSERT OR REPLACE INTO vision_capture_timings
                    (session_id, capture_id, station_id, capture_and_save_ms,
                     validation_ms, timeout_candidate_ms)
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    session_id,
                    str(payload.get("capture_id", "")),
                    int(payload.get("station_id", 0)),
                    float(payload.get("capture_and_save_ms", 0.0)),
                    float(payload.get("validation_ms", 0.0)),
                    float(payload.get("capture_timeout_candidate_ms", 0.0)),
                ),
            )
        elif event.event_type in {
            "VISION_SESSION_METRICS",
            "VISION_GPU_METRICS_SNAPSHOT",
        }:
            self._connection.execute(
                """
                INSERT INTO vision_session_gpu_metrics
                    (session_id, normal_shutdown, sample_count,
                     mean_gpu_utilization_pct, mean_vram_used_mib,
                     peak_vram_used_mib, total_vram_mib, payload_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(session_id) DO UPDATE SET
                    normal_shutdown=excluded.normal_shutdown,
                    sample_count=excluded.sample_count,
                    mean_gpu_utilization_pct=excluded.mean_gpu_utilization_pct,
                    mean_vram_used_mib=excluded.mean_vram_used_mib,
                    peak_vram_used_mib=excluded.peak_vram_used_mib,
                    total_vram_mib=excluded.total_vram_mib,
                    payload_json=excluded.payload_json
                WHERE excluded.sample_count >= vision_session_gpu_metrics.sample_count
                  AND (vision_session_gpu_metrics.normal_shutdown=0
                       OR excluded.normal_shutdown=1)
                """,
                (
                    session_id,
                    int(bool(payload.get("normal_shutdown", False))),
                    int(payload.get("gpu_sample_count", 0)),
                    float(payload.get("mean_gpu_utilization_pct", 0.0)),
                    float(payload.get("mean_vram_used_mib", 0.0)),
                    float(payload.get("peak_vram_used_mib", 0.0)),
                    float(payload.get("total_vram_mib", 0.0)),
                    event.payload_json,
                ),
            )

    @staticmethod
    def _optional_int(value) -> int | None:
        return None if value is None else int(value)

    @staticmethod
    def _optional_float(value) -> float | None:
        return None if value is None else float(value)

    def replay_station_terminals(
        self, session_id: str, limit: int, offset: int = 0
    ) -> tuple[list[ReplayStationTerminal], bool]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT terminal_kind, product_id, fifo_sequence, station_id,
                       capture_id, frame_batch_id, inference_job_id, revision,
                       verdict, score, model_version, error_code, reason
                FROM vision_station_terminals
                WHERE session_id=?
                ORDER BY fifo_sequence, station_id, revision
                LIMIT ? OFFSET ?
                """,
                (session_id, limit + 1, offset),
            ).fetchall()
        has_more = len(rows) > limit
        return [ReplayStationTerminal(*row) for row in rows[:limit]], has_more

    def session_report_rows(self, session_id: str) -> list[tuple]:
        with self._lock:
            return self._connection.execute(
                """
                WITH ranked AS (
                    SELECT *, ROW_NUMBER() OVER (
                        PARTITION BY session_id, product_id, station_id
                        ORDER BY revision DESC,
                                 CASE terminal_kind WHEN 'FAILURE' THEN 0 ELSE 1 END
                    ) AS terminal_rank
                    FROM vision_station_terminals
                    WHERE session_id=?
                )
                SELECT p.fifo_sequence, p.product_id, p.final_verdict, p.reason,
                       a.terminal_kind, a.verdict, a.error_code,
                       a.model_forward_ms, a.enqueue_to_result_ms,
                       b.terminal_kind, b.verdict, b.error_code,
                       b.model_forward_ms, b.enqueue_to_result_ms,
                       CASE
                         WHEN a.capture_completed_monotonic_ns IS NOT NULL
                          AND COALESCE(b.completed_monotonic_ns,
                                       a.completed_monotonic_ns) IS NOT NULL
                         THEN (COALESCE(b.completed_monotonic_ns,
                                        a.completed_monotonic_ns)
                               - a.capture_completed_monotonic_ns) / 1000000.0
                         ELSE NULL
                       END AS product_total_ms
                FROM product_terminal_results p
                LEFT JOIN ranked a
                  ON a.session_id=p.session_id AND a.product_id=p.product_id
                 AND a.station_id=1 AND a.terminal_rank=1
                LEFT JOIN ranked b
                  ON b.session_id=p.session_id AND b.product_id=p.product_id
                 AND b.station_id=2 AND b.terminal_rank=1
                WHERE p.session_id=?
                ORDER BY p.fifo_sequence
                """,
                (session_id, session_id),
            ).fetchall()

    def station_timeout_samples(
        self, session_id: str, station_id: int
    ) -> tuple[list[float], str, str, str]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT enqueue_to_result_ms, model_version, model_sha256,
                       config_fingerprint
                FROM vision_station_terminals
                WHERE session_id=? AND station_id=? AND terminal_kind='RESULT'
                  AND enqueue_to_result_ms IS NOT NULL
                ORDER BY fifo_sequence
                """,
                (session_id, station_id),
            ).fetchall()
        samples = [float(row[0]) for row in rows]
        versions = {str(row[1]) for row in rows}
        digests = {str(row[2]) for row in rows}
        fingerprints = {str(row[3]) for row in rows}
        return (
            samples,
            versions.pop() if len(versions) == 1 else "",
            digests.pop() if len(digests) == 1 else "",
            fingerprints.pop() if len(fingerprints) == 1 else "",
        )

    def capture_timeout_samples(self, session_id: str) -> list[float]:
        with self._lock:
            rows = self._connection.execute(
                "SELECT timeout_candidate_ms FROM vision_capture_timings WHERE session_id=?",
                (session_id,),
            ).fetchall()
        return [float(row[0]) for row in rows]

    def gpu_metrics(self, session_id: str) -> tuple | None:
        with self._lock:
            return self._connection.execute(
                """
                SELECT normal_shutdown, sample_count, mean_gpu_utilization_pct,
                       mean_vram_used_mib, peak_vram_used_mib, total_vram_mib
                FROM vision_session_gpu_metrics WHERE session_id=?
                """,
                (session_id,),
            ).fetchone()

    def retention_candidates(self, keep_latest: int) -> list[str]:
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT file_path FROM canonical_image_artifacts
                ORDER BY artifact_order DESC LIMIT -1 OFFSET ?
                """,
                (keep_latest,),
            ).fetchall()
        return [str(row[0]) for row in rows]

    def remove_image_records(self, paths: list[str]) -> None:
        if not paths:
            return
        with self._lock, self._connection:
            self._connection.executemany(
                "DELETE FROM canonical_image_artifacts WHERE file_path=?",
                [(path,) for path in paths],
            )

    def close(self) -> None:
        with self._lock:
            self._connection.close()
