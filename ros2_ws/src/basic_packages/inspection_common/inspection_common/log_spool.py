"""Log ACK 전까지 producer 이벤트를 보존하는 로컬 SQLite spool."""

from __future__ import annotations

import sqlite3
import threading
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class SpoolRecord:
    log_id: str
    revision: int
    payload_json: str
    payload_digest: str


class DurableLogSpool:
    def __init__(self, database_path: Path) -> None:
        database_path.parent.mkdir(parents=True, exist_ok=True)
        self._connection = sqlite3.connect(database_path, check_same_thread=False)
        self._lock = threading.Lock()
        with self._connection:
            self._connection.execute("PRAGMA journal_mode=WAL")
            self._connection.execute(
                """
                CREATE TABLE IF NOT EXISTS outbound_log_spool (
                    log_id TEXT NOT NULL,
                    revision INTEGER NOT NULL,
                    payload_json TEXT NOT NULL,
                    payload_digest TEXT NOT NULL,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    PRIMARY KEY (log_id, revision)
                )
                """
            )

    def enqueue(self, record: SpoolRecord) -> None:
        with self._lock, self._connection:
            existing = self._connection.execute(
                "SELECT payload_digest FROM outbound_log_spool WHERE log_id=? AND revision=?",
                (record.log_id, record.revision),
            ).fetchone()
            if existing is not None and existing[0] != record.payload_digest:
                raise ValueError("same log_id/revision has a conflicting digest")
            self._connection.execute(
                """
                INSERT OR IGNORE INTO outbound_log_spool
                    (log_id, revision, payload_json, payload_digest)
                VALUES (?, ?, ?, ?)
                """,
                (
                    record.log_id,
                    record.revision,
                    record.payload_json,
                    record.payload_digest,
                ),
            )

    def pending(self, limit: int = 100) -> list[SpoolRecord]:
        if limit < 1:
            raise ValueError("limit must be positive")
        with self._lock:
            rows = self._connection.execute(
                """
                SELECT log_id, revision, payload_json, payload_digest
                FROM outbound_log_spool
                ORDER BY created_at, log_id, revision LIMIT ?
                """,
                (limit,),
            ).fetchall()
        return [SpoolRecord(*row) for row in rows]

    def acknowledge(self, identities: list[tuple[str, int]]) -> None:
        with self._lock, self._connection:
            self._connection.executemany(
                "DELETE FROM outbound_log_spool WHERE log_id=? AND revision=?",
                identities,
            )

    def close(self) -> None:
        with self._lock:
            self._connection.close()
