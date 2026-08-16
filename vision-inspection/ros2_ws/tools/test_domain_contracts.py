"""ROS 없이 실행하는 v2 핵심 도메인 회귀 시험."""

from __future__ import annotations

import base64
import hashlib
import sys
import tempfile
import time
import unittest
from dataclasses import replace
from pathlib import Path

WORKSPACE = Path(__file__).parents[1]
for package_path in (
    WORKSPACE / "src" / "basic_packages" / "inspection_common",
    WORKSPACE / "src" / "nodes" / "inspection_master",
    WORKSPACE / "src" / "nodes" / "inspection_vision",
    WORKSPACE / "src" / "nodes" / "inspection_log",
):
    sys.path.insert(0, str(package_path))

from inspection_common import (  # noqa: E402
    IdempotencyStore,
    ReplayKind,
    StationId,
    Verdict,
    payload_digest,
)
from inspection_common.log_spool import DurableLogSpool, SpoolRecord  # noqa: E402
from inspection_log.storage import LogRepository, StoredLogEvent  # noqa: E402
from inspection_master.product_flow import (  # noqa: E402
    ProductLedger,
    ProductResultReorderBuffer,
    StationDecision,
)
from inspection_vision.capture_contract import CaptureBatch, ImageArtifact  # noqa: E402
from inspection_vision.inference_queue import InferenceJob, InferenceQueue  # noqa: E402


class CommonContractTests(unittest.TestCase):
    def test_digest_is_order_independent_and_idempotency_detects_conflict(self) -> None:
        first = payload_digest({"b": 2, "a": 1})
        self.assertEqual(first, payload_digest({"a": 1, "b": 2}))
        store: IdempotencyStore[str] = IdempotencyStore(capacity=2)
        self.assertEqual(store.inspect("command", first).kind, ReplayKind.NEW)
        store.remember("command", first, "result")
        replay = store.inspect("command", first)
        self.assertEqual(replay.kind, ReplayKind.REPLAY)
        self.assertEqual(replay.result, "result")
        self.assertEqual(store.inspect("command", "0" * 64).kind, ReplayKind.CONFLICT)


class MasterContractTests(unittest.TestCase):
    def test_station_aggregation_sensor3_and_fifo_reorder(self) -> None:
        ledger = ProductLedger()
        first = ledger.register("product-1", 1)
        self.assertTrue(
            first.apply_station_result(
                StationDecision(StationId.A, Verdict.PASS, 1, "capture-a", "job-a")
            )
        )
        self.assertIsNone(first.lock_if_complete())
        self.assertTrue(
            first.apply_station_result(
                StationDecision(StationId.B, Verdict.PASS, 1, "capture-b", "job-b")
            )
        )
        first_locked = first.lock_if_complete()
        self.assertIsNotNone(first_locked)
        self.assertEqual(first_locked.verdict, Verdict.PASS)
        self.assertFalse(
            first.apply_station_result(
                StationDecision(StationId.B, Verdict.NG, 2, "late", "late")
            )
        )

        second = ledger.register("product-2", 2)
        second_locked = second.lock_at_sensor3("sensor3-event")
        self.assertEqual(second_locked.verdict, Verdict.FORCED_NG)
        self.assertFalse(second_locked.station_a_completed)
        self.assertFalse(second_locked.station_b_completed)

        reorder = ProductResultReorderBuffer()
        self.assertEqual(reorder.add(second_locked), [])
        emitted = reorder.add(first_locked)
        self.assertEqual([item.fifo_sequence for item in emitted], [1, 2])

    def test_explicit_station_failure_locks_immediately(self) -> None:
        context = ProductLedger().register("product", 1)
        locked = context.lock_explicit_failure(StationId.A, "camera offline")
        self.assertEqual(locked.verdict, Verdict.FORCED_NG)
        self.assertIn("camera offline", locked.reason)


class VisionContractTests(unittest.TestCase):
    def _job(self, number: int, product: str | None = None) -> InferenceJob:
        return InferenceJob(
            inference_job_id=f"job-{number}",
            product_id=product or f"product-{number}",
            fifo_sequence=number,
            station_id=1,
            capture_id=f"capture-{number}",
            frame_batch_id=f"batch-{number}",
            image_paths=(f"/tmp/{number}.png",),
            enqueued_monotonic_ns=time.monotonic_ns(),
        )

    def test_queue_is_bounded_fifo_and_product_lock_removes_waiting_jobs(self) -> None:
        queue = InferenceQueue(capacity=2)
        one = self._job(1)
        two = self._job(2, product="locked-product")
        self.assertTrue(queue.try_enqueue(one))
        self.assertTrue(queue.try_enqueue(two))
        self.assertFalse(queue.try_enqueue(self._job(3)))
        self.assertEqual(queue.lock_product("locked-product"), 1)
        self.assertEqual(queue.get(), one)
        self.assertFalse(queue.try_enqueue(self._job(4, product="locked-product")))
        queue.close()

    def test_queue_uses_fifo_sequence_and_stamps_actual_enqueue_time(self) -> None:
        queue = InferenceQueue(capacity=3)
        later = self._job(2)
        earlier = self._job(1)
        unstamped = InferenceJob(
            inference_job_id="job-3",
            product_id="product-3",
            fifo_sequence=3,
            station_id=1,
            capture_id="capture-3",
            frame_batch_id="batch-3",
            image_paths=("/tmp/3.png",),
            enqueued_monotonic_ns=0,
        )
        self.assertTrue(queue.try_enqueue(later))
        self.assertTrue(queue.try_enqueue(earlier))
        self.assertTrue(queue.try_enqueue(unstamped))
        self.assertEqual(queue.get().fifo_sequence, 1)
        self.assertEqual(queue.get().fifo_sequence, 2)
        actual = queue.get()
        self.assertEqual(actual.fifo_sequence, 3)
        self.assertGreater(actual.enqueued_monotonic_ns, 0)
        queue.close()

    def test_capture_batch_validates_rgb_png_digest_and_host_skew(self) -> None:
        png = base64.b64decode(
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "image.png"
            path.write_bytes(png)
            digest = hashlib.sha256(png).hexdigest()
            artifacts = tuple(
                ImageArtifact(
                    camera_id=camera,
                    file_path=str(path.resolve()),
                    sha256=digest,
                    file_size_bytes=len(png),
                    width=2448,
                    height=2048,
                    pixel_format="RGB8_PNG",
                    camera_timestamp_raw=index,
                    camera_timestamp_domain="DEVICE_TICKS_UNSYNCED",
                    camera_timestamp_ns=index,
                    host_arrival_monotonic_ns=1_000_000 + index * 5_000,
                    host_arrival_timestamp_ns=1_000_000 + index * 5_000,
                )
                for index, camera in enumerate(("camera-a", "camera-b"))
            )
            batch = CaptureBatch("product", 1, "capture", "batch", 1, artifacts)
            batch.validate(("camera-a", "camera-b"))
            self.assertEqual(batch.frame_arrival_skew_us, 5)


class LogContractTests(unittest.TestCase):
    def test_spool_ack_and_log_repository_digest_conflict(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spool = DurableLogSpool(root / "spool.sqlite3")
            record = SpoolRecord("log", 1, "{}", hashlib.sha256(b"{}").hexdigest())
            spool.enqueue(record)
            self.assertEqual(spool.pending(), [record])
            spool.acknowledge([("log", 1)])
            self.assertEqual(spool.pending(), [])
            spool.close()

            repository = LogRepository(root / "log.sqlite3")
            event = StoredLogEvent(
                log_id="log",
                revision=1,
                severity=20,
                event_type="TEST",
                source_node="test",
                producer_instance_id="instance",
                product_id="",
                payload_json="{}",
                payload_digest=hashlib.sha256(b"{}").hexdigest(),
                occurred_at_ns=1,
            )
            repository.append_event(event)
            repository.append_event(event)
            conflicting = replace(event, payload_digest="0" * 64)
            with self.assertRaises(ValueError):
                repository.append_event(conflicting)
            repository.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
