"""ROS 없이 실행하는 VisionNode 저장·촬영·journal·worker 통합 회귀 시험."""

from __future__ import annotations

import asyncio
import json
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).parents[1]
for package_path in (
    WORKSPACE / "src" / "basic_packages" / "inspection_common",
    WORKSPACE / "src" / "nodes" / "inspection_vision",
):
    sys.path.insert(0, str(package_path))

from inspection_vision.artifact_store import (  # noqa: E402
    ArtifactStore,
    DiskPolicy,
)
from inspection_vision.camera_backend import (  # noqa: E402
    ActionCommandConfig,
    CameraSpec,
    SimulationCaptureBackend,
)
from inspection_vision.capture_contract import CameraFrame, RawCaptureBatch  # noqa: E402
from inspection_vision.capture_service import (  # noqa: E402
    CaptureProgress,
    CaptureRequest,
    CaptureService,
)
from inspection_vision.inference_queue import InferenceJob  # noqa: E402
from inspection_vision.model_adapter import (  # noqa: E402
    SimulationModel,
    StationInference,
    load_rgb_png_with_opencv,
)
from inspection_vision.mvs_backend import (  # noqa: E402
    MvsCaptureBackend,
    MvsSdkError,
    _AttemptWindow,
    _SdkFrame,
)
from inspection_vision.queue_journal import InferenceJournal  # noqa: E402
from inspection_vision.vision_runtime import VisionRuntime  # noqa: E402


def test_disk_policy() -> DiskPolicy:
    return DiskPolicy(
        warning_used_percent=99.7,
        pause_used_percent=99.8,
        critical_used_percent=99.9,
        warning_free_bytes=3,
        pause_free_bytes=2,
        critical_free_bytes=1,
    )


def make_frame(camera_id: str, *, arrival_ns: int, counter: int = 1) -> CameraFrame:
    return CameraFrame(
        camera_id=camera_id,
        width=2,
        height=2,
        rgb_bytes=bytes((counter, 2, 3)) * 4,
        frame_number=counter,
        external_trigger_count=counter,
        camera_timestamp_raw=counter * 100,
        camera_timestamp_domain="TEST_TICKS",
        camera_timestamp_ns=0,
        camera_timestamp_synchronized=False,
        sdk_host_timestamp_raw=counter * 200,
        host_arrival_monotonic_ns=arrival_ns,
        host_arrival_wall_time_ns=time.time_ns(),
    )


def make_raw(
    *,
    attempt: int,
    camera_ids: tuple[str, ...],
    spacing_us: int,
    capture_id: str = "capture-1",
) -> RawCaptureBatch:
    requested_mono = time.monotonic_ns()
    requested_wall = time.time_ns()
    frames = tuple(
        make_frame(
            camera_id,
            arrival_ns=requested_mono + 1_000_000 + index * spacing_us * 1_000,
            counter=attempt,
        )
        for index, camera_id in enumerate(camera_ids)
    )
    return RawCaptureBatch(
        product_id="product-1",
        station_id=1,
        capture_id=capture_id,
        attempt=attempt,
        trigger_requested_monotonic_ns=requested_mono,
        trigger_returned_monotonic_ns=requested_mono + 100,
        trigger_requested_wall_time_ns=requested_wall,
        trigger_returned_wall_time_ns=requested_wall + 100,
        frames=frames,
    )


class RetryBackend:
    def __init__(self, camera_ids: tuple[str, ...]) -> None:
        self.camera_ids = camera_ids
        self.attempts: list[tuple[str, int, tuple[str, ...]]] = []
        self.retry_preparations = 0

    async def initialize(self) -> dict[str, object]:
        return {"backend": "retry-test"}

    async def capture_station(self, **kwargs) -> RawCaptureBatch:
        self.attempts.append(
            (
                str(kwargs["capture_id"]),
                int(kwargs["attempt"]),
                tuple(kwargs["required_camera_ids"]),
            )
        )
        return make_raw(
            attempt=int(kwargs["attempt"]),
            camera_ids=self.camera_ids,
            spacing_us=1_000 if int(kwargs["attempt"]) == 1 else 10,
            capture_id=str(kwargs["capture_id"]),
        )

    async def prepare_retry(self, **_kwargs) -> None:
        self.retry_preparations += 1

    async def recover_station(self, **_kwargs) -> bool:
        return True

    async def close(self) -> None:
        return None


class ArtifactAndCaptureTests(unittest.TestCase):
    def test_atomic_rgb_png_manifest_and_restart_discovery(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            store = ArtifactStore(Path(directory), disk_policy=test_disk_policy())
            raw = make_raw(attempt=1, camera_ids=("CAM_A_1",), spacing_us=0)
            batch = store.save_batch(
                raw,
                frame_batch_id="batch-1",
                required_camera_ids=("CAM_A_1",),
                capture_outcome="SUCCESS",
            )
            batch.validate(("CAM_A_1",))
            manifest = json.loads(Path(batch.manifest_path).read_text(encoding="utf-8"))
            self.assertEqual(manifest["schema_version"], 2)
            self.assertEqual(manifest["capture_outcome"], "SUCCESS")
            self.assertEqual(manifest["images"][0]["frame_number"], 1)
            self.assertTrue(batch.images[0].file_path.endswith("CAM_A_1.png"))
            self.assertIn(
                ("product-1", 1, "capture-1"), store.discover_capture_keys()
            )
            self.assertEqual(store.cleanup_incomplete_files(), ())

    def test_skew_failure_retries_all_cameras_with_same_capture_id(self) -> None:
        async def scenario(root: Path) -> tuple[object, RetryBackend, list[CaptureProgress]]:
            camera_ids = ("CAM_A_1", "CAM_A_2", "CAM_A_3")
            backend = RetryBackend(camera_ids)
            progress: list[CaptureProgress] = []
            service = CaptureService(
                backend=backend,
                artifact_store=ArtifactStore(root, disk_policy=test_disk_policy()),
                max_attempts=2,
                frame_arrival_skew_limit_us=100,
                frame_timeout_ms=1000,
            )
            batch = await service.capture(
                CaptureRequest("product-1", 1, 1, "capture-same", camera_ids),
                on_progress=progress.append,
                is_cancel_requested=lambda: False,
            )
            return batch, backend, progress

        with tempfile.TemporaryDirectory() as directory:
            batch, backend, progress = asyncio.run(scenario(Path(directory)))
            self.assertEqual(batch.attempt, 2)
            self.assertEqual(backend.retry_preparations, 1)
            self.assertEqual(
                backend.attempts,
                [
                    ("capture-same", 1, ("CAM_A_1", "CAM_A_2", "CAM_A_3")),
                    ("capture-same", 2, ("CAM_A_1", "CAM_A_2", "CAM_A_3")),
                ],
            )
            manifests = sorted(Path(directory).rglob("manifest.json"))
            self.assertEqual(len(manifests), 2)
            outcomes = [
                json.loads(path.read_text(encoding="utf-8"))["capture_outcome"]
                for path in manifests
            ]
            self.assertEqual(sorted(outcomes), ["SKEW_FAILED", "SUCCESS"])
            self.assertTrue(any(item.stage.value == "RETRYING" for item in progress))


class JournalTests(unittest.TestCase):
    @staticmethod
    def _job(job_id: str, frame_batch_id: str, image_path: Path) -> InferenceJob:
        return InferenceJob(
            inference_job_id=job_id,
            product_id=f"product-{job_id}",
            fifo_sequence=1,
            station_id=1,
            capture_id=f"capture-{job_id}",
            frame_batch_id=frame_batch_id,
            image_paths=(str(image_path),),
            enqueued_monotonic_ns=time.monotonic_ns(),
            enqueued_wall_time_ns=time.time_ns(),
            queue_total_timeout_ms=60_000,
        )

    def test_restart_restores_only_committed_enqueue_and_lock_is_terminal(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / "image.png"
            image.write_bytes(b"png-path-only-test")
            database = root / "journal.sqlite3"
            journal = InferenceJournal(database)
            uncommitted = self._job("uncommitted", "batch-u", image)
            committed = self._job("committed", "batch-c", image)
            journal.prepare(uncommitted)
            journal.prepare(committed)
            journal.mark_enqueued(committed.inference_job_id)
            journal.close()

            reopened = InferenceJournal(database)
            recovered = reopened.recoverable_jobs()
            self.assertEqual(recovered, (committed,))
            snapshot = reopened.snapshot()
            self.assertEqual(snapshot["failed"], 1)
            self.assertEqual(snapshot["pending"], 1)
            locked = reopened.lock_product(committed.product_id)
            self.assertEqual(locked, (committed.inference_job_id,))
            self.assertTrue(reopened.is_product_locked(committed.product_id))
            self.assertEqual(reopened.recoverable_jobs(), ())
            reopened.close()


class RuntimeTests(unittest.TestCase):
    def test_restart_restores_enqueued_job_without_recapturing(self) -> None:
        async def scenario(root: Path) -> tuple[int, int, bool]:
            specs = (
                CameraSpec("A1", 1, "", ""),
                CameraSpec("A2", 1, "", ""),
                CameraSpec("A3", 1, "", ""),
                CameraSpec("B1", 2, "", ""),
            )

            def make_runtime(results: list[object]) -> VisionRuntime:
                return VisionRuntime(
                    backend=SimulationCaptureBackend(specs),
                    artifact_store=ArtifactStore(
                        root / "data", disk_policy=test_disk_policy()
                    ),
                    journal=InferenceJournal(root / "journal.sqlite3"),
                    model=SimulationModel(),
                    load_image=lambda path: path.read_bytes(),
                    queue_capacity=16,
                    worker_count=2,
                    queue_total_timeout_ms=10_000,
                    frame_arrival_skew_limit_us=1000,
                    frame_timeout_ms=1000,
                    serialize_model_access=True,
                    on_queue_state=lambda _event: None,
                    on_inference_success=lambda job, result: results.append(
                        (job, result)
                    ),
                    on_inference_failure=lambda failure: results.append(failure),
                    on_telemetry=lambda *_args: None,
                )

            first_results: list[object] = []
            first = make_runtime(first_results)
            await first.start()
            batch = await first.capture_service.capture(
                CaptureRequest("product-1", 1, 1, "capture-1", ("A1", "A2", "A3")),
                on_progress=lambda _progress: None,
                is_cancel_requested=lambda: False,
            )
            await first.enqueue_saved_batch(
                batch=batch,
                fifo_sequence=1,
                is_cancel_requested=lambda: False,
                on_blocked=lambda _job: None,
            )
            await first.close()

            restored_results: list[object] = []
            restored = make_runtime(restored_results)
            details = await restored.start()
            preexisting = restored.captured_before_runtime(
                "product-1", 1, "capture-1"
            )
            restored.activate_workers()
            for _ in range(100):
                if restored_results:
                    break
                await asyncio.sleep(0.01)
            await restored.close()
            return len(first_results), len(restored_results), bool(
                details["recovered_job_count"] == 1 and preexisting
            )

        with tempfile.TemporaryDirectory() as directory:
            before, after, recovery_evidence = asyncio.run(scenario(Path(directory)))
            self.assertEqual(before, 0)
            self.assertEqual(after, 1)
            self.assertTrue(recovery_evidence)

    def test_queue_timeout_clock_starts_after_enqueue_block_clears(self) -> None:
        async def scenario(root: Path) -> tuple[int, int]:
            specs = (
                CameraSpec("A1", 1, "", ""),
                CameraSpec("A2", 1, "", ""),
                CameraSpec("A3", 1, "", ""),
                CameraSpec("B1", 2, "", ""),
            )
            store = ArtifactStore(root / "data", disk_policy=test_disk_policy())
            runtime = VisionRuntime(
                backend=SimulationCaptureBackend(specs),
                artifact_store=store,
                journal=InferenceJournal(root / "journal.sqlite3"),
                model=SimulationModel(),
                load_image=lambda path: path.read_bytes(),
                queue_capacity=1,
                worker_count=1,
                queue_total_timeout_ms=10_000,
                frame_arrival_skew_limit_us=100,
                frame_timeout_ms=1000,
                serialize_model_access=True,
                on_queue_state=lambda _event: None,
                on_inference_success=lambda *_args: None,
                on_inference_failure=lambda _failure: None,
                on_telemetry=lambda *_args: None,
            )
            await runtime.start()
            first_batch = store.save_batch(
                make_raw(
                    attempt=1,
                    camera_ids=("A1",),
                    spacing_us=0,
                    capture_id="capture-first",
                ),
                frame_batch_id="batch-first",
                required_camera_ids=("A1",),
                capture_outcome="SUCCESS",
            )
            second_batch = store.save_batch(
                make_raw(
                    attempt=1,
                    camera_ids=("A1",),
                    spacing_us=0,
                    capture_id="capture-second",
                ),
                frame_batch_id="batch-second",
                required_camera_ids=("A1",),
                capture_outcome="SUCCESS",
            )
            await runtime.enqueue_saved_batch(
                batch=first_batch,
                fifo_sequence=1,
                is_cancel_requested=lambda: False,
                on_blocked=lambda _job: None,
            )
            blocked_started_wall_ns = time.time_ns()
            blocked = asyncio.create_task(
                runtime.enqueue_saved_batch(
                    batch=second_batch,
                    fifo_sequence=2,
                    is_cancel_requested=lambda: False,
                    on_blocked=lambda _job: None,
                )
            )
            await asyncio.sleep(0.05)
            self.assertFalse(blocked.done())
            await asyncio.to_thread(runtime.queue.get)
            second_job = await asyncio.wait_for(blocked, timeout=1.0)
            await runtime.close()
            return blocked_started_wall_ns, second_job.enqueued_wall_time_ns

        with tempfile.TemporaryDirectory() as directory:
            blocked_at, enqueued_at = asyncio.run(scenario(Path(directory)))
            self.assertGreaterEqual(enqueued_at - blocked_at, 40_000_000)

    def test_sim_runtime_capture_enqueue_and_worker_result(self) -> None:
        async def scenario(root: Path) -> tuple[list[object], list[object], dict[str, object]]:
            specs = (
                CameraSpec("A1", 1, "", ""),
                CameraSpec("A2", 1, "", ""),
                CameraSpec("A3", 1, "", ""),
                CameraSpec("B1", 2, "", ""),
            )
            results: list[object] = []
            failures: list[object] = []
            runtime = VisionRuntime(
                backend=SimulationCaptureBackend(specs, callback_spacing_us=10),
                artifact_store=ArtifactStore(
                    root / "data", disk_policy=test_disk_policy()
                ),
                journal=InferenceJournal(root / "journal.sqlite3"),
                model=SimulationModel(),
                load_image=lambda path: path.read_bytes(),
                queue_capacity=16,
                worker_count=2,
                queue_total_timeout_ms=10_000,
                frame_arrival_skew_limit_us=100,
                frame_timeout_ms=1000,
                serialize_model_access=True,
                on_queue_state=lambda _event: None,
                on_inference_success=lambda job, result: results.append((job, result)),
                on_inference_failure=failures.append,
                on_telemetry=lambda *_args: None,
            )
            await runtime.start()
            batch = await runtime.capture_service.capture(
                CaptureRequest("product-1", 1, 1, "capture-1", ("A1", "A2", "A3")),
                on_progress=lambda _progress: None,
                is_cancel_requested=lambda: False,
            )
            job = await runtime.enqueue_saved_batch(
                batch=batch,
                fifo_sequence=1,
                is_cancel_requested=lambda: False,
                on_blocked=lambda _job: None,
            )
            self.assertEqual(results, [])
            runtime.activate_workers()
            for _ in range(100):
                if results or failures:
                    break
                await asyncio.sleep(0.01)
            status = runtime.status_snapshot()
            self.assertEqual(results[0][0].inference_job_id, job.inference_job_id)
            self.assertEqual(results[0][1].model_version, "simulation-wiring-only-v1")
            await runtime.close()
            return results, failures, status

        with tempfile.TemporaryDirectory() as directory:
            results, failures, status = asyncio.run(scenario(Path(directory)))
            self.assertEqual(len(results), 1)
            self.assertEqual(failures, [])
            self.assertTrue(status["workers_started"])
            self.assertEqual(status["journal"]["done"], 1)

    def test_non_finite_model_score_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            StationInference(1, float("nan"), "model").validate()

    def test_opencv_loader_converts_bgr_decode_to_rgb(self) -> None:
        calls: list[object] = []
        fake_cv2 = types.SimpleNamespace(
            IMREAD_COLOR=1,
            COLOR_BGR2RGB=2,
            imread=lambda path, mode: calls.append((path, mode)) or "BGR_ARRAY",
            cvtColor=lambda image, conversion: calls.append(
                (image, conversion)
            )
            or "RGB_ARRAY",
        )
        previous = sys.modules.get("cv2")
        sys.modules["cv2"] = fake_cv2
        try:
            result = load_rgb_png_with_opencv(Path("canonical.png"))
        finally:
            if previous is None:
                del sys.modules["cv2"]
            else:
                sys.modules["cv2"] = previous
        self.assertEqual(result, "RGB_ARRAY")
        self.assertEqual(
            calls,
            [("canonical.png", 1), ("BGR_ARRAY", 2)],
        )


class MvsCorrelationTests(unittest.TestCase):
    def test_sdk_raw_version_mismatch_blocks_initialization(self) -> None:
        class FakeMvCamera:
            @staticmethod
            def MV_CC_Initialize() -> int:
                return 0

            @staticmethod
            def MV_CC_Finalize() -> int:
                return 0

            @staticmethod
            def MV_CC_GetSDKVersion() -> int:
                return 0x01020304

        async def scenario(wrapper_path: Path) -> None:
            backend = MvsCaptureBackend(
                (CameraSpec("CAM_A_1", 1, "SERIAL", "192.168.10.11"),),
                action_config=ActionCommandConfig(),
                python_module_path=wrapper_path,
                expected_sdk_version_raw=0x05000200,
            )
            backend._load_sdk_module = lambda: types.SimpleNamespace(  # type: ignore[method-assign]
                MvCamera=FakeMvCamera
            )
            with self.assertRaisesRegex(MvsSdkError, "SDK version mismatch"):
                await backend.initialize()

        with tempfile.TemporaryDirectory() as directory:
            asyncio.run(scenario(Path(directory)))

    def test_external_trigger_count_precedes_frame_number_and_wraps(self) -> None:
        window = _AttemptWindow(
            requested_monotonic_ns=100,
            baseline_frame_number=500,
            baseline_trigger_count=0xFFFFFFFF,
        )
        candidate = _SdkFrame(
            source_bytes=b"x",
            width=1,
            height=1,
            pixel_type=1,
            frame_number=1,
            external_trigger_count=0,
            device_timestamp_raw=0,
            sdk_host_timestamp_raw=0,
            host_arrival_monotonic_ns=101,
            host_arrival_wall_time_ns=101,
        )
        # trigger count 0은 metadata 미제공으로 보고 frame number fallback을 사용합니다.
        self.assertFalse(MvsCaptureBackend._frame_matches(window, candidate))
        self.assertTrue(MvsCaptureBackend._counter_after(0, 0xFFFFFFFF))
        self.assertFalse(MvsCaptureBackend._counter_after(10, 10))


if __name__ == "__main__":
    unittest.main(verbosity=2)
