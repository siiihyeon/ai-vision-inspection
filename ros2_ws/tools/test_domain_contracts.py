"""ROS 없이 실행하는 v2 핵심 도메인 회귀 시험."""

from __future__ import annotations

import hashlib
import struct
import sys
import tempfile
import threading
import time
import unittest
import zlib
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
    ConveyorId,
    IdempotencyStore,
    NodeHealthState,
    NodeId,
    ProductPhysicalState,
    ReplayKind,
    StationId,
    SystemState,
    Verdict,
    payload_digest,
)
from inspection_common.log_spool import DurableLogSpool, SpoolRecord  # noqa: E402
from inspection_log.storage import LogRepository, StoredLogEvent  # noqa: E402
from inspection_master.operation_runtime import EquipmentSnapshot  # noqa: E402
from inspection_master.product_flow import (  # noqa: E402
    ProductLedger,
    ProductResultReorderBuffer,
    SensorEventOutcome,
    SensorEventRegistry,
    StationDecision,
    StationProcessState,
    StationResultConflict,
)
from inspection_master.system_fsm import (  # noqa: E402
    InvalidSystemTransition,
    SystemEvent,
    allowed_system_events,
    decide_system_transition,
)
from inspection_master.worker_supervision import (  # noqa: E402
    WorkerInitPhase,
    WorkerRuntimeState,
)
from inspection_vision.capture_contract import CaptureBatch, ImageArtifact  # noqa: E402
from inspection_vision.inference_queue import (  # noqa: E402
    InferenceJob,
    InferenceQueue,
    WorkerPool,
)


def make_rgb8_png(width: int = 1, height: int = 1) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        checksum = zlib.crc32(kind + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", checksum)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    scanlines = b"".join(b"\x00" + b"\x00\x00\x00" * width for _ in range(height))
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(scanlines))
        + chunk(b"IEND", b"")
    )


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
    def _complete_station(
        self,
        context,
        station_id: StationId,
        *,
        verdict: Verdict | None,
        capture_failed: bool = False,
    ) -> None:
        """ROS 없이 station의 위치→촬영→재가동 규칙을 진행합니다."""

        suffix = station_id.name.lower()
        position_command_id = f"position-{suffix}"
        capture_id = f"capture-{suffix}-{context.product_id}"
        context.begin_station_cycle(
            station_id,
            position_command_id=position_command_id,
            target_step=100 * int(station_id),
            capture_id=capture_id,
        )
        context.mark_position_action_succeeded(station_id, position_command_id)
        context.mark_position_settled(station_id, position_command_id)
        context.mark_capture_requested(
            station_id,
            capture_id=capture_id,
            command_id=f"capture-command-{suffix}",
        )
        if capture_failed:
            context.mark_capture_failed(
                station_id,
                capture_id=capture_id,
                reason=f"station {suffix} final capture failed",
            )
        else:
            context.mark_capture_succeeded(
                station_id,
                capture_id=capture_id,
                frame_batch_id=f"batch-{suffix}",
                inference_job_id=f"job-{suffix}",
            )
            if verdict is not None:
                context.apply_station_result(
                    StationDecision(
                        station_id,
                        verdict,
                        1,
                        capture_id,
                        f"job-{suffix}",
                        f"batch-{suffix}",
                    )
                )
        context.mark_conveyor_resumed_after_capture(station_id)

    def _move_to_station_b(self, context) -> None:
        context.accept_sensor2(f"sensor2-{context.product_id}", 200)

    def test_worker_ready_requires_action_status_and_fresh_heartbeat(self) -> None:
        now_ns = 10_000_000_000
        state = WorkerRuntimeState(NodeId.VISION)
        state.begin_attempt(
            request_id="request-1",
            attempt=1,
            now_ns=now_ns,
            timeout_ns=10_000_000_000,
        )
        state.mark_goal_accepted()
        state.mark_action_ready(
            interface_version="2.0.0",
            software_version="0.2.0",
            active_session_id="session-1",
            reason="ready",
            now_ns=now_ns,
            timeout_ns=10_000_000_000,
        )
        self.assertFalse(
            state.can_mark_ready(
                session_id="session-1",
                expected_interface_version="2.0.0",
                command_epoch=0,
                now_ns=now_ns,
                heartbeat_timeout_ns=2_000_000_000,
            )
        )

        state.record_status(
            node_instance_id="instance-1",
            interface_version="2.0.0",
            software_version="0.2.0",
            active_session_id="session-1",
            command_epoch=0,
            heartbeat_sequence=1,
            health_state=NodeHealthState.READY,
            ready=True,
            master_heartbeat_alive=True,
        )
        state.accept_heartbeat(
            node_instance_id="instance-1",
            sequence=1,
            health_state=NodeHealthState.READY,
            interface_version="2.0.0",
            session_id="session-1",
            received_ns=now_ns,
        )
        self.assertTrue(
            state.can_mark_ready(
                session_id="session-1",
                expected_interface_version="2.0.0",
                command_epoch=0,
                now_ns=now_ns,
                heartbeat_timeout_ns=2_000_000_000,
            )
        )

    def test_worker_heartbeat_is_latest_only_and_detects_restart(self) -> None:
        state = WorkerRuntimeState(NodeId.CONTROL)
        first = state.accept_heartbeat(
            node_instance_id="instance-1",
            sequence=10,
            health_state=NodeHealthState.READY,
            interface_version="2.0.0",
            session_id="session-1",
            received_ns=1_000,
        )
        self.assertTrue(first.accepted)
        self.assertFalse(first.restarted)

        duplicate = state.accept_heartbeat(
            node_instance_id="instance-1",
            sequence=10,
            health_state=NodeHealthState.READY,
            interface_version="2.0.0",
            session_id="session-1",
            received_ns=2_000,
        )
        self.assertFalse(duplicate.accepted)

        restarted = state.accept_heartbeat(
            node_instance_id="instance-2",
            sequence=1,
            health_state=NodeHealthState.STARTING,
            interface_version="2.0.0",
            session_id="",
            received_ns=3_000,
        )
        self.assertTrue(restarted.accepted)
        self.assertTrue(restarted.restarted)

    def test_worker_retry_uses_new_request_and_links_previous_request(self) -> None:
        state = WorkerRuntimeState(NodeId.LOG)
        state.begin_attempt(
            request_id="request-1",
            attempt=1,
            now_ns=1_000,
            timeout_ns=10_000,
        )
        state.schedule_retry(
            now_ns=2_000,
            interval_ns=1_000,
            error_code=9002,
            reason="temporary failure",
            manual_intervention_required=False,
        )
        self.assertEqual(state.phase, WorkerInitPhase.RETRY_WAIT)
        state.begin_attempt(
            request_id="request-2",
            attempt=2,
            now_ns=3_000,
            timeout_ns=10_000,
        )
        self.assertEqual(state.request_id, "request-2")
        self.assertEqual(state.retry_of_request_id, "request-1")

    def test_worker_epoch_change_invalidates_only_stale_status_evidence(self) -> None:
        now_ns = 10_000
        state = WorkerRuntimeState(NodeId.VISION)
        state.action_ready = True
        state.active_session_id = "session-1"
        state.record_status(
            node_instance_id="instance-1",
            interface_version="2.0.0",
            software_version="0.2.0",
            active_session_id="session-1",
            command_epoch=0,
            heartbeat_sequence=1,
            health_state=NodeHealthState.READY,
            ready=True,
            master_heartbeat_alive=True,
        )
        state.accept_heartbeat(
            node_instance_id="instance-1",
            sequence=1,
            health_state=NodeHealthState.READY,
            interface_version="2.0.0",
            session_id="session-1",
            received_ns=now_ns,
        )
        state.mark_ready()

        state.invalidate_epoch_evidence(now_ns=now_ns, interval_ns=500)
        self.assertTrue(state.action_ready)
        self.assertFalse(state.status_verified)
        self.assertFalse(state.ready)
        self.assertEqual(state.phase, WorkerInitPhase.WAITING_STATUS)
        self.assertEqual(state.status_poll_due_ns, now_ns + 500)
        self.assertFalse(
            state.can_mark_ready(
                session_id="session-1",
                expected_interface_version="2.0.0",
                command_epoch=1,
                now_ns=now_ns,
                heartbeat_timeout_ns=2_000,
            )
        )

        state.record_status(
            node_instance_id="instance-1",
            interface_version="2.0.0",
            software_version="0.2.0",
            active_session_id="session-1",
            command_epoch=1,
            heartbeat_sequence=2,
            health_state=NodeHealthState.READY,
            ready=True,
            master_heartbeat_alive=True,
        )
        state.accept_heartbeat(
            node_instance_id="instance-1",
            sequence=2,
            health_state=NodeHealthState.READY,
            interface_version="2.0.0",
            session_id="session-1",
            received_ns=now_ns + 1,
        )
        self.assertTrue(
            state.can_mark_ready(
                session_id="session-1",
                expected_interface_version="2.0.0",
                command_epoch=1,
                now_ns=now_ns + 1,
                heartbeat_timeout_ns=2_000,
            )
        )

    def test_system_fsm_normal_run_pause_and_resume(self) -> None:
        state = SystemState.BOOT

        transition = decide_system_transition(
            state, SystemEvent.APP_STARTED, "application started"
        )
        self.assertEqual(transition.rule_id, "SYS-01")
        state = transition.current
        self.assertEqual(state, SystemState.INITIALIZING)

        state = decide_system_transition(
            state, SystemEvent.INIT_DONE, "workers ready"
        ).current
        self.assertEqual(state, SystemState.READY)

        # START 요청만으로 RUN_SYS가 되지 않습니다.
        start = decide_system_transition(
            state, SystemEvent.START_REQUEST, "operator start"
        )
        self.assertFalse(start.changed)
        self.assertEqual(start.current, SystemState.READY)
        state = decide_system_transition(
            state,
            SystemEvent.ALL_CONVEYORS_RUNNING,
            "both conveyors running",
        ).current
        self.assertEqual(state, SystemState.RUN_SYS)

        state = decide_system_transition(
            state, SystemEvent.PAUSE_REQUEST, "operator pause"
        ).current
        self.assertEqual(state, SystemState.PAUSING)
        state = decide_system_transition(
            state,
            SystemEvent.ALL_CONVEYORS_STOPPED,
            "both conveyors stopped",
        ).current
        self.assertEqual(state, SystemState.PAUSED)

        # RESUME 요청도 실제 RUN 확인 전까지 PAUSED를 유지합니다.
        resume = decide_system_transition(
            state, SystemEvent.RESUME_REQUEST, "operator resume"
        )
        self.assertFalse(resume.changed)
        state = decide_system_transition(
            resume.current,
            SystemEvent.ALL_CONVEYORS_RUNNING,
            "both conveyors running",
        ).current
        self.assertEqual(state, SystemState.RUN_SYS)

    def test_system_fsm_retries_safe_failures_without_fault_stop(self) -> None:
        initializing = decide_system_transition(
            SystemState.INITIALIZING,
            SystemEvent.INIT_TIMEOUT,
            "worker initialization timeout",
        )
        self.assertEqual(initializing.rule_id, "SYS-03")
        self.assertEqual(initializing.current, SystemState.INITIALIZING)

        start_failed = decide_system_transition(
            SystemState.READY,
            SystemEvent.START_FAILED,
            "conveyor did not start",
        )
        self.assertEqual(start_failed.current, SystemState.READY)

        resume_failed = decide_system_transition(
            SystemState.PAUSED,
            SystemEvent.RESUME_FAILED,
            "resume command failed",
        )
        self.assertEqual(resume_failed.current, SystemState.PAUSED)

        reset_failed = decide_system_transition(
            SystemState.RESETTING,
            SystemEvent.RESET_FAILED,
            "equipment check failed",
        )
        self.assertEqual(reset_failed.current, SystemState.RESETTING)

    def test_system_fsm_fault_and_reset_paths(self) -> None:
        pause_failed = decide_system_transition(
            SystemState.PAUSING,
            SystemEvent.PAUSE_TIMEOUT,
            "stop could not be confirmed",
        )
        self.assertEqual(pause_failed.rule_id, "SYS-07")
        self.assertEqual(pause_failed.current, SystemState.FAULT_STOP)

        critical = decide_system_transition(
            SystemState.RUN_SYS,
            SystemEvent.CRITICAL_FAULT,
            "FIFO identity lost",
        )
        self.assertEqual(critical.rule_id, "SYS-10")
        self.assertEqual(critical.current, SystemState.FAULT_STOP)

        resetting = decide_system_transition(
            critical.current,
            SystemEvent.RESET_REQUEST,
            "operator confirmed recovery guard",
        ).current
        self.assertEqual(resetting, SystemState.RESETTING)
        self.assertEqual(
            decide_system_transition(
                resetting,
                SystemEvent.RESET_SUCCEEDED_EMPTY_LINE,
                "line clear completed",
            ).current,
            SystemState.READY,
        )
        self.assertEqual(
            decide_system_transition(
                resetting,
                SystemEvent.RESET_SUCCEEDED_IN_PLACE,
                "equipment recovered with product context",
            ).current,
            SystemState.PAUSED,
        )

    def test_system_fsm_rejects_unlisted_transition_and_accepts_estop_anywhere(self) -> None:
        with self.assertRaises(InvalidSystemTransition):
            decide_system_transition(
                SystemState.BOOT,
                SystemEvent.START_REQUEST,
                "invalid direct start",
            )
        self.assertIn(SystemEvent.ESTOP_ASSERTED, allowed_system_events(SystemState.BOOT))
        estop = decide_system_transition(
            SystemState.BOOT,
            SystemEvent.ESTOP_ASSERTED,
            "physical E-stop",
        )
        self.assertEqual(estop.rule_id, "SYS-15")
        self.assertEqual(estop.current, SystemState.FAULT_STOP)

    def test_station_aggregation_sensor3_and_fifo_reorder(self) -> None:
        ledger = ProductLedger()
        first = ledger.register("product-1", 1)
        first.record_sensor(1, "sensor1-product-1", 100)
        self._complete_station(first, StationId.A, verdict=Verdict.PASS)
        self._move_to_station_b(first)
        self._complete_station(first, StationId.B, verdict=Verdict.PASS)
        self.assertIsNone(first.locked)
        first_locked = first.lock_at_sensor3("sensor3-product-1", 300)
        self.assertEqual(first_locked.verdict, Verdict.PASS)
        self.assertFalse(
            first.apply_station_result(
                StationDecision(
                    StationId.B,
                    Verdict.NG,
                    2,
                    "capture-b-product-1",
                    "job-b",
                    "batch-b",
                )
            )
        )

        second = ledger.register("product-2", 2)
        second.record_sensor(1, "sensor1-product-2", 400)
        self._complete_station(second, StationId.A, verdict=None)
        self._move_to_station_b(second)
        self._complete_station(second, StationId.B, verdict=Verdict.PASS)
        second_locked = second.lock_at_sensor3("sensor3-product-2", 600)
        self.assertEqual(second_locked.verdict, Verdict.FORCED_NG)
        self.assertFalse(second_locked.station_a_completed)
        self.assertTrue(second_locked.station_b_completed)

        reorder = ProductResultReorderBuffer()
        self.assertEqual(reorder.add(second_locked), [])
        emitted = reorder.add(first_locked)
        self.assertEqual([item.fifo_sequence for item in emitted], [1, 2])

    def test_capture_failure_becomes_forced_ng_only_at_sensor3(self) -> None:
        context = ProductLedger().register("product", 1)
        context.record_sensor(1, "sensor1-product", 100)
        self._complete_station(
            context,
            StationId.A,
            verdict=None,
            capture_failed=True,
        )
        self.assertIsNone(context.locked)
        self._move_to_station_b(context)
        self._complete_station(context, StationId.B, verdict=Verdict.PASS)
        locked = context.lock_at_sensor3("sensor3-product", 300)
        self.assertEqual(locked.verdict, Verdict.FORCED_NG)
        self.assertIn("final capture failed", locked.reason)

    def test_vision_result_can_precede_capture_action_result(self) -> None:
        context = ProductLedger().register("race-product", 1)
        context.record_sensor(1, "sensor1-race", 100)
        context.begin_station_cycle(
            StationId.A,
            position_command_id="position-a",
            target_step=100,
            capture_id="capture-race",
        )
        context.mark_position_action_succeeded(StationId.A, "position-a")
        context.mark_position_settled(StationId.A, "position-a")
        context.mark_capture_requested(
            StationId.A,
            capture_id="capture-race",
            command_id="capture-command-race",
        )
        context.apply_station_result(
            StationDecision(
                StationId.A,
                Verdict.PASS,
                1,
                "capture-race",
                "job-race",
                "batch-race",
            )
        )
        context.promote_capture_completion_from_vision(StationId.A)
        self.assertEqual(
            context.physical_state,
            ProductPhysicalState.STATION_A_DONE,
        )
        self.assertEqual(
            context.station(StationId.A).frame_batch_id,
            "batch-race",
        )
        context.mark_conveyor_resumed_after_capture(StationId.A)
        self.assertEqual(context.physical_state, ProductPhysicalState.FLIPPING)

    def test_same_station_revision_with_other_content_is_conflict(self) -> None:
        context = ProductLedger().register("product", 1)
        first = StationDecision(StationId.A, Verdict.PASS, 1, "capture", "job")
        conflicting = replace(first, verdict=Verdict.NG)
        self.assertTrue(context.apply_station_result(first))
        self.assertFalse(context.apply_station_result(first))
        with self.assertRaises(StationResultConflict):
            context.apply_station_result(conflicting)

    def test_duplicate_position_settled_does_not_rewind_capture_state(self) -> None:
        context = ProductLedger().register("product", 1)
        context.begin_station_cycle(
            StationId.A,
            position_command_id="position-a",
            target_step=100,
            capture_id="capture-a",
        )
        context.mark_position_action_succeeded(StationId.A, "position-a")
        context.mark_position_settled(StationId.A, "position-a")
        context.mark_capture_requested(
            StationId.A,
            capture_id="capture-a",
            command_id="capture-command-a",
        )
        revision = context.revision

        context.mark_position_settled(StationId.A, "position-a")

        station = context.station(StationId.A)
        self.assertEqual(
            station.process_state,
            StationProcessState.CAPTURE_REQUESTED,
        )
        self.assertEqual(context.revision, revision)

    def test_sensor_registry_rejects_duplicate_conflict_gap_and_out_of_order(self) -> None:
        registry = SensorEventRegistry()
        self.assertEqual(
            registry.accept("S1", "event-1", 1, "digest-1"),
            SensorEventOutcome.ACCEPTED,
        )
        self.assertEqual(
            registry.accept("S1", "event-1", 1, "digest-1"),
            SensorEventOutcome.DUPLICATE,
        )
        self.assertEqual(
            registry.accept("S1", "event-1", 1, "other"),
            SensorEventOutcome.CONFLICT,
        )
        self.assertEqual(
            registry.accept("S1", "event-3", 3, "digest-3"),
            SensorEventOutcome.SEQUENCE_GAP,
        )
        self.assertEqual(
            registry.accept("S1", "event-0", 0, "digest-0"),
            SensorEventOutcome.OUT_OF_ORDER,
        )

    def test_sensor_registry_bounds_remembered_event_ids(self) -> None:
        registry = SensorEventRegistry(event_capacity=2)
        for sequence in range(1, 4):
            self.assertEqual(
                registry.accept(
                    "S1",
                    f"event-{sequence}",
                    sequence,
                    f"digest-{sequence}",
                ),
                SensorEventOutcome.ACCEPTED,
            )
        self.assertEqual(registry.remembered_event_count, 2)
        self.assertEqual(
            registry.accept("S1", "event-1", 1, "digest-1"),
            SensorEventOutcome.OUT_OF_ORDER,
        )

    def test_fifo_removes_only_contiguous_completed_prefix(self) -> None:
        ledger = ProductLedger()
        first = ledger.register("first", 1)
        second = ledger.register("second", 2)
        first.physical_state = ProductPhysicalState.DONE
        second.physical_state = ProductPhysicalState.DONE
        second.completed = True
        self.assertEqual(ledger.remove_completed_prefix(), [])
        first.completed = True
        self.assertEqual(
            [item.product_id for item in ledger.remove_completed_prefix()],
            ["first", "second"],
        )

    def test_completed_context_is_pruned_after_retention_with_tombstone(self) -> None:
        ledger = ProductLedger()
        context = ledger.register("retained", 1)
        ledger.register_capture("capture-retained", "retained", StationId.A)
        context.completed = True
        context.physical_state = ProductPhysicalState.DONE
        ledger.remove_completed_prefix(now_ns=1_000)
        self.assertEqual(ledger.prune_removed(cutoff_ns=999), [])
        self.assertEqual(ledger.prune_removed(cutoff_ns=1_000), ["retained"])
        self.assertIsNone(ledger.get("retained", 1))
        self.assertTrue(ledger.is_retired_product("retained", 1))
        self.assertTrue(ledger.is_retired_capture("capture-retained"))

    def test_equipment_guards_require_explicit_physical_confirmation(self) -> None:
        snapshot = EquipmentSnapshot()
        self.assertFalse(snapshot.in_place_guards_satisfied())
        snapshot.mark_all_stopped()
        snapshot.actuator_safe = True
        self.assertTrue(snapshot.in_place_guards_satisfied())
        snapshot.sensor_clear = {1: True, 2: True, 3: True}
        snapshot.actuator_area_clear = True
        snapshot.line_clear_confirmed = True
        snapshot.operator_id = "operator"
        self.assertTrue(snapshot.line_clear_guards_satisfied())
        snapshot.conveyor_running[ConveyorId.UPPER] = True
        snapshot.conveyor_stopped[ConveyorId.UPPER] = False
        self.assertFalse(snapshot.line_clear_guards_satisfied())


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

    def test_queue_force_removes_jobs_after_total_deadline(self) -> None:
        queue = InferenceQueue(capacity=2)
        expired = replace(
            self._job(1),
            enqueued_monotonic_ns=time.monotonic_ns() - 10_000_000,
            queue_total_timeout_ms=1,
        )
        active = self._job(2)
        self.assertTrue(queue.try_enqueue(expired))
        self.assertTrue(queue.try_enqueue(active))
        removed = queue.discard_expired()
        self.assertEqual([job.inference_job_id for job in removed], ["job-1"])
        self.assertEqual(queue.get(), active)
        queue.close()

    def test_worker_does_not_publish_success_after_total_deadline(self) -> None:
        queue = InferenceQueue(capacity=1)
        shared_model = object()
        successes: list[str] = []
        failures: list[tuple[str, str]] = []
        completed = threading.Event()

        def infer(model: object, _images: tuple[bytes, ...]) -> str:
            self.assertIs(model, shared_model)
            time.sleep(0.02)
            return "PASS"

        pool: WorkerPool[object, bytes, str] = WorkerPool(
            queue=queue,
            model=shared_model,
            worker_count=1,
            load_image=lambda _path: b"rgb",
            infer=infer,
            on_success=lambda job, _result: (
                successes.append(job.inference_job_id),
                completed.set(),
            ),
            on_failure=lambda job, reason: (
                failures.append((job.inference_job_id, reason)),
                completed.set(),
            ),
        )
        job = replace(
            self._job(1),
            enqueued_monotonic_ns=0,
            queue_total_timeout_ms=5,
        )
        self.assertTrue(queue.try_enqueue(job))
        pool.start()
        self.assertTrue(completed.wait(1.0))
        pool.stop()
        self.assertEqual(successes, [])
        self.assertEqual(failures, [("job-1", "queue total inference timeout")])

    def test_capture_batch_validates_rgb_png_digest_and_host_skew(self) -> None:
        png = make_rgb8_png()
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
                    width=1,
                    height=1,
                    pixel_format="RGB8_PNG",
                    camera_timestamp_raw=index,
                    camera_timestamp_domain="DEVICE_TICKS_UNSYNCED",
                    camera_timestamp_ns=index,
                    camera_timestamp_synchronized=False,
                    host_arrival_monotonic_ns=1_000_000 + index * 5_000,
                    host_arrival_timestamp_ns=1_000_000 + index * 5_000,
                )
                for index, camera in enumerate(("camera-a", "camera-b"))
            )
            batch = CaptureBatch(
                product_id="product",
                station_id=1,
                capture_id="capture",
                frame_batch_id="batch",
                attempt=1,
                trigger_requested_monotonic_ns=900_000,
                trigger_returned_monotonic_ns=950_000,
                trigger_requested_wall_time_ns=900_000,
                trigger_returned_wall_time_ns=950_000,
                images=artifacts,
            )
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
