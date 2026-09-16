"""ROS 없이 실행하는 v2 핵심 도메인 회귀 시험."""

from __future__ import annotations

import hashlib
import json
import struct
import sys
import tempfile
import threading
import time
import unittest
import zlib
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

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
    canonical_json,
    payload_digest,
    sha256_text,
)
from inspection_common.log_spool import DurableLogSpool, SpoolRecord  # noqa: E402
from inspection_log.storage import LogRepository, StoredLogEvent  # noqa: E402
from inspection_log.reporting import generate_session_report  # noqa: E402
from inspection_master.operation_runtime import (  # noqa: E402
    EquipmentSnapshot,
    build_late_operation_diagnostic,
    finite_float_or_none,
)
from inspection_master.product_flow import (  # noqa: E402
    LockedProduct,
    ProductLedger,
    ProductResultReorderBuffer,
    SensorEventOutcome,
    SensorEventRegistry,
    StationContractDefect,
    StationDecision,
    StationMessageFacts,
    StationMessageOutcome,
    StationProcessState,
    StationResultConflict,
    classify_station_message,
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
from inspection_vision.capture_contract import (  # noqa: E402
    CaptureBatch,
    ImageArtifact,
    MONO8_PNG,
    RGB8_PNG,
    write_mono8_png_atomic,
)
from inspection_vision.hikrobot_mvs import (  # noqa: E402
    ActionGroup,
    CameraAcquisitionSettings,
    CameraInventoryEntry,
    MvsBackendSettings,
    MvsSdkError,
    validate_camera_inventory,
)
from inspection_vision.inference_queue import (  # noqa: E402
    InferenceFailureKind,
    InferenceJob,
    InferenceQueue,
    WorkerPool,
)


def make_png(pixel_format: str, width: int = 1, height: int = 1) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        checksum = zlib.crc32(kind + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", checksum)

    channels = 1 if pixel_format == MONO8_PNG else 3
    color_type = 0 if pixel_format == MONO8_PNG else 2
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    scanlines = b"".join(
        b"\x00" + b"\x00" * channels * width for _ in range(height)
    )
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
    def test_non_finite_vision_scores_are_rejected_from_contracts(self) -> None:
        self.assertEqual(finite_float_or_none(0.97), 0.97)
        self.assertIsNone(finite_float_or_none(float("nan")))
        self.assertIsNone(finite_float_or_none(float("inf")))
        self.assertIsNone(finite_float_or_none(float("-inf")))

    def test_late_operation_diagnostic_preserves_vision_evidence(self) -> None:
        payload = build_late_operation_diagnostic(
            "STATION_RESULT",
            "capture-001",
            {
                "operation": "caller-must-not-override",
                "correlation_id": "caller-must-not-override",
                "verdict": 1,
                "score": 0.97,
                "result_revision": 2,
                "model_version": "model-v3",
            },
        )

        self.assertEqual(payload["operation"], "STATION_RESULT")
        self.assertEqual(payload["correlation_id"], "capture-001")
        self.assertEqual(payload["verdict"], 1)
        self.assertEqual(payload["score"], 0.97)
        self.assertEqual(payload["result_revision"], 2)
        self.assertEqual(payload["model_version"], "model-v3")

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
            capture_id=capture_id,
        )
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

    def test_worker_health_latch_requires_initialize_retry(self) -> None:
        """RESET/DEGRADED 후 status poll이 아니라 InitializeNode를 재시도합니다."""

        state = WorkerRuntimeState(NodeId.CONTROL)
        state.action_ready = True
        state.status_verified = True
        state.ready = True

        state.invalidate_epoch_evidence(now_ns=10_000, interval_ns=500)
        self.assertEqual(state.phase, WorkerInitPhase.WAITING_STATUS)

        state.schedule_retry(
            now_ns=10_000,
            interval_ns=500,
            error_code=9002,
            reason="worker health must be restored by InitializeNode",
            manual_intervention_required=False,
        )
        self.assertEqual(state.phase, WorkerInitPhase.RETRY_WAIT)
        self.assertEqual(state.retry_due_ns, 10_500)
        self.assertFalse(state.action_ready)
        self.assertFalse(state.status_verified)
        self.assertFalse(state.ready)

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

    def test_station_a_terminal_ng_skips_station_b_and_cannot_be_revised(self) -> None:
        context = ProductLedger().register("terminal-ng", 1)
        context.record_sensor(1, "sensor1-terminal-ng", 100)
        self._complete_station(context, StationId.A, verdict=Verdict.NG)
        self.assertTrue(context.request_station_b_skip("Station A terminal NG"))
        context.accept_sensor2("sensor2-terminal-ng", 200)
        self.assertEqual(context.physical_state, ProductPhysicalState.SENSOR3_WAIT)
        self.assertEqual(
            context.station(StationId.B).process_state,
            StationProcessState.SKIPPED,
        )
        with self.assertRaises(StationResultConflict):
            context.apply_station_result(
                StationDecision(
                    StationId.A,
                    Verdict.PASS,
                    2,
                    "capture-a-terminal-ng",
                    "job-a",
                    "batch-a",
                )
            )
        locked = context.lock_at_sensor3("sensor3-terminal-ng", 300)
        self.assertIn(locked.verdict, {Verdict.NG, Verdict.FORCED_NG})
        self.assertFalse(locked.station_b_completed)

    def test_vision_result_can_precede_capture_action_result(self) -> None:
        context = ProductLedger().register("race-product", 1)
        context.record_sensor(1, "sensor1-race", 100)
        context.begin_station_cycle(
            StationId.A,
            position_command_id="position-a",
            capture_id="capture-race",
        )
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
            capture_id="capture-a",
        )
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
        snapshot.line_clear_confirmed = True
        snapshot.operator_id = "operator"
        self.assertTrue(snapshot.line_clear_guards_satisfied())
        snapshot.conveyor_running[ConveyorId.UPPER] = True
        snapshot.conveyor_stopped[ConveyorId.UPPER] = False
        self.assertFalse(snapshot.line_clear_guards_satisfied())


class StationMessageClassificationTests(unittest.TestCase):
    """ROS 없이 station result/failure 가드 체인을 직접 검증합니다."""

    CAPTURE_ID = "capture-a-product-1"

    def _facts(self, **overrides) -> StationMessageFacts:
        values: dict[str, object] = {
            "product_id": "product-1",
            "fifo_sequence": 1,
            "station_id_raw": int(StationId.A),
            "capture_id": self.CAPTURE_ID,
            "frame_batch_id": "batch-a",
            "inference_job_id": "job-a",
            "result_revision": 1,
            "verdict_raw": int(Verdict.PASS),
            "score": 0.5,
        }
        values.update(overrides)
        return StationMessageFacts(**values)  # type: ignore[arg-type]

    def _ledger_with_active_capture(self, capture_id: str | None = None):
        """Station A 촬영이 진행 중인 제품 하나를 가진 원장을 만듭니다."""

        active_capture_id = capture_id or self.CAPTURE_ID
        ledger = ProductLedger()
        context = ledger.register("product-1", 1)
        context.record_sensor(1, "sensor1-product-1", 100)
        context.begin_station_cycle(
            StationId.A,
            position_command_id="position-a",
            capture_id=active_capture_id,
        )
        ledger.register_capture(active_capture_id, "product-1", StationId.A)
        return ledger, context

    def _classify(self, ledger, facts, *, expects_verdict: bool = True):
        return classify_station_message(
            ledger, facts, expects_verdict=expects_verdict
        )

    def test_accepted_when_every_guard_passes(self) -> None:
        ledger, context = self._ledger_with_active_capture()
        decision = self._classify(ledger, self._facts())
        self.assertEqual(decision.outcome, StationMessageOutcome.ACCEPTED)
        self.assertEqual(decision.station_id, StationId.A)
        self.assertIs(decision.context, context)
        self.assertEqual(decision.verdict, Verdict.PASS)

    def test_invalid_station_id_is_reported_before_the_ledger_is_read(self) -> None:
        ledger = ProductLedger()
        decision = self._classify(ledger, self._facts(station_id_raw=7))
        self.assertEqual(decision.outcome, StationMessageOutcome.INVALID_STATION_ID)
        self.assertIsNone(decision.station_id)
        self.assertIsNone(decision.context)

    def test_retired_product_is_expired_not_unknown(self) -> None:
        ledger, _ = self._ledger_with_active_capture()
        ledger.clear_active(now_ns=1)
        ledger.prune_removed(cutoff_ns=2)
        self.assertIsNone(ledger.get("product-1", 1))
        decision = self._classify(ledger, self._facts())
        self.assertEqual(decision.outcome, StationMessageOutcome.EXPIRED_PRODUCT)

    def test_missing_product_with_registered_capture_is_owner_conflict(self) -> None:
        ledger = ProductLedger()
        ledger.register_capture(self.CAPTURE_ID, "another-product", StationId.A)
        decision = self._classify(ledger, self._facts())
        self.assertEqual(
            decision.outcome, StationMessageOutcome.CAPTURE_OWNER_CONFLICT
        )

    def test_missing_product_without_capture_owner_is_unknown(self) -> None:
        decision = self._classify(ProductLedger(), self._facts())
        self.assertEqual(decision.outcome, StationMessageOutcome.UNKNOWN_PRODUCT)

    def test_capture_not_registered_to_this_product_and_station(self) -> None:
        ledger = ProductLedger()
        context = ledger.register("product-1", 1)
        context.record_sensor(1, "sensor1-product-1", 100)
        decision = self._classify(ledger, self._facts())
        self.assertEqual(
            decision.outcome, StationMessageOutcome.UNREGISTERED_CAPTURE
        )

    def test_product_removed_from_the_active_fifo_is_ignored(self) -> None:
        ledger, _ = self._ledger_with_active_capture()
        ledger.clear_active(now_ns=1)
        decision = self._classify(ledger, self._facts())
        self.assertEqual(decision.outcome, StationMessageOutcome.REMOVED_PRODUCT)

    def test_result_for_a_superseded_capture_reports_the_active_capture_id(
        self,
    ) -> None:
        ledger, _ = self._ledger_with_active_capture()
        ledger.register_capture("capture-a-old", "product-1", StationId.A)
        decision = self._classify(ledger, self._facts(capture_id="capture-a-old"))
        self.assertEqual(decision.outcome, StationMessageOutcome.SUPERSEDED_CAPTURE)
        self.assertEqual(decision.active_capture_id, self.CAPTURE_ID)

    def test_result_arriving_after_sensor3_lock_is_late(self) -> None:
        ledger, context = self._ledger_with_active_capture()
        context.locked = LockedProduct(
            product_id="product-1",
            fifo_sequence=1,
            verdict=Verdict.FORCED_NG,
            station_a_completed=False,
            station_b_completed=False,
            reason="station result incomplete at Sensor3",
            sensor3_event_id="sensor3-product-1",
        )
        decision = self._classify(ledger, self._facts())
        self.assertEqual(decision.outcome, StationMessageOutcome.LATE_AFTER_LOCK)

    def test_non_finite_score_is_reported_as_its_own_defect(self) -> None:
        ledger, _ = self._ledger_with_active_capture()
        decision = self._classify(ledger, self._facts(score=None))
        self.assertEqual(decision.outcome, StationMessageOutcome.CONTRACT_INCOMPLETE)
        self.assertEqual(decision.defect, StationContractDefect.SCORE_NOT_FINITE)

    def test_incomplete_payload_fields_are_contract_defects(self) -> None:
        ledger, _ = self._ledger_with_active_capture()
        for override in (
            {"frame_batch_id": ""},
            {"inference_job_id": ""},
            {"result_revision": 0},
            {"verdict_raw": 99},
        ):
            with self.subTest(override=override):
                decision = self._classify(ledger, self._facts(**override))
                self.assertEqual(
                    decision.outcome, StationMessageOutcome.CONTRACT_INCOMPLETE
                )
                self.assertEqual(
                    decision.defect, StationContractDefect.PAYLOAD_INCOMPLETE
                )

    def test_failure_messages_do_not_require_verdict_or_score(self) -> None:
        """StationInferenceFailed에는 verdict/score가 없으므로 검사하지 않습니다."""

        ledger, _ = self._ledger_with_active_capture()
        decision = self._classify(
            ledger,
            self._facts(verdict_raw=None, score=None),
            expects_verdict=False,
        )
        self.assertEqual(decision.outcome, StationMessageOutcome.ACCEPTED)
        self.assertIsNone(decision.verdict)

    def test_classification_never_mutates_the_ledger(self) -> None:
        """판정은 읽기 전용입니다. 콜백이 반영하기 전에는 원장이 그대로여야 합니다."""

        ledger, context = self._ledger_with_active_capture()
        before = context.revision
        for facts in (
            self._facts(),
            self._facts(station_id_raw=7),
            self._facts(score=None),
            self._facts(capture_id="capture-a-old"),
        ):
            self._classify(ledger, facts)
        self.assertEqual(context.revision, before)
        self.assertIsNone(context.station(StationId.A).decision)
        self.assertFalse(context.station(StationId.A).failed)
        self.assertFalse(context.station(StationId.A).conflicted)


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
        failures: list[tuple[str, InferenceFailureKind, str]] = []
        completed = threading.Event()

        def infer(model: object, _images: tuple[bytes, ...]) -> str:
            self.assertIs(model, shared_model)
            time.sleep(0.02)
            return "PASS"

        pool: WorkerPool[object, bytes, str] = WorkerPool(
            queue=queue,
            model=shared_model,
            worker_count=1,
            load_image=lambda _path: b"mono",
            infer=infer,
            on_success=lambda job, _result: (
                successes.append(job.inference_job_id),
                completed.set(),
            ),
            on_failure=lambda job, failure: (
                failures.append((job.inference_job_id, failure.kind, failure.reason)),
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
        self.assertEqual(
            failures,
            [("job-1", InferenceFailureKind.TIMEOUT, "queue total inference timeout")],
        )

    def test_worker_loads_station_views_in_parallel_and_preserves_order(self) -> None:
        queue = InferenceQueue(capacity=1)
        barrier = threading.Barrier(3)
        completed = threading.Event()
        loaded_order: list[tuple[str, ...]] = []

        def load(path: Path) -> str:
            barrier.wait(timeout=1.0)
            return path.name

        pool: WorkerPool[object, str, str] = WorkerPool(
            queue=queue,
            model=object(),
            worker_count=1,
            image_load_worker_count=3,
            load_image=load,
            infer=lambda _model, images: loaded_order.append(images) or "PASS",
            on_success=lambda _job, _result: completed.set(),
            on_failure=lambda _job, _failure: completed.set(),
        )
        job = replace(
            self._job(1),
            image_paths=("/tmp/A1.png", "/tmp/A2.png", "/tmp/A3.png"),
        )
        self.assertTrue(queue.try_enqueue(job))
        pool.start()
        self.assertTrue(completed.wait(2.0))
        pool.stop()
        self.assertEqual(loaded_order, [("A1.png", "A2.png", "A3.png")])

    def test_active_forward_finishes_but_result_is_discarded_after_cancel(self) -> None:
        queue = InferenceQueue(capacity=1)
        forward_started = threading.Event()
        release_forward = threading.Event()
        terminal = threading.Event()
        successes: list[str] = []
        canceled: list[tuple[str, str]] = []

        def infer(_model: object, _images: tuple[bytes, ...]) -> str:
            forward_started.set()
            release_forward.wait(1.0)
            return "PASS"

        pool: WorkerPool[object, bytes, str] = WorkerPool(
            queue=queue,
            model=object(),
            worker_count=1,
            load_image=lambda _path: b"mono",
            infer=infer,
            on_success=lambda job, _result: successes.append(job.inference_job_id),
            on_failure=lambda _job, _failure: terminal.set(),
            on_canceled=lambda job, stage: (
                canceled.append((job.inference_job_id, stage)),
                terminal.set(),
            ),
        )
        job = self._job(1, product="cancel-active")
        self.assertTrue(queue.try_enqueue(job))
        pool.start()
        self.assertTrue(forward_started.wait(1.0))
        self.assertEqual(queue.cancel_scope("cancel-active", 1), ())
        release_forward.set()
        self.assertTrue(terminal.wait(1.0))
        pool.stop()
        self.assertEqual(successes, [])
        self.assertEqual(canceled, [("job-1", "DISCARDED_AFTER_FORWARD")])

    def test_model_failure_is_not_retried(self) -> None:
        queue = InferenceQueue(capacity=1)
        attempts = 0
        completed = threading.Event()
        failures: list[InferenceFailureKind] = []

        def infer(_model: object, _images: tuple[bytes, ...]) -> str:
            nonlocal attempts
            attempts += 1
            raise RuntimeError("model failed")

        pool: WorkerPool[object, bytes, str] = WorkerPool(
            queue=queue,
            model=object(),
            worker_count=1,
            load_image=lambda _path: b"mono",
            infer=infer,
            on_success=lambda _job, _result: None,
            on_failure=lambda _job, failure: (
                failures.append(failure.kind),
                completed.set(),
            ),
        )
        self.assertTrue(queue.try_enqueue(self._job(1)))
        pool.start()
        self.assertTrue(completed.wait(1.0))
        pool.stop()
        self.assertEqual(attempts, 1)
        self.assertEqual(failures, [InferenceFailureKind.MODEL])

    def test_preprocessing_failure_is_not_retried_or_misclassified(self) -> None:
        class ForegroundMissing(RuntimeError):
            is_preprocessing_failure = True

        queue = InferenceQueue(capacity=1)
        attempts = 0
        completed = threading.Event()
        failures: list[InferenceFailureKind] = []

        def load(_path: Path) -> bytes:
            nonlocal attempts
            attempts += 1
            raise ForegroundMissing("no foreground")

        pool: WorkerPool[object, bytes, str] = WorkerPool(
            queue=queue,
            model=object(),
            worker_count=1,
            load_image=load,
            infer=lambda _model, _images: "PASS",
            on_success=lambda _job, _result: None,
            on_failure=lambda _job, failure: (
                failures.append(failure.kind),
                completed.set(),
            ),
        )
        self.assertTrue(queue.try_enqueue(self._job(1)))
        pool.start()
        self.assertTrue(completed.wait(1.0))
        pool.stop()
        self.assertEqual(attempts, 1)
        self.assertEqual(failures, [InferenceFailureKind.PREPROCESSING])

    def test_cuda_out_of_memory_is_fail_closed_and_not_retried(self) -> None:
        class FakeOutOfMemoryError(RuntimeError):
            pass

        queue = InferenceQueue(capacity=1)
        attempts = 0
        completed = threading.Event()
        failures: list[InferenceFailureKind] = []

        def infer(_model: object, _images: tuple[bytes, ...]) -> str:
            nonlocal attempts
            attempts += 1
            raise FakeOutOfMemoryError("synthetic allocation failure")

        pool: WorkerPool[object, bytes, str] = WorkerPool(
            queue=queue,
            model=object(),
            worker_count=1,
            load_image=lambda _path: b"mono",
            infer=infer,
            on_success=lambda _job, _result: None,
            on_failure=lambda _job, failure: (
                failures.append(failure.kind),
                completed.set(),
            ),
        )
        self.assertTrue(queue.try_enqueue(self._job(1)))
        pool.start()
        self.assertTrue(completed.wait(1.0))
        pool.stop()
        self.assertEqual(attempts, 1)
        self.assertEqual(failures, [InferenceFailureKind.CUDA_OOM])

    def test_capture_batch_validates_mono_png_digest_and_host_skew(self) -> None:
        png = make_png(MONO8_PNG)
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
                    pixel_format=MONO8_PNG,
                    camera_timestamp_raw=index,
                    camera_timestamp_domain="DEVICE_TICKS_UNSYNCED",
                    camera_timestamp_ns=index,
                    camera_timestamp_synchronized=False,
                    host_arrival_monotonic_ns=1_000_000 + index * 5_000,
                    host_arrival_timestamp_ns=1_000_000 + index * 5_000,
                    packet_loss_count=0,
                    packet_resend_count=0,
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
            batch.validate(
                ("camera-a", "camera-b"), expected_pixel_format=MONO8_PNG
            )
            self.assertEqual(batch.frame_arrival_skew_us, 5)

            with self.assertRaisesRegex(ValueError, "unrecovered packet loss"):
                replace(
                    batch,
                    images=(replace(artifacts[0], packet_loss_count=1), artifacts[1]),
                ).validate(("camera-a", "camera-b"), expected_pixel_format=MONO8_PNG)

    def test_capture_batch_validates_station_a_images_in_parallel(self) -> None:
        png = make_png(MONO8_PNG)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "image.png"
            path.write_bytes(png)
            digest = hashlib.sha256(png).hexdigest()
            artifacts = tuple(
                ImageArtifact(
                    camera_id=f"camera-{index}",
                    file_path=str(path.resolve()),
                    sha256=digest,
                    file_size_bytes=len(png),
                    width=1,
                    height=1,
                    pixel_format=MONO8_PNG,
                    camera_timestamp_raw=index,
                    camera_timestamp_domain="DEVICE_TICKS_UNSYNCED",
                    camera_timestamp_ns=index,
                    camera_timestamp_synchronized=False,
                    host_arrival_monotonic_ns=1_000_000 + index,
                    host_arrival_timestamp_ns=1_000_000 + index,
                    packet_loss_count=0,
                    packet_resend_count=0,
                )
                for index in range(3)
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
            barrier = threading.Barrier(3)
            validated: list[str] = []

            def validate_image(image: ImageArtifact, **_kwargs) -> None:
                validated.append(image.camera_id)
                barrier.wait(timeout=1.0)

            with patch(
                "inspection_vision.capture_contract._validate_image_artifact",
                side_effect=validate_image,
            ):
                batch.validate(
                    tuple(image.camera_id for image in artifacts),
                    expected_pixel_format=MONO8_PNG,
                )
            self.assertCountEqual(
                validated,
                ["camera-0", "camera-1", "camera-2"],
            )

    def test_capture_batch_rejects_declared_mono_with_rgb_png(self) -> None:
        png = make_png(RGB8_PNG)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rgb.png"
            path.write_bytes(png)
            artifact = ImageArtifact(
                camera_id="camera-a",
                file_path=str(path.resolve()),
                sha256=hashlib.sha256(png).hexdigest(),
                file_size_bytes=len(png),
                width=1,
                height=1,
                pixel_format=MONO8_PNG,
                camera_timestamp_raw=1,
                camera_timestamp_domain="DEVICE_TICKS_UNSYNCED",
                camera_timestamp_ns=1,
                camera_timestamp_synchronized=False,
                host_arrival_monotonic_ns=1,
                host_arrival_timestamp_ns=1,
                packet_loss_count=0,
                packet_resend_count=0,
            )
            batch = CaptureBatch(
                product_id="product",
                station_id=1,
                capture_id="capture",
                frame_batch_id="batch",
                attempt=1,
                trigger_requested_monotonic_ns=1,
                trigger_returned_monotonic_ns=2,
                trigger_requested_wall_time_ns=1,
                trigger_returned_wall_time_ns=2,
                images=(artifact,),
            )
            with self.assertRaisesRegex(ValueError, "color type"):
                batch.validate(("camera-a",), expected_pixel_format=MONO8_PNG)

    def test_atomic_mono8_png_writer_refuses_completed_file_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory).resolve() / "mono.png"
            digest, size_bytes = write_mono8_png_atomic(
                path, bytes((0, 64, 128, 255)), 2, 2
            )
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
            self.assertEqual(path.stat().st_size, size_bytes)
            with self.assertRaises(FileExistsError):
                write_mono8_png_atomic(path, bytes(4), 2, 2)
            self.assertFalse(any(path.parent.glob("*.part")))

    @staticmethod
    def _mvs_inventory(firmware_a: str, firmware_b: str | None = None):
        serials = ("A1", "A2", "A3", "B1")
        addresses = ("192.168.10.13", "192.168.10.11", "192.168.10.14", "192.168.10.12")
        return tuple(
            CameraInventoryEntry(
                serial=serial,
                station_id=1 if index < 3 else 2,
                ip_address=addresses[index],
                model_name="MV-CS050-10GC",
                firmware_version=(
                    firmware_b if index == 3 and firmware_b is not None else firmware_a
                ),
            )
            for index, serial in enumerate(serials)
        )

    def test_mvs_firmware_empty_expectation_requires_four_versions_to_match(self) -> None:
        entries = self._mvs_inventory("V1.2.3")
        network_map = {entry.serial: entry.ip_address for entry in entries}
        version = validate_camera_inventory(
            entries,
            expected_serials=("A1", "A2", "A3", "B1"),
            expected_network_map=network_map,
            expected_model="MV-CS050-10GC",
            expected_firmware_version="",
        )
        self.assertEqual(version, "V1.2.3")

        with self.assertRaisesRegex(MvsSdkError, "firmware versions differ"):
            validate_camera_inventory(
                self._mvs_inventory("V1.2.3", "V1.2.4"),
                expected_serials=("A1", "A2", "A3", "B1"),
                expected_network_map=network_map,
                expected_model="MV-CS050-10GC",
                expected_firmware_version="",
            )

    def test_mvs_firmware_exact_expectation_is_fail_closed(self) -> None:
        entries = self._mvs_inventory("V4.0.43 250414 1530132")
        network_map = {entry.serial: entry.ip_address for entry in entries}
        version = validate_camera_inventory(
            entries,
            expected_serials=("A1", "A2", "A3", "B1"),
            expected_network_map=network_map,
            expected_model="MV-CS050-10GC",
            expected_firmware_version="V4.0.43 250414 1530132",
        )
        self.assertEqual(version, "V4.0.43 250414 1530132")
        with self.assertRaisesRegex(MvsSdkError, "configured expected version"):
            validate_camera_inventory(
                entries,
                expected_serials=("A1", "A2", "A3", "B1"),
                expected_network_map=network_map,
                expected_model="MV-CS050-10GC",
                expected_firmware_version="V4.0.44",
            )

    def test_mvs_action_groups_and_initial_operating_defaults(self) -> None:
        settings = MvsBackendSettings(
            station_camera_ids={1: ("A1", "A2", "A3"), 2: ("B1",)},
            camera_network_map={
                "A1": "192.168.10.13",
                "A2": "192.168.10.11",
                "A3": "192.168.10.14",
                "B1": "192.168.10.12",
            },
            camera_acquisition_settings={
                serial: CameraAcquisitionSettings(5000.0, 0.0)
                for serial in ("A1", "A2", "A3", "B1")
            },
            action_device_key=1,
            action_groups={1: ActionGroup(1, 1), 2: ActionGroup(2, 2)},
            data_root=Path(tempfile.gettempdir()).resolve() / "inspection-images",
            acquisition_timeout_ms=250,
        )
        settings.validate()
        self.assertEqual(settings.packet_size, 1500)
        self.assertEqual(settings.acquisition_timeout_ms, 250)
        self.assertEqual(settings.action_groups[1], ActionGroup(1, 1))
        self.assertEqual(settings.action_groups[2], ActionGroup(2, 2))

        config_root = (
            WORKSPACE
            / "src"
            / "basic_packages"
            / "inspection_bringup"
            / "config"
        )
        capture_config = (config_root / "vision_capture.hardware.yaml").read_text(
            encoding="utf-8"
        )
        runtime_config = (config_root / "vision_runtime.hardware.yaml").read_text(
            encoding="utf-8"
        )
        model_config = (config_root / "vision_model.hardware.yaml").read_text(
            encoding="utf-8"
        )
        for expected_line in (
            "vision.image.sensor_width: 2448",
            "vision.image.sensor_height: 2048",
            "vision.gige_action.station_a.group_key: 1",
            "vision.gige_action.station_b.group_key: 2",
            'vision.camera.expected_firmware_version: "V4.0.43 250414 1530132"',
            'vision.nic.ipv4: "192.168.10.10"',
            "vision.nic.prefix_length: 24",
            "vision.frame_arrival_skew_limit_us: 50000",
            "vision.capture.acquisition_timeout_ms: 250",
        ):
            self.assertIn(expected_line, capture_config)
        hardware_config = (config_root / "hardware.yaml").read_text(encoding="utf-8")
        self.assertIn('control.mega.port: "/dev/ttyACM0"', hardware_config)
        self.assertIn("control.mega.baud_rate: 115200", hardware_config)
        self.assertIn("vision.queue.capacity: 16", runtime_config)
        self.assertIn("vision.inference.station_a.total_timeout_ms: 3000", runtime_config)
        self.assertIn("vision.inference.station_b.total_timeout_ms: 1500", runtime_config)
        self.assertIn("vision.worker_count: 1", model_config)
        self.assertIn("vision.model.serialize_access: true", model_config)


class LogContractTests(unittest.TestCase):
    @staticmethod
    def _durable_event(
        *, event_type: str, payload: dict[str, object], log_id: str
    ) -> StoredLogEvent:
        envelope = {
            "schema_version": 2,
            "event_type": event_type,
            "severity": 20,
            "source_node": "vision",
            "producer_instance_id": "vision-instance",
            "session_id": "session-1",
            "product_id": str(payload.get("product_id", "")),
            "payload": payload,
        }
        payload_json = canonical_json(envelope)
        return StoredLogEvent(
            log_id=log_id,
            revision=1,
            severity=20,
            event_type=event_type,
            source_node="vision",
            producer_instance_id="vision-instance",
            product_id=str(payload.get("product_id", "")),
            payload_json=payload_json,
            payload_digest=sha256_text(payload_json),
            occurred_at_ns=1,
        )

    def test_durable_vision_terminal_can_be_replayed_and_reported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            repository = LogRepository(root / "log.sqlite3")
            result_payload = {
                "product_id": "product-1",
                "fifo_sequence": 1,
                "station_id": 1,
                "capture_id": "capture-a",
                "frame_batch_id": "batch-a",
                "inference_job_id": "job-a",
                "result_revision": 1,
                "verdict": int(Verdict.NG),
                "score": 0.9,
                "model_version": "Model_v_1",
                "model_sha256": "0" * 64,
                "config_fingerprint": "1" * 64,
                "model_forward_ms": 2.0,
                "enqueue_to_result_ms": 3.0,
                "capture_completed_monotonic_ns": 10,
                "completed_monotonic_ns": 20,
                "image_paths": [],
            }
            repository.append_event(
                self._durable_event(
                    event_type="VISION_STATION_RESULT_DURABLE",
                    payload=result_payload,
                    log_id="vision-result",
                )
            )
            terminals, has_more = repository.replay_station_terminals(
                "session-1", 100
            )
            self.assertFalse(has_more)
            self.assertEqual(len(terminals), 1)
            self.assertEqual(terminals[0].inference_job_id, "job-a")
            report = generate_session_report(
                repository,
                session_id="session-1",
                report_root=root / "reports",
                timeout_tuning_path=root / "tuning.json",
                auto_apply_timeouts=False,
                minimum_samples=10000,
                safety_factor=1.2,
            )
            self.assertTrue(report.csv_path.is_file())
            self.assertTrue(report.summary_path.is_file())
            self.assertIsNone(report.tuning_path)
            tuned = generate_session_report(
                repository,
                session_id="session-1",
                report_root=root / "reports-auto",
                timeout_tuning_path=root / "tuning.json",
                auto_apply_timeouts=True,
                minimum_samples=1,
                safety_factor=1.2,
            )
            self.assertIsNone(tuned.tuning_path)
            self.assertFalse((root / "tuning.json").exists())

            station_b_payload = dict(result_payload)
            station_b_payload.update(
                {
                    "station_id": 2,
                    "capture_id": "capture-b",
                    "frame_batch_id": "batch-b",
                    "inference_job_id": "job-b",
                    "enqueue_to_result_ms": 2.0,
                }
            )
            repository.append_event(
                self._durable_event(
                    event_type="VISION_STATION_RESULT_DURABLE",
                    payload=station_b_payload,
                    log_id="vision-result-b",
                )
            )
            tuned = generate_session_report(
                repository,
                session_id="session-1",
                report_root=root / "reports-auto-complete",
                timeout_tuning_path=root / "tuning.json",
                auto_apply_timeouts=True,
                minimum_samples=1,
                safety_factor=1.2,
            )
            self.assertEqual(tuned.tuning_path, root / "tuning.json")
            tuning_document = json.loads(
                (root / "tuning.json").read_text(encoding="utf-8")
            )
            self.assertEqual(tuning_document["timeouts_ms"]["station_a"], 4)
            self.assertEqual(tuning_document["timeouts_ms"]["station_b"], 3)
            repository.close()

    def test_repository_replacement_can_close_previous_connection(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            database_path = Path(directory) / "log.sqlite3"
            previous_repository = LogRepository(database_path)
            replacement_repository = LogRepository(database_path)

            previous_repository.close()
            event = StoredLogEvent(
                log_id="replacement-log",
                revision=1,
                severity=20,
                event_type="REINITIALIZED",
                source_node="log",
                producer_instance_id="instance",
                product_id="",
                payload_json="{}",
                payload_digest=hashlib.sha256(b"{}").hexdigest(),
                occurred_at_ns=1,
            )
            replacement_repository.append_event(event)
            replacement_repository.close()

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
