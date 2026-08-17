"""MasterNode v2 공정 조율, 통신과 결과 소유권 구현."""

from __future__ import annotations

import json
import threading
import time
from functools import partial
from pathlib import Path

import rclpy
from inspection_common import (
    ConveyorId,
    DurableLogSpool,
    ErrorCode,
    IdempotencyStore,
    NodeHealthState,
    NodeId,
    PauseReason,
    ProductPhysicalState,
    ReplayKind,
    SpoolRecord,
    StationId,
    SystemState,
    Verdict,
    canonical_json,
    is_sha256_hex,
    is_uuid4,
    new_uuid,
    sha256_text,
)
from inspection_common.digest import payload_digest
from inspection_common.node_base import (
    InspectionNodeBase,
    heartbeat_qos,
    reliable_event_qos,
    spin_node,
    state_qos,
)
from inspection_interfaces.action import (
    ActuateProduct,
    CaptureProduct,
    InitializeNode,
    PositionProduct,
)
from inspection_interfaces.msg import (
    LogEvent,
    LogPersistedAck,
    NodeHeartbeat,
    PositionSettled,
    ProductResultLocked,
    SensorEvent,
    StationInferenceFailed,
    StationResult,
    SystemCommand,
    VisionQueueState,
)
from inspection_interfaces.srv import GetNodeStatus, OperatorCommand
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from .operation_runtime import (
    ActuationCycle,
    ActuationPhase,
    EquipmentSnapshot,
    ShutdownPhase,
    StationCycle,
    StationCyclePhase,
    build_late_operation_diagnostic,
    finite_float_or_none,
)
from .product_flow import (
    LockedProduct,
    ProductLedger,
    ProductFlowError,
    ProductIdentityConflict,
    ProductResultReorderBuffer,
    SensorEventOutcome,
    SensorEventRegistry,
    StationDecision,
    StationResultConflict,
)
from .system_fsm import (
    InvalidSystemTransition,
    RecoveryPolicy,
    SystemEvent,
    SystemTransition,
    decide_system_transition,
)
from .worker_supervision import WorkerInitPhase, WorkerRuntimeState


class MasterNode(InspectionNodeBase):
    """전체 공정 조율과 제품 추적을 단독 소유하는 Master.

    구현 블록은 다음 순서로 완성합니다.
    1. 전체 시스템 FSM과 운전 명령
    2. 작업 노드 초기화와 생존 감시
    3. 단일 물리 FIFO와 Sensor1/2/3 매핑
    4. Station A/B 위치 이동과 촬영
    5. 비동기 비전 결과와 최종 판정
    6. Sensor3 액추에이터 분류와 FIFO 제거
    7. PAUSE/RESET/FAULT_STOP 복구
    8. 로그 spool/ACK와 프로그램 종료

    남은 TODO 표기:
    - TODO(HARDWARE): 장비 시험 뒤 수치나 신호를 확정할 항목
    - TODO(OPERATION): 실제 운영 환경에서 임계값을 확정할 항목
    - TODO(TEST): ROS 2·실장비 통합 시험에서 확인할 항목
    """

    def __init__(self) -> None:
        super().__init__(NodeId.MASTER, provides_initialize_action=False)

        self.declare_parameter("system.config_version", "dev-001")
        self.declare_parameter("retry.init_interval_ms", 1000)
        self.declare_parameter("retry.warning_after_attempts", 3)
        self.declare_parameter("comm.node_heartbeat_timeout_ms", 2000)
        self.declare_parameter("system.init_timeout_ms", 10000)
        self.declare_parameter("master.fifo.soft_limit", 18)
        self.declare_parameter("master.fifo.hard_capacity", 20)
        self.declare_parameter("master.sensor_ids.sensor_1", "SENSOR_1")
        self.declare_parameter("master.sensor_ids.sensor_2", "SENSOR_2")
        self.declare_parameter("master.sensor_ids.sensor_3", "SENSOR_3")
        self.declare_parameter("master.sensor.accepted_edge", int(SensorEvent.RISING))
        self.declare_parameter("master.hardware_mapping_confirmed", False)
        self.declare_parameter("master.station_a.position_offset_steps", 0)
        self.declare_parameter("master.station_b.position_offset_steps", 0)
        self.declare_parameter("master.position_tolerance_steps", 0)
        self.declare_parameter(
            "master.camera_ids.station_a", Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter(
            "master.camera_ids.station_b", Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter("master.action.position_timeout_ms", 10000)
        self.declare_parameter("master.action.capture_timeout_ms", 30000)
        self.declare_parameter("master.action.actuation_timeout_ms", 10000)
        self.declare_parameter("master.action.conveyor_resume_timeout_ms", 10000)
        self.declare_parameter("master.pause_stop_timeout_ms", 10000)
        self.declare_parameter("master.shutdown_stop_timeout_ms", 10000)
        self.declare_parameter(
            "master.log_spool_path", "/tmp/inspection/spool/master.sqlite3"
        )
        self.declare_parameter("master.log_flush_period_ms", 1000)
        self.declare_parameter("master.log_spool_warning_bytes", 0)
        self.declare_parameter("master.log_spool_hard_bytes", 0)
        self.declare_parameter("master.completed_context_retention_ms", 600000)

        self.config_version = str(self.get_parameter("system.config_version").value)
        self.init_retry_interval_ms = int(
            self.get_parameter("retry.init_interval_ms").value
        )
        self.init_warning_after_attempts = int(
            self.get_parameter("retry.warning_after_attempts").value
        )
        self.node_heartbeat_timeout_ms = int(
            self.get_parameter("comm.node_heartbeat_timeout_ms").value
        )
        self.init_timeout_ms = int(self.get_parameter("system.init_timeout_ms").value)
        self.fifo_soft_limit = int(self.get_parameter("master.fifo.soft_limit").value)
        self.fifo_hard_capacity = int(
            self.get_parameter("master.fifo.hard_capacity").value
        )
        self.sensor_id_to_index = {
            str(self.get_parameter("master.sensor_ids.sensor_1").value): 1,
            str(self.get_parameter("master.sensor_ids.sensor_2").value): 2,
            str(self.get_parameter("master.sensor_ids.sensor_3").value): 3,
        }
        self.accepted_sensor_edge = int(
            self.get_parameter("master.sensor.accepted_edge").value
        )
        self.hardware_mapping_confirmed = bool(
            self.get_parameter("master.hardware_mapping_confirmed").value
        )
        self.station_position_offsets = {
            StationId.A: int(
                self.get_parameter("master.station_a.position_offset_steps").value
            ),
            StationId.B: int(
                self.get_parameter("master.station_b.position_offset_steps").value
            ),
        }
        self.position_tolerance_steps = int(
            self.get_parameter("master.position_tolerance_steps").value
        )
        station_a_camera_ids = self.get_parameter(
            "master.camera_ids.station_a"
        ).value
        station_b_camera_ids = self.get_parameter(
            "master.camera_ids.station_b"
        ).value
        self.station_camera_ids = {
            # Type-only STRING_ARRAY 선언은 ROS 배포판이나 빈 YAML override에
            # 따라 None으로 보일 수 있으므로, 미설정값을 빈 튜플로 정규화해
            # 프로세스 crash 대신 기존 fail-closed START guard로 보냅니다.
            StationId.A: tuple(station_a_camera_ids or ()),
            StationId.B: tuple(station_b_camera_ids or ()),
        }
        self.position_timeout_ms = int(
            self.get_parameter("master.action.position_timeout_ms").value
        )
        self.capture_timeout_ms = int(
            self.get_parameter("master.action.capture_timeout_ms").value
        )
        self.actuation_timeout_ms = int(
            self.get_parameter("master.action.actuation_timeout_ms").value
        )
        self.conveyor_resume_timeout_ms = int(
            self.get_parameter("master.action.conveyor_resume_timeout_ms").value
        )
        self.pause_stop_timeout_ms = int(
            self.get_parameter("master.pause_stop_timeout_ms").value
        )
        self.shutdown_stop_timeout_ms = int(
            self.get_parameter("master.shutdown_stop_timeout_ms").value
        )
        self.log_spool_path = str(
            self.get_parameter("master.log_spool_path").value
        )
        self.log_flush_period_ms = int(
            self.get_parameter("master.log_flush_period_ms").value
        )
        self.log_spool_warning_bytes = int(
            self.get_parameter("master.log_spool_warning_bytes").value
        )
        self.log_spool_hard_bytes = int(
            self.get_parameter("master.log_spool_hard_bytes").value
        )
        self.completed_context_retention_ms = int(
            self.get_parameter("master.completed_context_retention_ms").value
        )
        if not self.config_version:
            raise ValueError("system.config_version must not be empty")
        if not 100 <= self.init_retry_interval_ms <= 10000:
            raise ValueError("retry.init_interval_ms must be between 100 and 10000")
        if not 1 <= self.init_warning_after_attempts <= 100:
            raise ValueError("retry.warning_after_attempts must be between 1 and 100")
        if not 100 <= self.node_heartbeat_timeout_ms <= 60000:
            raise ValueError("comm.node_heartbeat_timeout_ms must be 100..60000")
        if not 1000 <= self.init_timeout_ms <= 60000:
            raise ValueError("system.init_timeout_ms must be between 1000 and 60000")
        if not 1 <= self.fifo_soft_limit <= self.fifo_hard_capacity:
            raise ValueError("FIFO limits must satisfy 1 <= soft_limit <= hard_capacity")
        if len(self.sensor_id_to_index) != 3 or any(
            not sensor_id for sensor_id in self.sensor_id_to_index
        ):
            raise ValueError("three unique non-empty sensor IDs are required")
        if self.accepted_sensor_edge not in {SensorEvent.RISING, SensorEvent.FALLING}:
            raise ValueError("master.sensor.accepted_edge must be RISING or FALLING")
        station_a_camera_count = len(self.station_camera_ids[StationId.A])
        station_b_camera_count = len(self.station_camera_ids[StationId.B])
        camera_mapping_is_present = bool(
            station_a_camera_count or station_b_camera_count
        )
        # hardware.yaml의 빈 배열은 장비 미확정을 나타내므로 프로세스를
        # 즉시 종료하지 않습니다. 대신 START guard가 fail-closed로 차단합니다.
        # 카메라 ID를 하나라도 입력했거나 sim이면 A 3대/B 1대를 엄격히
        # 검증합니다.
        if self.profile == "sim" or camera_mapping_is_present:
            if station_a_camera_count != 3:
                raise ValueError("master.camera_ids.station_a must contain 3 cameras")
            if station_b_camera_count != 1:
                raise ValueError("master.camera_ids.station_b must contain 1 camera")
            if set(self.station_camera_ids[StationId.A]) & set(
                self.station_camera_ids[StationId.B]
            ):
                raise ValueError("station camera sets must be disjoint")
        for name, value in (
            ("position", self.position_timeout_ms),
            ("capture", self.capture_timeout_ms),
            ("actuation", self.actuation_timeout_ms),
            ("conveyor resume", self.conveyor_resume_timeout_ms),
            ("pause stop", self.pause_stop_timeout_ms),
            ("shutdown stop", self.shutdown_stop_timeout_ms),
        ):
            if not 100 <= value <= 300000:
                raise ValueError(f"{name} timeout must be between 100 and 300000 ms")
        if not 100 <= self.log_flush_period_ms <= 60000:
            raise ValueError("master.log_flush_period_ms must be 100..60000")
        if (
            self.log_spool_warning_bytes < 0
            or self.log_spool_hard_bytes < 0
            or (
                self.log_spool_hard_bytes
                and self.log_spool_warning_bytes > self.log_spool_hard_bytes
            )
        ):
            raise ValueError("invalid Master log spool thresholds")
        if not 60_000 <= self.completed_context_retention_ms <= 86_400_000:
            raise ValueError(
                "master.completed_context_retention_ms must be 60000..86400000"
            )

        self.session_id = new_uuid()
        self.config_digest = payload_digest(
            {
                "config_version": self.config_version,
                "expected_interface_version": self.expected_interface_version,
                "profile": self.profile,
                "fifo_limits": [self.fifo_soft_limit, self.fifo_hard_capacity],
                "sensor_ids": sorted(self.sensor_id_to_index.items()),
                "accepted_sensor_edge": self.accepted_sensor_edge,
                "hardware_mapping_confirmed": self.hardware_mapping_confirmed,
                "station_position_offsets": {
                    "A": self.station_position_offsets[StationId.A],
                    "B": self.station_position_offsets[StationId.B],
                },
                "position_tolerance_steps": self.position_tolerance_steps,
                "station_camera_ids": {
                    "A": self.station_camera_ids[StationId.A],
                    "B": self.station_camera_ids[StationId.B],
                },
                "timeouts_ms": {
                    "position": self.position_timeout_ms,
                    "capture": self.capture_timeout_ms,
                    "actuation": self.actuation_timeout_ms,
                    "conveyor_resume": self.conveyor_resume_timeout_ms,
                    "pause_stop": self.pause_stop_timeout_ms,
                    "shutdown_stop": self.shutdown_stop_timeout_ms,
                },
                "completed_context_retention_ms": (
                    self.completed_context_retention_ms
                ),
            }
        )
        self.ledger = ProductLedger()
        self.result_reorder = ProductResultReorderBuffer()
        self.sensor_events = SensorEventRegistry()
        self.equipment = (
            EquipmentSnapshot.sim_safe()
            if self.profile == "sim"
            else EquipmentSnapshot()
        )
        self._flow_lock = threading.RLock()
        self._station_cycles: dict[StationId, StationCycle] = {}
        self._actuation_cycles: dict[str, ActuationCycle] = {}
        self._actuation_id_owners: IdempotencyStore[str] = IdempotencyStore(
            capacity=4096
        )
        self._deferred_station_starts: set[tuple[str, StationId]] = set()
        self._deferred_capture_resumes: set[tuple[str, StationId]] = set()
        self.pause_reason: PauseReason | None = None
        self.recovery_policy = RecoveryPolicy.NONE
        self._pending_run_confirmation = False
        self._shutdown_requested = False
        self._paused_by_queue = False
        self._queue_recovered_pending = False
        self._pause_deadline_ns = 0
        self._shutdown_deadline_ns = 0
        self._next_context_prune_ns = time.monotonic_ns() + 60_000_000_000
        self.shutdown_phase = ShutdownPhase.IDLE
        self._log_spool: DurableLogSpool | None = None
        self._log_spool_open_failure_reported = False
        if self.log_spool_path:
            try:
                self._log_spool = DurableLogSpool(Path(self.log_spool_path))
            except Exception as exc:
                self._log_spool_open_failure_reported = True
                self.set_health_state(NodeHealthState.DEGRADED)
                self.get_logger().error(
                    f"Master log spool open failed; retrying: {type(exc).__name__}"
                )
        elif self.profile == "hardware":
            self._log_spool_open_failure_reported = True
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().error(
                "master.log_spool_path is empty; INITIALIZING and START remain "
                "blocked until a durable path is configured"
            )
        self.worker_heartbeats: dict[NodeId, NodeHeartbeat] = {}
        self.worker_received_ns: dict[NodeId, int] = {}
        self.worker_states = {
            worker: WorkerRuntimeState(worker)
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        }
        self._initialize_goal_handles: dict[NodeId, object] = {}
        self._reported_worker_outages: set[NodeId] = set()
        self._initialization_completion_reported = False
        self._operator_results: IdempotencyStore[dict[str, object]] = (
            IdempotencyStore(capacity=512)
        )

        self._worker_heartbeat_subscriptions = [
            self.create_subscription(
                NodeHeartbeat,
                f"/inspection/{worker.value}/heartbeat",
                lambda message, worker_id=worker: self._handle_worker_heartbeat(
                    worker_id, message
                ),
                heartbeat_qos(),
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        ]
        self.initialize_clients = {
            worker: ActionClient(
                self, InitializeNode, f"/inspection/{worker.value}/initialize"
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        }
        self.status_clients = {
            worker: self.create_client(
                GetNodeStatus, f"/inspection/{worker.value}/get_status"
            )
            for worker in (NodeId.CONTROL, NodeId.VISION, NodeId.LOG)
        }
        self.position_client = ActionClient(
            self, PositionProduct, "/inspection/control/position_product"
        )
        self.capture_client = ActionClient(
            self, CaptureProduct, "/inspection/vision/capture_product"
        )
        self.actuate_client = ActionClient(
            self, ActuateProduct, "/inspection/control/actuate_product"
        )
        self._station_result_subscription = self.create_subscription(
            StationResult,
            "/inspection/vision/station_result",
            self._handle_station_result,
            reliable_event_qos(),
        )
        self._station_failure_subscription = self.create_subscription(
            StationInferenceFailed,
            "/inspection/vision/station_inference_failed",
            self._handle_station_failure,
            reliable_event_qos(),
        )
        self._sensor_subscription = self.create_subscription(
            SensorEvent,
            "/inspection/control/sensor_event",
            self._handle_sensor_event,
            reliable_event_qos(),
        )
        self._position_settled_subscription = self.create_subscription(
            PositionSettled,
            "/inspection/control/position_settled",
            self._handle_position_settled,
            reliable_event_qos(),
        )
        self._queue_subscription = self.create_subscription(
            VisionQueueState,
            "/inspection/vision/queue_state",
            self._handle_queue_state,
            state_qos(),
        )
        self._locked_publisher = self.create_publisher(
            ProductResultLocked,
            "master/product_result_locked",
            reliable_event_qos(),
        )
        self._system_command_publisher = self.create_publisher(
            SystemCommand,
            "master/system_command",
            reliable_event_qos(),
        )
        self._log_event_publisher = self.create_publisher(
            LogEvent,
            "log/event",
            reliable_event_qos(),
        )
        self._log_ack_subscription = self.create_subscription(
            LogPersistedAck,
            "/inspection/log/persisted_ack",
            self._handle_log_persisted_ack,
            reliable_event_qos(),
        )
        self._operator_command_service = self.create_service(
            OperatorCommand,
            "master/operator_command",
            self._handle_operator_command,
        )
        self._worker_watchdog = self.create_timer(0.5, self._check_worker_heartbeats)
        self._operation_watchdog = self.create_timer(
            0.1, self._check_operation_deadlines
        )
        self._log_flush_timer = self.create_timer(
            self.log_flush_period_ms / 1000.0, self._flush_log_spool
        )
        self.get_logger().info("MasterNode v2 orchestration started")

    # region BLOCK 1 - 전체 시스템 FSM과 운전 명령

    def _handle_operator_command(
        self,
        request: OperatorCommand.Request,
        response: OperatorCommand.Response,
    ) -> OperatorCommand.Response:
        """개발 CLI/HMI 요청을 Master의 공식 FSM 함수로만 전달합니다."""

        request_id = request.request_id or new_uuid()
        response.request_id = request_id
        if not is_uuid4(request_id):
            response.accepted = False
            response.system_state = int(self.system_state)
            response.system_state_name = self.system_state.name
            response.message = "request_id must be empty or UUIDv4"
            return response
        digest = payload_digest(
            {
                "command_type": int(request.command_type),
                "reason": request.reason,
                "operator_id": request.operator_id,
            }
        )
        replay = self._operator_results.inspect(request_id, digest)
        if replay.kind == ReplayKind.CONFLICT:
            values = {
                "accepted": False,
                "system_state": int(self.system_state),
                "system_state_name": self.system_state.name,
                "message": "same request_id received with another payload",
            }
            return self._fill_operator_response(response, values)
        if replay.kind == ReplayKind.REPLAY and replay.result is not None:
            return self._fill_operator_response(response, replay.result)

        reason = request.reason or "operator command"
        command_handlers = {
            int(OperatorCommand.Request.INITIALIZE): lambda: self.request_initialize(
                reason
            ),
            int(OperatorCommand.Request.START): lambda: self.request_run(reason),
            int(OperatorCommand.Request.PAUSE): lambda: self.request_pause(reason),
            int(OperatorCommand.Request.RESUME): lambda: self.request_resume(reason),
            int(OperatorCommand.Request.RESET): lambda: self.request_reset(reason),
            int(OperatorCommand.Request.CONFIRM_LINE_CLEAR): lambda: (
                self.confirm_line_cleared(request.operator_id)
            ),
        }
        handler = command_handlers.get(int(request.command_type))
        if handler is None:
            accepted = False
            message = "unknown operator command_type"
        elif (
            int(request.command_type)
            == int(OperatorCommand.Request.CONFIRM_LINE_CLEAR)
            and not request.operator_id
        ):
            accepted = False
            message = "operator_id is required for line-clear confirmation"
        else:
            accepted = bool(handler())
            message = (
                f"{reason}: accepted"
                if accepted
                else f"{reason}: rejected by current state or safety guard"
            )
        values = {
            "accepted": accepted,
            "system_state": int(self.system_state),
            "system_state_name": self.system_state.name,
            "message": message,
        }
        self._operator_results.remember(request_id, digest, values)
        return self._fill_operator_response(response, values)

    @staticmethod
    def _fill_operator_response(
        response: OperatorCommand.Response,
        values: dict[str, object],
    ) -> OperatorCommand.Response:
        response.accepted = bool(values["accepted"])
        response.system_state = int(values["system_state"])
        response.system_state_name = str(values["system_state_name"])
        response.message = str(values["message"])
        return response

    def _status_snapshot(self, **extra: object) -> str:
        """GetNodeStatus에 Master의 FIFO·복구·장비 mirror를 함께 보고합니다."""

        base = json.loads(super()._status_snapshot())
        with self._flow_lock:
            active = self.ledger.active_contexts()
        base.update(
            {
                "fifo_active_size": len(active),
                "fifo_soft_limit": self.fifo_soft_limit,
                "fifo_hard_capacity": self.fifo_hard_capacity,
                "active_product_ids": [item.product_id for item in active],
                "pause_reason": (
                    self.pause_reason.name if self.pause_reason is not None else ""
                ),
                "recovery_policy": self.recovery_policy.value,
                "shutdown_phase": self.shutdown_phase.value,
                "conveyor_running": {
                    key.name: value
                    for key, value in self.equipment.conveyor_running.items()
                },
                "conveyor_stopped": {
                    key.name: value
                    for key, value in self.equipment.conveyor_stopped.items()
                },
                "sensor_clear": dict(self.equipment.sensor_clear),
                "actuator_safe": self.equipment.actuator_safe,
                "line_clear_confirmed": self.equipment.line_clear_confirmed,
                **extra,
            }
        )
        return canonical_json(base)

    def request_initialize(self, reason: str = "operator initialize request") -> bool:
        """BOOT에서 INITIALIZING으로 진입하는 공통 시작점입니다."""

        transition = self._apply_system_event(SystemEvent.APP_STARTED, reason)
        if transition is not None:
            self._start_worker_initialization()
        return transition is not None

    def report_initialization_done(self, reason: str = "all workers ready") -> bool:
        """Block 2가 모든 노드의 현재 session 준비를 확인한 뒤 호출합니다."""

        if not self._all_workers_ready_for_session():
            self.get_logger().warning("INIT_DONE rejected: workers are not READY")
            return False
        if self._log_spool is None:
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().warning(
                "INIT_DONE delayed: Master durable log spool is unavailable"
            )
            return False
        transition = self._apply_system_event(SystemEvent.INIT_DONE, reason)
        return transition is not None

    def report_initialization_failed(
        self, reason: str, *, timed_out: bool = False
    ) -> bool:
        """안전 정지 상태를 유지하면서 INITIALIZING 재시도를 기록합니다."""

        event = SystemEvent.INIT_TIMEOUT if timed_out else SystemEvent.INIT_FAILED
        transition = self._apply_system_event(event, reason)
        return transition is not None

    def request_run(self, reason: str = "operator run request") -> bool:
        """READY에서 운전 명령을 보내되 실제 RUN 확인 전까지 READY를 유지합니다."""

        if not self._verify_start_conditions():
            self.get_logger().warning("START_REQUEST rejected: start guard failed")
            return False
        transition = self._apply_system_event(SystemEvent.START_REQUEST, reason)
        if transition is None:
            return False
        self._pending_run_confirmation = True
        self._request_conveyor_run(reason)
        return True

    def confirm_all_conveyors_running(
        self, reason: str = "all conveyors confirmed RUN_CONV"
    ) -> bool:
        """Control의 상·하층 실제 RUN 확인 후에만 RUN_SYS를 확정합니다."""

        if not self._pending_run_confirmation:
            self.get_logger().warning(
                "unexpected conveyor RUN confirmation without a pending request"
            )
            return False
        previous_state = self.system_state
        transition = self._apply_system_event(
            SystemEvent.ALL_CONVEYORS_RUNNING, reason
        )
        if transition is None:
            return False
        self._pending_run_confirmation = False
        self.equipment.mark_all_running()
        if previous_state == SystemState.READY:
            # 빈 라인 확인은 신규 RUN 시작에 한 번만 소비합니다.
            self.equipment.clear_operator_confirmation()
        self.pause_reason = None
        self._resume_deferred_operations()
        return True

    def report_run_failed(self, reason: str) -> bool:
        """시작·재개 실패 시 기존 안전 정지 상태를 유지하고 재시도합니다."""

        event = (
            SystemEvent.START_FAILED
            if self.system_state == SystemState.READY
            else SystemEvent.RESUME_FAILED
        )
        transition = self._apply_system_event(event, reason)
        if transition is not None:
            self._pending_run_confirmation = False
        return transition is not None

    def request_pause(self, reason: str = "operator pause request") -> bool:
        """작업자 일시정지 요청으로 RUN_SYS에서 PAUSING에 진입합니다."""

        transition = self._apply_system_event(SystemEvent.PAUSE_REQUEST, reason)
        if transition is None:
            return False
        self.pause_reason = PauseReason.OPERATOR
        self._pending_run_confirmation = False
        self._request_all_conveyors_stop(reason)
        return True

    def request_recoverable_device_pause(
        self,
        reason: str,
        *,
        pause_reason: PauseReason = PauseReason.DEVICE_RECOVERY_AUTO,
    ) -> bool:
        """추적 정합성이 유지되는 장치 문제를 PAUSING으로 전환합니다."""

        if pause_reason not in {
            PauseReason.DEVICE_RECOVERY_AUTO,
            PauseReason.DEVICE_RECOVERY_MANUAL,
            PauseReason.STORAGE_RECOVERY,
        }:
            raise ValueError("invalid pause reason for recoverable device fault")
        transition = self._apply_system_event(
            SystemEvent.RECOVERABLE_DEVICE_FAULT, reason
        )
        if transition is None:
            return False
        self.pause_reason = pause_reason
        self._pending_run_confirmation = False
        self.set_health_state(NodeHealthState.DEGRADED)
        self._request_all_conveyors_stop(reason)
        return True

    def report_pause_failed(
        self, reason: str, *, timed_out: bool = False
    ) -> bool:
        """실제 정지를 확인할 수 없을 때만 SYS-07 FAULT_STOP으로 전환합니다."""

        event = SystemEvent.PAUSE_TIMEOUT if timed_out else SystemEvent.PAUSE_FAILED
        self.recovery_policy = RecoveryPolicy.EQUIPMENT_CHECK_REQUIRED
        transition = self._apply_system_event(event, reason)
        if transition is None:
            return False
        self._request_all_conveyors_stop(reason)
        return True

    def request_resume(
        self,
        reason: str = "operator resume request",
        *,
        automatic: bool = False,
    ) -> bool:
        """PAUSED guard를 확인하고 재가동을 요청합니다.

        실제 RUN_SYS 전이는 Control의 양쪽 RUN 확인 뒤 수행합니다.
        """

        if automatic and self.pause_reason != PauseReason.DEVICE_RECOVERY_AUTO:
            self.get_logger().warning(
                "automatic resume rejected: pause reason requires operator"
            )
            return False
        if not self._verify_resume_conditions():
            self.get_logger().warning("resume rejected: consistency guard failed")
            return False
        event = (
            SystemEvent.DEVICE_RECOVERED
            if automatic
            else SystemEvent.RESUME_REQUEST
        )
        transition = self._apply_system_event(event, reason)
        if transition is None:
            return False
        self._pending_run_confirmation = True
        self._request_conveyor_run(reason)
        return True

    def request_reset(self, reason: str = "operator reset request") -> bool:
        """FAULT_STOP 또는 복구 불가 PAUSED 상태의 Reset 진입점입니다."""

        if not self._verify_reset_entry_conditions():
            self.get_logger().warning("RESET_REQUEST rejected: reset guard failed")
            return False
        transition = self._apply_system_event(SystemEvent.RESET_REQUEST, reason)
        if transition is None:
            return False
        self._start_reset_sequence(reason)
        return True

    def report_reset_succeeded(
        self, *, line_cleared: bool, reason: str = "reset guard completed"
    ) -> bool:
        """빈 라인 복구는 READY, 제품 보존 복구는 PAUSED로 완료합니다."""

        event = (
            SystemEvent.RESET_SUCCEEDED_EMPTY_LINE
            if line_cleared
            else SystemEvent.RESET_SUCCEEDED_IN_PLACE
        )
        transition = self._apply_system_event(event, reason)
        if transition is None:
            return False
        self.recovery_policy = RecoveryPolicy.NONE
        if line_cleared:
            self.pause_reason = None
        else:
            self.pause_reason = PauseReason.FAULT_RECOVERY
            self.set_health_state(NodeHealthState.READY)
        return True

    def report_reset_failed(
        self, reason: str, *, timed_out: bool = False
    ) -> bool:
        """SAFE_STOP을 유지하며 RESETTING에서 재시도합니다."""

        event = SystemEvent.RESET_TIMEOUT if timed_out else SystemEvent.RESET_FAILED
        transition = self._apply_system_event(event, reason)
        return transition is not None

    def report_critical_fault(
        self,
        reason: str,
        *,
        recovery_policy: RecoveryPolicy,
    ) -> bool:
        """추적 붕괴·정지 불명·중대 장비 오류를 FAULT_STOP으로 래치합니다."""

        if recovery_policy == RecoveryPolicy.NONE:
            raise ValueError("critical fault requires a recovery policy")
        self.recovery_policy = recovery_policy
        self._pending_run_confirmation = False
        transition = self._apply_system_event(SystemEvent.CRITICAL_FAULT, reason)
        if transition is None:
            return False
        self._request_all_conveyors_stop(reason)
        if recovery_policy == RecoveryPolicy.LINE_CLEAR_REQUIRED:
            self._request_line_clear(reason)
        return True

    def report_estop_asserted(self, reason: str = "physical E-stop asserted") -> bool:
        """물리 E-stop을 모든 운전 상태에서 FAULT_STOP으로 래치합니다."""

        self.recovery_policy = RecoveryPolicy.EQUIPMENT_CHECK_REQUIRED
        self._pending_run_confirmation = False
        transition = self._apply_system_event(SystemEvent.ESTOP_ASSERTED, reason)
        if transition is None:
            return False
        self._request_all_conveyors_stop(reason)
        return True

    def request_shutdown(self, reason: str = "operator exit request") -> bool:
        """운전 FSM과 별도인 안전 종료 절차를 모든 상태에서 시작합니다."""

        if self._shutdown_requested:
            return False
        self._shutdown_requested = True
        self.get_logger().info(f"shutdown requested: {reason}")
        self._begin_shutdown_sequence()
        return True

    def _verify_start_conditions(self) -> bool:
        """READY 신규 운전의 빈 라인·노드·장비 guard를 확인합니다."""

        # Block 2와 3이 구현되기 전에는 실제 START를 허용하지 않습니다.
        return (
            self.system_state == SystemState.READY
            and self._all_workers_ready_for_session()
            and self._verify_empty_line_for_new_run()
        )

    def _apply_system_event(
        self, event: SystemEvent, reason: str
    ) -> SystemTransition | None:
        """전이표에 등록된 이벤트만 적용하고 나머지는 거부합니다."""

        try:
            transition = decide_system_transition(self.system_state, event, reason)
        except InvalidSystemTransition as exc:
            self.get_logger().warning(f"system event rejected: {exc}")
            return None
        self.system_state = transition.current
        self._on_system_state_entered(transition)
        return transition

    def _on_system_state_entered(
        self, transition: SystemTransition
    ) -> None:
        """상태 전이 확정 후 공통 플래그·Health·로그를 갱신합니다."""

        if transition.current in {SystemState.READY, SystemState.RUN_SYS}:
            self.set_health_state(NodeHealthState.READY)
        elif transition.current == SystemState.RESETTING:
            self.set_health_state(NodeHealthState.RECOVERING)
        elif transition.current == SystemState.FAULT_STOP:
            self.set_health_state(NodeHealthState.FAULT)
        self.get_logger().info(
            f"[{transition.rule_id}] {transition.previous.name} -> "
            f"{transition.current.name}; event={transition.event.value}; "
            f"reason={transition.reason}"
        )
        self._emit_log_event(
            severity=(
                LogEvent.CRITICAL
                if transition.current == SystemState.FAULT_STOP
                else LogEvent.INFO
            ),
            event_type="SYSTEM_STATE_CHANGED",
            payload={
                "rule_id": transition.rule_id,
                "previous": transition.previous.name,
                "event": transition.event.value,
                "current": transition.current.name,
                "reason": transition.reason,
                "recovery_policy": self.recovery_policy.value,
            },
        )

    # endregion

    # region BLOCK 2 - 작업 노드 초기화와 생존 감시

    def _start_worker_initialization(self) -> None:
        """Control/Vision/Log 초기화 Action 묶음을 시작합니다."""

        if self.system_state != SystemState.INITIALIZING:
            self.get_logger().warning(
                "worker initialization rejected outside INITIALIZING"
            )
            return
        self._initialization_completion_reported = False
        self.worker_heartbeats.clear()
        self.worker_received_ns.clear()
        self._reported_worker_outages.clear()
        self.worker_states = {
            worker_id: WorkerRuntimeState(worker_id)
            for worker_id in self.worker_states
        }

        # 세 노드는 서로 다른 자원을 초기화하므로 병렬로 요청합니다. 각 노드
        # 내부의 중복 초기화는 InspectionNodeBase Action Server가 직렬화합니다.
        for worker_id in self.worker_states:
            self._send_initialize_goal(worker_id, attempt=1)

    def _send_initialize_goal(self, worker_id: NodeId, attempt: int) -> None:
        """한 작업 노드에 InitializeNode Goal을 전송합니다."""

        if not self._worker_initialization_allowed(worker_id):
            return
        now_ns = time.monotonic_ns()
        state = self.worker_states[worker_id]
        state.begin_attempt(
            request_id=new_uuid(),
            attempt=attempt,
            now_ns=now_ns,
            timeout_ns=self.init_timeout_ms * 1_000_000,
        )
        client = self.initialize_clients[worker_id]
        if not client.wait_for_server(timeout_sec=0.0):
            self._schedule_initialize_retry(
                worker_id,
                attempt,
                "InitializeNode Action server is unavailable",
            )
            return

        goal = InitializeNode.Goal()
        goal.request_id = state.request_id
        goal.retry_of_request_id = state.retry_of_request_id
        goal.session_id = self.session_id
        goal.config_version = self.config_version
        goal.config_digest = self.config_digest
        goal.expected_interface_version = self.expected_interface_version
        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=partial(
                    self._handle_initialize_feedback,
                    worker_id,
                    state.request_id,
                ),
            )
        except Exception as exc:
            self._schedule_initialize_retry(
                worker_id,
                attempt,
                f"failed to send InitializeNode Goal: {type(exc).__name__}",
            )
            return
        future.add_done_callback(
            partial(
                self._handle_initialize_goal_response,
                worker_id,
                state.request_id,
            )
        )

    def _handle_initialize_feedback(
        self,
        worker_id: NodeId,
        request_id: str,
        feedback_message,
    ) -> None:
        """현재 request의 Action feedback만 진단 로그에 반영합니다."""

        state = self.worker_states[worker_id]
        if state.request_id != request_id:
            return
        feedback = feedback_message.feedback
        self.get_logger().debug(
            f"{worker_id.value} init feedback: stage={feedback.stage}, "
            f"attempt={feedback.attempt}"
        )

    def _handle_initialize_goal_response(
        self,
        worker_id: NodeId,
        request_id: str,
        future,
    ) -> None:
        """Goal 수락 여부를 확인하고 최종 Result callback을 연결합니다."""

        state = self.worker_states[worker_id]
        if (
            state.request_id != request_id
            or state.phase != WorkerInitPhase.WAITING_GOAL_RESPONSE
        ):
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                f"InitializeNode Goal response failed: {type(exc).__name__}",
            )
            return
        if not goal_handle.accepted:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "InitializeNode Goal was rejected",
            )
            return

        state.mark_goal_accepted()
        self._initialize_goal_handles[worker_id] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._handle_initialize_result, worker_id, request_id)
        )

    def _handle_initialize_result(
        self,
        worker_id: NodeId,
        request_id: str,
        future,
    ) -> None:
        """초기화 성공·재시도 가능 실패·수동 복구 필요 실패를 분기합니다."""

        state = self.worker_states[worker_id]
        if (
            state.request_id != request_id
            or state.phase != WorkerInitPhase.WAITING_RESULT
        ):
            return
        self._initialize_goal_handles.pop(worker_id, None)
        try:
            result = future.result().result
        except Exception as exc:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                f"InitializeNode Result failed: {type(exc).__name__}",
            )
            return

        if result.request_id != request_id or result.node_id != worker_id.value:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "InitializeNode Result identity mismatch",
                error_code=int(ErrorCode.COMMAND_CONFLICT),
                manual_intervention_required=True,
            )
            return
        if not result.success:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                result.reason or "worker initialization failed",
                error_code=int(result.error_code),
                manual_intervention_required=not result.retryable,
            )
            return
        if result.interface_version != self.expected_interface_version:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "interface version mismatch: "
                f"expected={self.expected_interface_version}, "
                f"actual={result.interface_version}",
                error_code=int(ErrorCode.INTERFACE_VERSION_MISMATCH),
                manual_intervention_required=True,
            )
            return
        if result.active_session_id != self.session_id:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "InitializeNode Result session mismatch",
                error_code=int(ErrorCode.NODE_INIT_FAILED),
            )
            return

        now_ns = time.monotonic_ns()
        state.mark_action_ready(
            interface_version=result.interface_version,
            software_version=result.software_version,
            active_session_id=result.active_session_id,
            reason=result.reason,
            now_ns=now_ns,
            timeout_ns=self.init_timeout_ms * 1_000_000,
        )
        self._request_worker_status(worker_id)

    def _schedule_initialize_retry(
        self,
        worker_id: NodeId,
        attempt: int,
        reason: str,
        *,
        error_code: int = int(ErrorCode.NODE_INIT_FAILED),
        timed_out: bool = False,
        manual_intervention_required: bool = False,
    ) -> None:
        """초기화 실패를 기록하고 다음 재시도를 예약합니다."""

        state = self.worker_states[worker_id]
        goal_handle = self._initialize_goal_handles.pop(worker_id, None)
        if timed_out and goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                self.get_logger().warning(
                    f"{worker_id.value} init timeout cancel request failed"
                )
        state.schedule_retry(
            now_ns=time.monotonic_ns(),
            interval_ns=self.init_retry_interval_ms * 1_000_000,
            error_code=error_code,
            reason=reason,
            manual_intervention_required=manual_intervention_required,
        )
        if self.system_state == SystemState.INITIALIZING:
            self.report_initialization_failed(reason, timed_out=timed_out)

        message = (
            f"{worker_id.value} initialization attempt {attempt} failed; "
            f"retry in {self.init_retry_interval_ms} ms: {reason}"
        )
        warning_due = (
            attempt >= self.init_warning_after_attempts
            and attempt % self.init_warning_after_attempts == 0
        )
        if warning_due or manual_intervention_required:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _request_worker_status(self, worker_id: NodeId) -> None:
        """Action 성공 뒤 GetNodeStatus로 READY/session/epoch를 교차 검증합니다."""

        state = self.worker_states[worker_id]
        if not state.action_ready or state.phase != WorkerInitPhase.WAITING_STATUS:
            return
        client = self.status_clients[worker_id]
        if not client.wait_for_service(timeout_sec=0.0):
            state.schedule_status_poll(
                now_ns=time.monotonic_ns(),
                interval_ns=self.init_retry_interval_ms * 1_000_000,
            )
            return

        request = GetNodeStatus.Request()
        request.request_id = new_uuid()
        request.session_id = self.session_id
        state.begin_status_check(request_id=request.request_id)
        try:
            future = client.call_async(request)
        except Exception as exc:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                f"GetNodeStatus request failed: {type(exc).__name__}",
            )
            return
        future.add_done_callback(
            partial(
                self._handle_worker_status,
                worker_id,
                request.request_id,
            )
        )

    def _handle_worker_status(
        self,
        worker_id: NodeId,
        request_id: str,
        future,
    ) -> None:
        """GetNodeStatus 응답을 현재 초기화 시도와 대조합니다."""

        state = self.worker_states[worker_id]
        if (
            state.phase != WorkerInitPhase.WAITING_STATUS
            or state.status_request_id != request_id
        ):
            return
        state.status_request_id = ""
        try:
            response = future.result()
        except Exception as exc:
            state.schedule_status_poll(
                now_ns=time.monotonic_ns(),
                interval_ns=self.init_retry_interval_ms * 1_000_000,
            )
            self.get_logger().warning(
                f"{worker_id.value} GetNodeStatus failed: {type(exc).__name__}"
            )
            return

        if response.node_id != worker_id.value:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "GetNodeStatus node_id mismatch",
                error_code=int(ErrorCode.COMMAND_CONFLICT),
                manual_intervention_required=True,
            )
            return
        if response.interface_version != self.expected_interface_version:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "GetNodeStatus interface version mismatch",
                error_code=int(ErrorCode.INTERFACE_VERSION_MISMATCH),
                manual_intervention_required=True,
            )
            return
        if response.active_session_id != self.session_id:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "GetNodeStatus session mismatch",
            )
            return
        try:
            health_state = NodeHealthState(response.health_state)
        except ValueError:
            self._schedule_initialize_retry(
                worker_id,
                state.attempt,
                "GetNodeStatus contains invalid health_state",
                manual_intervention_required=True,
            )
            return

        state.record_status(
            node_instance_id=response.node_instance_id,
            interface_version=response.interface_version,
            software_version=response.software_version,
            active_session_id=response.active_session_id,
            command_epoch=response.command_epoch,
            heartbeat_sequence=response.heartbeat_sequence,
            health_state=health_state,
            ready=response.ready,
            master_heartbeat_alive=response.master_heartbeat_alive,
        )
        if not response.ready or not response.master_heartbeat_alive:
            state.schedule_status_poll(
                now_ns=time.monotonic_ns(),
                interval_ns=self.init_retry_interval_ms * 1_000_000,
            )
            return
        if not self._try_mark_worker_ready(worker_id):
            state.schedule_status_poll(
                now_ns=time.monotonic_ns(),
                interval_ns=self.init_retry_interval_ms * 1_000_000,
            )

    def _all_workers_ready_for_session(self) -> bool:
        """세 작업 노드가 현재 session/epoch에서 READY인지 검사합니다."""

        now_ns = time.monotonic_ns()
        timeout_ns = self.node_heartbeat_timeout_ms * 1_000_000
        return all(
            state.ready
            and state.can_mark_ready(
                session_id=self.session_id,
                expected_interface_version=self.expected_interface_version,
                command_epoch=self.command_epoch,
                now_ns=now_ns,
                heartbeat_timeout_ns=timeout_ns,
            )
            for state in self.worker_states.values()
        )

    def _advance_command_epoch(
        self,
        reason: str,
        *,
        restarting_worker: NodeId | None = None,
        reinitialize_all_workers: bool = False,
    ) -> None:
        """새 명령 epoch를 발급하고 worker 증거를 다시 생성하게 합니다.

        재시작한 worker는 InitializeNode부터 다시 수행합니다. 나머지 worker는
        일반적으로 GetNodeStatus만 다시 조회합니다. RESET처럼 worker health가
        RECOVERING으로 래치되는 경로에서는 모든 worker의 InitializeNode Action을
        다시 실행하여 READY 복귀 근거를 새로 만듭니다.
        """

        self.command_epoch += 1
        now_ns = time.monotonic_ns()
        interval_ns = self.init_retry_interval_ms * 1_000_000
        for worker_id, state in self.worker_states.items():
            if worker_id == restarting_worker:
                continue
            if reinitialize_all_workers:
                state.schedule_retry(
                    now_ns=now_ns,
                    interval_ns=interval_ns,
                    error_code=int(ErrorCode.NODE_INIT_FAILED),
                    reason=f"reinitialize after command epoch change: {reason}",
                    manual_intervention_required=False,
                )
            else:
                state.invalidate_epoch_evidence(
                    now_ns=now_ns,
                    interval_ns=interval_ns,
                )
        self.get_logger().info(
            f"command_epoch advanced to {self.command_epoch}: {reason}"
        )

    def _try_mark_worker_ready(self, worker_id: NodeId) -> bool:
        state = self.worker_states[worker_id]
        if not state.can_mark_ready(
            session_id=self.session_id,
            expected_interface_version=self.expected_interface_version,
            command_epoch=self.command_epoch,
            now_ns=time.monotonic_ns(),
            heartbeat_timeout_ns=self.node_heartbeat_timeout_ms * 1_000_000,
        ):
            return False
        if not state.ready:
            state.mark_ready()
            self.get_logger().info(
                f"{worker_id.value} READY confirmed by Action, status and heartbeat"
            )
        self._maybe_complete_initialization()
        if (
            self.system_state
            in {SystemState.READY, SystemState.RUN_SYS, SystemState.PAUSED}
            and self._all_workers_ready_for_session()
            and self._log_spool is not None
        ):
            self.set_health_state(NodeHealthState.READY)
        return True

    def _maybe_complete_initialization(self) -> None:
        if (
            self.system_state != SystemState.INITIALIZING
            or self._initialization_completion_reported
            or not self._all_workers_ready_for_session()
        ):
            return
        self._initialization_completion_reported = True
        if not self.report_initialization_done():
            self._initialization_completion_reported = False

    def _handle_worker_heartbeat(
        self, worker_id: NodeId, message: NodeHeartbeat
    ) -> None:
        if message.node_id != worker_id.value:
            self.get_logger().error(f"heartbeat node_id mismatch for {worker_id.value}")
            return
        state = self.worker_states[worker_id]
        if message.interface_version != self.expected_interface_version:
            state.interface_version = message.interface_version
            self._handle_worker_unavailable(
                worker_id,
                f"{worker_id.value} interface version mismatch",
                restarted=True,
            )
            return
        try:
            health_state = NodeHealthState(message.health_state)
        except ValueError:
            self.get_logger().error(
                f"heartbeat health_state is invalid for {worker_id.value}"
            )
            return
        now_ns = time.monotonic_ns()
        update = state.accept_heartbeat(
            node_instance_id=message.node_instance_id,
            sequence=message.sequence,
            health_state=health_state,
            interface_version=message.interface_version,
            session_id=message.header.session_id,
            received_ns=now_ns,
        )
        if not update.accepted:
            return
        self.worker_heartbeats[worker_id] = message
        self.worker_received_ns[worker_id] = now_ns

        recovered = worker_id in self._reported_worker_outages
        if recovered:
            self._reported_worker_outages.discard(worker_id)
            self.get_logger().info(f"{worker_id.value} heartbeat recovered")
        if update.restarted:
            self._advance_command_epoch(
                f"{worker_id.value} process restart",
                restarting_worker=worker_id,
            )
            self._handle_worker_unavailable(
                worker_id,
                f"{worker_id.value} process restarted",
                restarted=True,
            )
            return
        if health_state != NodeHealthState.READY:
            if state.ready:
                self._handle_worker_unavailable(
                    worker_id,
                    f"{worker_id.value} health changed to {health_state.name}",
                    restarted=False,
                )
            return
        if recovered and state.action_ready:
            state.phase = WorkerInitPhase.WAITING_STATUS
            self._request_worker_status(worker_id)
            return
        if state.status_verified:
            self._try_mark_worker_ready(worker_id)

    def _check_worker_heartbeats(self) -> None:
        now = time.monotonic_ns()
        self._advance_worker_initialization(now)
        if self.system_state in {
            SystemState.BOOT,
            SystemState.INITIALIZING,
            SystemState.FAULT_STOP,
        }:
            return

        timeout_ns = self.node_heartbeat_timeout_ms * 1_000_000
        for worker_id, state in self.worker_states.items():
            if state.heartbeat_is_fresh(now_ns=now, timeout_ns=timeout_ns):
                continue
            if worker_id in self._reported_worker_outages:
                continue
            self._handle_worker_unavailable(
                worker_id,
                f"{worker_id.value} heartbeat timeout",
                restarted=False,
            )

    def _advance_worker_initialization(self, now_ns: int) -> None:
        """watchdog tick에서 Action timeout·status poll·재시도를 진행합니다."""

        for worker_id, state in self.worker_states.items():
            waiting = state.phase in {
                WorkerInitPhase.WAITING_GOAL_RESPONSE,
                WorkerInitPhase.WAITING_RESULT,
                WorkerInitPhase.WAITING_STATUS,
            }
            if waiting and state.deadline_ns and now_ns > state.deadline_ns:
                self._schedule_initialize_retry(
                    worker_id,
                    state.attempt,
                    "worker initialization timed out",
                    error_code=int(ErrorCode.NODE_INIT_FAILED),
                    timed_out=True,
                )
                continue
            if (
                state.phase == WorkerInitPhase.WAITING_STATUS
                and not state.status_request_id
                and state.status_poll_due_ns
                and now_ns >= state.status_poll_due_ns
            ):
                self._request_worker_status(worker_id)
                continue
            if (
                state.phase == WorkerInitPhase.RETRY_WAIT
                and now_ns >= state.retry_due_ns
                and self._worker_initialization_allowed(worker_id)
            ):
                self._send_initialize_goal(worker_id, state.attempt + 1)
                continue
            if (
                self.system_state == SystemState.INITIALIZING
                and state.phase == WorkerInitPhase.READY
                and not state.heartbeat_is_fresh(
                    now_ns=now_ns,
                    timeout_ns=self.node_heartbeat_timeout_ms * 1_000_000,
                )
            ):
                self._schedule_initialize_retry(
                    worker_id,
                    state.attempt,
                    "worker heartbeat expired during initialization",
                    timed_out=True,
                )

    def _worker_initialization_allowed(self, worker_id: NodeId) -> bool:
        if self.system_state in {
            SystemState.INITIALIZING,
            SystemState.READY,
            SystemState.PAUSED,
            SystemState.RESETTING,
        }:
            return True
        return self.system_state == SystemState.RUN_SYS and worker_id == NodeId.LOG

    def _handle_worker_unavailable(
        self,
        worker_id: NodeId,
        reason: str,
        *,
        restarted: bool,
    ) -> None:
        """노드 영향도와 현재 동작 여부에 따라 정지 수준을 선택합니다."""

        state = self.worker_states[worker_id]
        state.ready = False
        state.status_verified = False
        self._reported_worker_outages.add(worker_id)

        if self.system_state == SystemState.INITIALIZING:
            if state.phase != WorkerInitPhase.RETRY_WAIT:
                self._schedule_initialize_retry(
                    worker_id,
                    max(state.attempt, 1),
                    reason,
                    error_code=(
                        int(ErrorCode.INTERFACE_VERSION_MISMATCH)
                        if "version mismatch" in reason
                        else int(ErrorCode.NODE_INIT_FAILED)
                    ),
                    manual_intervention_required="version mismatch" in reason,
                )
            return

        # node_base의 READY 복귀 경로는 InitializeNode 성공뿐입니다. 프로세스
        # 재시작뿐 아니라 heartbeat 단절로 DEGRADED/RECOVERING이 된 경우에도
        # status poll만 반복하지 않고 초기화 Action을 다시 예약합니다.
        state.schedule_retry(
            now_ns=time.monotonic_ns(),
            interval_ns=self.init_retry_interval_ms * 1_000_000,
            error_code=int(ErrorCode.NODE_INIT_FAILED),
            reason=reason,
            manual_intervention_required=False,
        )

        control_faulted_while_moving = (
            worker_id == NodeId.CONTROL
            and self.system_state
            in {
                SystemState.RUN_SYS,
                SystemState.PAUSING,
            }
        )
        if control_faulted_while_moving:
            self.report_critical_fault(
                reason,
                recovery_policy=RecoveryPolicy.LINE_CLEAR_REQUIRED,
            )
        elif (
            worker_id == NodeId.VISION
            and self.system_state == SystemState.RUN_SYS
        ):
            self.request_recoverable_device_pause(
                reason,
                pause_reason=(
                    PauseReason.DEVICE_RECOVERY_MANUAL
                    if restarted
                    else PauseReason.DEVICE_RECOVERY_AUTO
                ),
            )
        else:
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().warning(reason)

    # endregion

    # region BLOCK 3 - 단일 물리 FIFO와 Sensor1/2/3 매핑

    def register_product(self, product_id: str, fifo_sequence: int) -> None:
        """제품 원장과 활성 FIFO에 같은 identity를 원자적으로 등록합니다."""

        with self._flow_lock:
            context = self.ledger.register(product_id, fifo_sequence)
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="PRODUCT_REGISTERED",
            product_id=product_id,
            payload=context.snapshot(),
        )

    def _handle_sensor_event(self, message: SensorEvent) -> None:
        """Control이 정제한 센서 이벤트를 Sensor1/2/3 블록으로 전달합니다."""

        if message.header.session_id != self.session_id:
            return
        sensor_index = self._resolve_sensor_index(message.sensor_id)
        if sensor_index is None:
            self.get_logger().warning(
                f"unmapped sensor event ignored: {message.sensor_id}"
            )
            return
        digest = payload_digest(
            {
                "sensor_id": message.sensor_id,
                "event_id": message.event_id,
                "edge": int(message.edge),
                "sensor_sequence": int(message.sensor_sequence),
                "estimated_step": int(message.estimated_step),
                "observed_at": [
                    int(message.observed_at.sec),
                    int(message.observed_at.nanosec),
                ],
            }
        )
        outcome = self.sensor_events.accept(
            message.sensor_id,
            message.event_id,
            int(message.sensor_sequence),
            digest,
        )
        if outcome == SensorEventOutcome.DUPLICATE:
            return
        if outcome != SensorEventOutcome.ACCEPTED:
            self._fault_stop(
                f"sensor event integrity failure: sensor={message.sensor_id}, "
                f"event={message.event_id}, outcome={outcome.value}"
            )
            return
        if int(message.edge) != self.accepted_sensor_edge:
            return
        if sensor_index == 1:
            self._handle_sensor1_entry(message)
        elif sensor_index == 2:
            self._handle_sensor2_entry(message)
        elif sensor_index == 3:
            self._handle_sensor3_entry(message)

    def _resolve_sensor_index(self, sensor_id: str) -> int | None:
        """Control의 물리 sensor_id를 공정 Sensor1/2/3 역할로 변환합니다."""

        # TODO(HARDWARE): hardware.yaml의 논리 ID를 실제 Mega 배선·polarity와
        # 대조한 뒤 master.hardware_mapping_confirmed=true로 승인해야 합니다.
        return self.sensor_id_to_index.get(sensor_id)

    def _handle_sensor1_entry(self, message: SensorEvent) -> None:
        """새 제품 identity를 발급하고 단일 FIFO 맨 뒤에 등록합니다."""

        if self.system_state not in {SystemState.RUN_SYS, SystemState.PAUSING}:
            self.get_logger().warning(
                f"Sensor1 event ignored in {self.system_state.name}"
            )
            return
        with self._flow_lock:
            if self.ledger.active_size >= self.fifo_hard_capacity:
                self._fault_stop(
                    "FIFO hard capacity exceeded while accepting Sensor1 event"
                )
                return
            fifo_sequence = self.ledger.next_sequence
            product_id = f"P-{self.session_id[:8]}-{fifo_sequence:06d}"
            context = self.ledger.register(product_id, fifo_sequence)
            context.record_sensor(1, message.event_id, int(message.estimated_step))
            fifo_size = self.ledger.active_size

        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="SENSOR1_PRODUCT_ACCEPTED",
            product_id=product_id,
            payload=context.snapshot(),
        )
        if fifo_size >= self.fifo_soft_limit:
            self._deferred_station_starts.add((product_id, StationId.A))
            if self.system_state == SystemState.RUN_SYS:
                self.request_recoverable_device_pause(
                    f"FIFO soft limit reached ({fifo_size}/{self.fifo_hard_capacity})",
                    pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
                )
            return
        if self.system_state == SystemState.RUN_SYS:
            self._start_station_cycle(product_id, fifo_sequence, StationId.A)
        else:
            self._deferred_station_starts.add((product_id, StationId.A))

    def _handle_sensor2_entry(self, message: SensorEvent) -> None:
        """뒤집기 완료 제품을 Station B 대기 제품과 매핑합니다."""

        if self.system_state not in {SystemState.RUN_SYS, SystemState.PAUSING}:
            return
        try:
            with self._flow_lock:
                context = self.ledger.oldest_in_state(ProductPhysicalState.FLIPPING)
                context.accept_sensor2(message.event_id, int(message.estimated_step))
        except ProductFlowError as exc:
            self._fault_stop(f"Sensor2/FIFO mismatch: {exc}")
            return
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="SENSOR2_PRODUCT_ACCEPTED",
            product_id=context.product_id,
            payload=context.snapshot(),
        )
        if self.system_state == SystemState.RUN_SYS:
            self._start_station_cycle(
                context.product_id, context.fifo_sequence, StationId.B
            )
        else:
            self._deferred_station_starts.add((context.product_id, StationId.B))

    def _handle_sensor3_entry(self, message: SensorEvent) -> None:
        """분류 지점 제품을 매핑하고 판정 잠금·액추에이터 블록으로 넘깁니다."""

        if self.system_state not in {SystemState.RUN_SYS, SystemState.PAUSING}:
            return
        try:
            with self._flow_lock:
                context = self.ledger.oldest_in_state(
                    ProductPhysicalState.SENSOR3_WAIT
                )
                locked = context.lock_at_sensor3(
                    message.event_id, int(message.estimated_step)
                )
        except ProductFlowError as exc:
            self._fault_stop(f"Sensor3/FIFO mismatch: {exc}")
            return
        self._accept_locked(locked)
        self._schedule_actuation(locked)

    def _validate_fifo_alignment(self) -> bool:
        """FIFO 순서, 제품 물리 상태와 최신 센서 관측의 정합성을 확인합니다."""

        with self._flow_lock:
            return self.ledger.validate_alignment()

    def _verify_empty_line_for_new_run(self) -> bool:
        """READY 신규 시작에 필요한 빈 FIFO·빈 라인 조건을 확인합니다."""

        mapping_ready = self.profile == "sim" or self.hardware_mapping_confirmed
        camera_mapping_ready = (
            len(self.station_camera_ids[StationId.A]) == 3
            and len(self.station_camera_ids[StationId.B]) == 1
        )
        return (
            mapping_ready
            and camera_mapping_ready
            and self._log_spool is not None
            and self.ledger.active_size == 0
            and self.equipment.line_clear_guards_satisfied()
        )

    # endregion

    # region BLOCK 4 - Station A/B 위치 이동과 촬영

    def _start_station_cycle(
        self, product_id: str, fifo_sequence: int, station_id: StationId
    ) -> None:
        """제품의 위치 이동→정지 확인→촬영 흐름을 시작합니다."""

        if self.system_state != SystemState.RUN_SYS:
            self._deferred_station_starts.add((product_id, station_id))
            return
        existing = self._station_cycles.get(station_id)
        if existing is not None:
            if existing.product_id == product_id:
                return
            self._fault_stop(
                f"station {station_id.name} already owns product {existing.product_id}"
            )
            return
        context = self.ledger.get(product_id, fifo_sequence)
        if context is None:
            self._fault_stop("station cycle references unknown product identity")
            return
        sensor_index = 1 if station_id == StationId.A else 2
        if sensor_index not in context.sensor_steps:
            self._fault_stop(
                f"station {station_id.name} has no source sensor step"
            )
            return
        conveyor_id = (
            ConveyorId.UPPER if station_id == StationId.A else ConveyorId.LOWER
        )
        target_step = (
            context.sensor_steps[sensor_index]
            + self.station_position_offsets[station_id]
        )
        position_command_id = new_uuid()
        capture_id = new_uuid()
        try:
            with self._flow_lock:
                context.begin_station_cycle(
                    station_id,
                    position_command_id=position_command_id,
                    target_step=target_step,
                    capture_id=capture_id,
                )
                self.ledger.register_capture(capture_id, product_id, station_id)
        except ProductFlowError as exc:
            self._fault_stop(f"station cycle start failed: {exc}")
            return
        self._station_cycles[station_id] = StationCycle(
            product_id=product_id,
            fifo_sequence=fifo_sequence,
            station_id=station_id,
            conveyor_id=conveyor_id,
            position_command_id=position_command_id,
            capture_id=capture_id,
            target_step=target_step,
        )
        self._deferred_station_starts.discard((product_id, station_id))
        self._send_position_goal(product_id, fifo_sequence, station_id)

    def _send_position_goal(
        self, product_id: str, fifo_sequence: int, station_id: StationId
    ) -> None:
        """ControlNode에 PositionProduct Goal을 전송합니다."""

        cycle = self._station_cycles.get(station_id)
        if (
            cycle is None
            or cycle.product_id != product_id
            or cycle.fifo_sequence != fifo_sequence
        ):
            self._fault_stop("position cycle identity mismatch")
            return
        # TODO(HARDWARE): position offset과 허용오차는 제품 크기·TB6600 보정 후
        # hardware.yaml의 승인값으로 교체합니다.
        if not self.position_client.wait_for_server(timeout_sec=0.0):
            self._pause_station_for_recovery(
                station_id, "PositionProduct Action server is unavailable"
            )
            return
        goal = PositionProduct.Goal()
        payload = {
            "product_id": product_id,
            "fifo_sequence": fifo_sequence,
            "station_id": int(station_id),
            "conveyor_id": int(cycle.conveyor_id),
            "target_step": cycle.target_step,
        }
        self._fill_command_header(
            goal.command,
            command_id=cycle.position_command_id,
            correlation_id=product_id,
            payload=payload,
        )
        goal.product_id = product_id
        goal.fifo_sequence = fifo_sequence
        goal.station_id = int(station_id)
        goal.conveyor_id = int(cycle.conveyor_id)
        goal.target_step = cycle.target_step
        cycle.phase = StationCyclePhase.POSITION_GOAL
        cycle.deadline_ns = (
            time.monotonic_ns() + self.position_timeout_ms * 1_000_000
        )
        try:
            future = self.position_client.send_goal_async(goal)
        except Exception as exc:
            self._pause_station_for_recovery(
                station_id,
                f"PositionProduct Goal send failed: {type(exc).__name__}",
            )
            return
        future.add_done_callback(
            partial(
                self._handle_position_goal_response,
                station_id,
                cycle.position_command_id,
            )
        )

    def _handle_position_goal_response(
        self, station_id: StationId, command_id: str, future
    ) -> None:
        cycle = self._station_cycles.get(station_id)
        if cycle is None or cycle.position_command_id != command_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "POSITION_GOAL_RESPONSE",
            product_id=cycle.product_id,
            correlation_id=command_id,
        ):
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._pause_station_for_recovery(
                station_id,
                f"PositionProduct Goal response failed: {type(exc).__name__}",
            )
            return
        if not goal_handle.accepted:
            self._pause_station_for_recovery(
                station_id, "PositionProduct Goal was rejected"
            )
            return
        cycle.position_goal_handle = goal_handle
        cycle.phase = StationCyclePhase.WAITING_POSITION
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._handle_position_goal_result, station_id, command_id)
        )

    def _handle_position_goal_result(
        self, station_id: StationId, command_id: str, future
    ) -> None:
        """위치 이동 Action 결과와 후속 PositionSettled를 연결합니다."""

        cycle = self._station_cycles.get(station_id)
        if cycle is None or cycle.position_command_id != command_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "POSITION_RESULT",
            product_id=cycle.product_id,
            correlation_id=command_id,
        ):
            return
        try:
            result = future.result().result
        except Exception as exc:
            self._fault_stop(
                "PositionProduct Result became unavailable after Goal acceptance; "
                f"physical position is unknown: {type(exc).__name__}"
            )
            return
        if (
            result.product_id != cycle.product_id
            or int(result.station_id) != int(station_id)
            or result.position_command_id != command_id
        ):
            self._fault_stop("PositionProduct Result identity mismatch")
            return
        if not result.success:
            error_code = int(result.error_code)
            if error_code == int(ErrorCode.COMMAND_CONFLICT):
                self._fault_stop(f"PositionProduct command conflict: {result.reason}")
            elif error_code == int(ErrorCode.POSITION_FAILED):
                self._fault_stop(
                    "PositionProduct reported an untrustworthy physical position: "
                    + (result.reason or "POSITION_FAILED")
                )
            else:
                self._pause_station_for_recovery(
                    station_id, result.reason or "PositionProduct failed"
                )
            return
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("PositionProduct Result references unknown product")
            return
        try:
            with self._flow_lock:
                context.mark_position_action_succeeded(station_id, command_id)
        except ProductFlowError as exc:
            self._fault_stop(f"position result application failed: {exc}")
            return
        self._maybe_request_station_capture(station_id)

    def _handle_position_settled(self, message: PositionSettled) -> None:
        """Control이 보고한 정지·안정화 완료 이벤트를 검증합니다."""

        if message.header.session_id != self.session_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "POSITION_SETTLED",
            product_id=message.product_id,
            correlation_id=message.position_command_id,
        ):
            return
        try:
            station_id = StationId(message.station_id)
        except ValueError:
            self._fault_stop("PositionSettled contains invalid station_id")
            return
        cycle = self._station_cycles.get(station_id)
        if cycle is None:
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="SUPERSEDED_POSITION_SETTLED_IGNORED",
                product_id=message.product_id,
                payload={
                    "station_id": station_id.name,
                    "position_command_id": message.position_command_id,
                    "reason": "no active station cycle",
                },
            )
            return
        if (
            message.product_id != cycle.product_id
            or message.position_command_id != cycle.position_command_id
            or int(message.conveyor_id) != int(cycle.conveyor_id)
            or int(message.target_step) != cycle.target_step
        ):
            self._fault_stop("PositionSettled identity or target mismatch")
            return
        if abs(int(message.position_error_steps)) > self.position_tolerance_steps:
            self._pause_station_for_recovery(
                station_id,
                "PositionSettled exceeded configured position tolerance",
            )
            return
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("PositionSettled references unknown product")
            return
        try:
            with self._flow_lock:
                context.mark_position_settled(
                    station_id, cycle.position_command_id
                )
        except ProductFlowError as exc:
            self._fault_stop(f"PositionSettled application failed: {exc}")
            return
        self._maybe_request_station_capture(station_id)

    def _maybe_request_station_capture(self, station_id: StationId) -> None:
        cycle = self._station_cycles.get(station_id)
        if cycle is None:
            return
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None or not context.can_request_capture(station_id):
            return
        self._request_station_capture(
            cycle.product_id, cycle.fifo_sequence, station_id
        )

    def _request_station_capture(
        self, product_id: str, fifo_sequence: int, station_id: StationId
    ) -> None:
        """VisionNode에 CaptureProduct Goal을 전송합니다."""

        cycle = self._station_cycles.get(station_id)
        context = self.ledger.get(product_id, fifo_sequence)
        if cycle is None or context is None or cycle.product_id != product_id:
            self._fault_stop("capture cycle identity mismatch")
            return
        if not self.capture_client.wait_for_server(timeout_sec=0.0):
            self._pause_station_for_recovery(
                station_id, "CaptureProduct Action server is unavailable"
            )
            return
        command_id = new_uuid()
        goal = CaptureProduct.Goal()
        required_camera_ids = self.station_camera_ids[station_id]
        payload = {
            "product_id": product_id,
            "fifo_sequence": fifo_sequence,
            "station_id": int(station_id),
            "capture_id": cycle.capture_id,
            "required_camera_ids": required_camera_ids,
        }
        self._fill_command_header(
            goal.command,
            command_id=command_id,
            correlation_id=cycle.position_command_id,
            payload=payload,
        )
        goal.product_id = product_id
        goal.fifo_sequence = fifo_sequence
        goal.station_id = int(station_id)
        goal.capture_id = cycle.capture_id
        goal.required_camera_ids = list(required_camera_ids)
        goal.requested_at = self.get_clock().now().to_msg()
        try:
            with self._flow_lock:
                context.mark_capture_requested(
                    station_id,
                    capture_id=cycle.capture_id,
                    command_id=command_id,
                )
        except ProductFlowError as exc:
            self._fault_stop(f"capture request application failed: {exc}")
            return
        cycle.phase = StationCyclePhase.CAPTURE_GOAL
        cycle.deadline_ns = (
            time.monotonic_ns() + self.capture_timeout_ms * 1_000_000
        )
        try:
            future = self.capture_client.send_goal_async(
                goal,
                feedback_callback=partial(
                    self._handle_capture_feedback,
                    station_id,
                    cycle.capture_id,
                ),
            )
        except Exception as exc:
            self._pause_station_for_recovery(
                station_id,
                f"CaptureProduct Goal send failed: {type(exc).__name__}",
            )
            return
        future.add_done_callback(
            partial(
                self._handle_capture_goal_response,
                station_id,
                cycle.capture_id,
            )
        )

    def _handle_capture_feedback(
        self, station_id: StationId, capture_id: str, feedback_message
    ) -> None:
        cycle = self._station_cycles.get(station_id)
        if cycle is None or cycle.capture_id != capture_id:
            return
        # Capture feedback은 queue 대기 중 10 Hz로 반복될 수 있는 telemetry입니다.
        # FAULT_STOP의 제품 원장은 보존하되 고빈도 진행 신호는 기록하지 않습니다.
        if self.system_state == SystemState.FAULT_STOP:
            return
        feedback = feedback_message.feedback
        if int(feedback.stage) == int(CaptureProduct.Feedback.ENQUEUE_BLOCKED):
            self._paused_by_queue = True
            self._queue_recovered_pending = False
            self._pause(
                f"Vision queue full; preserving FrameBatch {feedback.frame_batch_id}"
            )

    def _handle_capture_goal_response(
        self, station_id: StationId, capture_id: str, future
    ) -> None:
        cycle = self._station_cycles.get(station_id)
        if cycle is None or cycle.capture_id != capture_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "CAPTURE_GOAL_RESPONSE",
            product_id=cycle.product_id,
            correlation_id=capture_id,
        ):
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._pause_station_for_recovery(
                station_id,
                f"CaptureProduct Goal response failed: {type(exc).__name__}",
            )
            return
        if not goal_handle.accepted:
            self._pause_station_for_recovery(
                station_id, "CaptureProduct Goal was rejected"
            )
            return
        cycle.capture_goal_handle = goal_handle
        cycle.phase = StationCyclePhase.WAITING_CAPTURE_RESULT
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._handle_capture_result, station_id, capture_id)
        )

    def _handle_capture_result(
        self, station_id: StationId, capture_id: str, future
    ) -> None:
        """촬영·저장·Queue 등록 결과에 따라 재가동 또는 FORCED_NG를 결정합니다."""

        cycle = self._station_cycles.get(station_id)
        if cycle is None or cycle.capture_id != capture_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "CAPTURE_RESULT",
            product_id=cycle.product_id,
            correlation_id=capture_id,
        ):
            return
        try:
            result = future.result().result
        except Exception as exc:
            self._pause_station_for_recovery(
                station_id,
                f"CaptureProduct Result failed: {type(exc).__name__}",
            )
            return
        if (
            result.product_id != cycle.product_id
            or int(result.station_id) != int(station_id)
            or result.capture_id != capture_id
        ):
            self._fault_stop("CaptureProduct Result identity mismatch")
            return
        if not result.success:
            if int(result.error_code) == int(ErrorCode.COMMAND_CONFLICT):
                self._fault_stop(f"CaptureProduct command conflict: {result.reason}")
                return
            self._finish_capture_failure(
                station_id, result.reason or "final station capture failed"
            )
            return
        returned_camera_ids = tuple(image.camera_id for image in result.images)
        required_camera_ids = self.station_camera_ids[station_id]
        if (
            not result.frame_batch_id
            or not result.inference_job_id
            or int(result.attempt_count) < 1
            or len(returned_camera_ids) != len(set(returned_camera_ids))
            or set(returned_camera_ids) != set(required_camera_ids)
            or any(
                not image.file_path
                or not is_sha256_hex(image.sha256)
                or int(image.file_size_bytes) <= 0
                or int(image.width) <= 0
                or int(image.height) <= 0
                or image.pixel_format != "RGB8_PNG"
                for image in result.images
            )
        ):
            self._finish_capture_failure(
                station_id,
                "CaptureProduct success payload is incomplete or camera set differs",
            )
            return
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("CaptureProduct Result references unknown product")
            return
        try:
            with self._flow_lock:
                context.mark_capture_succeeded(
                    station_id,
                    capture_id=capture_id,
                    frame_batch_id=result.frame_batch_id,
                    inference_job_id=result.inference_job_id,
                )
        except ProductFlowError as exc:
            self._fault_stop(f"capture result application failed: {exc}")
            return
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="STATION_CAPTURE_COMPLETED",
            product_id=context.product_id,
            payload={
                **context.snapshot(),
                "station_id": station_id.name,
                "capture_id": capture_id,
                "frame_batch_id": result.frame_batch_id,
                "image_count": len(result.images),
                "frame_arrival_skew_us": int(result.frame_arrival_skew_us),
            },
        )
        self._resume_conveyor_after_capture(cycle.product_id, station_id)

    def _finish_capture_failure(self, station_id: StationId, reason: str) -> None:
        cycle = self._station_cycles.get(station_id)
        if cycle is None:
            return
        if self._ignore_operation_while_fault_stopped(
            "CAPTURE_FAILURE",
            product_id=cycle.product_id,
            correlation_id=cycle.capture_id,
        ):
            return
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("capture failure references unknown product")
            return
        try:
            with self._flow_lock:
                context.mark_capture_failed(
                    station_id, capture_id=cycle.capture_id, reason=reason
                )
        except ProductFlowError as exc:
            self._fault_stop(f"capture failure application failed: {exc}")
            return
        self._emit_log_event(
            severity=LogEvent.ERROR,
            event_type="STATION_CAPTURE_FORCED_NG",
            product_id=context.product_id,
            payload={**context.snapshot(), "station_id": station_id.name, "reason": reason},
        )
        self._resume_conveyor_after_capture(cycle.product_id, station_id)

    def _resume_conveyor_after_capture(
        self, product_id: str, station_id: StationId
    ) -> None:
        """Capture 성공 후 추론 완료를 기다리지 않고 공정을 재개합니다."""

        cycle = self._station_cycles.get(station_id)
        if cycle is None or cycle.product_id != product_id:
            self._fault_stop("capture resume cycle identity mismatch")
            return
        cycle.phase = StationCyclePhase.RESUME_PENDING
        cycle.deadline_ns = (
            time.monotonic_ns() + self.conveyor_resume_timeout_ms * 1_000_000
        )
        if self.system_state != SystemState.RUN_SYS:
            self._deferred_capture_resumes.add((product_id, station_id))
            return
        self._publish_system_command(
            SystemCommand.RESUME,
            f"resume {cycle.conveyor_id.name} after station {station_id.name} capture",
            target_conveyor_id=int(cycle.conveyor_id),
        )
        # sim profile에는 실제 conveyor status adapter가 없으므로 즉시 확인합니다.
        # TODO(HARDWARE): Control의 station별 실제 RUN 확인 이벤트가 연결되면
        # hardware profile에서 아래 공개 확인 진입점을 호출해야 합니다.
        if self.profile == "sim":
            self.confirm_conveyor_resumed(product_id, station_id)

    def confirm_conveyor_resumed(
        self, product_id: str, station_id: StationId
    ) -> bool:
        """Control의 해당 층 실제 RUN 확인 뒤 제품 물리 상태를 전진시킵니다."""

        cycle = self._station_cycles.get(station_id)
        if (
            cycle is None
            or cycle.product_id != product_id
            or cycle.phase != StationCyclePhase.RESUME_PENDING
        ):
            return False
        if self._ignore_operation_while_fault_stopped(
            "CONVEYOR_RESUME_CONFIRMATION",
            product_id=product_id,
            correlation_id=station_id.name,
        ):
            return False
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("conveyor resume references unknown product")
            return False
        try:
            with self._flow_lock:
                context.mark_conveyor_resumed_after_capture(station_id)
        except ProductFlowError as exc:
            self._fault_stop(f"conveyor resume application failed: {exc}")
            return False
        cycle.deadline_ns = 0
        self._station_cycles.pop(station_id, None)
        self._deferred_capture_resumes.discard((product_id, station_id))
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="CONVEYOR_RESUMED_AFTER_CAPTURE",
            product_id=product_id,
            payload=context.snapshot(),
        )
        return True

    def _pause_station_for_recovery(self, station_id: StationId, reason: str) -> None:
        if self._ignore_operation_while_fault_stopped(
            "STATION_RECOVERY",
            correlation_id=station_id.name,
        ):
            return
        cycle = self._station_cycles.get(station_id)
        if cycle is not None:
            context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
            if context is not None:
                try:
                    with self._flow_lock:
                        station = context.station(station_id)
                        if station.result_completed:
                            context.promote_capture_completion_from_vision(station_id)
                            cycle.phase = StationCyclePhase.RESUME_PENDING
                            cycle.deadline_ns = 0
                            self._deferred_capture_resumes.add(
                                (cycle.product_id, station_id)
                            )
                        else:
                            context.reset_unfinished_station_cycle(station_id)
                            self._station_cycles.pop(station_id, None)
                            self._deferred_station_starts.add(
                                (cycle.product_id, station_id)
                            )
                except ProductFlowError as exc:
                    self._fault_stop(
                        f"station recovery state cannot be reconciled: {exc}"
                    )
                    return
        self.request_recoverable_device_pause(
            reason, pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL
        )

    # endregion

    # region BLOCK 5 - 비동기 비전 결과와 최종 판정

    def _handle_station_result(self, message: StationResult) -> None:
        if message.header.session_id != self.session_id:
            return
        score = finite_float_or_none(message.score)
        if self._ignore_operation_while_fault_stopped(
            "STATION_RESULT",
            product_id=message.product_id,
            correlation_id=message.capture_id,
            diagnostic_details={
                "fifo_sequence": int(message.fifo_sequence),
                "station_id": int(message.station_id),
                "frame_batch_id": message.frame_batch_id,
                "inference_job_id": message.inference_job_id,
                "result_revision": int(message.result_revision),
                "verdict": int(message.verdict),
                "score": score,
                "score_is_finite": score is not None,
                "model_version": message.model_version,
                "completed_at_sec": int(message.completed_at.sec),
                "completed_at_nanosec": int(message.completed_at.nanosec),
            },
        ):
            return
        try:
            station_id = StationId(message.station_id)
        except ValueError:
            self._fault_stop("station result contains an invalid station_id")
            return
        try:
            verdict = Verdict(message.verdict)
        except ValueError:
            verdict = None
        context = self.ledger.get(message.product_id, message.fifo_sequence)
        if context is None:
            if self.ledger.is_retired_product(
                message.product_id, message.fifo_sequence
            ) or self.ledger.is_retired_capture(message.capture_id):
                self._emit_log_event(
                    severity=LogEvent.WARNING,
                    event_type="EXPIRED_PRODUCT_RESULT_IGNORED",
                    product_id=message.product_id,
                    payload={"capture_id": message.capture_id},
                )
                return
            owner = self.ledger.capture_owner(message.capture_id)
            if owner is not None:
                self._fault_stop(
                    "station result product identity conflicts with capture owner"
                )
            else:
                self._fault_stop("station result references unknown product identity")
            return
        owner = self.ledger.capture_owner(message.capture_id)
        if owner != (context.product_id, station_id):
            self._fault_stop("station result capture identity is not registered")
            return
        if context.removed:
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="REMOVED_PRODUCT_RESULT_IGNORED",
                product_id=context.product_id,
                payload={"capture_id": message.capture_id},
            )
            return
        if context.station(station_id).capture_id != message.capture_id:
            # 장치 복구 중 이전 capture를 폐기하고 새 capture_id로
            # 재시작했다면 이전 작업의 늦은 결과는 물리 상태를 바꾸지 않습니다.
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="SUPERSEDED_STATION_RESULT_IGNORED",
                product_id=context.product_id,
                payload={
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "active_capture_id": context.station(station_id).capture_id,
                },
            )
            return
        if context.locked is not None:
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="LATE_STATION_RESULT_IGNORED",
                product_id=context.product_id,
                payload={
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "result_revision": int(message.result_revision),
                    "locked_verdict": context.locked.verdict.name,
                },
            )
            return
        if (
            verdict is None
            or score is None
            or int(message.result_revision) < 1
            or not message.frame_batch_id
            or not message.inference_job_id
        ):
            reason = (
                "station result score must be finite"
                if score is None
                else "station result payload is invalid or incomplete"
            )
            with self._flow_lock:
                context.record_station_contract_failure(station_id, reason)
            self._emit_log_event(
                severity=LogEvent.ERROR,
                event_type="VISION_RESULT_CONTRACT_FORCED_NG",
                product_id=context.product_id,
                payload={
                    **context.snapshot(),
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "reason": reason,
                },
            )
            return
        decision = StationDecision(
            station_id=station_id,
            verdict=verdict,
            revision=message.result_revision,
            capture_id=message.capture_id,
            inference_job_id=message.inference_job_id,
            frame_batch_id=message.frame_batch_id,
        )
        try:
            with self._flow_lock:
                if not context.apply_station_result(decision):
                    return
        except StationResultConflict as exc:
            # 같은 제품·station·capture의 결과값만 충돌한 경우에는 두 결과를
            # 진단 보존하고 Sensor3에서 FORCED_NG로 잠급니다.
            self._emit_log_event(
                severity=LogEvent.ERROR,
                event_type="VISION_RESULT_CONFLICT_FORCED_NG",
                product_id=context.product_id,
                payload={
                    **context.snapshot(),
                    "station_id": decision.station_id.name,
                    "capture_id": decision.capture_id,
                    "reason": str(exc),
                },
            )
            return
        except ProductIdentityConflict as exc:
            self._fault_stop(f"station result identity conflict: {exc}")
            return
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="STATION_RESULT_APPLIED",
            product_id=context.product_id,
            payload={
                **context.snapshot(),
                "station_id": decision.station_id.name,
                "capture_id": decision.capture_id,
                "result_revision": decision.revision,
            },
        )

    def _handle_station_failure(self, message: StationInferenceFailed) -> None:
        if message.header.session_id != self.session_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "STATION_FAILURE",
            product_id=message.product_id,
            correlation_id=message.capture_id,
            diagnostic_details={
                "fifo_sequence": int(message.fifo_sequence),
                "station_id": int(message.station_id),
                "frame_batch_id": message.frame_batch_id,
                "inference_job_id": message.inference_job_id,
                "result_revision": int(message.result_revision),
                "error_code": int(message.error_code),
                "reason": message.reason,
                "failed_at_sec": int(message.failed_at.sec),
                "failed_at_nanosec": int(message.failed_at.nanosec),
            },
        ):
            return
        context = self.ledger.get(message.product_id, message.fifo_sequence)
        if context is None:
            if self.ledger.is_retired_product(
                message.product_id, message.fifo_sequence
            ) or self.ledger.is_retired_capture(message.capture_id):
                self._emit_log_event(
                    severity=LogEvent.WARNING,
                    event_type="EXPIRED_PRODUCT_FAILURE_IGNORED",
                    product_id=message.product_id,
                    payload={"capture_id": message.capture_id},
                )
                return
            self._fault_stop("station failure references unknown product identity")
            return
        try:
            station_id = StationId(message.station_id)
        except ValueError:
            self.get_logger().error("station failure contains an invalid station_id")
            return
        owner = self.ledger.capture_owner(message.capture_id)
        if owner != (context.product_id, station_id):
            self._fault_stop("station failure capture identity is not registered")
            return
        if context.removed:
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="REMOVED_PRODUCT_FAILURE_IGNORED",
                product_id=context.product_id,
                payload={"capture_id": message.capture_id},
            )
            return
        if context.station(station_id).capture_id != message.capture_id:
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="SUPERSEDED_STATION_FAILURE_IGNORED",
                product_id=context.product_id,
                payload={
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "active_capture_id": context.station(station_id).capture_id,
                },
            )
            return
        if context.locked is not None:
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="LATE_STATION_FAILURE_IGNORED",
                product_id=context.product_id,
                payload={
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "reason": message.reason,
                },
            )
            return
        if (
            int(message.result_revision) < 1
            or not message.frame_batch_id
            or not message.inference_job_id
        ):
            reason = "station failure payload is incomplete"
            with self._flow_lock:
                context.record_station_contract_failure(station_id, reason)
            self._emit_log_event(
                severity=LogEvent.ERROR,
                event_type="VISION_FAILURE_CONTRACT_FORCED_NG",
                product_id=context.product_id,
                payload={
                    **context.snapshot(),
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "reason": reason,
                },
            )
            return
        try:
            with self._flow_lock:
                applied = context.record_station_failure(
                    station_id,
                    capture_id=message.capture_id,
                    frame_batch_id=message.frame_batch_id,
                    inference_job_id=message.inference_job_id,
                    revision=int(message.result_revision),
                    reason=message.reason,
                )
        except ProductIdentityConflict as exc:
            self._fault_stop(f"station failure identity conflict: {exc}")
            return
        if applied:
            self._emit_log_event(
                severity=LogEvent.ERROR,
                event_type="STATION_INFERENCE_FORCED_NG",
                product_id=context.product_id,
                payload={
                    **context.snapshot(),
                    "station_id": station_id.name,
                    "capture_id": message.capture_id,
                    "error_code": int(message.error_code),
                    "reason": message.reason,
                },
            )

    # endregion

    # region BLOCK 6 - Sensor3 액추에이터 분류와 FIFO 제거

    def lock_product_at_sensor3(
        self, product_id: str, fifo_sequence: int, sensor3_event_id: str
    ) -> None:
        """물리 FIFO 추적기가 Sensor3 이벤트를 제품에 매핑한 뒤 호출합니다."""

        context = self.ledger.get(product_id, fifo_sequence)
        if context is None:
            raise KeyError("Sensor3 mapping references unknown product")
        locked = context.lock_at_sensor3(sensor3_event_id)
        self._accept_locked(locked)
        self._schedule_actuation(locked)

    def _schedule_actuation(self, locked: LockedProduct) -> None:
        """Sensor3에 매핑된 잠금 판정을 실제 분류 명령으로 변환합니다."""

        context = self.ledger.get(locked.product_id, locked.fifo_sequence)
        if context is None or context.locked != locked:
            self._fault_stop("actuation schedule references an unknown locked product")
            return
        actuator_job_id = new_uuid()
        command_id = new_uuid()
        try:
            with self._flow_lock:
                context.begin_actuation(
                    actuator_job_id=actuator_job_id,
                    command_id=command_id,
                )
        except ProductFlowError as exc:
            self._fault_stop(f"actuation schedule failed: {exc}")
            return
        cycle = ActuationCycle(
            product_id=locked.product_id,
            fifo_sequence=locked.fifo_sequence,
            actuator_job_id=actuator_job_id,
            command_id=command_id,
        )
        self._actuation_cycles[actuator_job_id] = cycle
        self._send_actuation_goal(locked)

    def _send_actuation_goal(self, locked: LockedProduct) -> None:
        """ControlNode에 ActuateProduct Goal을 전송합니다."""

        context = self.ledger.get(locked.product_id, locked.fifo_sequence)
        if context is None or not context.actuator_job_id:
            self._fault_stop("ActuateProduct Goal has no product job identity")
            return
        cycle = self._actuation_cycles.get(context.actuator_job_id)
        if cycle is None:
            self._fault_stop("ActuateProduct cycle is missing")
            return
        if not self.actuate_client.wait_for_server(timeout_sec=0.0):
            self.request_recoverable_device_pause(
                "ActuateProduct Action server is unavailable",
                pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
            )
            return
        actuator_command = (
            ActuateProduct.Goal.PASS_THROUGH
            if locked.verdict == Verdict.PASS
            else ActuateProduct.Goal.DIVERT_NG
        )
        goal = ActuateProduct.Goal()
        payload = {
            "product_id": locked.product_id,
            "fifo_sequence": locked.fifo_sequence,
            "sensor3_event_id": locked.sensor3_event_id,
            "actuator_job_id": cycle.actuator_job_id,
            "actuator_command": int(actuator_command),
        }
        self._fill_command_header(
            goal.command,
            command_id=cycle.command_id,
            correlation_id=cycle.actuator_job_id,
            payload=payload,
        )
        goal.product_id = locked.product_id
        goal.fifo_sequence = locked.fifo_sequence
        goal.sensor3_event_id = locked.sensor3_event_id
        goal.actuator_command = int(actuator_command)
        cycle.phase = ActuationPhase.GOAL_PENDING
        cycle.deadline_ns = (
            time.monotonic_ns() + self.actuation_timeout_ms * 1_000_000
        )
        try:
            future = self.actuate_client.send_goal_async(goal)
        except Exception as exc:
            self.request_recoverable_device_pause(
                f"ActuateProduct Goal send failed: {type(exc).__name__}",
                pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
            )
            return
        future.add_done_callback(
            partial(
                self._handle_actuation_goal_response,
                cycle.actuator_job_id,
                cycle.command_id,
            )
        )

    def _handle_actuation_goal_response(
        self, actuator_job_id: str, command_id: str, future
    ) -> None:
        cycle = self._actuation_cycles.get(actuator_job_id)
        if cycle is None or cycle.command_id != command_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "ACTUATION_GOAL_RESPONSE",
            product_id=cycle.product_id,
            correlation_id=command_id,
        ):
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_recoverable_device_pause(
                f"ActuateProduct Goal response failed: {type(exc).__name__}",
                pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
            )
            return
        if not goal_handle.accepted:
            self.request_recoverable_device_pause(
                "ActuateProduct Goal was rejected",
                pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
            )
            return
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("accepted actuation references unknown product")
            return
        try:
            with self._flow_lock:
                context.mark_actuation_accepted(command_id)
        except ProductFlowError as exc:
            self._fault_stop(f"actuation acceptance failed: {exc}")
            return
        cycle.goal_handle = goal_handle
        cycle.phase = ActuationPhase.WAITING_RESULT
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._handle_actuation_result, actuator_job_id, command_id)
        )

    def _handle_actuation_result(
        self, actuator_job_id: str, command_id: str, future
    ) -> None:
        """실제 액추에이터 완료를 제품에 반영하고 FIFO 제거를 시도합니다."""

        cycle = self._actuation_cycles.get(actuator_job_id)
        if cycle is None or cycle.command_id != command_id:
            return
        if self._ignore_operation_while_fault_stopped(
            "ACTUATION_RESULT",
            product_id=cycle.product_id,
            correlation_id=command_id,
        ):
            return
        try:
            result = future.result().result
        except Exception as exc:
            self._fault_stop(
                f"ActuateProduct Result unavailable after Goal acceptance: "
                f"{type(exc).__name__}"
            )
            return
        if result.product_id != cycle.product_id:
            self._fault_stop("ActuateProduct Result product identity mismatch")
            return
        if not result.success:
            self._fault_stop(
                "actuation result is not physically trustworthy: "
                + (result.reason or "ActuateProduct failed")
            )
            return
        if not result.actuation_id:
            self._fault_stop("ActuateProduct success has no actuation_id")
            return
        actuation_identity = self._actuation_id_owners.inspect(
            result.actuation_id, actuator_job_id
        )
        if actuation_identity.kind == ReplayKind.CONFLICT:
            self._fault_stop("actuation_id was reused for another actuator job")
            return
        if actuation_identity.kind == ReplayKind.NEW:
            self._actuation_id_owners.remember(
                result.actuation_id, actuator_job_id, actuator_job_id
            )
        context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
        if context is None:
            self._fault_stop("ActuateProduct completion references unknown product")
            return
        try:
            with self._flow_lock:
                context.mark_actuation_completed(command_id, result.actuation_id)
        except ProductFlowError as exc:
            self._fault_stop(f"actuation completion failed: {exc}")
            return
        cycle.phase = ActuationPhase.COMPLETED
        cycle.deadline_ns = 0
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="ACTUATOR_COMPLETED",
            product_id=context.product_id,
            payload=context.snapshot(),
        )
        self._remove_completed_fifo_prefix()

    def _remove_completed_fifo_prefix(self) -> None:
        """FIFO 맨 앞부터 연속된 DONE 제품만 순서대로 제거합니다."""

        if self._ignore_operation_while_fault_stopped("FIFO_PREFIX_REMOVAL"):
            return
        with self._flow_lock:
            removed = self.ledger.remove_completed_prefix(
                now_ns=time.monotonic_ns()
            )
        for context in removed:
            self._actuation_cycles.pop(context.actuator_job_id, None)
            self._emit_log_event(
                severity=LogEvent.INFO,
                event_type="PRODUCT_REMOVED_FROM_ACTIVE_FIFO",
                product_id=context.product_id,
                payload=context.snapshot(),
            )

    # endregion

    # region BLOCK 7 - PAUSE/RESET/FAULT_STOP 복구

    def _request_all_conveyors_stop(self, reason: str) -> None:
        """Control에 상·하층 컨베이어 안전 정지를 요청합니다."""

        if self.system_state == SystemState.PAUSING:
            self._pause_deadline_ns = (
                time.monotonic_ns() + self.pause_stop_timeout_ms * 1_000_000
            )
        else:
            # FAULT_STOP은 PAUSING timeout 전이를 사용하지 않습니다. 이전
            # 정지 요청의 deadline을 남겨 다음 운전에서 오해하지 않습니다.
            self._pause_deadline_ns = 0
        self._publish_system_command(SystemCommand.PAUSE, reason)
        if self.profile == "sim":
            self.confirm_all_conveyors_stopped()

    def _request_conveyor_run(self, reason: str) -> None:
        """정합성 검증을 통과한 뒤 Control에 운전 재개를 요청합니다."""

        self._publish_system_command(SystemCommand.RESUME, reason)
        # TODO(HARDWARE): Control의 상·하층 실제 RUN 확인 이벤트가 연결되면
        # hardware profile에서 confirm_all_conveyors_running()을 호출합니다.
        if self.profile == "sim":
            self.confirm_all_conveyors_running()

    def _verify_resume_conditions(self) -> bool:
        """재개 전에 노드·FIFO·센서·컨베이어 정합성을 검사합니다."""

        return (
            self.system_state == SystemState.PAUSED
            and self._all_workers_ready_for_session()
            and self.equipment.in_place_guards_satisfied()
            and self._validate_fifo_alignment()
            and self.recovery_policy != RecoveryPolicy.LINE_CLEAR_REQUIRED
            and not (
                self._paused_by_queue and not self._queue_recovered_pending
            )
        )

    def _verify_reset_entry_conditions(self) -> bool:
        """FAULT_STOP의 recovery policy에 맞는 RESET guard를 확인합니다."""

        if self.system_state != SystemState.FAULT_STOP:
            return False
        if self.recovery_policy == RecoveryPolicy.LINE_CLEAR_REQUIRED:
            return self.equipment.line_clear_guards_satisfied()
        if self.recovery_policy in {
            RecoveryPolicy.EQUIPMENT_CHECK_REQUIRED,
            RecoveryPolicy.RETRY_IN_PLACE,
        }:
            return (
                self.equipment.in_place_guards_satisfied()
                and self._validate_fifo_alignment()
            )
        return False

    def _start_reset_sequence(self, reason: str) -> None:
        """오류 종류에 맞는 소프트 복구 또는 line clear 절차를 시작합니다."""

        self._advance_command_epoch(
            "reset sequence started",
            reinitialize_all_workers=True,
        )
        self._publish_system_command(SystemCommand.RESET, reason)
        if self.profile == "sim":
            self.confirm_reset_completed()

    def _request_line_clear(self, reason: str) -> None:
        """운영자에게 라인의 모든 제품 제거를 요구하고 재가동을 차단합니다."""

        self.equipment.clear_operator_confirmation()
        message = (
            "[FAULT_STOP] 제품 추적 정보를 신뢰할 수 없습니다. 상·하층 "
            "컨베이어와 액추에이터가 정지했는지 확인한 뒤 라인의 모든 "
            "제품을 제거하고 LINE_CLEAR_CONFIRMED를 입력해 주세요."
        )
        self.get_logger().error(f"{message} reason={reason}")
        self._emit_log_event(
            severity=LogEvent.CRITICAL,
            event_type="LINE_CLEAR_REQUIRED",
            payload={"reason": reason, "operator_message": message},
        )

    def update_equipment_snapshot(
        self,
        *,
        upper_running: bool | None = None,
        lower_running: bool | None = None,
        upper_stopped: bool | None = None,
        lower_stopped: bool | None = None,
        sensor_1_clear: bool | None = None,
        sensor_2_clear: bool | None = None,
        sensor_3_clear: bool | None = None,
        actuator_safe: bool | None = None,
        actuator_area_clear: bool | None = None,
        estop_asserted: bool | None = None,
    ) -> None:
        """향후 Control typed 상태 event가 갱신할 안전 guard 진입점입니다."""

        updates = (
            (self.equipment.conveyor_running, ConveyorId.UPPER, upper_running),
            (self.equipment.conveyor_running, ConveyorId.LOWER, lower_running),
            (self.equipment.conveyor_stopped, ConveyorId.UPPER, upper_stopped),
            (self.equipment.conveyor_stopped, ConveyorId.LOWER, lower_stopped),
            (self.equipment.sensor_clear, 1, sensor_1_clear),
            (self.equipment.sensor_clear, 2, sensor_2_clear),
            (self.equipment.sensor_clear, 3, sensor_3_clear),
        )
        for target, key, value in updates:
            if value is not None:
                target[key] = value
        if actuator_safe is not None:
            self.equipment.actuator_safe = actuator_safe
        if actuator_area_clear is not None:
            self.equipment.actuator_area_clear = actuator_area_clear
        if estop_asserted is not None:
            self.equipment.estop_asserted = estop_asserted
            if estop_asserted:
                self.report_estop_asserted()

    def confirm_line_cleared(self, operator_id: str) -> bool:
        """신규 RUN 전 또는 FAULT 복구 중의 물리 빈 라인 확인입니다."""

        if not operator_id or self.system_state not in {
            SystemState.READY,
            SystemState.FAULT_STOP,
        }:
            return False
        self.equipment.line_clear_confirmed = True
        self.equipment.operator_id = operator_id
        self._emit_log_event(
            severity=LogEvent.WARNING,
            event_type="LINE_CLEAR_OPERATOR_CONFIRMED",
            payload={"operator_id": operator_id},
        )
        return self.equipment.line_clear_guards_satisfied()

    def _cancel_tracked_operation_goals(self, *, include_actuation: bool) -> None:
        """복구·종료 전에 Master가 보유한 Action Goal에 취소를 요청합니다.

        취소 요청은 best-effort입니다. 실제 장비 안전 정지는 RESET/PAUSE 명령과
        Control adapter의 안전 출력 적용으로 별도 보장해야 합니다.
        """

        goal_handles = [
            goal_handle
            for cycle in tuple(self._station_cycles.values())
            for goal_handle in (cycle.position_goal_handle, cycle.capture_goal_handle)
            if goal_handle is not None
        ]
        if include_actuation:
            goal_handles.extend(
                cycle.goal_handle
                for cycle in tuple(self._actuation_cycles.values())
                if cycle.goal_handle is not None
                and cycle.phase != ActuationPhase.COMPLETED
            )
        for goal_handle in goal_handles:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                # RESET/PAUSE가 실제 안전 정지를 별도로 수행하며, Action 취소
                # 실패 자체가 복구 callback을 중단시키지는 않게 합니다.
                pass

    def confirm_reset_completed(self) -> bool:
        """Control의 RESET·안전 출력 확인 후 복구 정책에 맞게 완료합니다."""

        if self.system_state != SystemState.RESETTING:
            return False
        if self.recovery_policy == RecoveryPolicy.LINE_CLEAR_REQUIRED:
            if not self.equipment.line_clear_guards_satisfied():
                self.report_reset_failed("line clear guards are incomplete")
                return False
            self._cancel_tracked_operation_goals(include_actuation=True)
            with self._flow_lock:
                cleared = self.ledger.clear_active(now_ns=time.monotonic_ns())
                self.sensor_events.reset()
                self.result_reorder.reset(self.ledger.next_sequence)
                self._station_cycles.clear()
                self._actuation_cycles.clear()
                self._actuation_id_owners = IdempotencyStore(capacity=4096)
                self._deferred_station_starts.clear()
                self._deferred_capture_resumes.clear()
                self._paused_by_queue = False
                self._queue_recovered_pending = False
            self._emit_log_event(
                severity=LogEvent.WARNING,
                event_type="LINE_CLEAR_APPLIED",
                payload={
                    "cleared_product_ids": [item.product_id for item in cleared],
                    "operator_id": self.equipment.operator_id,
                },
            )
            return self.report_reset_succeeded(line_cleared=True)
        if not (
            self.equipment.in_place_guards_satisfied()
            and self._validate_fifo_alignment()
        ):
            self.report_reset_failed("in-place reset guards are incomplete")
            return False
        if not self._prepare_in_place_recovery():
            return False
        # RESET 뒤에는 이전 Vision queue 상태도 새 command epoch의 증거가 아닙니다.
        self._paused_by_queue = False
        self._queue_recovered_pending = False
        return self.report_reset_succeeded(line_cleared=False)

    def _prepare_in_place_recovery(self) -> bool:
        """정지 확인 후 미완료 Action을 안전하게 재시도 대기 상태로 변환합니다."""

        # 액추에이터 Goal이 수락된 뒤 결과를 잃은 경우에는
        # 제품이 어느 경로로 지나갔는지 알 수 없으므로 in-place를
        # 허용하지 않습니다.
        if any(
            cycle.phase == ActuationPhase.WAITING_RESULT
            for cycle in self._actuation_cycles.values()
        ):
            self.report_critical_fault(
                "in-place reset rejected: accepted actuation result is unknown",
                recovery_policy=RecoveryPolicy.LINE_CLEAR_REQUIRED,
            )
            return False

        self._cancel_tracked_operation_goals(include_actuation=False)
        for station_id, cycle in tuple(self._station_cycles.items()):
            cycle.deadline_ns = 0
            context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
            if context is None:
                self.report_critical_fault(
                    "in-place reset found an unknown station product",
                    recovery_policy=RecoveryPolicy.LINE_CLEAR_REQUIRED,
                )
                return False
            try:
                with self._flow_lock:
                    station = context.station(station_id)
                    if cycle.phase == StationCyclePhase.RESUME_PENDING:
                        self._deferred_capture_resumes.add(
                            (cycle.product_id, station_id)
                        )
                    elif station.result_completed:
                        context.promote_capture_completion_from_vision(station_id)
                        cycle.phase = StationCyclePhase.RESUME_PENDING
                        self._deferred_capture_resumes.add(
                            (cycle.product_id, station_id)
                        )
                    else:
                        context.reset_unfinished_station_cycle(station_id)
                        self._station_cycles.pop(station_id, None)
                        self._deferred_station_starts.add(
                            (cycle.product_id, station_id)
                        )
            except ProductFlowError as exc:
                self.report_critical_fault(
                    f"in-place station recovery failed: {exc}",
                    recovery_policy=RecoveryPolicy.LINE_CLEAR_REQUIRED,
                )
                return False

        for cycle in self._actuation_cycles.values():
            if cycle.phase == ActuationPhase.GOAL_PENDING:
                cycle.deadline_ns = 0
        return True

    def _handle_queue_state(self, message: VisionQueueState) -> None:
        if message.header.session_id != self.session_id:
            return
        # QueueState도 고빈도 telemetry이므로 FAULT_STOP 중에는 상태를 바꾸거나
        # 로그를 만들지 않습니다. RESET 성공 후 새 queue 상태만 수락합니다.
        if self.system_state == SystemState.FAULT_STOP:
            return
        if message.state == VisionQueueState.ENQUEUE_BLOCKED:
            if not self._paused_by_queue:
                self._paused_by_queue = True
                self._queue_recovered_pending = False
                self._pause(
                    f"Vision queue full; preserving FrameBatch {message.blocked_frame_batch_id}"
                )
        elif message.state == VisionQueueState.ACCEPTING and self._paused_by_queue:
            self._queue_recovered_pending = True
            # ENQUEUE_BLOCKED 동안은 보존된 FrameBatch가 queue 자리를
            # 기다리므로 capture deadline을 소진시키지 않습니다. 수락 가능
            # 신호를 받은 시점부터 유한한 결과 대기 시간을 다시 부여합니다.
            now_ns = time.monotonic_ns()
            for cycle in self._station_cycles.values():
                if (
                    cycle.phase == StationCyclePhase.WAITING_CAPTURE_RESULT
                    and cycle.deadline_ns == 0
                ):
                    cycle.deadline_ns = (
                        now_ns + self.capture_timeout_ms * 1_000_000
                    )
            if self.system_state == SystemState.PAUSED:
                self._resume_after_queue_recovery()

    def _pause(self, reason: str) -> None:
        if self.system_state in {
            SystemState.PAUSING,
            SystemState.PAUSED,
            SystemState.FAULT_STOP,
        }:
            return
        if self.system_state == SystemState.RUN_SYS:
            self.request_recoverable_device_pause(reason)
        else:
            # INITIALIZING/READY/RESETTING은 이미 장비가 정지된 단계이므로
            # 바로 FAULT_STOP하지 않고 해당 블록의 재시도 경로를 유지합니다.
            self.set_health_state(NodeHealthState.DEGRADED)
        self.get_logger().error(reason)

    def _fault_stop(self, reason: str) -> None:
        """추적·결과 무결성 충돌을 자동 재개 불가 상태로 전환합니다."""

        self.report_critical_fault(
            reason,
            recovery_policy=RecoveryPolicy.LINE_CLEAR_REQUIRED,
        )
        self.get_logger().error(reason)

    def _ignore_operation_while_fault_stopped(
        self,
        operation: str,
        *,
        product_id: str = "",
        correlation_id: str = "",
        diagnostic_details: dict[str, object] | None = None,
    ) -> bool:
        """FAULT_STOP 이후 도착한 비동기 결과가 제품 원장을 바꾸지 못하게 합니다."""

        if self.system_state != SystemState.FAULT_STOP:
            return False
        self._emit_log_event(
            severity=LogEvent.WARNING,
            event_type="LATE_OPERATION_IGNORED_DURING_FAULT_STOP",
            product_id=product_id,
            payload=build_late_operation_diagnostic(
                operation,
                correlation_id,
                diagnostic_details,
            ),
        )
        return True

    def confirm_all_conveyors_stopped(self) -> None:
        """Control의 실제 정지 완료를 받은 뒤 PAUSING을 확정합니다.

        실제 Sensor/Conveyor 상태 매핑이 확정되면 PositionSettled가 아닌 별도의
        typed 장비 상태 이벤트에서 이 확장점을 호출해야 합니다.
        """

        self.equipment.mark_all_stopped()
        self._pause_deadline_ns = 0
        if self.system_state == SystemState.PAUSING:
            if not self._validate_fifo_alignment():
                self.report_critical_fault(
                    "FIFO/position mismatch after conveyor stop",
                    recovery_policy=RecoveryPolicy.LINE_CLEAR_REQUIRED,
                )
                return
            transition = self._apply_system_event(
                SystemEvent.ALL_CONVEYORS_STOPPED,
                "all conveyors confirmed STOP_CONV",
            )
            if transition is None:
                return
            if self.pause_reason == PauseReason.OPERATOR:
                self.set_health_state(NodeHealthState.READY)
        if self._shutdown_requested:
            self._finalize_shutdown_sequence()
            return
        if self.system_state != SystemState.PAUSED:
            return
        if self._paused_by_queue and self._queue_recovered_pending:
            self._resume_after_queue_recovery()

    def _resume_after_queue_recovery(self) -> None:
        """실제 정지 확인과 동일 FrameBatch enqueue 성공 뒤 자동 재개합니다."""

        if self.system_state != SystemState.PAUSED:
            return
        if self.request_resume("Vision queue recovered", automatic=True):
            self._paused_by_queue = False
            self._queue_recovered_pending = False

    def _resume_deferred_operations(self) -> None:
        """PAUSED 동안 보류한 물리 후속 작업을 제품 상태 기준으로 재개합니다."""

        for product_id, station_id in tuple(self._deferred_capture_resumes):
            self.confirm_conveyor_resumed(product_id, station_id)
        for product_id, station_id in tuple(self._deferred_station_starts):
            context = self.ledger.get_by_id(product_id)
            if context is None or context.removed:
                self._deferred_station_starts.discard((product_id, station_id))
                continue
            self._start_station_cycle(
                context.product_id, context.fifo_sequence, station_id
            )
        for cycle in tuple(self._actuation_cycles.values()):
            if cycle.phase != ActuationPhase.GOAL_PENDING:
                continue
            context = self.ledger.get(cycle.product_id, cycle.fifo_sequence)
            if context is not None and context.locked is not None:
                self._send_actuation_goal(context.locked)

    def _fill_command_header(
        self,
        command,
        *,
        command_id: str,
        correlation_id: str,
        payload: dict[str, object],
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        command.header.stamp = stamp
        command.header.session_id = self.session_id
        command.header.message_id = new_uuid()
        command.header.correlation_id = correlation_id
        command.command_epoch = self.command_epoch
        command.command_id = command_id
        command.payload_digest = payload_digest(payload)
        command.issued_at = stamp

    def _publish_system_command(
        self,
        command_type: int,
        reason: str,
        *,
        target_conveyor_id: int = int(SystemCommand.ALL_CONVEYORS),
    ) -> None:
        message = SystemCommand()
        command_id = new_uuid()
        self._fill_command_header(
            message.command,
            command_id=command_id,
            correlation_id="",
            payload={
                "command_type": int(command_type),
                "target_conveyor_id": int(target_conveyor_id),
                "reason": reason,
                "epoch": self.command_epoch,
            },
        )
        message.command_type = command_type
        message.target_conveyor_id = int(target_conveyor_id)
        message.reason = reason
        self._system_command_publisher.publish(message)

    # endregion

    # region BLOCK 8 - 판정 발행, 로그 spool/ACK와 프로그램 종료

    def _accept_locked(self, locked: LockedProduct) -> None:
        try:
            ready_results = self.result_reorder.add(locked)
        except ValueError as exc:
            self._fault_stop(f"locked result reorder conflict: {exc}")
            return
        for ready in ready_results:
            message = ProductResultLocked()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.session_id = self.session_id
            message.header.message_id = new_uuid()
            message.header.correlation_id = ready.product_id
            message.product_id = ready.product_id
            message.fifo_sequence = ready.fifo_sequence
            message.final_verdict = int(ready.verdict)
            message.station_a_completed = ready.station_a_completed
            message.station_b_completed = ready.station_b_completed
            message.lock_reason = ready.reason
            message.sensor3_event_id = ready.sensor3_event_id
            message.locked_at = message.header.stamp
            self._locked_publisher.publish(message)
            self._emit_log_event(
                severity=(
                    LogEvent.INFO
                    if ready.verdict == Verdict.PASS
                    else LogEvent.WARNING
                ),
                event_type="PRODUCT_RESULT_LOCKED",
                product_id=ready.product_id,
                payload={
                    "fifo_sequence": ready.fifo_sequence,
                    "verdict": ready.verdict.name,
                    "station_a_completed": ready.station_a_completed,
                    "station_b_completed": ready.station_b_completed,
                    "reason": ready.reason,
                    "sensor3_event_id": ready.sensor3_event_id,
                },
            )

    def _emit_log_event(
        self,
        *,
        severity: int,
        event_type: str,
        payload: dict[str, object],
        product_id: str = "",
    ) -> None:
        """중요 이벤트를 보존하고 LogNode로 발행합니다.

        정상 경로는 local spool 선기록 후 발행입니다. spool이 고장 난
        비정상 경로에서도 LogNode까지 함께 차단하지 않고 직접 발행하여,
        최소 한 곳에는 이벤트가 남을 가능성을 보존합니다.
        """

        log_id = new_uuid()
        revision = 1
        envelope = {
            "schema_version": 2,
            "event_type": event_type,
            "severity": int(severity),
            "source_node": NodeId.MASTER.value,
            "producer_instance_id": self.node_instance_id,
            "session_id": self.session_id,
            "product_id": product_id,
            "payload": payload,
        }
        try:
            payload_json = canonical_json(envelope)
            digest = sha256_text(payload_json)
            record = SpoolRecord(log_id, revision, payload_json, digest)
        except (TypeError, ValueError, OverflowError) as exc:
            # 로그 payload 하나가 잘못되어도 ROS callback 전체를 죽이지 않습니다.
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().error(
                "Master log event serialization failed; "
                f"event={event_type}; error={type(exc).__name__}"
            )
            return
        if self._log_spool is None:
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().error(
                "Master local log spool unavailable; publishing without durable "
                f"producer copy; event={event_type}"
            )
            self._publish_spool_record(record)
            return
        try:
            self._log_spool.enqueue(record)
        except Exception as exc:
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().error(
                "Master log spool enqueue failed; publishing without durable "
                f"producer copy: {type(exc).__name__}"
            )
            self._publish_spool_record(record)
            return
        self._publish_spool_record(record)

    def _publish_spool_record(self, record: SpoolRecord) -> None:
        try:
            envelope = json.loads(record.payload_json)
        except (TypeError, ValueError):
            self.get_logger().error(f"invalid spool JSON for {record.log_id}")
            return
        message = LogEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.header.correlation_id = str(envelope.get("product_id", ""))
        message.log_id = record.log_id
        message.revision = record.revision
        message.severity = int(envelope.get("severity", LogEvent.INFO))
        message.event_type = str(envelope.get("event_type", "UNKNOWN"))
        message.source_node = NodeId.MASTER.value
        message.producer_instance_id = self.node_instance_id
        message.product_id = str(envelope.get("product_id", ""))
        message.payload_json = record.payload_json
        message.payload_digest = record.payload_digest
        message.occurred_at = message.header.stamp
        self._log_event_publisher.publish(message)

    def _handle_log_persisted_ack(self, message: LogPersistedAck) -> None:
        """LogNode commit ACK와 일치하는 Master spool 항목만 제거합니다."""

        if message.header.session_id != self.session_id:
            return
        if message.producer_node != NodeId.MASTER.value:
            return
        if message.producer_instance_id != self.node_instance_id:
            return
        if len(message.acked_log_ids) != len(message.acked_revisions):
            self.get_logger().error("LogPersistedAck identity arrays have other lengths")
            return
        if self._log_spool is None:
            return
        identities = [
            (log_id, int(revision))
            for log_id, revision in zip(
                message.acked_log_ids, message.acked_revisions
            )
            if log_id and int(revision) > 0
        ]
        if identities:
            self._log_spool.acknowledge(identities)

    def _flush_log_spool(self) -> None:
        """미ACK 로그를 조회해 순서대로 재발행합니다."""

        if self._log_spool is None:
            if not self.log_spool_path:
                return
            try:
                self._log_spool = DurableLogSpool(Path(self.log_spool_path))
            except Exception as exc:
                if not self._log_spool_open_failure_reported:
                    self._log_spool_open_failure_reported = True
                    self.get_logger().error(
                        "Master log spool reopen failed; retrying: "
                        f"{type(exc).__name__}"
                    )
                return
            self._log_spool_open_failure_reported = False
            self.get_logger().info("Master log spool recovered")
            if (
                self.system_state
                in {SystemState.READY, SystemState.RUN_SYS, SystemState.PAUSED}
                and self._all_workers_ready_for_session()
            ):
                self.set_health_state(NodeHealthState.READY)
            elif self.system_state == SystemState.INITIALIZING:
                self._maybe_complete_initialization()
        try:
            pending = self._log_spool.pending(limit=100)
        except Exception as exc:
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().error(
                f"Master log spool read failed: {type(exc).__name__}"
            )
            return
        for record in pending:
            self._publish_spool_record(record)
        self._check_log_spool_capacity()

    def _check_log_spool_capacity(self) -> None:
        if not self.log_spool_path:
            return
        # SQLite WAL mode에서는 본 파일보다 ``-wal``에 미ACK 데이터가
        # 먼저 쌓일 수 있으므로 관련 파일의 실제 디스크 사용량을 합산합니다.
        paths = (
            Path(self.log_spool_path),
            Path(f"{self.log_spool_path}-wal"),
            Path(f"{self.log_spool_path}-shm"),
        )
        size = 0
        for path in paths:
            try:
                size += path.stat().st_size
            except OSError:
                continue
        if self.log_spool_warning_bytes and size >= self.log_spool_warning_bytes:
            self.set_health_state(NodeHealthState.DEGRADED)
            self.get_logger().warning(
                f"Master log spool warning threshold reached: {size} bytes"
            )
        if (
            self.log_spool_hard_bytes
            and size >= self.log_spool_hard_bytes
            and self.system_state == SystemState.RUN_SYS
        ):
            self.request_recoverable_device_pause(
                "Master log spool hard threshold reached",
                pause_reason=PauseReason.STORAGE_RECOVERY,
            )

    def _begin_shutdown_sequence(self) -> None:
        """Ctrl+C 종료 전에 안전 정지와 미ACK 로그 보존을 시도합니다."""

        if self.shutdown_phase != ShutdownPhase.IDLE:
            return
        if self.system_state == SystemState.RUN_SYS:
            self.shutdown_phase = ShutdownPhase.WAITING_STOP
            self._shutdown_deadline_ns = (
                time.monotonic_ns() + self.shutdown_stop_timeout_ms * 1_000_000
            )
            if not self.request_pause("shutdown safe stop"):
                self.shutdown_phase = ShutdownPhase.BLOCKED
            return
        if self.system_state == SystemState.PAUSING:
            self.shutdown_phase = ShutdownPhase.WAITING_STOP
            self._shutdown_deadline_ns = (
                time.monotonic_ns() + self.shutdown_stop_timeout_ms * 1_000_000
            )
            self._request_all_conveyors_stop("shutdown awaiting safe stop")
            return
        if not self.equipment.all_conveyors_stopped():
            self.shutdown_phase = ShutdownPhase.WAITING_STOP
            self._shutdown_deadline_ns = (
                time.monotonic_ns() + self.shutdown_stop_timeout_ms * 1_000_000
            )
            self._request_all_conveyors_stop("shutdown stop confirmation")
            return
        self._finalize_shutdown_sequence()

    def _finalize_shutdown_sequence(self) -> None:
        if self.shutdown_phase == ShutdownPhase.READY_TO_EXIT:
            return
        self.shutdown_phase = ShutdownPhase.FINALIZING
        self._shutdown_deadline_ns = 0
        self._cancel_tracked_operation_goals(include_actuation=True)
        self._emit_log_event(
            severity=LogEvent.INFO,
            event_type="SHUTDOWN_READY",
            payload={
                "system_state": self.system_state.name,
                "active_fifo_size": self.ledger.active_size,
            },
        )
        self._flush_log_spool()
        self.shutdown_phase = ShutdownPhase.READY_TO_EXIT
        self.get_logger().info(
            "safe stop confirmed and local spool preserved; process may exit"
        )

    @property
    def shutdown_ready(self) -> bool:
        return self.shutdown_phase == ShutdownPhase.READY_TO_EXIT

    def _check_operation_deadlines(self) -> None:
        now_ns = time.monotonic_ns()
        if now_ns >= self._next_context_prune_ns:
            cutoff_ns = (
                now_ns - self.completed_context_retention_ms * 1_000_000
            )
            with self._flow_lock:
                pruned = self.ledger.prune_removed(cutoff_ns=cutoff_ns)
            if pruned:
                self.get_logger().info(
                    f"pruned {len(pruned)} completed product contexts"
                )
            self._next_context_prune_ns = now_ns + 60_000_000_000
        if (
            self._pause_deadline_ns
            and now_ns > self._pause_deadline_ns
            and self.system_state == SystemState.PAUSING
        ):
            self._pause_deadline_ns = 0
            self.report_pause_failed(
                "conveyor stop confirmation timed out", timed_out=True
            )
        if (
            self._shutdown_deadline_ns
            and now_ns > self._shutdown_deadline_ns
            and self.shutdown_phase == ShutdownPhase.WAITING_STOP
        ):
            self._shutdown_deadline_ns = 0
            self.shutdown_phase = ShutdownPhase.BLOCKED
            self.report_critical_fault(
                "shutdown blocked because safe stop was not confirmed",
                recovery_policy=RecoveryPolicy.EQUIPMENT_CHECK_REQUIRED,
            )
            self.get_logger().critical(
                "shutdown remains BLOCKED: verify Control/Mega communication and "
                "physical conveyor stop; a second Ctrl+C forces an unsafe exit"
            )
        if self.system_state == SystemState.FAULT_STOP:
            # FAULT_STOP 진입 시점의 제품·FIFO·cycle을 복구 근거로 보존합니다.
            # 이미 도착한 개별 callback도 동일한 원칙으로 진입부에서 무시합니다.
            return
        for station_id, cycle in tuple(self._station_cycles.items()):
            if not cycle.deadline_ns or now_ns <= cycle.deadline_ns:
                continue
            cycle.deadline_ns = 0
            if cycle.phase in {
                StationCyclePhase.CAPTURE_GOAL,
                StationCyclePhase.WAITING_CAPTURE_RESULT,
            }:
                if self._paused_by_queue:
                    continue
                if cycle.capture_goal_handle is not None:
                    try:
                        cycle.capture_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._finish_capture_failure(
                    station_id, "CaptureProduct timed out"
                )
            elif cycle.phase == StationCyclePhase.POSITION_GOAL:
                if cycle.position_goal_handle is not None:
                    try:
                        cycle.position_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._pause_station_for_recovery(
                    station_id,
                    "PositionProduct Goal response timed out before command acceptance",
                )
            elif cycle.phase == StationCyclePhase.WAITING_POSITION:
                if cycle.position_goal_handle is not None:
                    try:
                        cycle.position_goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                self._fault_stop(
                    "PositionProduct timed out after Goal acceptance; physical "
                    "position is unknown"
                )
            elif cycle.phase == StationCyclePhase.RESUME_PENDING:
                self._deferred_capture_resumes.add(
                    (cycle.product_id, station_id)
                )
                self.request_recoverable_device_pause(
                    f"{cycle.conveyor_id.name} conveyor RUN confirmation timed out",
                    pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
                )
        for cycle in tuple(self._actuation_cycles.values()):
            if not cycle.deadline_ns or now_ns <= cycle.deadline_ns:
                continue
            cycle.deadline_ns = 0
            if cycle.phase == ActuationPhase.WAITING_RESULT:
                self._fault_stop("ActuateProduct timed out after Goal acceptance")
            elif cycle.phase == ActuationPhase.GOAL_PENDING:
                self.request_recoverable_device_pause(
                    "ActuateProduct Goal response timed out",
                    pause_reason=PauseReason.DEVICE_RECOVERY_MANUAL,
                )

    def destroy_node(self) -> None:
        if self._log_spool is not None:
            self._log_spool.close()
            self._log_spool = None
        super().destroy_node()

    # endregion


def main(args: list[str] | None = None) -> None:
    # OS 기본 SIGINT handler가 ROS context를 먼저 닫지 않게 해야
    # ``spin_node``가 Ctrl+C 후에도 정지 명령과 확인 callback을 처리할 수 있습니다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    spin_node(MasterNode())


if __name__ == "__main__":
    main()
