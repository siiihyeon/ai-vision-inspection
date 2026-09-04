"""네 노드가 공유하는 ROS 2 생존·상태·초기화 계약."""

from __future__ import annotations

import json
import os
import platform
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass, field

import rclpy
from inspection_interfaces.action import InitializeNode
from inspection_interfaces.msg import (
    MasterHeartbeat,
    NodeHeartbeat,
    NodeRuntimeEnvironment,
    SystemCommand,
)
from inspection_interfaces.srv import GetNodeStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .constants import (
    ErrorCode,
    NODE_EXECUTABLE_NAME,
    NodeHealthState,
    NodeId,
    SystemState,
)
from .digest import is_sha256_hex
from .identifiers import is_uuid4, new_uuid
from .idempotency import IdempotencyStore, ReplayKind
from .package_version import read_installed_package_version


def heartbeat_qos() -> QoSProfile:
    """Heartbeat: Best Effort, volatile, keep-last 1."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def reliable_event_qos(depth: int = 100) -> QoSProfile:
    """업무 이벤트 기본 QoS. 유실 복구는 애플리케이션 보존 정책과 병행합니다."""

    if depth < 1:
        raise ValueError("depth must be positive")
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def state_qos(depth: int = 1) -> QoSProfile:
    """최신 상태를 재구독자에게 전달하는 QoS.

    depth=1(기본값)은 발행자 대부분에 맞는 "최신 스냅샷 한 건"이지만,
    한 발행 주기 안에서 상태가 연달아 여러 번 바뀔 수 있는 구독자는 더 큰
    depth를 지정해야 중간 전이가 유실되지 않습니다.
    """

    if depth < 1:
        raise ValueError("depth must be positive")
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


@dataclass(frozen=True, slots=True)
class NodeInitializationOutcome:
    success: bool
    reason: str
    retryable: bool = False
    error_code: int = int(ErrorCode.NONE)
    status_details: dict[str, object] = field(default_factory=dict)


class InspectionNodeBase(Node):
    """Heartbeat, 상태조회, 직렬화된 초기화 Action을 구현합니다."""

    def __init__(self, node_id: NodeId, *, provides_initialize_action: bool) -> None:
        super().__init__(NODE_EXECUTABLE_NAME[node_id])
        self.node_id = node_id
        self.node_instance_id = new_uuid()
        self.started_monotonic_ns = time.monotonic_ns()
        self.interface_version = read_installed_package_version("inspection_interfaces")
        self.software_version = read_installed_package_version(f"inspection_{node_id.value}")

        self.declare_parameter("profile", "sim")
        self.declare_parameter("comm.master_heartbeat_timeout_ms", 2000)
        if node_id == NodeId.MASTER:
            self.declare_parameter("system.expected_interface_version", self.interface_version)
        heartbeat_key = (
            "comm.master_heartbeat_period_ms"
            if node_id == NodeId.MASTER
            else "comm.node_heartbeat_period_ms"
        )
        self.declare_parameter(heartbeat_key, 500)

        self.profile = str(self.get_parameter("profile").value)
        if self.profile not in {"sim", "hardware"}:
            raise ValueError("profile must be exactly 'sim' or 'hardware'")
        self.expected_interface_version = (
            str(self.get_parameter("system.expected_interface_version").value)
            if node_id == NodeId.MASTER
            else self.interface_version
        )
        heartbeat_period_ms = int(self.get_parameter(heartbeat_key).value)
        self.master_heartbeat_timeout_ms = int(
            self.get_parameter("comm.master_heartbeat_timeout_ms").value
        )
        if not 100 <= heartbeat_period_ms <= 5000:
            raise ValueError(f"{heartbeat_key} must be between 100 and 5000 ms")
        if self.master_heartbeat_timeout_ms < heartbeat_period_ms * 2:
            raise ValueError("master heartbeat timeout must be at least two periods")

        self.health_state = NodeHealthState.STARTING
        self.session_id = ""
        self.config_version = ""
        self.config_digest = ""
        self.command_epoch = 0
        self.system_state = SystemState.BOOT
        self._heartbeat_sequence = 0
        self.last_master_heartbeat_monotonic_ns: int | None = None
        self.last_master_instance_id = ""
        self._initialize_requests: dict[
            str, tuple[tuple[str, ...], dict[str, object]]
        ] = {}
        self._initialize_lock = threading.RLock()
        self._initialize_in_progress = False
        self._seen_system_commands: IdempotencyStore[bool] = IdempotencyStore(
            capacity=4096
        )

        self._heartbeat_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._action_group = MutuallyExclusiveCallbackGroup()
        endpoint = node_id.value

        heartbeat_type = MasterHeartbeat if node_id == NodeId.MASTER else NodeHeartbeat
        self._heartbeat_publisher = self.create_publisher(
            heartbeat_type, f"{endpoint}/heartbeat", heartbeat_qos()
        )
        self._status_service = self.create_service(
            GetNodeStatus,
            f"{endpoint}/get_status",
            self._handle_get_status,
            callback_group=self._service_group,
        )
        self._heartbeat_timer = self.create_timer(
            heartbeat_period_ms / 1000.0,
            self._publish_heartbeat,
            callback_group=self._heartbeat_group,
        )
        self._master_watchdog_timer = None
        self._master_heartbeat_subscription = None
        self._system_command_subscription = None
        if node_id != NodeId.MASTER:
            self._master_heartbeat_subscription = self.create_subscription(
                MasterHeartbeat,
                "/inspection/master/heartbeat",
                self._handle_master_heartbeat,
                heartbeat_qos(),
                callback_group=self._heartbeat_group,
            )
            self._master_watchdog_timer = self.create_timer(
                heartbeat_period_ms / 1000.0,
                self._check_master_heartbeat,
                callback_group=self._heartbeat_group,
            )
            self._system_command_subscription = self.create_subscription(
                SystemCommand,
                "/inspection/master/system_command",
                self._handle_system_command,
                reliable_event_qos(),
                callback_group=self._heartbeat_group,
            )

        self._initialize_action_server: ActionServer | None = None
        if provides_initialize_action:
            self._initialize_action_server = ActionServer(
                self,
                InitializeNode,
                f"{endpoint}/initialize",
                execute_callback=self._execute_initialize,
                goal_callback=self._handle_initialize_goal,
                cancel_callback=self._handle_initialize_cancel,
                callback_group=self._action_group,
            )

    def set_health_state(self, state: NodeHealthState) -> None:
        self.health_state = state

    def validate_command_header(
        self,
        command,
        *,
        allowed_system_states: set[SystemState] | None = None,
    ) -> tuple[bool, ErrorCode, str]:
        """공통 멱등 저장소에 넣기 전 명령 envelope를 fail-closed 검증합니다."""

        if self.health_state != NodeHealthState.READY:
            return False, ErrorCode.NODE_INIT_FAILED, "node is not READY"
        if command.header.session_id != self.session_id:
            return False, ErrorCode.COMMAND_CONFLICT, "session_id mismatch"
        if command.command_epoch != self.command_epoch:
            return False, ErrorCode.COMMAND_CONFLICT, "command_epoch mismatch"
        if not is_uuid4(command.command_id):
            return False, ErrorCode.COMMAND_CONFLICT, "command_id must be UUIDv4"
        if not is_sha256_hex(command.payload_digest):
            return False, ErrorCode.COMMAND_CONFLICT, "payload_digest must be SHA-256"
        allowed = allowed_system_states or {SystemState.RUN_SYS}
        if self.system_state not in allowed:
            return (
                False,
                ErrorCode.COMMAND_CONFLICT,
                f"equipment command is not allowed in {self.system_state.name}",
            )
        return True, ErrorCode.NONE, ""

    def required_hardware_parameters(self) -> Sequence[str]:
        return ()

    def validate_hardware_profile(self) -> list[str]:
        if self.profile != "hardware":
            return []
        required_keys = tuple(self.required_hardware_parameters())
        if not required_keys:
            return [f"{self.node_id.value}.hardware_validation_not_implemented"]
        missing: list[str] = []
        for key in required_keys:
            if not self.has_parameter(key):
                missing.append(key)
                continue
            value = self.get_parameter(key).value
            if value is None or value == "" or value == []:
                missing.append(key)
        return missing

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        """자식 노드의 SDK/모델/저장소 초기화 확장점."""

        if self.profile == "hardware":
            return NodeInitializationOutcome(
                success=False,
                error_code=int(ErrorCode.NODE_INIT_FAILED),
                reason=f"{self.node_id.value} hardware adapter is not implemented",
                retryable=True,
            )
        return NodeInitializationOutcome(
            success=True,
            reason="sim skeleton resource initialization completed",
        )

    def on_initialization_succeeded(
        self, previous_session_id: str, new_session_id: str
    ) -> None:
        """READY 전환 직후 자식 노드가 session 전용 메모리를 정리하는 hook."""

        del previous_session_id, new_session_id

    def _publish_heartbeat(self) -> None:
        self._heartbeat_sequence += 1
        message = MasterHeartbeat() if self.node_id == NodeId.MASTER else NodeHeartbeat()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = f"{self.node_id.value}-heartbeat-{self._heartbeat_sequence}"
        message.header.correlation_id = ""
        message.sequence = self._heartbeat_sequence
        if self.node_id == NodeId.MASTER:
            message.master_instance_id = self.node_instance_id
            message.interface_version = self.interface_version
            message.command_epoch = self.command_epoch
            message.system_state = int(self.system_state)
        else:
            message.node_id = self.node_id.value
            message.node_instance_id = self.node_instance_id
            message.health_state = int(self.health_state)
            message.interface_version = self.interface_version
        self._heartbeat_publisher.publish(message)

    def _handle_master_heartbeat(self, message: MasterHeartbeat) -> None:
        if message.interface_version != self.interface_version:
            return
        if self.session_id and message.header.session_id != self.session_id:
            return
        if message.command_epoch < self.command_epoch:
            return
        self.command_epoch = message.command_epoch
        self.last_master_instance_id = message.master_instance_id
        try:
            self.system_state = SystemState(message.system_state)
        except ValueError:
            self.health_state = NodeHealthState.FAULT
            self.get_logger().error("Master heartbeat contains an invalid system_state")
            return
        self.last_master_heartbeat_monotonic_ns = time.monotonic_ns()

    def _handle_system_command(self, message: SystemCommand) -> None:
        command = message.command
        if command.header.session_id != self.session_id or not self.session_id:
            return
        if not is_uuid4(command.command_id) or not is_sha256_hex(command.payload_digest):
            return
        replay = self._seen_system_commands.inspect(
            command.command_id, command.payload_digest
        )
        if replay.kind == ReplayKind.CONFLICT:
            self.health_state = NodeHealthState.FAULT
            return
        if replay.kind == ReplayKind.REPLAY:
            return
        if command.command_epoch < self.command_epoch:
            return
        self._seen_system_commands.remember(
            command.command_id, command.payload_digest, True
        )
        self.command_epoch = command.command_epoch
        if int(message.target_conveyor_id) != int(SystemCommand.ALL_CONVEYORS):
            # Station 촬영 후 특정 컨베이어만 재가동하는 명령은
            # 전체 SystemState를 바꾸지 않습니다. ControlNode의 장비
            # adapter가 아래 확장점에서 실제 motor 명령을 수행합니다.
            self.handle_targeted_conveyor_command(message)
            return
        if message.command_type == SystemCommand.PAUSE:
            self.system_state = SystemState.PAUSING
        elif message.command_type == SystemCommand.RESUME:
            self.system_state = SystemState.RUN_SYS
        elif message.command_type == SystemCommand.RESET:
            self.system_state = SystemState.RESETTING
            self.health_state = NodeHealthState.RECOVERING
        self.handle_all_conveyors_command(message)

    def handle_targeted_conveyor_command(self, _message: SystemCommand) -> None:
        """특정 컨베이어 명령을 Control adapter가 구현할 확장점입니다."""

    def handle_all_conveyors_command(self, _message: SystemCommand) -> None:
        """전체 컨베이어 대상 명령을 Control adapter가 구현할 확장점입니다."""

    def _master_heartbeat_alive(self) -> bool:
        if self.node_id == NodeId.MASTER:
            return True
        if self.last_master_heartbeat_monotonic_ns is None:
            return False
        elapsed_ms = (
            time.monotonic_ns() - self.last_master_heartbeat_monotonic_ns
        ) / 1_000_000
        return elapsed_ms <= self.master_heartbeat_timeout_ms

    def _check_master_heartbeat(self) -> None:
        if self.health_state == NodeHealthState.READY and not self._master_heartbeat_alive():
            self.health_state = NodeHealthState.DEGRADED
            self.get_logger().error("Master heartbeat timed out; command execution must stop")

    def _handle_get_status(
        self, request: GetNodeStatus.Request, response: GetNodeStatus.Response
    ) -> GetNodeStatus.Response:
        session_matches = bool(request.session_id) and request.session_id == self.session_id
        response.ready = (
            self.health_state == NodeHealthState.READY
            and session_matches
            and self._master_heartbeat_alive()
        )
        response.node_id = self.node_id.value
        response.node_instance_id = self.node_instance_id
        response.health_state = int(self.health_state)
        response.interface_version = self.interface_version
        response.software_version = self.software_version
        response.active_session_id = self.session_id
        response.command_epoch = self.command_epoch
        response.heartbeat_sequence = self._heartbeat_sequence
        response.master_heartbeat_alive = self._master_heartbeat_alive()
        response.uptime_ms = (time.monotonic_ns() - self.started_monotonic_ns) // 1_000_000
        response.status_json = self._status_snapshot(session_matches=session_matches)
        return response

    def _handle_initialize_goal(self, goal_request) -> GoalResponse:
        if not is_uuid4(goal_request.request_id) or not is_uuid4(goal_request.session_id):
            return GoalResponse.REJECT
        if goal_request.retry_of_request_id and not is_uuid4(goal_request.retry_of_request_id):
            return GoalResponse.REJECT
        if goal_request.retry_of_request_id == goal_request.request_id:
            return GoalResponse.REJECT
        with self._initialize_lock:
            if self._initialize_in_progress:
                return GoalResponse.REJECT
            self._initialize_in_progress = True
        return GoalResponse.ACCEPT

    def _handle_initialize_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _status_snapshot(self, **extra: object) -> str:
        snapshot: dict[str, object] = {
            "schema_version": 2,
            "profile": self.profile,
            "health_state": int(self.health_state),
            "health_state_name": self.health_state.name,
            "system_state": int(self.system_state),
            "system_state_name": self.system_state.name,
            "node_instance_id": self.node_instance_id,
            "command_epoch": self.command_epoch,
        }
        snapshot.update(extra)
        return json.dumps(snapshot, ensure_ascii=False, sort_keys=True)

    def _runtime_environment(self) -> NodeRuntimeEnvironment:
        environment = NodeRuntimeEnvironment()
        environment.os_name = platform.system()
        environment.os_version = platform.version()
        environment.python_version = platform.python_version()
        environment.ros_distribution = os.environ.get("ROS_DISTRO", "unknown")
        environment.architecture = platform.machine()
        environment.sdk_versions = []
        environment.firmware_versions = []
        return environment

    def _fill_initialize_result(
        self,
        result: InitializeNode.Result,
        request_id: str,
        values: dict[str, object],
    ) -> None:
        result.success = bool(values["success"])
        result.node_id = self.node_id.value
        result.error_code = int(values["error_code"])
        result.reason = str(values["reason"])
        result.interface_version = self.interface_version
        result.software_version = self.software_version
        result.request_id = request_id
        result.active_session_id = self.session_id
        result.runtime_environment = self._runtime_environment()
        result.status_snapshot = str(values["status_snapshot"])
        result.retryable = bool(values["retryable"])

    async def _execute_initialize(self, goal_handle) -> InitializeNode.Result:
        request = goal_handle.request
        signature = (
            request.retry_of_request_id,
            request.session_id,
            request.config_version,
            request.config_digest,
            request.expected_interface_version,
        )
        result = InitializeNode.Result()
        try:
            with self._initialize_lock:
                cached = self._initialize_requests.get(request.request_id)
            if cached is not None:
                cached_signature, cached_values = cached
                if cached_signature != signature:
                    values = self._init_failure(
                        ErrorCode.COMMAND_CONFLICT,
                        "same request_id received with different payload",
                        retryable=False,
                    )
                    self._fill_initialize_result(result, request.request_id, values)
                    goal_handle.abort()
                    return result
                self._fill_initialize_result(result, request.request_id, cached_values)
                goal_handle.succeed() if result.success else goal_handle.abort()
                return result

            feedback = InitializeNode.Feedback()
            feedback.stage = "VALIDATING"
            feedback.attempt = 1
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                values = self._init_failure(
                    ErrorCode.NODE_INIT_FAILED, "initialization canceled", retryable=True
                )
                self._fill_initialize_result(result, request.request_id, values)
                goal_handle.canceled()
                return result
            if request.expected_interface_version != self.interface_version:
                values = self._init_failure(
                    ErrorCode.INTERFACE_VERSION_MISMATCH,
                    f"expected={request.expected_interface_version}, actual={self.interface_version}",
                    retryable=False,
                )
            elif not request.config_version or not is_sha256_hex(request.config_digest):
                values = self._init_failure(
                    ErrorCode.NODE_INIT_FAILED,
                    "config_version and lowercase SHA-256 config_digest are required",
                    retryable=True,
                )
            else:
                missing_keys = self.validate_hardware_profile()
                if missing_keys:
                    self.set_health_state(NodeHealthState.INIT_BLOCKED)
                    values = self._init_failure(
                        ErrorCode.NODE_INIT_FAILED,
                        "missing hardware configuration: " + ", ".join(missing_keys),
                        retryable=True,
                        missing_hardware_keys=missing_keys,
                    )
                else:
                    feedback.stage = "INITIALIZING_RESOURCES"
                    goal_handle.publish_feedback(feedback)
                    try:
                        outcome = await self.initialize_node_resources()
                    except Exception as exc:
                        self.get_logger().error(
                            f"resource initialization raised {type(exc).__name__}"
                        )
                        outcome = NodeInitializationOutcome(
                            success=False,
                            error_code=int(ErrorCode.NODE_INIT_FAILED),
                            reason=f"resource initialization raised {type(exc).__name__}",
                            retryable=True,
                        )
                    if not outcome.success:
                        self.set_health_state(NodeHealthState.INIT_BLOCKED)
                        values = self._init_failure(
                            outcome.error_code,
                            outcome.reason,
                            retryable=outcome.retryable,
                            resource_details=outcome.status_details,
                        )
                    else:
                        previous_session_id = self.session_id
                        previous_config_version = self.config_version
                        previous_config_digest = self.config_digest
                        self.session_id = request.session_id
                        self.config_version = request.config_version
                        self.config_digest = request.config_digest
                        try:
                            self.on_initialization_succeeded(
                                previous_session_id, self.session_id
                            )
                        except Exception as exc:
                            self.session_id = previous_session_id
                            self.config_version = previous_config_version
                            self.config_digest = previous_config_digest
                            self.get_logger().error(
                                "post-initialization hook raised "
                                f"{type(exc).__name__}"
                            )
                            self.set_health_state(NodeHealthState.INIT_BLOCKED)
                            values = self._init_failure(
                                ErrorCode.NODE_INIT_FAILED,
                                "post-initialization session cleanup failed",
                                retryable=True,
                            )
                            with self._initialize_lock:
                                self._initialize_requests[request.request_id] = (
                                    signature,
                                    values,
                                )
                            self._fill_initialize_result(
                                result, request.request_id, values
                            )
                            goal_handle.abort()
                            return result
                        self.set_health_state(NodeHealthState.READY)
                        self.system_state = SystemState.READY
                        feedback.stage = "READY"
                        goal_handle.publish_feedback(feedback)
                        values = {
                            "success": True,
                            "error_code": int(ErrorCode.NONE),
                            "reason": outcome.reason,
                            "status_snapshot": self._status_snapshot(
                                resource_details=outcome.status_details
                            ),
                            "retryable": False,
                        }

            with self._initialize_lock:
                self._initialize_requests[request.request_id] = (signature, values)
            self._fill_initialize_result(result, request.request_id, values)
            goal_handle.succeed() if result.success else goal_handle.abort()
            return result
        finally:
            with self._initialize_lock:
                self._initialize_in_progress = False

    def _init_failure(
        self,
        code: ErrorCode | int,
        reason: str,
        *,
        retryable: bool,
        **details: object,
    ) -> dict[str, object]:
        return {
            "success": False,
            "error_code": int(code),
            "reason": reason,
            "status_snapshot": self._status_snapshot(**details),
            "retryable": retryable,
        }


def spin_node(node: InspectionNodeBase) -> None:
    """공통 executor와 종료 순서."""

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(f"{node.get_name()} shutdown requested")
        request_shutdown = getattr(node, "request_shutdown", None)
        if callable(request_shutdown):
            # Master는 Ctrl+C에서도 즉시 destroy하지 않고 Control의
            # 실제 정지 확인과 로컬 spool 보존을 끝낸 뒤 종료합니다.
            request_shutdown("Ctrl+C")
            try:
                while rclpy.ok() and not bool(
                    getattr(node, "shutdown_ready", False)
                ):
                    executor.spin_once(timeout_sec=0.1)
            except KeyboardInterrupt:
                node.get_logger().critical(
                    "second Ctrl+C forced process exit before safe shutdown completed"
                )
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
