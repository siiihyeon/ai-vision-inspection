"""네 노드가 공유하는 최소 ROS 2 통신·실행 골격입니다.

Heartbeat, 짧은 상태조회, 작업 노드 초기화 Action만 공통화합니다.
각 노드의 상태머신과 장비·업무 로직은 이 모듈에 넣지 않습니다.
"""

from __future__ import annotations

import json
import time
from collections.abc import Sequence

import rclpy
from inspection_interfaces.action import InitializeNode
from inspection_interfaces.msg import MasterHeartbeat, NodeHeartbeat
from inspection_interfaces.srv import GetNodeStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .constants import ErrorCode, NODE_EXECUTABLE_NAME, NodeHealthState, NodeId
from .package_version import read_installed_package_version


def heartbeat_qos() -> QoSProfile:
    """최신 생존신호 하나만 보존하는 Heartbeat QoS를 만듭니다."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class InspectionNodeBase(Node):
    """Heartbeat와 상태조회 계약을 구현하는 공통 Node 기반 클래스입니다."""

    def __init__(self, node_id: NodeId, *, provides_initialize_action: bool) -> None:
        super().__init__(NODE_EXECUTABLE_NAME[node_id])
        self.node_id = node_id
        self.interface_version = read_installed_package_version("inspection_interfaces")
        self.software_version = read_installed_package_version(
            f"inspection_{node_id.value}"
        )
        self.declare_parameter("profile", "sim")
        if node_id == NodeId.MASTER:
            self.declare_parameter(
                "system.expected_interface_version",
                self.interface_version,
            )
        heartbeat_key = (
            "comm.master_heartbeat_period_ms"
            if node_id == NodeId.MASTER
            else "comm.node_heartbeat_period_ms"
        )
        self.declare_parameter(heartbeat_key, 500)

        self.profile = str(self.get_parameter("profile").value)
        self.expected_interface_version = (
            str(self.get_parameter("system.expected_interface_version").value)
            if node_id == NodeId.MASTER
            else ""
        )
        self.health_state = NodeHealthState.STARTING
        self.session_id = ""
        self.config_version = ""
        self.config_digest = ""
        self.command_epoch = 0
        self.system_state = "BOOT"
        self._heartbeat_sequence = 0
        self.last_master_heartbeat_monotonic_ns: int | None = None
        self._initialize_requests: dict[str, tuple[tuple[str, ...], dict[str, object]]] = {}

        heartbeat_period_ms = int(self.get_parameter(heartbeat_key).value)
        if not 100 <= heartbeat_period_ms <= 5000:
            raise ValueError(f"{heartbeat_key} must be between 100 and 5000 ms")

        self._heartbeat_callback_group = MutuallyExclusiveCallbackGroup()
        self._service_callback_group = ReentrantCallbackGroup()
        self._action_callback_group = ReentrantCallbackGroup()

        endpoint = node_id.value
        if node_id == NodeId.MASTER:
            self._heartbeat_publisher = self.create_publisher(
                MasterHeartbeat,
                f"{endpoint}/heartbeat",
                heartbeat_qos(),
            )
        else:
            self._heartbeat_publisher = self.create_publisher(
                NodeHeartbeat,
                f"{endpoint}/heartbeat",
                heartbeat_qos(),
            )
        self._status_service = self.create_service(
            GetNodeStatus,
            f"{endpoint}/get_status",
            self._handle_get_status,
            callback_group=self._service_callback_group,
        )
        self._heartbeat_timer = self.create_timer(
            heartbeat_period_ms / 1000.0,
            self._publish_heartbeat,
            callback_group=self._heartbeat_callback_group,
        )

        self._master_heartbeat_subscription = None
        if node_id != NodeId.MASTER:
            self._master_heartbeat_subscription = self.create_subscription(
                MasterHeartbeat,
                "/inspection/master/heartbeat",
                self._handle_master_heartbeat,
                heartbeat_qos(),
                callback_group=self._heartbeat_callback_group,
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
                callback_group=self._action_callback_group,
            )

    def set_health_state(self, state: NodeHealthState) -> None:
        """노드가 소유한 health 상태를 변경합니다."""

        self.health_state = state

    def required_hardware_parameters(self) -> Sequence[str]:
        """각 작업 노드가 실제 장비 구현 단계에서 필수 키를 반환합니다."""

        return ()

    def validate_hardware_profile(self) -> list[str]:
        """hardware 프로필의 누락 설정을 반환하며 미구현 검증은 차단합니다."""

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
            if value is None or value == "":
                missing.append(key)
        return missing

    def _publish_heartbeat(self) -> None:
        self._heartbeat_sequence += 1
        if self.node_id == NodeId.MASTER:
            message = MasterHeartbeat()
            message.command_epoch = self.command_epoch
            message.system_state = self.system_state
        else:
            message = NodeHeartbeat()
            message.node_id = self.node_id.value
            message.health_state = self.health_state.value
            message.interface_version = self.interface_version
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = (
            f"{self.node_id.value}-heartbeat-{self._heartbeat_sequence}"
        )
        message.header.correlation_id = ""
        message.sequence = self._heartbeat_sequence
        self._heartbeat_publisher.publish(message)

    def _handle_master_heartbeat(self, message: MasterHeartbeat) -> None:
        """작업 노드가 Master 생존신호를 받은 monotonic 시각을 보존합니다."""

        del message
        self.last_master_heartbeat_monotonic_ns = time.monotonic_ns()

    def _handle_get_status(
        self,
        request: GetNodeStatus.Request,
        response: GetNodeStatus.Response,
    ) -> GetNodeStatus.Response:
        session_matches = bool(request.session_id) and request.session_id == self.session_id
        response.ready = self.health_state == NodeHealthState.READY and session_matches
        response.node_id = self.node_id.value
        response.health_state = self.health_state.value
        response.interface_version = self.interface_version
        response.status_json = json.dumps(
            {
                "schema_version": 1,
                "profile": self.profile,
                "heartbeat_sequence": self._heartbeat_sequence,
                "master_heartbeat_seen": (
                    self.last_master_heartbeat_monotonic_ns is not None
                ),
                "session_matches": session_matches,
            },
            ensure_ascii=False,
        )
        return response

    def _handle_initialize_goal(self, goal_request) -> GoalResponse:
        """형식상 유효한 요청만 실행 단계로 넘깁니다."""

        if not goal_request.request_id or not goal_request.session_id:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_initialize_cancel(self, _goal_handle) -> CancelResponse:
        """초기화 취소 요청을 수락하며 실제 장비 중단은 노드 구현이 담당합니다."""

        return CancelResponse.ACCEPT

    def _status_snapshot(self, **extra: object) -> str:
        snapshot: dict[str, object] = {
            "schema_version": 1,
            "profile": self.profile,
            "health_state": self.health_state.value,
        }
        snapshot.update(extra)
        return json.dumps(snapshot, ensure_ascii=False)

    def _fill_initialize_result(
        self,
        result: InitializeNode.Result,
        values: dict[str, object],
    ) -> None:
        result.success = bool(values["success"])
        result.node_id = self.node_id.value
        result.error_code = str(values["error_code"])
        result.reason = str(values["reason"])
        result.interface_version = self.interface_version
        result.software_version = self.software_version
        result.status_snapshot = str(values["status_snapshot"])
        result.retryable = bool(values["retryable"])

    async def _execute_initialize(self, goal_handle) -> InitializeNode.Result:
        """통신 시험용 sim 초기화와 fail-safe hardware 차단을 수행합니다."""

        request = goal_handle.request
        signature = (
            request.session_id,
            request.config_version,
            request.config_digest,
            request.expected_interface_version,
        )
        cached = self._initialize_requests.get(request.request_id)
        result = InitializeNode.Result()
        if cached is not None:
            cached_signature, cached_values = cached
            if cached_signature != signature:
                values = {
                    "success": False,
                    "error_code": ErrorCode.COMMAND_CONFLICT.value,
                    "reason": "same request_id received with different payload",
                    "status_snapshot": self._status_snapshot(),
                    "retryable": False,
                }
                self._fill_initialize_result(result, values)
                goal_handle.abort()
                return result
            self._fill_initialize_result(result, cached_values)
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result

        feedback = InitializeNode.Feedback()
        feedback.stage = "VALIDATING"
        feedback.attempt = 1
        goal_handle.publish_feedback(feedback)

        if goal_handle.is_cancel_requested:
            values = {
                "success": False,
                "error_code": ErrorCode.NODE_INIT_FAILED.value,
                "reason": "initialization canceled",
                "status_snapshot": self._status_snapshot(),
                "retryable": True,
            }
            self._fill_initialize_result(result, values)
            goal_handle.canceled()
            return result

        if request.expected_interface_version != self.interface_version:
            values = {
                "success": False,
                "error_code": ErrorCode.INTERFACE_VERSION_MISMATCH.value,
                "reason": (
                    f"expected={request.expected_interface_version}, "
                    f"actual={self.interface_version}"
                ),
                "status_snapshot": self._status_snapshot(),
                "retryable": False,
            }
        elif not request.config_version or not request.config_digest:
            values = {
                "success": False,
                "error_code": ErrorCode.NODE_INIT_FAILED.value,
                "reason": "config_version and config_digest are required",
                "status_snapshot": self._status_snapshot(),
                "retryable": True,
            }
        else:
            missing_keys = self.validate_hardware_profile()
            if missing_keys:
                self.set_health_state(NodeHealthState.INIT_BLOCKED)
                values = {
                    "success": False,
                    "error_code": ErrorCode.NODE_INIT_FAILED.value,
                    "reason": "missing hardware configuration: " + ", ".join(missing_keys),
                    "status_snapshot": self._status_snapshot(
                        missing_hardware_keys=missing_keys
                    ),
                    "retryable": True,
                }
            else:
                self.session_id = request.session_id
                self.config_version = request.config_version
                self.config_digest = request.config_digest
                self.set_health_state(NodeHealthState.READY)
                feedback.stage = "READY"
                goal_handle.publish_feedback(feedback)
                values = {
                    "success": True,
                    "error_code": "",
                    "reason": "sim skeleton initialization completed",
                    "status_snapshot": self._status_snapshot(),
                    "retryable": False,
                }

        self._initialize_requests[request.request_id] = (signature, values)
        self._fill_initialize_result(result, values)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def spin_node(node: InspectionNodeBase) -> None:
    """공통 executor와 안전한 종료 순서로 노드를 실행합니다."""

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(f"{node.get_name()} shutdown requested")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
