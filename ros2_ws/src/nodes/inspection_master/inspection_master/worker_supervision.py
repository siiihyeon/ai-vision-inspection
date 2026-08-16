"""작업 노드 초기화와 heartbeat 상태를 보관하는 순수 도메인 모델."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from inspection_common import NodeHealthState, NodeId


class WorkerInitPhase(StrEnum):
    """Master가 바라보는 작업 노드 초기화 진행 단계."""

    IDLE = "IDLE"
    WAITING_GOAL_RESPONSE = "WAITING_GOAL_RESPONSE"
    WAITING_RESULT = "WAITING_RESULT"
    WAITING_STATUS = "WAITING_STATUS"
    RETRY_WAIT = "RETRY_WAIT"
    READY = "READY"


@dataclass(frozen=True, slots=True)
class HeartbeatUpdate:
    """heartbeat 수락 여부와 프로세스 재시작 감지 결과."""

    accepted: bool
    restarted: bool
    reason: str = ""


@dataclass(slots=True)
class WorkerRuntimeState:
    """Control/Vision/Log 한 노드에 대한 Master의 최신 미러."""

    worker_id: NodeId
    phase: WorkerInitPhase = WorkerInitPhase.IDLE
    attempt: int = 0
    request_id: str = ""
    retry_of_request_id: str = ""
    deadline_ns: int = 0
    retry_due_ns: int = 0
    status_request_id: str = ""
    status_poll_due_ns: int = 0
    action_ready: bool = False
    status_verified: bool = False
    ready: bool = False
    status_node_instance_id: str = ""
    heartbeat_node_instance_id: str = ""
    interface_version: str = ""
    software_version: str = ""
    active_session_id: str = ""
    command_epoch: int = 0
    status_heartbeat_sequence: int = 0
    status_health_state: NodeHealthState = NodeHealthState.STARTING
    heartbeat_health_state: NodeHealthState = NodeHealthState.STARTING
    status_ready: bool = False
    status_master_heartbeat_alive: bool = False
    heartbeat_sequence: int = 0
    heartbeat_received_ns: int = 0
    heartbeat_session_id: str = ""
    last_error_code: int = 0
    last_reason: str = ""
    manual_intervention_required: bool = False

    def begin_attempt(
        self,
        *,
        request_id: str,
        attempt: int,
        now_ns: int,
        timeout_ns: int,
    ) -> None:
        """새 request ID로 초기화 Action 1회를 시작합니다."""

        previous_request_id = self.request_id
        self.phase = WorkerInitPhase.WAITING_GOAL_RESPONSE
        self.attempt = attempt
        self.retry_of_request_id = previous_request_id
        self.request_id = request_id
        self.deadline_ns = now_ns + timeout_ns
        self.retry_due_ns = 0
        self.status_request_id = ""
        self.status_poll_due_ns = 0
        self.action_ready = False
        self.status_verified = False
        self.ready = False
        self.last_error_code = 0
        self.last_reason = ""
        self.manual_intervention_required = False

    def mark_goal_accepted(self) -> None:
        self.phase = WorkerInitPhase.WAITING_RESULT

    def mark_action_ready(
        self,
        *,
        interface_version: str,
        software_version: str,
        active_session_id: str,
        reason: str,
        now_ns: int,
        timeout_ns: int,
    ) -> None:
        self.phase = WorkerInitPhase.WAITING_STATUS
        self.action_ready = True
        self.interface_version = interface_version
        self.software_version = software_version
        self.active_session_id = active_session_id
        self.last_reason = reason
        self.deadline_ns = now_ns + timeout_ns

    def begin_status_check(
        self,
        *,
        request_id: str,
    ) -> None:
        self.phase = WorkerInitPhase.WAITING_STATUS
        self.status_request_id = request_id
        self.status_poll_due_ns = 0

    def schedule_status_poll(self, *, now_ns: int, interval_ns: int) -> None:
        self.phase = WorkerInitPhase.WAITING_STATUS
        self.status_request_id = ""
        self.status_poll_due_ns = now_ns + interval_ns

    def record_status(
        self,
        *,
        node_instance_id: str,
        interface_version: str,
        software_version: str,
        active_session_id: str,
        command_epoch: int,
        heartbeat_sequence: int,
        health_state: NodeHealthState,
        ready: bool,
        master_heartbeat_alive: bool,
    ) -> None:
        self.status_verified = True
        self.status_node_instance_id = node_instance_id
        self.interface_version = interface_version
        self.software_version = software_version
        self.active_session_id = active_session_id
        self.command_epoch = command_epoch
        self.status_heartbeat_sequence = heartbeat_sequence
        self.status_health_state = health_state
        self.status_ready = ready
        self.status_master_heartbeat_alive = master_heartbeat_alive

    def schedule_retry(
        self,
        *,
        now_ns: int,
        interval_ns: int,
        error_code: int,
        reason: str,
        manual_intervention_required: bool,
    ) -> None:
        self.phase = WorkerInitPhase.RETRY_WAIT
        self.deadline_ns = 0
        self.retry_due_ns = now_ns + interval_ns
        self.status_request_id = ""
        self.status_poll_due_ns = 0
        self.action_ready = False
        self.status_verified = False
        self.ready = False
        self.last_error_code = error_code
        self.last_reason = reason
        self.manual_intervention_required = manual_intervention_required

    def accept_heartbeat(
        self,
        *,
        node_instance_id: str,
        sequence: int,
        health_state: NodeHealthState,
        interface_version: str,
        session_id: str,
        received_ns: int,
    ) -> HeartbeatUpdate:
        """latest-only 규칙으로 heartbeat를 수락하고 재시작을 감지합니다."""

        if not node_instance_id or sequence <= 0:
            return HeartbeatUpdate(False, False, "invalid identity or sequence")
        same_instance = self.heartbeat_node_instance_id == node_instance_id
        if same_instance and sequence <= self.heartbeat_sequence:
            return HeartbeatUpdate(False, False, "stale or duplicate sequence")

        restarted = bool(self.heartbeat_node_instance_id and not same_instance)
        if restarted:
            self.action_ready = False
            self.status_verified = False
            self.ready = False

        self.heartbeat_node_instance_id = node_instance_id
        self.heartbeat_sequence = sequence
        self.heartbeat_health_state = health_state
        self.interface_version = interface_version
        self.heartbeat_session_id = session_id
        self.heartbeat_received_ns = received_ns
        return HeartbeatUpdate(True, restarted)

    def heartbeat_is_fresh(self, *, now_ns: int, timeout_ns: int) -> bool:
        return (
            self.heartbeat_received_ns > 0
            and now_ns - self.heartbeat_received_ns <= timeout_ns
        )

    def can_mark_ready(
        self,
        *,
        session_id: str,
        expected_interface_version: str,
        command_epoch: int,
        now_ns: int,
        heartbeat_timeout_ns: int,
    ) -> bool:
        """Action·Status·heartbeat 세 근거가 모두 같은 노드를 가리키는지 확인합니다."""

        return (
            self.action_ready
            and self.status_verified
            and self.status_node_instance_id == self.heartbeat_node_instance_id
            and self.active_session_id == session_id
            and self.heartbeat_session_id == session_id
            and self.interface_version == expected_interface_version
            and self.command_epoch == command_epoch
            and self.heartbeat_sequence >= self.status_heartbeat_sequence
            and self.status_ready
            and self.status_master_heartbeat_alive
            and self.status_health_state == NodeHealthState.READY
            and self.heartbeat_health_state == NodeHealthState.READY
            and self.heartbeat_is_fresh(
                now_ns=now_ns,
                timeout_ns=heartbeat_timeout_ns,
            )
        )

    def mark_ready(self) -> None:
        self.phase = WorkerInitPhase.READY
        self.ready = True
        self.deadline_ns = 0
        self.retry_due_ns = 0
        self.status_poll_due_ns = 0
