"""ControlNode 통신 골격. Mega/TB6600 구현은 adapter 확장점에 격리합니다."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import threading
import time

import rclpy
from inspection_common import (
    ConveyorId,
    ErrorCode,
    IdempotencyStore,
    NodeHealthState,
    NodeId,
    SystemState,
    canonical_json,
    new_uuid,
    sha256_text,
)
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
    state_qos,
)
from inspection_interfaces.action import ActuateProduct
from inspection_interfaces.msg import (
    EquipmentState,
    LogEvent,
    PositionSettled,
    SensorEvent,
    SystemCommand,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future

from .mega_protocol import (
    Sensor3Telemetry,
    decode_frame_diagnostic,
    encode_frame,
    parse_event,
    parse_sensor3_telemetry,
)

try:
    import serial
except ImportError:  # Serial is required only for the hardware profile.
    serial = None


class ControlNode(InspectionNodeBase):
    """센서 입력과 위치/선별 명령 실행을 소유합니다."""

    def __init__(self) -> None:
        super().__init__(NodeId.CONTROL, provides_initialize_action=True)
        self.declare_parameter("control.mega.port", "")
        self.declare_parameter("control.mega.baud_rate", 0)
        self.declare_parameter("control.tb6600.upper_config", "")
        self.declare_parameter("control.tb6600.lower_config", "")
        self.declare_parameter("control.sensor_config", "")
        self.declare_parameter("control.actuator_config", "")
        self.declare_parameter("control.actuator.timeout_ms", 10000)
        self.declare_parameter("control.station_a.position_offset_steps", 0)
        self.declare_parameter("control.station_b.position_offset_steps", 0)

        self._actuation_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._equipment_action_group = MutuallyExclusiveCallbackGroup()
        self._sensor_publisher = self.create_publisher(
            SensorEvent, "control/sensor_event", reliable_event_qos()
        )
        self._position_settled_publisher = self.create_publisher(
            PositionSettled, "control/position_settled", reliable_event_qos()
        )
        self._equipment_state_publisher = self.create_publisher(
            EquipmentState, "control/equipment_state", state_qos()
        )
        self._log_event_publisher = self.create_publisher(
            LogEvent, "log/event", reliable_event_qos(depth=1000)
        )
        self._latest_equipment_state: tuple[int, int, bool, bool, bool, bool] | None = None
        self._mega = None
        self._mega_thread: threading.Thread | None = None
        self._mega_lock = threading.RLock()
        self._mega_sequence = 0
        self._mega_events: dict[str, tuple[bool, str]] = {}
        self._mega_event = threading.Condition(self._mega_lock)
        self._mega_diagnostic_counts: dict[str, int] = {
            "ascii_decode_error": 0,
            "missing_crc_field": 0,
            "invalid_crc_text": 0,
            "crc_mismatch": 0,
            "sensor3_format_rejected": 0,
            "sensor3_parsed": 0,
            "sensor3_publish_attempted": 0,
        }
        self._mega_last_diagnostic_report: dict[str, float] = {}
        self._blocking_pool = ThreadPoolExecutor(
            max_workers=2, thread_name_prefix="control-blocking"
        )
        self._mega_prune_timer = self.create_timer(1.0, self._prune_stale_mega_events)
        self._actuate_server = ActionServer(
            self,
            ActuateProduct,
            "control/actuate_product",
            execute_callback=self._execute_actuation,
            goal_callback=self._accept_equipment_goal,
            cancel_callback=self._cancel_equipment_goal,
            callback_group=self._equipment_action_group,
        )
        self.get_logger().info("ControlNode v2 communication skeleton started")

    def required_hardware_parameters(self) -> tuple[str, ...]:
        return (
            "control.mega.port",
            "control.mega.baud_rate",
            "control.tb6600.upper_config",
            "control.tb6600.lower_config",
            "control.sensor_config",
            "control.actuator_config",
        )

    def validate_hardware_profile(self) -> list[str]:
        missing = super().validate_hardware_profile()
        if self.profile == "hardware":
            if int(self.get_parameter("control.mega.baud_rate").value) <= 0:
                missing.append("control.mega.baud_rate")
            if int(self.get_parameter("control.station_a.position_offset_steps").value) <= 0:
                missing.append("control.station_a.position_offset_steps")
            if int(self.get_parameter("control.station_b.position_offset_steps").value) <= 0:
                missing.append("control.station_b.position_offset_steps")
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        if self.profile == "hardware":
            if serial is None:
                return NodeInitializationOutcome(False, "pyserial is not installed")
            try:
                if self._mega_thread is None or not self._mega_thread.is_alive():
                    self._mega = serial.Serial(
                        str(self.get_parameter("control.mega.port").value),
                        int(self.get_parameter("control.mega.baud_rate").value),
                        timeout=0.1,
                    )
                    self._mega_thread = threading.Thread(
                        target=self._read_mega, daemon=True
                    )
                    self._mega_thread.start()
                sequence = self._send_mega("HELLO", 2)
                status = await self._run_blocking(
                    self._wait_for_mega, f"ACK:{sequence}", 2.0
                )
                if status != "OK":
                    return NodeInitializationOutcome(
                        False, self._mega_wait_failure("Mega HELLO", status)
                    )
                offsets = {
                    ConveyorId.UPPER: int(
                        self.get_parameter(
                            "control.station_a.position_offset_steps"
                        ).value
                    ),
                    ConveyorId.LOWER: int(
                        self.get_parameter(
                            "control.station_b.position_offset_steps"
                        ).value
                    ),
                }
                for conveyor_id, steps in offsets.items():
                    sequence = self._send_mega("SET_OFFSET", int(conveyor_id), steps)
                    status = await self._run_blocking(
                        self._wait_for_mega, f"ACK:{sequence}", 2.0
                    )
                    if status != "OK":
                        return NodeInitializationOutcome(
                            False,
                            self._mega_wait_failure(
                                f"Mega SET_OFFSET ({conveyor_id.name})", status
                            ),
                        )
            except (OSError, RuntimeError) as exc:
                return NodeInitializationOutcome(False, f"Mega connection failed: {exc}", True)
            return NodeInitializationOutcome(True, "Mega handshake completed")
        return await super().initialize_node_resources()

    def publish_sensor_observation(self, message: SensorEvent) -> None:
        """향후 serial adapter가 debounced 센서 이벤트를 전달할 확장점."""

        self._sensor_publisher.publish(message)

    def on_initialization_succeeded(
        self, previous_session_id: str, new_session_id: str
    ) -> None:
        """세션이 확정된 뒤 Mega의 최신 안전 상태를 다시 발행합니다.

        Mega는 포트 연결 직후 상태를 한 번만 보낼 수 있다. 그 시점은 NodeBase가
        새 session_id를 적용하기 전이므로, 그대로 두면 Master가 session mismatch로
        버리고 START guard 값이 None으로 남는다.
        """

        del previous_session_id, new_session_id
        if self._latest_equipment_state is not None:
            self._publish_equipment_state(*self._latest_equipment_state)

    def _publish_position_settled(self, *, conveyor_id: int, estimated_step: int) -> None:
        """Mega가 자율로 이동·정지한 결과를 그대로 옮깁니다.

        Master가 이동을 명령하지 않으므로 product_id/station_id는 없습니다.
        conveyor_id만으로 Master가 어느 station cycle인지 매칭합니다.
        """

        settled = PositionSettled()
        settled.header.stamp = self.get_clock().now().to_msg()
        settled.header.session_id = self.session_id
        settled.header.message_id = new_uuid()
        settled.conveyor_id = conveyor_id
        settled.estimated_step = estimated_step
        settled.position_source = PositionSettled.OPEN_LOOP_ESTIMATE
        settled.position_verified = False
        settled.settled_at = settled.header.stamp
        self._position_settled_publisher.publish(settled)

    def handle_targeted_conveyor_command(self, message: SystemCommand) -> None:
        """Station 촬영 후 해당 층 컨베이어만 재가동하는 진입점."""

        try:
            conveyor_id = ConveyorId(message.target_conveyor_id)
        except ValueError:
            self.get_logger().error("targeted SystemCommand has invalid conveyor_id")
            return
        if int(message.command_type) != int(SystemCommand.RESUME):
            self.get_logger().error(
                "only RESUME is allowed for a targeted conveyor command"
            )
            return
        if self.profile == "sim":
            self.get_logger().info(
                f"sim conveyor {conveyor_id.name} resume command accepted"
            )
            return
        if self.profile == "hardware":
            try:
                self._send_mega("RUN", int(conveyor_id))
            except (RuntimeError, OSError) as exc:
                self.get_logger().error(str(exc))

    def handle_all_conveyors_command(self, message: SystemCommand) -> None:
        """전체 컨베이어 대상 PAUSE/RESUME을 Mega STOP/RUN으로 전달합니다."""

        if self.profile != "hardware":
            return
        operation = {
            SystemCommand.PAUSE: "STOP",
            SystemCommand.RESUME: "RUN",
        }.get(int(message.command_type))
        if operation is None:
            return
        for conveyor_id in (ConveyorId.UPPER, ConveyorId.LOWER):
            try:
                self._send_mega(operation, int(conveyor_id))
            except (RuntimeError, OSError) as exc:
                self.get_logger().error(str(exc))

    def _next_sequence(self) -> int:
        with self._mega_lock:
            self._mega_sequence = (self._mega_sequence + 1) & 0x7FFFFFFF
            return self._mega_sequence

    def _send_mega(self, operation: str, *values: object) -> int:
        if self._mega is None:
            raise RuntimeError("Mega is not connected")
        sequence = self._next_sequence()
        with self._mega_lock:
            self._mega.write(encode_frame("C", sequence, operation, *values))
        return sequence

    def _wait_for_mega(self, key: str, timeout: float) -> str | None:
        """수신한 status 문자열을 그대로 돌려주고, 타임아웃이면 None입니다.

        타임아웃(응답 없음)과 명시적 ERR(응답 왔지만 거부)을 호출부가
        구분할 수 있게, 성공 여부(bool)가 아니라 원본 status를 돌려줍니다.
        """

        deadline = time.monotonic() + timeout
        with self._mega_event:
            while time.monotonic() < deadline:
                if key in self._mega_events:
                    return self._mega_events.pop(key)[1]
                self._mega_event.wait(max(0.01, deadline - time.monotonic()))
        return None

    @staticmethod
    def _mega_wait_failure(prefix: str, status: str | None) -> str:
        return f"{prefix} timeout" if status is None else f"{prefix} rejected: {status}"

    def _prune_stale_mega_events(self) -> None:
        """수거되지 않은 mega 이벤트가 무한정 쌓이는 것을 막습니다.

        RUN 명령처럼 응답을 기다리지 않는 호출은 ACK를 아무도 pop하지
        않아 계속 쌓이므로, 오래된 항목을 조용히 정리합니다. 정상 흐름
        (RUN 명령마다 매번 발생)과 실제 이상 상황을 구분할 수 없어
        로그는 남기지 않습니다. position/actuator 타임아웃(기본 10초)
        보다 넉넉하게 잡아, 정상적으로 응답을 기다리는 중인 항목을
        먼저 지워버리지 않습니다.
        """

        cutoff = time.monotonic() - 15.0
        with self._mega_event:
            stale = [key for key, value in self._mega_events.items() if value[2] < cutoff]
            for key in stale:
                del self._mega_events[key]

    def _run_blocking(self, fn, *args) -> Future:
        """블로킹 호출을 스레드 풀에 넘기고 rclpy Future로 결과를 받습니다.

        rclpy executor에는 asyncio 이벤트 루프가 없어 asyncio.to_thread를
        쓸 수 없습니다. rclpy Future는 executor가 직접 깨우므로 await 시
        executor 스레드가 정상 반납됩니다.
        """

        rclpy_future = Future()
        pool_future = self._blocking_pool.submit(fn, *args)

        def _relay(done_future) -> None:
            try:
                rclpy_future.set_result(done_future.result())
            except Exception as exc:  # 호출부 await에서 다시 발생시킵니다.
                rclpy_future.set_exception(exc)

        pool_future.add_done_callback(_relay)
        return rclpy_future

    def destroy_node(self) -> bool:
        """종료 시 블로킹 스레드 풀을 정리합니다."""

        self._blocking_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _read_mega(self) -> None:
        while self._mega is not None:
            try:
                raw_line = self._mega.readline()
            except OSError as exc:
                self.get_logger().error(f"Mega serial disconnected: {exc}")
                with self._mega_lock:
                    mega, self._mega = self._mega, None
                try:
                    mega.close()
                except OSError:
                    pass
                self.set_health_state(NodeHealthState.DEGRADED)
                return
            decoded = decode_frame_diagnostic(raw_line)
            fields = decoded.fields
            if fields is None:
                if decoded.rejection_reason != "empty_read":
                    self._record_mega_frame_rejection(
                        decoded.rejection_reason or "unknown_decode_error",
                        raw_line,
                    )
                continue
            if fields[0] == "A" and len(fields) >= 3:
                with self._mega_event:
                    self._mega_events[f"ACK:{fields[1]}"] = (
                        fields[2] == "OK", fields[2], time.monotonic(),
                    )
                    self._mega_event.notify_all()
                continue
            telemetry = parse_sensor3_telemetry(fields)
            if telemetry is not None:
                self._mega_diagnostic_counts["sensor3_parsed"] += 1
                self._publish_sensor3_telemetry(telemetry)
                continue
            if fields[:2] == ["LOG", "SENSOR3"]:
                self._record_mega_frame_rejection(
                    "sensor3_format_rejected",
                    raw_line,
                )
                continue
            event = parse_event(fields)
            if event is None:
                continue
            if event.kind == "SENSOR" and len(event.values) >= 4:
                self._publish_sensor_event(*event.values[:4])
            elif event.kind == "POSITION" and len(event.values) >= 3:
                conveyor_text, step_text, _sequence_text = event.values[:3]
                try:
                    estimated_step = int(step_text)
                    conveyor_id = int(conveyor_text)
                except ValueError:
                    continue
                self._publish_position_settled(
                    conveyor_id=conveyor_id,
                    estimated_step=estimated_step,
                )
            elif event.kind == "ACTUATION" and len(event.values) >= 2:
                status_text, sequence_text = event.values[:2]
                try:
                    actuation_sequence = int(sequence_text)
                except ValueError:
                    continue
                with self._mega_event:
                    self._mega_events[f"ACTUATION:{actuation_sequence}"] = (
                        status_text == "OK",
                        status_text,
                        time.monotonic(),
                    )
                    self._mega_event.notify_all()
            elif event.kind == "STATE" and len(event.values) >= 6:
                try:
                    upper_state = int(event.values[0])
                    lower_state = int(event.values[1])
                    sensor_1_clear = event.values[2] == "1"
                    sensor_2_clear = event.values[3] == "1"
                    sensor_3_clear = event.values[4] == "1"
                    actuator_safe = event.values[5] == "1"
                except ValueError:
                    continue
                self._publish_equipment_state(
                    upper_state,
                    lower_state,
                    sensor_1_clear,
                    sensor_2_clear,
                    sensor_3_clear,
                    actuator_safe,
                )

    @staticmethod
    def _safe_serial_preview(raw_line: bytes, limit: int = 240) -> str:
        preview = raw_line[:limit].decode("ascii", errors="backslashreplace")
        return preview.rstrip("\r\n")

    def _record_mega_frame_rejection(self, reason: str, raw_line: bytes) -> None:
        """Count every rejection and rate-limit durable diagnostic summaries."""

        self._mega_diagnostic_counts.setdefault(reason, 0)
        self._mega_diagnostic_counts[reason] += 1
        now = time.monotonic()
        last_report = self._mega_last_diagnostic_report.get(reason, float("-inf"))
        if now - last_report < 5.0:
            return
        self._mega_last_diagnostic_report[reason] = now
        preview = self._safe_serial_preview(raw_line)
        self.get_logger().warning(
            f"Mega serial frame rejected: reason={reason}, "
            f"length={len(raw_line)}, preview={preview!r}"
        )
        self._publish_control_log_event(
            "MEGA_SERIAL_FRAME_REJECTED",
            {
                "reason": reason,
                "raw_length": len(raw_line),
                "raw_preview": preview,
                "preview_truncated": len(raw_line) > 240,
                "diagnostic_counts": dict(self._mega_diagnostic_counts),
            },
            severity=LogEvent.WARNING,
        )

    def _publish_sensor3_telemetry(self, telemetry: Sensor3Telemetry) -> None:
        """Mega Sensor3 진단 표본을 LogNode가 SQLite에 저장할 형태로 발행합니다."""

        event_type = f"MEGA_SENSOR3_{telemetry.event}"
        payload = {
            "firmware_event": telemetry.event,
            "firmware_millis": telemetry.firmware_millis,
            "firmware_micros": telemetry.firmware_micros,
            "distance_cm": telemetry.distance_cm,
            "detection_armed": telemetry.detection_armed,
            "consecutive_detect_count": telemetry.consecutive_detect_count,
            "consecutive_release_count": telemetry.consecutive_release_count,
            "echo_timeout": telemetry.event in {"TIMEOUT", "REARMED_TIMEOUT"},
            "release_check": telemetry.event == "RELEASE_CHECK",
            "rearmed": telemetry.event in {"REARMED", "REARMED_TIMEOUT"},
        }
        if self._publish_control_log_event(event_type, payload, severity=LogEvent.DEBUG):
            self._mega_diagnostic_counts["sensor3_publish_attempted"] += 1

    def _publish_control_log_event(
        self,
        event_type: str,
        payload: dict[str, object],
        *,
        severity: int,
    ) -> bool:
        """Publish one Control-owned LogEvent for the active session."""

        if not self.session_id:
            return False
        envelope = {
            "schema_version": 2,
            "event_type": event_type,
            "severity": int(severity),
            "source_node": NodeId.CONTROL.value,
            "producer_instance_id": self.node_instance_id,
            "session_id": self.session_id,
            "product_id": "",
            "payload": payload,
        }
        try:
            payload_json = canonical_json(envelope)
            payload_digest = sha256_text(payload_json)
        except (TypeError, ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Control diagnostic serialization failed: {type(exc).__name__}"
            )
            return False
        message = LogEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.log_id = new_uuid()
        message.revision = 1
        message.severity = severity
        message.event_type = event_type
        message.source_node = NodeId.CONTROL.value
        message.producer_instance_id = self.node_instance_id
        message.product_id = ""
        message.payload_json = payload_json
        message.payload_digest = payload_digest
        message.occurred_at = message.header.stamp
        self._log_event_publisher.publish(message)
        return True

    def _publish_sensor_event(self, sensor_id: str, edge: str, sequence: str, step: str) -> None:
        message = SensorEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.event_id = new_uuid()
        message.sensor_id = sensor_id
        message.edge = int(edge)
        message.sensor_sequence = int(sequence)
        message.estimated_step = int(step)
        message.observed_at = message.header.stamp
        self.publish_sensor_observation(message)

    def _publish_equipment_state(
        self,
        upper_state: int,
        lower_state: int,
        sensor_1_clear: bool,
        sensor_2_clear: bool,
        sensor_3_clear: bool,
        actuator_safe: bool,
    ) -> None:
        """Mega의 E|STATE 이벤트를 EquipmentState 토픽으로 옮깁니다.

        upper_state/lower_state는 Mega의 ConveyorState enum 값과
        동일합니다: 0=RUNNING, 1=POSITIONING, 2=WAIT_CAMERA, 3=STOPPED.
        """

        self._latest_equipment_state = (
            upper_state,
            lower_state,
            sensor_1_clear,
            sensor_2_clear,
            sensor_3_clear,
            actuator_safe,
        )
        message = EquipmentState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.session_id = self.session_id
        message.header.message_id = new_uuid()
        message.upper_running = upper_state == 0
        message.upper_stopped = upper_state == 3
        message.lower_running = lower_state == 0
        message.lower_stopped = lower_state == 3
        message.sensor_1_clear = sensor_1_clear
        message.sensor_2_clear = sensor_2_clear
        message.sensor_3_clear = sensor_3_clear
        message.actuator_safe = actuator_safe
        self._equipment_state_publisher.publish(message)

    def _accept_equipment_goal(self, goal_request) -> GoalResponse:
        return (
            GoalResponse.ACCEPT
            if goal_request.product_id and goal_request.command.command_id
            else GoalResponse.REJECT
        )

    def _cancel_equipment_goal(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_actuation(self, goal_handle) -> ActuateProduct.Result:
        request = goal_handle.request
        result = ActuateProduct.Result()
        valid, code, reason = self.validate_command_header(
            request.command,
            allowed_system_states={SystemState.RUN_SYS, SystemState.PAUSING},
        )
        if not valid:
            result.error_code = int(code)
            result.reason = reason
            goal_handle.abort()
            return result
        replay = self._actuation_results.inspect(
            request.command.command_id, request.command.payload_digest
        )
        if replay.kind.value == "CONFLICT":
            result.error_code = int(ErrorCode.COMMAND_CONFLICT)
            result.reason = "same command_id received with another digest"
            goal_handle.abort()
            return result
        if replay.result is not None:
            values = replay.result
        elif self.profile == "hardware":
            status: str | None = None
            try:
                sequence = self._send_mega("ACTUATE", request.actuator_command)
                status = await self._run_blocking(
                    self._wait_for_mega,
                    f"ACTUATION:{sequence}",
                    int(self.get_parameter("control.actuator.timeout_ms").value) / 1000,
                )
            except (RuntimeError, TimeoutError, OSError):
                status = None
            success = status == "OK"
            values = {"success": success, "product_id": request.product_id,
                      "actuation_id": new_uuid() if success else "",
                      "error_code": int(ErrorCode.NONE if success else ErrorCode.ACTUATOR_FAILED),
                      "reason": (
                          "Mega actuation completed" if success
                          else self._mega_wait_failure("Mega actuation", status)
                      )}
        else:
            values = {
                "success": True,
                "product_id": request.product_id,
                "actuation_id": new_uuid(),
                "error_code": int(ErrorCode.NONE),
                "reason": "sim actuation completed",
            }
        self._actuation_results.remember(
            request.command.command_id, request.command.payload_digest, values
        )
        result.success = bool(values["success"])
        result.product_id = str(values["product_id"])
        result.actuation_id = str(values["actuation_id"])
        result.error_code = int(values["error_code"])
        result.reason = str(values["reason"])
        goal_handle.succeed() if result.success else goal_handle.abort()
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    spin_node(ControlNode())


if __name__ == "__main__":
    main()
