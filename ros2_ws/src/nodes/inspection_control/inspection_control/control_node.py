"""ControlNode 통신 골격. Mega/TB6600 구현은 adapter 확장점에 격리합니다."""

from __future__ import annotations

import rclpy
from .mega_protocol import decode_frame, encode_frame, parse_event
from inspection_common import (
    ConveyorId,
    ErrorCode,
    IdempotencyStore,
    NodeId,
    SystemState,
    new_uuid,
)
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
)
from inspection_interfaces.action import ActuateProduct, PositionProduct
from inspection_interfaces.msg import PositionSettled, SensorEvent, SystemCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import threading
import time

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
        self.declare_parameter("control.position.upper_steps", 0)
        self.declare_parameter("control.position.lower_steps", 0)
        self.declare_parameter("control.position.timeout_ms", 10000)
        self.declare_parameter("control.actuator.timeout_ms", 10000)

        self._position_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._actuation_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._equipment_action_group = MutuallyExclusiveCallbackGroup()
        self._sensor_publisher = self.create_publisher(
            SensorEvent, "control/sensor_event", reliable_event_qos()
        )
        self._position_settled_publisher = self.create_publisher(
            PositionSettled, "control/position_settled", reliable_event_qos()
        )
        self._mega = None
        self._mega_lock = threading.RLock()
        self._mega_sequence = 0
        self._mega_events: dict[str, tuple[bool, str]] = {}
        self._pending_position_requests: dict[int, dict[str, object]] = {}
        self._mega_event = threading.Condition(self._mega_lock)
        self._position_server = ActionServer(
            self,
            PositionProduct,
            "control/position_product",
            execute_callback=self._execute_position,
            goal_callback=self._accept_equipment_goal,
            cancel_callback=self._cancel_equipment_goal,
            callback_group=self._equipment_action_group,
        )
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
        return list(dict.fromkeys(missing))

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        if self.profile == "hardware":
            if serial is None:
                return NodeInitializationOutcome(False, "pyserial is not installed")
            try:
                self._mega = serial.Serial(
                    str(self.get_parameter("control.mega.port").value),
                    int(self.get_parameter("control.mega.baud_rate").value),
                    timeout=0.1,
                )
                threading.Thread(target=self._read_mega, daemon=True).start()
                sequence = self._send_mega("HELLO", 2)
                if not self._wait_for_mega(f"ACK:{sequence}", 2.0):
                    return NodeInitializationOutcome(False, "Mega HELLO timeout")
            except (OSError, RuntimeError) as exc:
                return NodeInitializationOutcome(False, f"Mega connection failed: {exc}", True)
            return NodeInitializationOutcome(True, "Mega handshake completed")
        return await super().initialize_node_resources()

    def publish_sensor_observation(self, message: SensorEvent) -> None:
        """향후 serial adapter가 debounced 센서 이벤트를 전달할 확장점."""

        self._sensor_publisher.publish(message)

    def _publish_position_settled(
        self,
        request: dict[str, object],
        *,
        conveyor_id: int,
        estimated_step: int,
    ) -> None:
        settled = PositionSettled()
        settled.header.stamp = self.get_clock().now().to_msg()
        settled.header.session_id = self.session_id
        settled.header.message_id = new_uuid()
        settled.header.correlation_id = str(request["product_id"])
        settled.product_id = str(request["product_id"])
        settled.station_id = int(request["station_id"])
        settled.position_command_id = str(request["command_id"])
        settled.conveyor_id = conveyor_id
        settled.target_step = int(request["target_step"])
        settled.estimated_step = estimated_step
        settled.position_error_steps = estimated_step - settled.target_step
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
            except RuntimeError as exc:
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

    def _wait_for_mega(self, key: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        with self._mega_event:
            while time.monotonic() < deadline:
                if key in self._mega_events:
                    return self._mega_events.pop(key)[0]
                self._mega_event.wait(max(0.01, deadline - time.monotonic()))
        return False

    def _read_mega(self) -> None:
        while self._mega is not None:
            fields = decode_frame(self._mega.readline())
            if not fields:
                continue
            if fields[0] == "A" and len(fields) >= 3:
                with self._mega_event:
                    self._mega_events[f"ACK:{fields[1]}"] = (fields[2] == "OK", fields[2])
                    self._mega_event.notify_all()
                continue
            event = parse_event(fields)
            if event is None:
                continue
            if event.kind == "SENSOR" and len(event.values) >= 4:
                self._publish_sensor_event(*event.values[:4])
            elif event.kind == "POSITION" and len(event.values) >= 3:
                conveyor_text, step_text, sequence_text = event.values[:3]
                try:
                    position_sequence = int(sequence_text)
                    estimated_step = int(step_text)
                    conveyor_id = int(conveyor_text)
                except ValueError:
                    continue
                with self._mega_event:
                    self._mega_events[f"POSITION:{position_sequence}"] = (
                        True,
                        f"{conveyor_id}|{estimated_step}",
                    )
                    request = self._pending_position_requests.pop(
                        position_sequence, None
                    )
                    self._mega_event.notify_all()
                if request is not None:
                    self._publish_position_settled(
                        request,
                        conveyor_id=conveyor_id,
                        estimated_step=estimated_step,
                    )
            elif event.kind == "ACTUATION" and event.values:
                with self._mega_event:
                    self._mega_events["ACTUATION"] = (event.values[0] == "OK", "")
                    self._mega_event.notify_all()

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

    def _accept_equipment_goal(self, goal_request) -> GoalResponse:
        return (
            GoalResponse.ACCEPT
            if goal_request.product_id and goal_request.command.command_id
            else GoalResponse.REJECT
        )

    def _cancel_equipment_goal(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_position(self, goal_handle) -> PositionProduct.Result:
        request = goal_handle.request
        result = PositionProduct.Result()
        valid, code, reason = self.validate_command_header(request.command)
        if not valid:
            return self._finish_position(goal_handle, result, False, code, reason)

        replay = self._position_results.inspect(
            request.command.command_id, request.command.payload_digest
        )
        if replay.kind.value == "CONFLICT":
            return self._finish_position(
                goal_handle,
                result,
                False,
                ErrorCode.COMMAND_CONFLICT,
                "same command_id received with another digest",
            )
        if replay.result is not None:
            self._apply_position_values(result, replay.result)
            goal_handle.succeed() if result.success else goal_handle.abort()
            return result
        if goal_handle.is_cancel_requested:
            return self._finish_position(
                goal_handle,
                result,
                False,
                ErrorCode.CONTROL_FAILED,
                "position canceled",
                canceled=True,
            )
        if self.profile == "hardware":
            target_step = request.target_step or (
                int(self.get_parameter("control.position.upper_steps").value)
                if request.conveyor_id == int(ConveyorId.UPPER)
                else int(self.get_parameter("control.position.lower_steps").value)
            )
            sequence = 0
            try:
                sequence = self._next_sequence()
                with self._mega_lock:
                    self._pending_position_requests[sequence] = {
                        "product_id": request.product_id,
                        "station_id": request.station_id,
                        "command_id": request.command.command_id,
                        "target_step": target_step,
                    }
                    self._mega.write(
                        encode_frame(
                            "C", sequence, "POSITION", request.conveyor_id, target_step
                        )
                    )
                if not self._wait_for_mega(
                    f"POSITION:{sequence}",
                    int(self.get_parameter("control.position.timeout_ms").value) / 1000,
                ):
                    raise TimeoutError("position feedback timeout")
            except (RuntimeError, TimeoutError) as exc:
                with self._mega_lock:
                    self._pending_position_requests.pop(sequence, None)
                return self._finish_position(goal_handle, result, False, ErrorCode.POSITION_FAILED, str(exc))
            values = {"success": True, "product_id": request.product_id, "station_id": request.station_id,
                      "position_command_id": request.command.command_id, "estimated_step": target_step,
                      "position_error_steps": 0, "position_source": PositionSettled.OPEN_LOOP_ESTIMATE,
                      "error_code": int(ErrorCode.NONE), "reason": "Mega position settled"}
            self._position_results.remember(request.command.command_id, request.command.payload_digest, values)
            self._apply_position_values(result, values)
            goal_handle.succeed()
            return result

        values: dict[str, object] = {
            "success": True,
            "product_id": request.product_id,
            "station_id": request.station_id,
            "position_command_id": request.command.command_id,
            "estimated_step": request.target_step,
            "position_error_steps": 0,
            "position_source": PositionSettled.OPEN_LOOP_ESTIMATE,
            "error_code": int(ErrorCode.NONE),
            "reason": "sim open-loop estimate settled",
        }
        self._position_results.remember(
            request.command.command_id, request.command.payload_digest, values
        )
        self._apply_position_values(result, values)
        settled = PositionSettled()
        settled.header.stamp = self.get_clock().now().to_msg()
        settled.header.session_id = self.session_id
        settled.header.message_id = new_uuid()
        settled.header.correlation_id = request.command.command_id
        settled.product_id = request.product_id
        settled.station_id = request.station_id
        settled.position_command_id = request.command.command_id
        settled.conveyor_id = request.conveyor_id
        settled.target_step = request.target_step
        settled.estimated_step = request.target_step
        settled.position_error_steps = 0
        settled.position_source = PositionSettled.OPEN_LOOP_ESTIMATE
        settled.position_verified = False
        settled.settled_at = settled.header.stamp
        self._position_settled_publisher.publish(settled)
        goal_handle.succeed()
        return result

    def _finish_position(
        self,
        goal_handle,
        result,
        success: bool,
        code: ErrorCode,
        reason: str,
        *,
        canceled: bool = False,
    ) -> PositionProduct.Result:
        result.success = success
        result.product_id = goal_handle.request.product_id
        result.station_id = goal_handle.request.station_id
        result.position_command_id = goal_handle.request.command.command_id
        result.error_code = int(code)
        result.reason = reason
        if success:
            goal_handle.succeed()
        elif canceled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    @staticmethod
    def _apply_position_values(result, values: dict[str, object]) -> None:
        result.success = bool(values["success"])
        result.product_id = str(values["product_id"])
        result.station_id = int(values["station_id"])
        result.position_command_id = str(values["position_command_id"])
        result.estimated_step = int(values["estimated_step"])
        result.position_error_steps = int(values["position_error_steps"])
        result.position_source = int(values["position_source"])
        result.error_code = int(values["error_code"])
        result.reason = str(values["reason"])

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
            try:
                self._send_mega("ACTUATE", request.actuator_command)
                success = self._wait_for_mega("ACTUATION", int(self.get_parameter("control.actuator.timeout_ms").value) / 1000)
            except (RuntimeError, TimeoutError):
                success = False
            values = {"success": success, "product_id": request.product_id,
                      "actuation_id": new_uuid() if success else "",
                      "error_code": int(ErrorCode.NONE if success else ErrorCode.ACTUATOR_FAILED),
                      "reason": "Mega actuation completed" if success else "Mega actuation failed"}
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
    rclpy.init(args=args)
    spin_node(ControlNode())


if __name__ == "__main__":
    main()
