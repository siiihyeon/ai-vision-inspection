"""ControlNode 통신 골격. Mega/TB6600 구현은 adapter 확장점에 격리합니다."""

from __future__ import annotations

import rclpy
from inspection_common import ErrorCode, IdempotencyStore, NodeId, new_uuid
from inspection_common.node_base import (
    InspectionNodeBase,
    NodeInitializationOutcome,
    reliable_event_qos,
    spin_node,
)
from inspection_interfaces.action import ActuateProduct, PositionProduct
from inspection_interfaces.msg import PositionSettled, SensorEvent
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


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

        self._position_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._actuation_results: IdempotencyStore[dict[str, object]] = IdempotencyStore()
        self._equipment_action_group = MutuallyExclusiveCallbackGroup()
        self._sensor_publisher = self.create_publisher(
            SensorEvent, "control/sensor_event", reliable_event_qos()
        )
        self._position_settled_publisher = self.create_publisher(
            PositionSettled, "control/position_settled", reliable_event_qos()
        )
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

    async def initialize_node_resources(self) -> NodeInitializationOutcome:
        # TODO(IMPLEMENTATION): Mega handshake, safe outputs, sensors and TB6600 self-test.
        return await super().initialize_node_resources()

    def publish_sensor_observation(self, message: SensorEvent) -> None:
        """향후 serial adapter가 debounced 센서 이벤트를 전달할 확장점."""

        self._sensor_publisher.publish(message)

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
            return self._finish_position(
                goal_handle,
                result,
                False,
                ErrorCode.IMPLEMENTATION_PENDING,
                "Mega/TB6600 position adapter is not implemented",
            )

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
        valid, code, reason = self.validate_command_header(request.command)
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
            values = {
                "success": False,
                "product_id": request.product_id,
                "actuation_id": "",
                "error_code": int(ErrorCode.IMPLEMENTATION_PENDING),
                "reason": "Mega actuator adapter is not implemented",
            }
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
