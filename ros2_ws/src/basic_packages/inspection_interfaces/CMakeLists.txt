cmake_minimum_required(VERSION 3.8)
project(inspection_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommonHeader.msg"
  "msg/CommandHeader.msg"
  "msg/ErrorCode.msg"
  "msg/NodeHealth.msg"
  "msg/SystemState.msg"
  "msg/StationId.msg"
  "msg/ConveyorId.msg"
  "msg/NodeRuntimeEnvironment.msg"
  "msg/MasterHeartbeat.msg"
  "msg/NodeHeartbeat.msg"
  "msg/SystemCommand.msg"
  "msg/SensorEvent.msg"
  "msg/PositionSettled.msg"
  "msg/ImageReference.msg"
  "msg/StationResult.msg"
  "msg/StationInferenceFailed.msg"
  "msg/ProductResultLocked.msg"
  "msg/VisionQueueState.msg"
  "msg/LogEvent.msg"
  "msg/LogPersistedAck.msg"
  "srv/GetNodeStatus.srv"
  "srv/OperatorCommand.srv"
  "action/InitializeNode.action"
  "action/PositionProduct.action"
  "action/CaptureProduct.action"
  "action/ActuateProduct.action"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
