"""ROS IDL과 숫자값을 공유하는 공통 enum입니다."""

from enum import IntEnum, StrEnum


class NodeId(StrEnum):
    MASTER = "master"
    CONTROL = "control"
    VISION = "vision"
    LOG = "log"


class NodeHealthState(IntEnum):
    STARTING = 0
    READY = 1
    DEGRADED = 2
    RECOVERING = 3
    INIT_BLOCKED = 4
    FAULT = 5


class SystemState(IntEnum):
    BOOT = 0
    INITIALIZING = 1
    READY = 2
    RUN_SYS = 3
    PAUSING = 4
    PAUSED = 5
    FAULT_STOP = 6
    RESETTING = 7


class PauseReason(IntEnum):
    OPERATOR = 1
    DEVICE_RECOVERY_AUTO = 2
    DEVICE_RECOVERY_MANUAL = 3
    FAULT_RECOVERY = 4
    STORAGE_RECOVERY = 5


class ProductPhysicalState(IntEnum):
    STATION_A_WAIT = 1
    STATION_A_DONE = 2
    FLIPPING = 3
    STATION_B_WAIT = 4
    STATION_B_DONE = 5
    SENSOR3_WAIT = 6
    AT_SENSOR3 = 7
    ACTUATING = 8
    DONE = 9


class PhysicalZone(IntEnum):
    UPPER_INSPECTION = 1
    FLIP_TRANSFER = 2
    LOWER_INSPECTION = 3
    LOWER_TO_SENSOR3 = 4


class StationId(IntEnum):
    A = 1
    B = 2


class ConveyorId(IntEnum):
    UPPER = 1
    LOWER = 2


class Verdict(IntEnum):
    PASS = 1
    NG = 2
    FORCED_NG = 3


class ErrorCode(IntEnum):
    NONE = 0
    CAPTURE_FAILED = 1000
    CAPTURE_SKEW_EXCEEDED = 1001
    CAPTURE_SAVE_FAILED = 1002
    CAPTURE_CANCELED = 1003
    VISION_DISK_STOP = 1004
    INFERENCE_FAILED = 2000
    INFERENCE_TIMEOUT = 2001
    INFERENCE_FILE_READ_FAILED = 2002
    GPU_OUT_OF_MEMORY = 2003
    CONTROL_FAILED = 3000
    POSITION_FAILED = 3001
    ACTUATOR_FAILED = 4000
    LOG_COMMIT_FAILED = 5000
    COMMAND_CONFLICT = 9000
    INTERFACE_VERSION_MISMATCH = 9001
    NODE_INIT_FAILED = 9002
    MASTER_HEARTBEAT_TIMEOUT = 9003
    IMPLEMENTATION_PENDING = 9098
    INTERNAL_ERROR = 9099


NODE_EXECUTABLE_NAME = {
    NodeId.MASTER: "master_node",
    NodeId.CONTROL: "control_node",
    NodeId.VISION: "vision_node",
    NodeId.LOG: "log_node",
}

NODE_PACKAGE_NAME = {
    NodeId.MASTER: "inspection_master",
    NodeId.CONTROL: "inspection_control",
    NodeId.VISION: "inspection_vision",
    NodeId.LOG: "inspection_log",
}
