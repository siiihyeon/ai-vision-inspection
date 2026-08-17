"""ROS 2 설치 없이 v2 골격의 구조·IDL·금지 계약을 검사합니다."""

from __future__ import annotations

import ast
import re
from pathlib import Path
from xml.etree import ElementTree

WORKSPACE = Path(__file__).parents[1]
ROOT = WORKSPACE.parent
SOURCE = WORKSPACE / "src"
INTERFACES = SOURCE / "basic_packages" / "inspection_interfaces"

REQUIRED_PACKAGES = {
    "inspection_interfaces",
    "inspection_common",
    "inspection_bringup",
    "inspection_master",
    "inspection_control",
    "inspection_vision",
    "inspection_log",
}
REQUIRED_INTERFACES = {
    "msg": {
        "CommonHeader.msg",
        "CommandHeader.msg",
        "ErrorCode.msg",
        "NodeHealth.msg",
        "SystemState.msg",
        "StationId.msg",
        "ConveyorId.msg",
        "NodeRuntimeEnvironment.msg",
        "MasterHeartbeat.msg",
        "NodeHeartbeat.msg",
        "SystemCommand.msg",
        "SensorEvent.msg",
        "PositionSettled.msg",
        "ImageReference.msg",
        "StationResult.msg",
        "StationInferenceFailed.msg",
        "ProductResultLocked.msg",
        "VisionQueueState.msg",
        "LogEvent.msg",
        "LogPersistedAck.msg",
    },
    "srv": {"GetNodeStatus.srv", "OperatorCommand.srv"},
    "action": {
        "InitializeNode.action",
        "PositionProduct.action",
        "CaptureProduct.action",
        "ActuateProduct.action",
    },
}
EXPECTED_FIELDS = {
    "msg/CommonHeader.msg": [["stamp", "session_id", "message_id", "correlation_id"]],
    "msg/CommandHeader.msg": [["header", "command_epoch", "command_id", "payload_digest", "issued_at"]],
    "msg/SystemCommand.msg": [["command", "command_type", "target_conveyor_id", "reason"]],
    "msg/MasterHeartbeat.msg": [["header", "master_instance_id", "interface_version", "command_epoch", "sequence", "system_state"]],
    "msg/NodeHeartbeat.msg": [["header", "node_id", "node_instance_id", "sequence", "health_state", "interface_version"]],
    "msg/PositionSettled.msg": [["header", "product_id", "station_id", "position_command_id", "conveyor_id", "target_step", "estimated_step", "position_error_steps", "position_source", "position_verified", "settled_at"]],
    "msg/ImageReference.msg": [["camera_id", "file_path", "sha256", "file_size_bytes", "width", "height", "pixel_format", "camera_timestamp_raw", "camera_timestamp_domain", "camera_timestamp_ns", "camera_timestamp_synchronized", "host_arrival_monotonic_ns", "host_arrival_wall_time"]],
    "msg/ProductResultLocked.msg": [["header", "product_id", "fifo_sequence", "final_verdict", "station_a_completed", "station_b_completed", "lock_reason", "sensor3_event_id", "locked_at"]],
    "msg/LogPersistedAck.msg": [["header", "producer_node", "producer_instance_id", "acked_log_ids", "acked_revisions", "committed_at"]],
    "srv/GetNodeStatus.srv": [
        ["request_id", "session_id"],
        ["ready", "node_id", "node_instance_id", "health_state", "interface_version", "software_version", "active_session_id", "command_epoch", "heartbeat_sequence", "master_heartbeat_alive", "uptime_ms", "status_json"],
    ],
    "srv/OperatorCommand.srv": [
        ["request_id", "command_type", "reason", "operator_id"],
        ["request_id", "accepted", "system_state", "system_state_name", "message"],
    ],
    "action/CaptureProduct.action": [
        ["command", "product_id", "fifo_sequence", "station_id", "capture_id", "required_camera_ids", "requested_at"],
        ["success", "product_id", "station_id", "capture_id", "frame_batch_id", "attempt_count", "images", "frame_arrival_skew_us", "inference_job_id", "error_code", "reason"],
        ["stage", "attempt", "frame_batch_id", "completed_camera_ids", "pending_camera_ids", "progress", "reason"],
    ],
}
ERROR_CODES = {
    "NONE": 0,
    "CAPTURE_FAILED": 1000,
    "INFERENCE_FAILED": 2000,
    "CONTROL_FAILED": 3000,
    "ACTUATOR_FAILED": 4000,
    "LOG_COMMIT_FAILED": 5000,
    "COMMAND_CONFLICT": 9000,
    "IMPLEMENTATION_PENDING": 9098,
}
ENUM_CATALOGS = {
    "NodeHealth.msg": {
        "STARTING": 0,
        "READY": 1,
        "DEGRADED": 2,
        "RECOVERING": 3,
        "INIT_BLOCKED": 4,
        "FAULT": 5,
    },
    "SystemState.msg": {
        "BOOT": 0,
        "INITIALIZING": 1,
        "READY": 2,
        "RUN_SYS": 3,
        "PAUSING": 4,
        "PAUSED": 5,
        "FAULT_STOP": 6,
        "RESETTING": 7,
    },
    "StationId.msg": {"UNKNOWN": 0, "A": 1, "B": 2},
    "ConveyorId.msg": {"UNKNOWN": 0, "UPPER": 1, "LOWER": 2},
}
REQUIRED_READMES = [
    ROOT / "README.md",
    ROOT / "구현_전_상세설계" / "README.md",
    ROOT / "firmware" / "arduino_mega" / "README.md",
    WORKSPACE / "README.md",
    WORKSPACE / "tools" / "README.md",
    INTERFACES / "README.md",
    INTERFACES / "msg" / "README.md",
    INTERFACES / "srv" / "README.md",
    INTERFACES / "action" / "README.md",
    SOURCE / "basic_packages" / "inspection_common" / "README.md",
    SOURCE / "basic_packages" / "inspection_bringup" / "README.md",
    SOURCE / "basic_packages" / "inspection_bringup" / "config" / "README.md",
    SOURCE / "nodes" / "inspection_master" / "README.md",
    SOURCE / "nodes" / "inspection_master" / "마스터노드_읽기가이드.md",
    SOURCE / "nodes" / "inspection_control" / "README.md",
    SOURCE / "nodes" / "inspection_vision" / "README.md",
    SOURCE / "nodes" / "inspection_vision" / "코드 읽기 가이드.md",
    SOURCE / "nodes" / "inspection_vision" / "실험_파라미터와_미결정사항.md",
    SOURCE / "nodes" / "inspection_vision" / "MVS_실장비_검증절차.md",
    SOURCE / "nodes" / "inspection_vision" / "검증결과.md",
    SOURCE / "nodes" / "inspection_log" / "README.md",
]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def parse_idl_sections(path: Path) -> list[list[str]]:
    sections: list[list[str]] = [[]]
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line == "---":
            sections.append([])
            continue
        parts = line.split()
        require(len(parts) == 2, f"invalid IDL line {path}: {raw}")
        if "=" not in parts[1]:
            sections[-1].append(parts[1])
    return sections


manifests = list(SOURCE.rglob("package.xml"))
found_packages: dict[str, Path] = {}
for manifest in manifests:
    root = ElementTree.parse(manifest).getroot()
    name = root.findtext("name")
    require(bool(name), f"package name missing: {manifest}")
    found_packages[str(name)] = manifest
require(REQUIRED_PACKAGES <= found_packages.keys(), "required ROS package is missing")

for python_file in SOURCE.rglob("*.py"):
    ast.parse(python_file.read_text(encoding="utf-8"), filename=str(python_file))
for python_file in (WORKSPACE / "tools").glob("*.py"):
    ast.parse(python_file.read_text(encoding="utf-8"), filename=str(python_file))

cmake = (INTERFACES / "CMakeLists.txt").read_text(encoding="utf-8")
for kind, names in REQUIRED_INTERFACES.items():
    actual = {path.name for path in (INTERFACES / kind).glob(f"*.{kind if kind != 'action' else 'action'}")}
    # glob suffix for msg/srv/action is identical to directory name.
    require(names <= actual, f"missing {kind} interfaces: {sorted(names - actual)}")
    for name in names:
        relative = f"{kind}/{name}"
        require(f'"{relative}"' in cmake, f"CMake does not register {relative}")
        sections = parse_idl_sections(INTERFACES / relative)
        for section in sections:
            require(len(section) == len(set(section)), f"duplicate field in {relative}")

for relative, expected in EXPECTED_FIELDS.items():
    actual = parse_idl_sections(INTERFACES / relative)
    require(actual == expected, f"v2 field contract mismatch: {relative}: {actual}")

error_text = (INTERFACES / "msg" / "ErrorCode.msg").read_text(encoding="utf-8")
actual_codes = {
    match.group(1): int(match.group(2))
    for match in re.finditer(r"^uint16\s+([A-Z0-9_]+)=(\d+)$", error_text, re.MULTILINE)
}
for name, value in ERROR_CODES.items():
    require(actual_codes.get(name) == value, f"ErrorCode mismatch: {name}")

for filename, expected_catalog in ENUM_CATALOGS.items():
    text = (INTERFACES / "msg" / filename).read_text(encoding="utf-8")
    actual_catalog = {
        match.group(1): int(match.group(2))
        for match in re.finditer(r"^uint8\s+([A-Z0-9_]+)=(\d+)$", text, re.MULTILINE)
    }
    require(actual_catalog == expected_catalog, f"numeric enum mismatch: {filename}")

constants_tree = ast.parse(
    (SOURCE / "basic_packages" / "inspection_common" / "inspection_common" / "constants.py").read_text(encoding="utf-8")
)
python_enums: dict[str, dict[str, int]] = {}
for item in constants_tree.body:
    if not isinstance(item, ast.ClassDef):
        continue
    values: dict[str, int] = {}
    for statement in item.body:
        if (
            isinstance(statement, ast.Assign)
            and len(statement.targets) == 1
            and isinstance(statement.targets[0], ast.Name)
            and isinstance(statement.value, ast.Constant)
            and isinstance(statement.value.value, int)
        ):
            values[statement.targets[0].id] = statement.value.value
    if values:
        python_enums[item.name] = values
for filename, expected_catalog in ENUM_CATALOGS.items():
    message_name = filename.removesuffix(".msg")
    class_name = "NodeHealthState" if message_name == "NodeHealth" else message_name
    expected_python = {
        key: value for key, value in expected_catalog.items() if key != "UNKNOWN"
    }
    require(
        python_enums.get(class_name) == expected_python,
        f"Python/ROS enum mismatch: {class_name}",
    )
require(
    python_enums.get("ErrorCode") == actual_codes,
    "Python/ROS ErrorCode catalogs differ",
)

interface_manifest = ElementTree.parse(INTERFACES / "package.xml").getroot()
require(interface_manifest.findtext("version") == "2.0.0", "interface version must be 2.0.0")

for manifest in manifests:
    package_root = manifest.parent
    setup = package_root / "setup.py"
    if not setup.is_file():
        continue
    xml_version = ElementTree.parse(manifest).getroot().findtext("version")
    setup_match = re.search(r'version="([^"]+)"', setup.read_text(encoding="utf-8"))
    require(setup_match is not None and setup_match.group(1) == xml_version, f"version mismatch: {package_root}")

for readme in REQUIRED_READMES:
    require(readme.is_file() and readme.stat().st_size > 100, f"required README missing/empty: {readme}")

sim = (SOURCE / "basic_packages" / "inspection_bringup" / "config" / "sim.yaml").read_text(encoding="utf-8")
hardware = (SOURCE / "basic_packages" / "inspection_bringup" / "config" / "hardware.yaml").read_text(encoding="utf-8")
for config in (sim, hardware):
    require('system.expected_interface_version: "2.0.0"' in config, "interface version config mismatch")
    require("comm.master_heartbeat_period_ms: 500" in config, "Master heartbeat period mismatch")
    require(config.count("comm.node_heartbeat_period_ms: 500") == 3, "worker heartbeat period mismatch")
    require("comm.master_heartbeat_timeout_ms: 2000" in config, "heartbeat timeout mismatch")
    require("comm.node_heartbeat_timeout_ms: 2000" in config, "worker heartbeat timeout mismatch")
    require("system.init_timeout_ms: 10000" in config, "worker init timeout mismatch")
    require("retry.init_interval_ms: 1000" in config, "worker init retry interval mismatch")
    require(
        "master.action.conveyor_resume_timeout_ms: 10000" in config,
        "target conveyor resume timeout missing",
    )
    require(
        "master.completed_context_retention_ms: 600000" in config,
        "completed product context retention mismatch",
    )
    require("GIGE_ACTION_COMMAND" in config, "GigE Action trigger mode missing")
    require("warning_codes" not in config, "warning_codes must not return")
require("TODO(HARDWARE_REQUIRED)" in hardware, "hardware fail-closed marker missing")
require("hardware_ready" not in hardware, "manual hardware_ready bypass is forbidden")

code_and_idl = "\n".join(
    path.read_text(encoding="utf-8")
    for path in SOURCE.rglob("*")
    if path.is_file() and path.suffix in {".py", ".msg", ".srv", ".action", ".yaml"}
)
for forbidden in (
    "TriggerCapture",
    "trigger_capture",
    "LightRuntime",
    "warning_codes",
    "control.light",
    "control.trigger",
    "led_brightness",
    "set_led",
):
    require(forbidden not in code_and_idl, f"forbidden legacy contract found: {forbidden}")

node_base = (SOURCE / "basic_packages" / "inspection_common" / "inspection_common" / "node_base.py").read_text(encoding="utf-8")
for token in (
    "ReliabilityPolicy.BEST_EFFORT",
    "depth=1",
    "MultiThreadedExecutor",
    "node_instance_id",
    "master_heartbeat_timeout_ms",
    "is_sha256_hex",
    "_initialize_in_progress",
    "validate_command_header",
):
    require(token in node_base, f"NodeBase contract missing: {token}")

master = (SOURCE / "nodes" / "inspection_master" / "inspection_master" / "master_node.py").read_text(encoding="utf-8")
product_flow = (SOURCE / "nodes" / "inspection_master" / "inspection_master" / "product_flow.py").read_text(encoding="utf-8")
system_fsm = (SOURCE / "nodes" / "inspection_master" / "inspection_master" / "system_fsm.py").read_text(encoding="utf-8")
worker_supervision = (SOURCE / "nodes" / "inspection_master" / "inspection_master" / "worker_supervision.py").read_text(encoding="utf-8")
operation_runtime = (SOURCE / "nodes" / "inspection_master" / "inspection_master" / "operation_runtime.py").read_text(encoding="utf-8")
vision = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "vision_node.py").read_text(encoding="utf-8")
vision_runtime = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "vision_runtime.py").read_text(encoding="utf-8")
capture_service = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "capture_service.py").read_text(encoding="utf-8")
artifact_store = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "artifact_store.py").read_text(encoding="utf-8")
mvs_backend = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "mvs_backend.py").read_text(encoding="utf-8")
queue_journal = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "queue_journal.py").read_text(encoding="utf-8")
queue = (SOURCE / "nodes" / "inspection_vision" / "inspection_vision" / "inference_queue.py").read_text(encoding="utf-8")
log_storage = (SOURCE / "nodes" / "inspection_log" / "inspection_log" / "storage.py").read_text(encoding="utf-8")
for token in ("ProductResultReorderBuffer", "lock_product_at_sensor3", "StationInferenceFailed", "ENQUEUE_BLOCKED"):
    require(token in master + product_flow, f"Master ownership contract missing: {token}")
for token in (
    "START_REQUEST",
    "ALL_CONVEYORS_RUNNING",
    "PAUSE_REQUEST",
    "ALL_CONVEYORS_STOPPED",
    "CRITICAL_FAULT",
    "RESET_SUCCEEDED_EMPTY_LINE",
    "RESET_SUCCEEDED_IN_PLACE",
    "ESTOP_ASSERTED",
    "InvalidSystemTransition",
):
    require(token in system_fsm, f"Master system FSM contract missing: {token}")
for token in (
    "WorkerRuntimeState",
    "WorkerInitPhase",
    "accept_heartbeat",
    "can_mark_ready",
    "retry_of_request_id",
):
    require(token in worker_supervision, f"Master worker supervision contract missing: {token}")
for token in (
    "StationCycle",
    "ActuationCycle",
    "EquipmentSnapshot",
    "line_clear_guards_satisfied",
    "in_place_guards_satisfied",
):
    require(token in operation_runtime, f"Master operation runtime missing: {token}")
for token in (
    "request_initialize",
    "_handle_operator_command",
    "_start_worker_initialization",
    "_handle_sensor1_entry",
    "_handle_sensor2_entry",
    "_handle_sensor3_entry",
    "_handle_position_settled",
    "_request_station_capture",
    "_schedule_actuation",
    "_remove_completed_fifo_prefix",
    "_verify_resume_conditions",
    "_handle_log_persisted_ack",
    "request_shutdown",
    "conveyor_resume_timeout_ms",
    "OperatorCommand",
):
    require(token in master, f"Master implementation block skeleton missing: {token}")
master_readme = (SOURCE / "nodes" / "inspection_master" / "README.md").read_text(
    encoding="utf-8"
)
require(
    "`SKELETON`" not in master_readme and "`PARTIAL`" not in master_readme,
    "Master README still reports incomplete implementation blocks",
)
vision_implementation = "\n".join(
    (
        vision,
        vision_runtime,
        capture_service,
        artifact_store,
        mvs_backend,
        queue_journal,
    )
)
for token in (
    "capture_id=request.capture_id",
    "vision.capture.max_attempts",
    "try_enqueue",
    "ENQUEUE_BLOCKED",
    "frame_arrival_skew_us",
    '"reason": ""',
    "MV_GIGE_IssueActionCommand",
    "MV_CC_RegisterImageCallBackEx",
    "MV_CC_ConvertPixelTypeEx",
    "_atomic_write",
    "enqueue_committed",
    "expected_sdk_version_raw",
):
    require(token in vision_implementation, f"Vision implementation contract missing: {token}")
for token in ("deque", "queue_total_timeout_ms", "discard_expired", "_sweep_expired", "_load_with_one_retry", "_infer_with_one_retry", "_model_lock"):
    require(token in queue, f"Inference worker contract missing: {token}")
for table in ("products_latest", "frame_batch_attempts", "inference_jobs_latest", "camera_result_revisions", "station_result_revisions", "faults", "pending_projections"):
    require(table in log_storage, f"Log schema table missing: {table}")

launch = (SOURCE / "basic_packages" / "inspection_bringup" / "launch" / "inspection_system.launch.py").read_text(encoding="utf-8")
require(launch.count('namespace="inspection"') == 4, "four node namespaces are required")
require(not (ROOT / "vision-inspection").exists(), "nested previous skeleton must not be shipped")
require(
    (WORKSPACE / "ROS2_핵심개념_코드읽기가이드.md").is_file(),
    "updated ROS 2 code-reading guide is missing",
)

print("inspection modified v2 skeleton verification: PASS")
