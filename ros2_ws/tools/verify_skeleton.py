"""ROS 2 설치 없이 통신 골격의 구조와 핵심 계약을 검사합니다."""

from __future__ import annotations

import ast
import re
from pathlib import Path
from xml.etree import ElementTree


WORKSPACE = Path(__file__).parents[1]
SOURCE = WORKSPACE / "src"
REQUIRED_PACKAGES = {
    "inspection_interfaces",
    "inspection_common",
    "inspection_bringup",
    "inspection_master",
    "inspection_control",
    "inspection_vision",
    "inspection_log",
}
INTERFACE_FIELDS = {
    "msg/CommonHeader.msg": ["stamp", "session_id", "message_id", "correlation_id"],
    "msg/MasterHeartbeat.msg": [
        "header",
        "command_epoch",
        "sequence",
        "system_state",
    ],
    "msg/NodeHeartbeat.msg": [
        "header",
        "node_id",
        "sequence",
        "health_state",
        "interface_version",
    ],
    "srv/GetNodeStatus.srv": [
        "request_id",
        "session_id",
        "ready",
        "node_id",
        "health_state",
        "interface_version",
        "status_json",
    ],
    "action/InitializeNode.action": [
        "request_id",
        "session_id",
        "config_version",
        "config_digest",
        "expected_interface_version",
        "success",
        "node_id",
        "error_code",
        "reason",
        "interface_version",
        "software_version",
        "status_snapshot",
        "retryable",
        "stage",
        "attempt",
    ],
}
NODE_CONTRACTS = {
    "inspection_master": ("MasterNode", "MASTER", False, False),
    "inspection_control": ("ControlNode", "CONTROL", True, True),
    "inspection_vision": ("VisionNode", "VISION", True, True),
    "inspection_log": ("LogNode", "LOG", True, True),
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def interface_field_names(path: Path) -> list[str]:
    fields: list[str] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or line == "---":
            continue
        parts = line.split()
        require(len(parts) == 2, f"invalid interface line: {path}: {raw_line}")
        fields.append(parts[1])
    return fields


manifests = list(SOURCE.rglob("package.xml"))
found_packages: dict[str, Path] = {}
for manifest in manifests:
    root = ElementTree.parse(manifest).getroot()
    name = root.findtext("name")
    require(bool(name), f"package name missing: {manifest}")
    found_packages[str(name)] = manifest

require(
    REQUIRED_PACKAGES <= found_packages.keys(),
    f"required packages missing: {REQUIRED_PACKAGES - found_packages.keys()}",
)

for python_file in SOURCE.rglob("*.py"):
    ast.parse(python_file.read_text(encoding="utf-8"), filename=str(python_file))

interfaces = SOURCE / "basic_packages" / "inspection_interfaces"
cmake = (interfaces / "CMakeLists.txt").read_text(encoding="utf-8")
for relative, expected_fields in INTERFACE_FIELDS.items():
    interface_file = interfaces / relative
    require(interface_file.is_file(), f"missing interface: {relative}")
    require(relative in cmake, f"interface not registered in CMake: {relative}")
    require(
        interface_field_names(interface_file) == expected_fields,
        f"interface contract mismatch: {relative}",
    )

interface_manifest = ElementTree.parse(interfaces / "package.xml").getroot()
interface_version = interface_manifest.findtext("version")
require(interface_version == "1.0.0", "unexpected inspection_interfaces version")

for profile in ("sim", "hardware"):
    config = (
        SOURCE
        / "basic_packages"
        / "inspection_bringup"
        / "config"
        / f"{profile}.yaml"
    ).read_text(encoding="utf-8")
    require(
        re.search(r"^\s+interface_version:\s*", config, re.MULTILINE) is None,
        "actual interface version must not be configurable",
    )
    require(
        f'system.expected_interface_version: "{interface_version}"' in config,
        f"Master expected interface version missing from {profile}.yaml",
    )
    require(
        "comm.master_heartbeat_period_ms: 500" in config,
        f"Master heartbeat setting missing from {profile}.yaml",
    )
    require(
        config.count("comm.node_heartbeat_period_ms: 500") == 3,
        f"worker heartbeat settings missing from {profile}.yaml",
    )

hardware = (
    SOURCE
    / "basic_packages"
    / "inspection_bringup"
    / "config"
    / "hardware.yaml"
).read_text(encoding="utf-8")
require("TODO(HARDWARE_REQUIRED)" in hardware, "hardware TODO markers are missing")
require("hardware_ready" not in hardware, "manual hardware_ready bypass must not exist")

node_base = (
    SOURCE
    / "basic_packages"
    / "inspection_common"
    / "inspection_common"
    / "node_base.py"
).read_text(encoding="utf-8")
require("ReliabilityPolicy.BEST_EFFORT" in node_base, "heartbeat QoS must be Best Effort")
require(re.search(r"depth\s*=\s*1", node_base) is not None, "heartbeat depth must be 1")
require("MultiThreadedExecutor" in node_base, "shared multi-threaded executor is missing")
require("schema_version" in node_base, "status JSON schema version is missing")
require(
    "bool(request.session_id) and request.session_id == self.session_id" in node_base,
    "GetNodeStatus must reject an empty or mismatched session",
)
require(
    "await self.initialize_node_resources()" in node_base,
    "shared initialization must call the node resource hook",
)
require(
    'if self.profile == "hardware"' in node_base,
    "unimplemented hardware resource initialization must be blocked",
)

for package_name, (
    class_name,
    expected_node_id,
    expected_action_server,
    requires_worker_hooks,
) in NODE_CONTRACTS.items():
    module_name = package_name.removeprefix("inspection_") + "_node.py"
    node_file = SOURCE / "nodes" / package_name / package_name / module_name
    tree = ast.parse(node_file.read_text(encoding="utf-8"), filename=str(node_file))
    class_node = next(
        (
            item
            for item in tree.body
            if isinstance(item, ast.ClassDef) and item.name == class_name
        ),
        None,
    )
    require(class_node is not None, f"node class missing: {class_name}")
    require(
        any(
            isinstance(base, ast.Name) and base.id == "InspectionNodeBase"
            for base in class_node.bases
        ),
        f"{class_name} must inherit InspectionNodeBase",
    )
    methods = {
        item.name: item
        for item in class_node.body
        if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
    }
    require("__init__" in methods, f"{class_name} constructor is missing")
    super_init = next(
        (
            call
            for call in ast.walk(methods["__init__"])
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == "__init__"
            and isinstance(call.func.value, ast.Call)
            and isinstance(call.func.value.func, ast.Name)
            and call.func.value.func.id == "super"
        ),
        None,
    )
    require(super_init is not None, f"{class_name} must call super().__init__")
    require(
        bool(super_init.args)
        and isinstance(super_init.args[0], ast.Attribute)
        and isinstance(super_init.args[0].value, ast.Name)
        and super_init.args[0].value.id == "NodeId"
        and super_init.args[0].attr == expected_node_id,
        f"{class_name} uses the wrong NodeId",
    )
    action_keyword = next(
        (
            keyword
            for keyword in super_init.keywords
            if keyword.arg == "provides_initialize_action"
        ),
        None,
    )
    require(
        action_keyword is not None
        and isinstance(action_keyword.value, ast.Constant)
        and action_keyword.value.value is expected_action_server,
        f"{class_name} has the wrong initialize Action ownership",
    )
    require(
        "_execute_initialize" not in methods,
        f"{class_name} must not override shared _execute_initialize",
    )
    if requires_worker_hooks:
        require(
            "required_hardware_parameters" in methods,
            f"{class_name} hardware parameter hook is missing",
        )
        require(
            "initialize_node_resources" in methods,
            f"{class_name} resource initialization hook is missing",
        )

    main_function = next(
        (
            item
            for item in tree.body
            if isinstance(item, ast.FunctionDef) and item.name == "main"
        ),
        None,
    )
    require(main_function is not None, f"{package_name} main function is missing")
    require(
        any(
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "spin_node"
            for call in ast.walk(main_function)
        ),
        f"{package_name} main must use spin_node",
    )

launch = (
    SOURCE
    / "basic_packages"
    / "inspection_bringup"
    / "launch"
    / "inspection_system.launch.py"
).read_text(encoding="utf-8")
require(launch.count('namespace="inspection"') == 4, "node namespaces are incomplete")

print("inspection skeleton verification: PASS")
