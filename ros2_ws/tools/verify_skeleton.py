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

launch = (
    SOURCE
    / "basic_packages"
    / "inspection_bringup"
    / "launch"
    / "inspection_system.launch.py"
).read_text(encoding="utf-8")
require(launch.count('namespace="inspection"') == 4, "node namespaces are incomplete")

print("inspection skeleton verification: PASS")
