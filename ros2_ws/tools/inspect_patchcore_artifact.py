#!/usr/bin/env python3
"""4-view PatchCore artifact v3를 CUDA 없이 검증하고 통합 SHA를 출력합니다."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

WORKSPACE = Path(__file__).parents[1]
for package_path in (
    WORKSPACE / "src" / "basic_packages" / "inspection_common",
    WORKSPACE / "src" / "nodes" / "inspection_vision",
):
    sys.path.insert(0, str(package_path))

from inspection_vision.model_backend import (  # noqa: E402
    ArtifactContractError,
    PatchCoreArtifactModel,
    artifact_directory_sha256,
)


SERIAL_TO_VIEW = {
    "DA9880512": "CAM_A_1",
    "DA9880516": "CAM_A_2",
    "DA7552836": "CAM_A_3",
    "DA7838410": "CAM_B_1",
}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="4-view PatchCore artifact v3 contract와 통합 SHA-256을 검사합니다."
    )
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--version", required=True)
    args = parser.parse_args()
    root = args.artifact.expanduser().resolve()
    try:
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        PatchCoreArtifactModel._validate_manifest(
            root,
            manifest,
            expected_version=args.version,
            serial_to_view=SERIAL_TO_VIEW,
        )
        digest = artifact_directory_sha256(root)
    except (ArtifactContractError, json.JSONDecodeError, OSError, ValueError) as exc:
        print(f"artifact contract failed: {exc}", file=sys.stderr)
        return 1
    print(f"artifact={root}")
    print(f"version={args.version}")
    print(f"sha256={digest}")
    print("contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
