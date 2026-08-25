#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""저장된 v2 PatchCore artifact로 데이터셋 성능 평가만 수행한다.

실행 예:
    python3 patchcore_AD.py --artifact-name MB_1

Memory bank 생성·threshold 산출은 하지 않으며 반드시 MB_construction.py가
완성한 artifact를 읽습니다. 터미널에서는 artifact 이름만 받습니다.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import torch

from ad_common import (
    GpuProcessLock,
    PatchCoreViewModel,
    artifact_directory_sha256,
    create_result_directory,
    evaluate_multiview_models,
    index_multiview_dataset,
    load_model_input,
    load_patchcore_state,
    read_json,
    require_supported_versions,
    resolve_cuda_device,
    sorted_product_names,
    validate_all_indexed_pngs,
    validate_artifact_manifest,
    validate_safe_name,
    verify_saved_parallelism_memory_safety,
    write_json,
)


# =============================================================================
# 사용자 지정 설정
# =============================================================================

# MB_construction.py의 DATASET_ROOT와 같은 데이터 상위 폴더입니다.
PROJECT_DIR = Path(__file__).resolve().parent
DATASET_ROOT = PROJECT_DIR / "data_set"

# MB_construction.py가 MB_1 등의 폴더를 만든 PatchCore artifact 상위 경로입니다.
ARTIFACT_ROOT = PROJECT_DIR / "artifact"

# 평가 결과는 이 경로 아래 result_MB_1 같은 이름으로 생성됩니다.
RESULT_PARENT_DIR = PROJECT_DIR / "result"

# False이면 result_MB_1이 이미 있을 때 중단합니다. True일 때만 교체합니다.
OVERWRITE_EXISTING_RESULT = True

# 서로 다른 프로그램 프로세스끼리 CUDA GPU 0을 동시에 점유하지 않게 합니다.
# 한 프로세스 내부의 view별 CUDA stream에는 이 lock이 적용되지 않습니다.
GPU_LOCK_TIMEOUT_SECONDS = 300
GPU_LOCK_POLL_INTERVAL_SECONDS = 1

# 한 제품 view의 CUDA event가 이 시간 안에 끝나지 않으면 무한 polling 대신
# TimeoutError로 중단합니다. 일반적인 millisecond 추론보다 충분히 큰 안전값입니다.
CUDA_EVENT_TIMEOUT_SECONDS = 60.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="다중-view PatchCore anomaly detection")
    parser.add_argument("--artifact-name", required=True, help="불러올 artifact 이름. 예: MB_1")
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=DATASET_ROOT,
        help=f"test dataset 상위 경로 (기본: {DATASET_ROOT})",
    )
    parser.add_argument(
        "--artifact-root",
        type=Path,
        default=ARTIFACT_ROOT,
        help=f"artifact 상위 경로 (기본: {ARTIFACT_ROOT})",
    )
    parser.add_argument(
        "--result-root",
        type=Path,
        default=RESULT_PARENT_DIR,
        help=f"성능시험 결과 상위 경로 (기본: {RESULT_PARENT_DIR})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    artifact_name = validate_safe_name(args.artifact_name, "artifact name")
    artifact_root = args.artifact_root.expanduser().resolve()
    dataset_root = args.dataset_root.expanduser().resolve()
    result_parent = args.result_root.expanduser().resolve()
    artifact_dir = artifact_root / artifact_name
    if not artifact_dir.is_dir():
        raise FileNotFoundError(f"PatchCore artifact 폴더가 없습니다: {artifact_dir}")

    manifest = read_json(artifact_dir / "manifest.json")
    validate_artifact_manifest(
        manifest,
        "patchcore",
        artifact_name,
        artifact_dir=artifact_dir,
    )
    artifact_sha256 = artifact_directory_sha256(artifact_dir)
    print(f"Vision-compatible artifact sha256: {artifact_sha256}")
    runtime_versions = require_supported_versions()
    parameters_by_view = manifest["parameters_by_view"]
    view_names = [str(value) for value in manifest["view_names"]]

    # Test split의 view별 파일명 집합과 모든 PNG 무결성을 긴 GPU 작업 전에 검사합니다.
    index = index_multiview_dataset(
        dataset_root,
        view_names,
        required_splits=("test_set_normal", "test_set_anomaly"),
    )
    validate_all_indexed_pngs(index, require_mono8_compatible=True)
    expected_result = result_parent / f"result_{artifact_name}"
    if expected_result.exists() and not OVERWRITE_EXISTING_RESULT:
        raise FileExistsError(f"결과 폴더가 이미 존재합니다: {expected_result}")
    with GpuProcessLock(0, GPU_LOCK_TIMEOUT_SECONDS, GPU_LOCK_POLL_INTERVAL_SECONDS):
        device = resolve_cuda_device()
        models: list[PatchCoreViewModel] = []
        loaded_shapes: dict[str, tuple[int, ...]] = {}
        for view in view_names:
            config = parameters_by_view[view]
            model = PatchCoreViewModel(
                backbone=config["backbone"],
                layer_numbers=config["feature_layers"],
                num_neighbors=config["k"],
                pretrained=False,
                distance_chunk_size=config["distance_chunk_size"],
            )
            payload = torch.load(
                artifact_dir / view / "model.pt",
                map_location="cpu",
                weights_only=True,
            )
            load_patchcore_state(model, payload)
            loaded_shapes[view] = tuple(int(value) for value in model.memory_bank.shape)
            expected_shape = tuple(
                int(value) for value in manifest["memory_bank_shapes"][view]
            )
            if loaded_shapes[view] != expected_shape:
                raise RuntimeError(
                    f"{view} memory bank shape가 manifest와 다릅니다: "
                    f"expected={expected_shape}, actual={loaded_shapes[view]}"
                )
            models.append(model.to(device).eval())

        first_name = sorted_product_names(index, "test_set_normal")[0]
        sample_inputs = [
            load_model_input(
                index.files["test_set_normal"][view][first_name],
                parameters_by_view[view]["input_resolution"],
                parameters_by_view[view]["resize_mode"],
            )[None].to(device)
            for view in view_names
        ]

        # Construction에서 결정한 stream 수는 유지한다. 실행 시에는 속도를 다시
        # 비교하지 않고, 저장된 수가 현재 free VRAM - reserve에서 안전한지만 확인한다.
        saved_benchmark = manifest["parallel_benchmark"]
        saved_count = int(saved_benchmark["selected_parallel_count"])
        runtime_memory_safety = verify_saved_parallelism_memory_safety(
            models=models,
            sample_inputs=sample_inputs,
            parallel_count=saved_count,
            reserve_mib=int(saved_benchmark["reserve_mib"]),
            warmup_runs=int(saved_benchmark["warmup_runs"]),
        )
        if not runtime_memory_safety.memory_safe:
            print("저장된 병렬 개수 VRAM 안전성 실패")
            raise RuntimeError(
                f"저장된 streams={saved_count}가 현재 free VRAM-reserve 조건을 통과하지 못했습니다.",
            )

        with create_result_directory(
            result_parent,
            artifact_name,
            OVERWRITE_EXISTING_RESULT,
        ) as result_dir:
            summary = evaluate_multiview_models(
                models=models,
                index=index,
                manifest=manifest,
                result_root=result_dir,
                device=device,
                parallel_count=saved_count,
                cuda_event_timeout_seconds=CUDA_EVENT_TIMEOUT_SECONDS,
            )
            write_json(
                result_dir / "run_information.json",
                {
                    "artifact_sha256": artifact_sha256,
                    "runtime_library_versions": runtime_versions,
                    "runtime_parallel_memory_safety": runtime_memory_safety.to_dict(),
                    "cuda_event_timeout_seconds": CUDA_EVENT_TIMEOUT_SECONDS,
                    "summary": summary,
                },
            )

    print(f"\nPatchCore evaluation complete: {result_parent / ('result_' + artifact_name)}")


if __name__ == "__main__":
    main()
