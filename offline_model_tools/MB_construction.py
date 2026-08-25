#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정상 다중-view 이미지로 PatchCore 추론 전용 memory-bank artifact를 만든다.

실행 예:
    python3 MB_construction.py --artifact-name MB_1

사용자는 아래 '사용자 지정 설정'의 경로와 MB_CONFIGS를 먼저 수정해야 한다.
각 view는 서로 다른 CNN 입력 해상도와 PatchCore 파라미터를 사용할 수 있다.
"""

from __future__ import annotations

import argparse
import gc
from pathlib import Path
from typing import Any

import torch

from ad_common import (
    GpuProcessLock,
    PATCHCORE_ARTIFACT_FORMAT_VERSION,
    PATCHCORE_CAMERA_SERIAL_BY_VIEW,
    PATCHCORE_EXPECTED_VIEWS,
    PATCHCORE_STATION_VIEWS,
    PatchCoreViewModel,
    artifact_directory_sha256,
    benchmark_parallelism,
    export_patchcore_state,
    index_multiview_dataset,
    load_model_input,
    managed_output_directory,
    patchcore_select_coreset,
    percentile_threshold,
    require_supported_versions,
    resolve_cuda_device,
    set_reproducible_seed,
    sorted_product_names,
    torch_save_verified,
    validate_percentile_margin,
    validate_all_indexed_pngs,
    validate_artifact_manifest,
    validate_safe_name,
    verify_saved_parallelism_memory_safety,
    write_json,
)


# =============================================================================
# 사용자 지정 설정: 경로 및 안전 정책
# =============================================================================

# 이 폴더 바로 아래에 training_set, val_set, test_set_normal,
# test_set_anomaly 폴더가 있어야 합니다. 각 split 아래에는 view 폴더가 있습니다.
PROJECT_DIR = Path(__file__).resolve().parent
DATASET_ROOT = PROJECT_DIR / "data_set"

# 완성된 MB_1, MB_2 등의 artifact 폴더가 생성될 상위 경로입니다.
ARTIFACT_ROOT = PROJECT_DIR / "artifact"

# False이면 같은 artifact 이름이 존재할 때 즉시 중단합니다.
# True일 때만 완성된 새 artifact로 기존 폴더를 교체합니다.
OVERWRITE_EXISTING_ARTIFACT = True

# 서로 다른 프로그램 프로세스가 같은 CUDA GPU를 점유하지 않도록 기다리는 시간입니다.
GPU_LOCK_TIMEOUT_SECONDS = 300
# Lock 확인 재시도 간격입니다. 너무 작으면 불필요하게 CPU를 자주 깨웁니다.
GPU_LOCK_POLL_INTERVAL_SECONDS = 1

# 추론 시 GPU가 최소한으로 남겨야 할 VRAM입니다.
GPU_MEMORY_SAFETY_RESERVE_MB = 2048
# 후보별 timing 전에 CUDA kernel/allocator를 안정화하는 사전 반복 횟수입니다.
PARALLEL_WARMUP_RUNS = 3
# 후보 1/2/3 각각의 동일 workload 반복 횟수입니다(후보 개수가 아닙니다).
PARALLEL_BENCHMARK_RUNS = 10
# 더 높은 병렬 후보를 채택하기 위해 필요한 최소 median 개선율(%)입니다.
MIN_PARALLEL_SPEEDUP_PERCENT = 5.0
# 본 프로젝트에서 동시에 실행할 view forward의 상한입니다.
MAX_PARALLEL_VIEWS = 3


# =============================================================================
# 사용자 지정 설정: artifact별 PatchCore 파라미터
# =============================================================================

# 터미널의 --artifact-name 값으로 아래 항목 하나를 선택합니다.
# 같은 데이터로 설정이 다른 MB_1, MB_2 등을 독립적으로 만들 수 있습니다.
# parameters_by_view 아래의 각 view 설정은 완전히 독립적입니다. 한 view의 해상도,
# backbone/layer, coreset, k, margin 등을 바꿔도 다른 view에는 적용되지 않습니다.
MB_CONFIGS: dict[str, dict[str, Any]] = {
    "MB_resol_180_default": {
        # Dataset split 내부에 실제로 존재하는 view 폴더 이름입니다.
        # 순서는 실제 AD 추론 순서가 됩니다.
        "view_names": list(PATCHCORE_EXPECTED_VIEWS),

        "parameters_by_view": {
            "CAM_A_1": {
                "backbone": "resnet34",
                "feature_layers": [1, 2],
                "coreset_ratio": 0.1,
                "k": 9,
                "threshold_percentile": 100,
                "customized_margin": 0.02,
                "threshold_epsilon": 1e-8,
                "input_resolution": (180, 180),
                "resize_mode": "padding",
                "construction_batch_size": 1,
                "distance_chunk_size": 1024,
                "seed": 42,
            },
            "CAM_A_2": {
                "backbone": "resnet34",
                "feature_layers": [1, 2],
                "coreset_ratio": 0.1,
                "k": 9,
                "threshold_percentile": 100,
                "customized_margin": 0.02,
                "threshold_epsilon": 1e-8,
                "input_resolution": (180, 180),
                "resize_mode": "padding",
                "construction_batch_size": 1,
                "distance_chunk_size": 1024,
                "seed": 42,
            },
            "CAM_A_3": {
                "backbone": "resnet34",
                "feature_layers": [1, 2],
                "coreset_ratio": 0.1,
                "k": 9,
                "threshold_percentile": 100,
                "customized_margin": 0.02,
                "threshold_epsilon": 1e-8,
                "input_resolution": (180, 180),
                "resize_mode": "padding",
                "construction_batch_size": 1,
                "distance_chunk_size": 1024,
                "seed": 42,
            },
            "CAM_B_1": {
                "backbone": "resnet34",
                "feature_layers": [1, 2],
                "coreset_ratio": 0.1,
                "k": 9,
                "threshold_percentile": 100,
                "customized_margin": 0.02,
                "threshold_epsilon": 1e-8,
                "input_resolution": (180, 180),
                "resize_mode": "padding",
                "construction_batch_size": 1,
                "distance_chunk_size": 1024,
                "seed": 42,
            },
        },
    }
}

# Vision Node가 full-frame Mono8에서 crop_2를 만들 때 사용하는 view별 전처리
# 계약입니다. 데이터셋에는 동일 알고리즘으로 완성한 crop_2 PNG만 넣습니다.
# 조명 실험 후 필요한 view의 v_threshold만 독립적으로 조정할 수 있습니다.
PREPROCESSING_BY_VIEW: dict[str, dict[str, Any]] = {
    view: {
        "v_threshold": 40,
        "connectivity": 8,
        "remove_disconnected_noise": True,
        "check_connection": False,
    }
    for view in PATCHCORE_EXPECTED_VIEWS
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="다중-view PatchCore memory-bank artifact construction")
    parser.add_argument(
        "--artifact-name",
        action="append",
        required=True,
        help="MB_CONFIGS에서 선택할 이름. 여러 artifact는 이 옵션을 반복합니다. 예: --artifact-name MB_1 --artifact-name MB_2",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=DATASET_ROOT,
        help=f"training/validation dataset 상위 경로 (기본: {DATASET_ROOT})",
    )
    parser.add_argument(
        "--artifact-root",
        type=Path,
        default=ARTIFACT_ROOT,
        help=f"완성 artifact 상위 경로 (기본: {ARTIFACT_ROOT})",
    )
    return parser.parse_args()


def validate_config(config: dict[str, Any]) -> None:
    if set(config) != {"view_names", "parameters_by_view"}:
        raise ValueError(
            "MB config는 view_names와 parameters_by_view만 가져야 합니다."
        )
    if tuple(config["view_names"]) != PATCHCORE_EXPECTED_VIEWS:
        raise ValueError(
            f"운영 PatchCore view 순서는 {list(PATCHCORE_EXPECTED_VIEWS)}여야 합니다."
        )
    parameters_by_view = config["parameters_by_view"]
    if not isinstance(parameters_by_view, dict) or set(parameters_by_view) != set(
        PATCHCORE_EXPECTED_VIEWS
    ):
        raise ValueError("parameters_by_view는 운영 view 네 개를 정확히 포함해야 합니다.")
    required = {
        "backbone",
        "feature_layers",
        "coreset_ratio",
        "k",
        "threshold_percentile",
        "customized_margin",
        "threshold_epsilon",
        "input_resolution",
        "resize_mode",
        "construction_batch_size",
        "distance_chunk_size",
        "seed",
    }
    for view in PATCHCORE_EXPECTED_VIEWS:
        parameters = parameters_by_view[view]
        if not isinstance(parameters, dict) or set(parameters) != required:
            actual = sorted(parameters) if isinstance(parameters, dict) else type(parameters).__name__
            raise ValueError(
                f"{view} parameter key가 정확하지 않습니다. "
                f"필요={sorted(required)}, 실제={actual}"
            )
        numeric_fields = (
            "coreset_ratio",
            "threshold_percentile",
            "customized_margin",
            "threshold_epsilon",
        )
        integer_fields = (
            "k",
            "construction_batch_size",
            "distance_chunk_size",
            "seed",
        )
        if (
            type(parameters["backbone"]) is not str
            or type(parameters["resize_mode"]) is not str
            or any(type(parameters[field]) not in {int, float} for field in numeric_fields)
            or any(type(parameters[field]) is not int for field in integer_fields)
        ):
            raise ValueError(f"{view}: parameter type이 올바르지 않습니다.")
        validate_percentile_margin(
            parameters["threshold_percentile"],
            parameters["customized_margin"],
            parameters["threshold_epsilon"],
        )
        if not 0.0 < float(parameters["coreset_ratio"]) <= 1.0:
            raise ValueError(f"{view}: coreset_ratio는 0보다 크고 1 이하여야 합니다.")
        if int(parameters["k"]) < 1:
            raise ValueError(f"{view}: k는 1 이상이어야 합니다.")
        if int(parameters["construction_batch_size"]) < 1:
            raise ValueError(f"{view}: construction_batch_size는 1 이상이어야 합니다.")
        if parameters["backbone"] not in {"resnet34", "efficientnet_b0"}:
            raise ValueError(f"{view}: 지원하지 않는 backbone입니다.")
        allowed_layers = (
            {1, 2, 3, 4}
            if parameters["backbone"] == "resnet34"
            else set(range(1, 8))
        )
        layers = parameters["feature_layers"]
        if (
            not isinstance(layers, list)
            or not layers
            or any(type(layer) is not int or layer not in allowed_layers for layer in layers)
            or len(set(layers)) != len(layers)
        ):
            raise ValueError(f"{view}: feature_layers가 backbone과 호환되지 않습니다.")
        if parameters["resize_mode"] != "padding":
            raise ValueError(f"{view}: 운영 artifact는 resize_mode='padding'만 허용합니다.")
        if int(parameters["distance_chunk_size"]) < 1:
            raise ValueError(f"{view}: distance_chunk_size는 1 이상이어야 합니다.")
        if int(parameters["seed"]) < 0:
            raise ValueError(f"{view}: seed는 0 이상이어야 합니다.")
        if len(parameters["input_resolution"]) != 2 or any(
            type(value) is not int or value < 1
            for value in parameters["input_resolution"]
        ):
            raise ValueError(f"{view}: input_resolution은 양수 (height, width)여야 합니다.")


def construct_one_view(
    view: str,
    config: dict[str, Any],
    index: Any,
    device: torch.device,
    view_output_dir: Path,
) -> tuple[PatchCoreViewModel, float, list[float], tuple[int, int]]:
    """한 view의 embedding을 수집하고 coreset/validation threshold를 완성한다."""
    print(f"\n[PatchCore construction] view={view}")
    model = PatchCoreViewModel(
        backbone=config["backbone"],
        layer_numbers=config["feature_layers"],
        num_neighbors=config["k"],
        pretrained=True,
        distance_chunk_size=config["distance_chunk_size"],
    ).to(device)
    model.eval()

    training_names = sorted_product_names(index, "training_set")
    batch_size = int(config["construction_batch_size"])
    embeddings: list[torch.Tensor] = []
    for start in range(0, len(training_names), batch_size):
        names = training_names[start : start + batch_size]
        images = torch.stack(
            [
                load_model_input(
                    index.files["training_set"][view][name],
                    config["input_resolution"],
                    config["resize_mode"],
                )
                for name in names
            ],
        ).to(device, non_blocking=True)
        embeddings.append(model.generate_embedding(images))
        print(f"  feature extraction: {min(start + len(names), len(training_names))}/{len(training_names)}")

    full_embedding = torch.cat(embeddings, dim=0)
    del embeddings
    print(f"  full embeddings: {tuple(full_embedding.shape)}")
    model.memory_bank = patchcore_select_coreset(full_embedding, config["coreset_ratio"])
    del full_embedding
    torch.cuda.empty_cache()
    print(f"  coreset memory bank: {tuple(model.memory_bank.shape)}")

    validation_scores: list[float] = []
    for name in sorted_product_names(index, "val_set"):
        image = load_model_input(
            index.files["val_set"][view][name],
            config["input_resolution"],
            config["resize_mode"],
        )[None].to(device, non_blocking=True)
        validation_scores.append(float(model(image)[0].detach().cpu()))

    threshold = percentile_threshold(
        validation_scores,
        config["threshold_percentile"],
        config["threshold_epsilon"],
    )
    print(f"  validation threshold: {threshold:.12g}")

    view_output_dir.mkdir(parents=True, exist_ok=False)
    torch_save_verified(view_output_dir / "model.pt", export_patchcore_state(model))
    write_json(
        view_output_dir / "calibration.json",
        {
            "view": view,
            "threshold": threshold,
            "validation_raw_scores": validation_scores,
            "memory_bank_shape": list(model.memory_bank.shape),
        },
    )
    return model, threshold, validation_scores, tuple(int(value) for value in model.memory_bank.shape)


def construct_artifact(
    artifact_name: str, *, dataset_root: Path, artifact_root: Path
) -> None:
    config = dict(MB_CONFIGS[artifact_name])
    validate_config(config)
    versions = require_supported_versions()

    index = index_multiview_dataset(
        dataset_root,
        config["view_names"],
        required_splits=("training_set", "val_set"),
    )
    validate_all_indexed_pngs(index, require_mono8_compatible=True)
    # 현재 프로젝트 PC에는 CUDA GPU가 하나이므로 construction은 보수적으로 view별
    # 순차 실행한다. 추론용 stream 개수는 모든 view artifact 완성 뒤 별도 실측한다.
    print("Construction scheduling: sequential view construction on the single CUDA GPU")

    with GpuProcessLock(0, GPU_LOCK_TIMEOUT_SECONDS, GPU_LOCK_POLL_INTERVAL_SECONDS):
        device = resolve_cuda_device()
        with managed_output_directory(
            artifact_root,
            artifact_name,
            OVERWRITE_EXISTING_ARTIFACT,
        ) as build_dir:
            models: list[PatchCoreViewModel] = []
            thresholds: dict[str, float] = {}
            validation_scores: dict[str, list[float]] = {}
            bank_shapes: dict[str, tuple[int, int]] = {}

            for view in index.view_names:
                view_config = config["parameters_by_view"][view]
                set_reproducible_seed(int(view_config["seed"]))
                model, threshold, scores, bank_shape = construct_one_view(
                    view,
                    view_config,
                    index,
                    device,
                    build_dir / view,
                )
                # 다음 view construction의 full embedding과 겹치지 않도록 완성 모델은
                # CPU로 내렸다가 모든 view가 끝난 후 추론 benchmark 때 다시 올린다.
                models.append(model.to("cpu"))
                thresholds[view] = threshold
                validation_scores[view] = scores
                bank_shapes[view] = bank_shape
                torch.cuda.empty_cache()

            models = [model.to(device).eval() for model in models]
            sample_inputs = [
                load_model_input(
                    index.files["val_set"][view][sorted_product_names(index, "val_set")[0]],
                    config["parameters_by_view"][view]["input_resolution"],
                    config["parameters_by_view"][view]["resize_mode"],
                )[None].to(device)
                for view in index.view_names
            ]
            benchmark = benchmark_parallelism(
                models=models,
                sample_inputs=sample_inputs,
                reserve_mib=GPU_MEMORY_SAFETY_RESERVE_MB,
                warmup_runs=PARALLEL_WARMUP_RUNS,
                benchmark_runs=PARALLEL_BENCHMARK_RUNS,
                min_speedup_percent=MIN_PARALLEL_SPEEDUP_PERCENT,
                maximum_parallel_count=MAX_PARALLEL_VIEWS,
            )
            all_view_memory_safety = verify_saved_parallelism_memory_safety(
                models=models,
                sample_inputs=sample_inputs,
                parallel_count=benchmark.selected_parallel_count,
                reserve_mib=GPU_MEMORY_SAFETY_RESERVE_MB,
                warmup_runs=PARALLEL_WARMUP_RUNS,
            )
            if not all_view_memory_safety.memory_safe:
                raise RuntimeError(
                    "선택한 병렬도에서 Station A와 독립 설정 Station B를 모두 포함한 "
                    "VRAM reserve 검증에 실패했습니다."
                )
            benchmark_document = benchmark.to_dict()
            benchmark_document["all_view_memory_safety"] = (
                all_view_memory_safety.to_dict()
            )

            manifest = {
                "format_version": PATCHCORE_ARTIFACT_FORMAT_VERSION,
                "algorithm": "patchcore",
                "artifact_name": artifact_name,
                "model_version": artifact_name,
                "library_versions": versions,
                "view_names": list(index.view_names),
                "station_views": PATCHCORE_STATION_VIEWS,
                "camera_serial_by_view": PATCHCORE_CAMERA_SERIAL_BY_VIEW,
                "parameters_by_view": config["parameters_by_view"],
                "thresholds": thresholds,
                "validation_raw_scores": validation_scores,
                "memory_bank_shapes": {view: list(shape) for view, shape in bank_shapes.items()},
                "preprocessing_by_view": PREPROCESSING_BY_VIEW,
                "parallel_benchmark": benchmark_document,
            }
            write_json(build_dir / "manifest.json", manifest)
            validate_artifact_manifest(
                manifest,
                "patchcore",
                artifact_name,
                artifact_dir=build_dir,
            )
            del models, sample_inputs
            gc.collect()
            torch.cuda.empty_cache()
    completed = artifact_root.expanduser().resolve() / artifact_name
    print(f"\nPatchCore artifact construction complete: {completed}")
    print(f"Vision-compatible directory sha256: {artifact_directory_sha256(completed)}")


def main() -> None:
    args = parse_args()
    artifact_names = [validate_safe_name(name, "artifact name") for name in args.artifact_name]
    missing_names = [name for name in artifact_names if name not in MB_CONFIGS]
    if missing_names:
        raise KeyError(f"MB_CONFIGS에 다음 설정이 없습니다: {missing_names}")

    for position, artifact_name in enumerate(artifact_names, start=1):
        print(f"\n=== PatchCore artifact {position}/{len(artifact_names)}: {artifact_name} ===")
        construct_artifact(
            artifact_name,
            dataset_root=args.dataset_root,
            artifact_root=args.artifact_root,
        )


if __name__ == "__main__":
    main()
