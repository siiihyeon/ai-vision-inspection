#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Full-frame 데이터로 위치 정규화 PatchCore artifact v3를 만든다.

기존 ``MB_construction.py``의 v2 계약은 변경하지 않는다. 이 도구는 training,
calibration A/B, labeled validation을 분리하고 정상 제품 4-view OR FPR 1% 이하에서
product recall이 가장 높은 공간 정규화/aggregation 후보를 선택한다.
"""

from __future__ import annotations

import argparse
import gc
from pathlib import Path
from typing import Any

import numpy as np
import torch

from ad_common import (
    GpuProcessLock,
    PATCHCORE_CAMERA_SERIAL_BY_VIEW,
    PATCHCORE_EXPECTED_VIEWS,
    PATCHCORE_STATION_VIEWS,
    PatchCoreViewModel,
    artifact_directory_sha256,
    benchmark_parallelism,
    export_patchcore_state,
    managed_output_directory,
    patchcore_select_coreset,
    require_supported_versions,
    resolve_cuda_device,
    set_reproducible_seed,
    torch_save_verified,
    validate_safe_name,
    verify_saved_parallelism_memory_safety,
    write_json,
)
from patchcore_v3_common import (
    PATCHCORE_ARTIFACT_FORMAT_VERSION,
    PATCHCORE_V3_PARAMETER_KEYS,
    aggregation_candidates,
    candidate_sort_key,
    dataset_provenance,
    evaluate_product_scores,
    find_product_fpr_thresholds,
    index_v3_dataset,
    native_patch_maps_and_legacy_scores,
    normalization_candidates,
    preprocessing_pipeline_document,
    preprocess_full_frame,
    score_candidate,
    sorted_product_names,
    spatial_calibration_payload,
    validate_preprocessing_settings,
    validate_v3_dataset_pngs,
    validate_v3_manifest,
)


PROJECT_DIR = Path(__file__).resolve().parent
DATASET_ROOT = PROJECT_DIR / "data_set"
ARTIFACT_ROOT = PROJECT_DIR / "artifact"
OVERWRITE_EXISTING_ARTIFACT = True

GPU_LOCK_TIMEOUT_SECONDS = 300
GPU_LOCK_POLL_INTERVAL_SECONDS = 1
GPU_MEMORY_SAFETY_RESERVE_MB = 2048
PARALLEL_WARMUP_RUNS = 3
PARALLEL_BENCHMARK_RUNS = 10
MIN_PARALLEL_SPEEDUP_PERCENT = 5.0
MAX_PARALLEL_VIEWS = 3

TARGET_PRODUCT_FPR = 0.01
CALIBRATION_B_MINIMUM_PRODUCTS = 100
CALIBRATION_B_RECOMMENDED_PRODUCTS = 1000


MB_CONFIGS: dict[str, dict[str, Any]] = {
    "MB_v3_resol_180": {
        "view_names": list(PATCHCORE_EXPECTED_VIEWS),
        "parameters_by_view": {
            # 각 view는 독립 설정이다. 서로 다른 backbone, feature layer와
            # input resolution을 사용해도 된다.
            "CAM_A_1": {
                "backbone": "resnet34",
                "feature_layers": [1, 2],
                "coreset_ratio": 0.1,
                # v3 판정은 raw patch map을 사용한다. k는 기존 reweighted
                # PatchCore baseline과 construction 기록을 위해 보존한다.
                "k": 9,
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
                "input_resolution": (180, 180),
                "resize_mode": "padding",
                "construction_batch_size": 1,
                "distance_chunk_size": 1024,
                "seed": 42,
            },
        },
    }
}

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
    parser = argparse.ArgumentParser(description="PatchCore spatial artifact v3 construction")
    parser.add_argument("--artifact-name", action="append", required=True)
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--artifact-root", type=Path, default=ARTIFACT_ROOT)
    return parser.parse_args()


def validate_config(config: dict[str, Any]) -> None:
    if set(config) != {"view_names", "parameters_by_view"}:
        raise ValueError("v3 MB config는 view_names/parameters_by_view만 가져야 합니다.")
    if tuple(config["view_names"]) != PATCHCORE_EXPECTED_VIEWS:
        raise ValueError(f"v3 view 순서는 {list(PATCHCORE_EXPECTED_VIEWS)}여야 합니다.")
    parameters_by_view = config["parameters_by_view"]
    if not isinstance(parameters_by_view, dict) or set(parameters_by_view) != set(PATCHCORE_EXPECTED_VIEWS):
        raise ValueError("parameters_by_view는 운영 view 네 개를 정확히 포함해야 합니다.")
    for view in PATCHCORE_EXPECTED_VIEWS:
        parameters = parameters_by_view[view]
        if not isinstance(parameters, dict) or set(parameters) != PATCHCORE_V3_PARAMETER_KEYS:
            raise ValueError(f"{view}: v3 parameter key가 올바르지 않습니다.")
        backbone = parameters["backbone"]
        if backbone not in {"resnet34", "efficientnet_b0"}:
            raise ValueError(f"{view}: 지원하지 않는 backbone입니다.")
        allowed_layers = {1, 2, 3, 4} if backbone == "resnet34" else set(range(1, 8))
        layers = parameters["feature_layers"]
        if (
            not isinstance(layers, list)
            or not layers
            or any(type(layer) is not int or layer not in allowed_layers for layer in layers)
            or len(set(layers)) != len(layers)
        ):
            raise ValueError(f"{view}: feature_layers가 backbone과 호환되지 않습니다.")
        if not 0 < float(parameters["coreset_ratio"]) <= 1:
            raise ValueError(f"{view}: coreset_ratio가 올바르지 않습니다.")
        if type(parameters["k"]) is not int or parameters["k"] < 1:
            raise ValueError(f"{view}: k가 올바르지 않습니다.")
        if parameters["resize_mode"] != "padding":
            raise ValueError(f"{view}: resize_mode는 padding이어야 합니다.")
        resolution = parameters["input_resolution"]
        if len(resolution) != 2 or any(type(value) is not int or value < 1 for value in resolution):
            raise ValueError(f"{view}: input_resolution이 올바르지 않습니다.")
        for field in ("construction_batch_size", "distance_chunk_size"):
            if type(parameters[field]) is not int or parameters[field] < 1:
                raise ValueError(f"{view}: {field}가 올바르지 않습니다.")
        if type(parameters["seed"]) is not int or parameters["seed"] < 0:
            raise ValueError(f"{view}: seed가 올바르지 않습니다.")
        validate_preprocessing_settings(view, PREPROCESSING_BY_VIEW[view])


def load_input(path: Path, view: str, config: dict[str, Any]) -> torch.Tensor:
    return preprocess_full_frame(
        path,
        view=view,
        settings=PREPROCESSING_BY_VIEW[view],
        resolution=config["input_resolution"],
        resize_mode=config["resize_mode"],
    ).tensor


def construct_memory_bank(
    *, view: str, config: dict[str, Any], index: Any, device: torch.device
) -> PatchCoreViewModel:
    print(f"\n[PatchCore v3 memory bank] view={view}")
    model = PatchCoreViewModel(
        backbone=config["backbone"],
        layer_numbers=config["feature_layers"],
        num_neighbors=config["k"],
        pretrained=True,
        distance_chunk_size=config["distance_chunk_size"],
    ).to(device).eval()
    names = sorted_product_names(index, "training_set")
    batch_size = int(config["construction_batch_size"])
    embeddings: list[torch.Tensor] = []
    for start in range(0, len(names), batch_size):
        batch_names = names[start : start + batch_size]
        images = torch.stack(
            [load_input(index.files["training_set"][view][name], view, config) for name in batch_names]
        ).to(device, non_blocking=True)
        embeddings.append(model.generate_embedding(images))
        print(f"  feature extraction: {min(start + len(batch_names), len(names))}/{len(names)}")
    full_embedding = torch.cat(embeddings, dim=0)
    del embeddings
    model.memory_bank = patchcore_select_coreset(full_embedding, config["coreset_ratio"])
    del full_embedding
    torch.cuda.empty_cache()
    print(f"  coreset memory bank: {tuple(model.memory_bank.shape)}")
    return model


def collect_split_maps(
    *,
    model: PatchCoreViewModel,
    index: Any,
    split: str,
    view: str,
    config: dict[str, Any],
    device: torch.device,
) -> tuple[np.ndarray, np.ndarray]:
    names = sorted_product_names(index, split)
    collected: list[np.ndarray] = []
    legacy_collected: list[np.ndarray] = []
    batch_size = int(config["construction_batch_size"])
    for start in range(0, len(names), batch_size):
        batch_names = names[start : start + batch_size]
        images = torch.stack(
            [load_input(index.files[split][view][name], view, config) for name in batch_names]
        ).to(device, non_blocking=True)
        maps, legacy_scores = native_patch_maps_and_legacy_scores(model, images)
        collected.append(maps.detach().cpu().numpy().astype(np.float32))
        legacy_collected.append(legacy_scores.detach().cpu().numpy().astype(np.float64))
    result = np.concatenate(collected, axis=0)
    print(f"  {split}: maps={tuple(result.shape)}")
    return result, np.concatenate(legacy_collected, axis=0)


def construct_artifact(artifact_name: str, *, dataset_root: Path, artifact_root: Path) -> None:
    config = dict(MB_CONFIGS[artifact_name])
    validate_config(config)
    versions = require_supported_versions()
    construction_splits = (
        "training_set",
        "calibration_set_A",
        "calibration_set_B",
        "validation_normal",
        "validation_anomaly",
    )
    index = index_v3_dataset(dataset_root, config["view_names"], construction_splits)
    validate_v3_dataset_pngs(index)
    calibration_b_count = len(sorted_product_names(index, "calibration_set_B"))
    if calibration_b_count < CALIBRATION_B_MINIMUM_PRODUCTS:
        raise ValueError(
            f"calibration_set_B는 {CALIBRATION_B_MINIMUM_PRODUCTS}개 이상이어야 합니다: "
            f"actual={calibration_b_count}"
        )
    if calibration_b_count < CALIBRATION_B_RECOMMENDED_PRODUCTS:
        print(
            "WARNING: 1% product FPR 안정성을 위해 calibration_set_B 1,000개 이상을 "
            f"권장합니다. actual={calibration_b_count}"
        )
    provenance = dataset_provenance(index)

    with GpuProcessLock(0, GPU_LOCK_TIMEOUT_SECONDS, GPU_LOCK_POLL_INTERVAL_SECONDS):
        device = resolve_cuda_device()
        with managed_output_directory(artifact_root, artifact_name, OVERWRITE_EXISTING_ARTIFACT) as build_dir:
            models: dict[str, PatchCoreViewModel] = {}
            raw_maps: dict[str, dict[str, np.ndarray]] = {
                split: {} for split in construction_splits if split != "training_set"
            }
            legacy_scores: dict[str, dict[str, np.ndarray]] = {
                split: {} for split in construction_splits if split != "training_set"
            }
            memory_bank_shapes: dict[str, list[int]] = {}
            patch_grid_shapes: dict[str, list[int]] = {}

            for view in index.view_names:
                view_config = config["parameters_by_view"][view]
                set_reproducible_seed(int(view_config["seed"]))
                model = construct_memory_bank(view=view, config=view_config, index=index, device=device)
                for split in raw_maps:
                    raw_maps[split][view], legacy_scores[split][view] = collect_split_maps(
                        model=model,
                        index=index,
                        split=split,
                        view=view,
                        config=view_config,
                        device=device,
                    )
                grid = raw_maps["calibration_set_A"][view].shape[1:]
                if any(tuple(raw_maps[split][view].shape[1:]) != tuple(grid) for split in raw_maps):
                    raise RuntimeError(f"{view}: split별 native patch grid가 다릅니다.")
                patch_grid_shapes[view] = [int(grid[0]), int(grid[1])]
                memory_bank_shapes[view] = [int(value) for value in model.memory_bank.shape]
                models[view] = model.to("cpu")
                torch.cuda.empty_cache()

            legacy_percentile, legacy_thresholds, legacy_calibration_fpr = (
                find_product_fpr_thresholds(
                    legacy_scores["calibration_set_B"], TARGET_PRODUCT_FPR
                )
            )
            candidate_records: list[dict[str, Any]] = [
                {
                    "candidate_id": "baseline-legacy-patchcore",
                    "eligible_for_selection": False,
                    "status": "valid",
                    "normalization": {"method": "legacy_reweighted_image_score"},
                    "aggregation": {"method": "legacy_max_patch_reweighting"},
                    "threshold_percentile": legacy_percentile,
                    "thresholds": legacy_thresholds,
                    "calibration_product_fpr": legacy_calibration_fpr,
                    "validation_metrics": evaluate_product_scores(
                        legacy_scores["validation_normal"],
                        legacy_scores["validation_anomaly"],
                        legacy_thresholds,
                    ),
                }
            ]
            candidate_number = 0
            normalization_set = [({"method": "none"}, False)] + [
                (candidate, True) for candidate in normalization_candidates()
            ]
            for normalization, eligible in normalization_set:
                for aggregation in aggregation_candidates():
                    candidate_number += 1
                    candidate_id = f"candidate-{candidate_number:03d}"
                    try:
                        record, _, _ = score_candidate(
                            candidate_id=candidate_id,
                            normalization=normalization,
                            aggregation=aggregation,
                            calibration_a_maps_by_view=raw_maps["calibration_set_A"],
                            calibration_b_maps_by_view=raw_maps["calibration_set_B"],
                            validation_normal_maps_by_view=raw_maps["validation_normal"],
                            validation_anomaly_maps_by_view=raw_maps["validation_anomaly"],
                            target_product_fpr=TARGET_PRODUCT_FPR,
                            eligible_for_selection=eligible,
                        )
                    except (RuntimeError, ValueError) as exc:
                        record = {
                            "candidate_id": candidate_id,
                            "eligible_for_selection": bool(eligible),
                            "normalization": normalization,
                            "aggregation": aggregation,
                            "status": "invalid",
                            "error": str(exc),
                        }
                    else:
                        record["status"] = "valid"
                    candidate_records.append(record)
                    print(
                        f"  candidate {candidate_number}: {normalization} + {aggregation} "
                        f"-> {record['status']}"
                    )

            eligible_records = [
                record
                for record in candidate_records
                if record.get("status") == "valid"
                and record.get("eligible_for_selection")
                and float(record["validation_metrics"]["fpr"]) <= TARGET_PRODUCT_FPR
            ]
            if not eligible_records:
                raise RuntimeError("validation product FPR 1% 이하를 만족하는 후보가 없습니다.")
            selected_record = min(eligible_records, key=candidate_sort_key)
            selected_id = str(selected_record["candidate_id"])
            print(f"Selected spatial candidate: {selected_id}")
            repeated, selected_calibrations, selected_b_scores = score_candidate(
                candidate_id=selected_id,
                normalization=selected_record["normalization"],
                aggregation=selected_record["aggregation"],
                calibration_a_maps_by_view=raw_maps["calibration_set_A"],
                calibration_b_maps_by_view=raw_maps["calibration_set_B"],
                validation_normal_maps_by_view=raw_maps["validation_normal"],
                validation_anomaly_maps_by_view=raw_maps["validation_anomaly"],
                target_product_fpr=TARGET_PRODUCT_FPR,
                eligible_for_selection=True,
            )
            if repeated["thresholds"] != selected_record["thresholds"]:
                raise RuntimeError("선택 후보 재계산 threshold가 달라졌습니다.")

            models_gpu = [models[view].to(device).eval() for view in index.view_names]
            first_name = sorted_product_names(index, "validation_normal")[0]
            sample_inputs = [
                load_input(
                    index.files["validation_normal"][view][first_name],
                    view,
                    config["parameters_by_view"][view],
                )[None].to(device)
                for view in index.view_names
            ]
            benchmark = benchmark_parallelism(
                models=models_gpu,
                sample_inputs=sample_inputs,
                reserve_mib=GPU_MEMORY_SAFETY_RESERVE_MB,
                warmup_runs=PARALLEL_WARMUP_RUNS,
                benchmark_runs=PARALLEL_BENCHMARK_RUNS,
                min_speedup_percent=MIN_PARALLEL_SPEEDUP_PERCENT,
                maximum_parallel_count=MAX_PARALLEL_VIEWS,
            )
            memory_safety = verify_saved_parallelism_memory_safety(
                models=models_gpu,
                sample_inputs=sample_inputs,
                parallel_count=benchmark.selected_parallel_count,
                reserve_mib=GPU_MEMORY_SAFETY_RESERVE_MB,
                warmup_runs=PARALLEL_WARMUP_RUNS,
            )
            if not memory_safety.memory_safe:
                raise RuntimeError("선택한 병렬도에서 VRAM reserve 검증에 실패했습니다.")
            benchmark_document = benchmark.to_dict()
            benchmark_document["all_view_memory_safety"] = memory_safety.to_dict()

            thresholds = {view: float(selected_record["thresholds"][view]) for view in index.view_names}
            for position, view in enumerate(index.view_names):
                view_dir = build_dir / view
                view_dir.mkdir(parents=True, exist_ok=False)
                model_cpu = models_gpu[position].to("cpu")
                torch_save_verified(view_dir / "model.pt", export_patchcore_state(model_cpu))
                torch_save_verified(
                    view_dir / "spatial_calibration.pt",
                    spatial_calibration_payload(selected_calibrations[view]),
                )
                write_json(
                    view_dir / "calibration.json",
                    {
                        "view": view,
                        "threshold": thresholds[view],
                        "threshold_percentile": float(selected_record["threshold_percentile"]),
                        "calibration_b_scores": selected_b_scores[view].tolist(),
                        "memory_bank_shape": memory_bank_shapes[view],
                        "patch_grid_shape": patch_grid_shapes[view],
                        "normalization": selected_record["normalization"],
                        "aggregation": selected_record["aggregation"],
                        "calibration_a_sample_count": selected_calibrations[view].sample_count,
                        "scale_reference": selected_calibrations[view].scale_reference,
                    },
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
                "preprocessing_pipeline": preprocessing_pipeline_document(),
                "preprocessing_by_view": PREPROCESSING_BY_VIEW,
                "spatial_scoring_policy": {
                    "normalization": selected_record["normalization"],
                    "aggregation": selected_record["aggregation"],
                    "decision": {
                        "target_product_fpr": TARGET_PRODUCT_FPR,
                        "threshold_percentile": float(selected_record["threshold_percentile"]),
                        "comparison": "strict_greater_than",
                        "view_score_normalization": "divide_by_threshold",
                    },
                },
                "thresholds": thresholds,
                "calibration_b_scores": {
                    view: selected_b_scores[view].tolist() for view in index.view_names
                },
                "memory_bank_shapes": memory_bank_shapes,
                "patch_grid_shapes": patch_grid_shapes,
                "candidate_selection": {
                    "selection_order": [
                        "validation_product_fpr_le_0.01",
                        "maximum_product_recall",
                        "minimum_product_fpr",
                        "minimum_validation_postprocess_p95_ms",
                        "minimum_normalization_complexity",
                    ],
                    "selected_candidate_id": selected_id,
                    "selected_validation_metrics": selected_record["validation_metrics"],
                    "candidates": candidate_records,
                },
                "dataset_provenance": provenance,
                "calibration_b_sample_policy": {
                    "minimum": CALIBRATION_B_MINIMUM_PRODUCTS,
                    "recommended": CALIBRATION_B_RECOMMENDED_PRODUCTS,
                    "actual": calibration_b_count,
                },
                "parallel_benchmark": benchmark_document,
            }
            write_json(build_dir / "manifest.json", manifest)
            validate_v3_manifest(manifest, artifact_name, build_dir)
            del models_gpu, models, sample_inputs, raw_maps, legacy_scores
            gc.collect()
            torch.cuda.empty_cache()

    completed = artifact_root.expanduser().resolve() / artifact_name
    print(f"\nPatchCore artifact v3 construction complete: {completed}")
    print(f"Vision-compatible directory sha256: {artifact_directory_sha256(completed)}")


def main() -> None:
    args = parse_args()
    artifact_names = [validate_safe_name(name, "artifact name") for name in args.artifact_name]
    missing = [name for name in artifact_names if name not in MB_CONFIGS]
    if missing:
        raise KeyError(f"MB_CONFIGS에 없는 설정입니다: {missing}")
    for position, artifact_name in enumerate(artifact_names, start=1):
        print(f"\n=== PatchCore v3 artifact {position}/{len(artifact_names)}: {artifact_name} ===")
        construct_artifact(
            artifact_name,
            dataset_root=args.dataset_root,
            artifact_root=args.artifact_root,
        )


if __name__ == "__main__":
    main()
