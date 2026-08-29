#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""고정된 PatchCore artifact v3를 full-frame test set에서 최종 평가한다.

이 도구는 model/calibration/threshold를 변경하지 않는다. 기존
``patchcore_AD.py``의 v2 평가 계약과 혼동되지 않도록 별도 파일로 제공한다.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import time
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np
import torch

from ad_common import (
    GpuProcessLock,
    PatchCoreViewModel,
    artifact_directory_sha256,
    create_result_directory,
    load_patchcore_state,
    read_json,
    require_supported_versions,
    resolve_cuda_device,
    save_confusion_matrix,
    validate_safe_name,
    verify_saved_parallelism_memory_safety,
    write_json,
)
from patchcore_v3_common import (
    dataset_provenance,
    index_v3_dataset,
    native_patch_maps,
    preprocess_full_frame,
    sorted_product_names,
    spatial_view_scores_torch,
    top_k_percent_patch_count,
    validate_spatial_payload,
    validate_v3_dataset_pngs,
    validate_v3_manifest,
)


PROJECT_DIR = Path(__file__).resolve().parent
DATASET_ROOT = PROJECT_DIR / "data_set"
ARTIFACT_ROOT = PROJECT_DIR / "artifact"
RESULT_PARENT_DIR = PROJECT_DIR / "result"
OVERWRITE_EXISTING_RESULT = True
GPU_LOCK_TIMEOUT_SECONDS = 300
GPU_LOCK_POLL_INTERVAL_SECONDS = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PatchCore artifact v3 final test")
    parser.add_argument("--artifact-name", required=True)
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--artifact-root", type=Path, default=ARTIFACT_ROOT)
    parser.add_argument("--result-root", type=Path, default=RESULT_PARENT_DIR)
    return parser.parse_args()


def _timing_summary(values: Sequence[float]) -> dict[str, float | int]:
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0 or not np.all(np.isfinite(array)):
        raise RuntimeError("timing sample이 비어 있거나 NaN/Inf를 포함합니다.")
    return {
        "count": int(array.size),
        "mean_ms": float(array.mean()),
        "median_ms": float(np.percentile(array, 50, method="linear")),
        "p95_ms": float(np.percentile(array, 95, method="linear")),
        "p99_ms": float(np.percentile(array, 99, method="linear")),
        "min_ms": float(array.min()),
        "max_ms": float(array.max()),
    }


def _classification_metrics(
    matrix_values: Sequence[int], sample_count: int
) -> dict[str, Any]:
    if len(matrix_values) != 4 or sample_count < 1:
        raise ValueError("confusion matrix 또는 sample count가 올바르지 않습니다.")
    tn, fp, fn, tp = (int(value) for value in matrix_values)
    if min(tn, fp, fn, tp) < 0 or tn + fp + fn + tp != sample_count:
        raise ValueError("confusion matrix count가 test sample 수와 다릅니다.")
    recall = tp / (tp + fn) if tp + fn else 0.0
    precision = tp / (tp + fp) if tp + fp else 0.0
    return {
        "confusion_matrix": {
            "labels": ["normal", "anomaly"],
            "matrix": [[tn, fp], [fn, tp]],
        },
        "tn": tn,
        "fp": fp,
        "fn": fn,
        "tp": tp,
        "accuracy": (tn + tp) / sample_count,
        "f1": 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0,
        "recall": recall,
        "precision": precision,
        "fpr": fp / (fp + tn) if fp + tn else 0.0,
    }


def _save_rgb(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if not ok:
        raise RuntimeError(f"PNG encoding failed: {path}")
    encoded.tofile(str(path))


def _tensor_rgb(tensor: torch.Tensor) -> np.ndarray:
    value = tensor.detach().cpu().clamp(0, 1)
    return value.mul(255).round().byte().permute(1, 2, 0).numpy()


def _save_normalized_heatmap(
    path: Path,
    tensor: torch.Tensor,
    normalized_map: np.ndarray,
    *,
    view_score: float,
    threshold: float,
    ratio_score: float,
    prediction: bool,
) -> None:
    image = _tensor_rgb(tensor)
    values = cv2.resize(
        normalized_map.astype(np.float32),
        (image.shape[1], image.shape[0]),
        interpolation=cv2.INTER_LINEAR,
    )
    # 표시 scale만 calibration threshold에 맞춘다. blur는 판정에 사용하지 않는다.
    high = max(float(threshold), float(np.percentile(values, 99.9)), 1e-6)
    low = min(0.0, float(np.percentile(values, 1.0)))
    scaled = np.clip((values - low) / max(high - low, 1e-6), 0.0, 1.0)
    heat_u8 = np.rint(scaled * 255).astype(np.uint8)
    heat = cv2.cvtColor(cv2.applyColorMap(heat_u8, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
    overlay = cv2.addWeighted(image, 0.55, heat, 0.45, 0)
    banner = np.zeros((42, overlay.shape[1], 3), dtype=np.uint8)
    verdict = "ANOMALY" if prediction else "NORMAL"
    cv2.putText(
        banner,
        f"view={view_score:.6f} threshold={threshold:.6f}",
        (4, 17),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        banner,
        f"ratio={ratio_score:.6f} strict>1 {verdict}",
        (4, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    _save_rgb(path, heat)
    _save_rgb(
        path.with_name(path.name.replace("_heatmap.png", "_overlay.png")),
        np.concatenate((banner, overlay), axis=0),
    )


def _copy_original(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _selected_policy_document(manifest: dict[str, Any]) -> dict[str, Any]:
    selection = manifest["candidate_selection"]
    policy = manifest["spatial_scoring_policy"]
    aggregation = dict(policy["aggregation"])
    effective_patch_counts: dict[str, int] = {}
    if aggregation["method"] == "top_k_percent_average":
        for view in manifest["view_names"]:
            grid_h, grid_w = manifest["patch_grid_shapes"][view]
            effective_patch_counts[view] = top_k_percent_patch_count(
                int(grid_h) * int(grid_w),
                float(aggregation["top_k_percent"]),
            )
    return {
        "selected_candidate_id": selection["selected_candidate_id"],
        "normalization": dict(policy["normalization"]),
        "aggregation": aggregation,
        "effective_top_k_patch_counts_by_view": effective_patch_counts,
        "decision": dict(policy["decision"]),
        "thresholds_by_view": dict(manifest["thresholds"]),
        "patch_grid_shapes_by_view": dict(manifest["patch_grid_shapes"]),
        "parameters_by_view": dict(manifest["parameters_by_view"]),
        "preprocessing_by_view": dict(manifest["preprocessing_by_view"]),
    }


def _print_startup_summary(
    *,
    artifact_name: str,
    artifact_sha256: str,
    manifest: dict[str, Any],
) -> dict[str, Any]:
    selected_policy = _selected_policy_document(manifest)
    print("\n=== PatchCore v3 artifact test configuration ===")
    print(f"Artifact: {artifact_name}")
    print(f"Artifact SHA-256: {artifact_sha256}")
    print("Selected policy and parameters:")
    print(json.dumps(selected_policy, ensure_ascii=False, indent=2, sort_keys=True))
    print("================================================\n")
    return selected_policy


def _save_score_plot(
    output: Path,
    calibration: Sequence[float],
    normal: Sequence[float],
    anomaly: Sequence[float],
    threshold: float,
    view: str,
) -> None:
    from matplotlib import pyplot as plt

    groups = [np.asarray(calibration), np.asarray(normal), np.asarray(anomaly)]
    combined = np.concatenate(groups)
    bins = max(10, min(80, int(math.sqrt(combined.size)) * 2))
    figure, axis = plt.subplots(figsize=(10, 6))
    axis.hist(groups, bins=bins, alpha=0.45, label=["calibration B normal", "test normal", "test anomaly"])
    axis.axvline(threshold, color="red", linestyle="--", label=f"strict > {threshold:.6g}")
    axis.set_title(f"Spatial PatchCore view score - {view}")
    axis.set_xlabel("Aggregated spatially normalized view score")
    axis.set_ylabel("Image count")
    axis.grid(alpha=0.2)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)


@torch.inference_mode()
def _infer_station(
    *,
    view_names: Sequence[str],
    source_paths: Sequence[Path],
    models: dict[str, PatchCoreViewModel],
    spatial: dict[str, dict[str, torch.Tensor]],
    manifest: dict[str, Any],
    device: torch.device,
    streams: Sequence[torch.cuda.Stream],
    parallel_count: int,
) -> dict[str, Any]:
    end_to_end_started = time.perf_counter()
    prepared = [
        preprocess_full_frame(
            path,
            view=view,
            settings=manifest["preprocessing_by_view"][view],
            resolution=manifest["parameters_by_view"][view]["input_resolution"],
            resize_mode=manifest["parameters_by_view"][view]["resize_mode"],
        )
        for view, path in zip(view_names, source_paths, strict=True)
    ]
    gpu_inputs = [item.tensor[None].to(device, non_blocking=True) for item in prepared]
    torch.cuda.synchronize(device)

    model_pipeline_started = time.perf_counter()
    raw_maps: list[torch.Tensor] = []
    model_forward_started = time.perf_counter()
    active_parallel = min(int(parallel_count), len(view_names))
    for start in range(0, len(view_names), active_parallel):
        wave_views = view_names[start : start + active_parallel]
        wave_inputs = gpu_inputs[start : start + active_parallel]
        for slot, (view, image) in enumerate(zip(wave_views, wave_inputs, strict=True)):
            with torch.cuda.stream(streams[slot]):
                maps = native_patch_maps(models[view], image)
                raw_maps.append(maps)
        for stream in streams[: len(wave_views)]:
            stream.synchronize()
    model_forward_only_ms = (time.perf_counter() - model_forward_started) * 1000.0
    outputs = [
        spatial_view_scores_torch(
            maps,
            spatial[view]["center"],
            spatial[view]["denominator"],
            manifest["spatial_scoring_policy"]["aggregation"],
        )
        for view, maps in zip(view_names, raw_maps, strict=True)
    ]
    view_scores = [float(output[0].detach().cpu()) for output in outputs]
    thresholds = [float(manifest["thresholds"][view]) for view in view_names]
    ratios = [score / threshold for score, threshold in zip(view_scores, thresholds, strict=True)]
    predictions = [score > threshold for score, threshold in zip(view_scores, thresholds, strict=True)]
    model_pipeline_ms = (time.perf_counter() - model_pipeline_started) * 1000.0
    end_to_end_ms = (time.perf_counter() - end_to_end_started) * 1000.0
    normalized_maps = [
        ((maps[0] - spatial[view]["center"]) / spatial[view]["denominator"]).detach().cpu().numpy()
        for view, maps in zip(view_names, raw_maps, strict=True)
    ]
    return {
        "prepared": prepared,
        "view_scores": view_scores,
        "ratios": ratios,
        "predictions": predictions,
        "normalized_maps": normalized_maps,
        "model_forward_only_ms": model_forward_only_ms,
        "model_pipeline_ms": model_pipeline_ms,
        "end_to_end_ms": end_to_end_ms,
        "station_prediction": bool(any(predictions)),
    }


def evaluate(
    *,
    models: dict[str, PatchCoreViewModel],
    spatial: dict[str, dict[str, torch.Tensor]],
    index: Any,
    manifest: dict[str, Any],
    result_root: Path,
    device: torch.device,
    parallel_count: int,
) -> dict[str, Any]:
    views = tuple(manifest["view_names"])
    station_views = (
        tuple(manifest["station_views"]["station_a"]),
        tuple(manifest["station_views"]["station_b"]),
    )
    streams = [torch.cuda.Stream(device=device) for _ in range(max(3, parallel_count))]
    labels: list[int] = []
    predictions: list[int] = []
    records: list[dict[str, Any]] = []
    station_forward: dict[str, list[float]] = {"station_a": [], "station_b": []}
    station_pipeline: dict[str, list[float]] = {"station_a": [], "station_b": []}
    station_end_to_end: dict[str, list[float]] = {"station_a": [], "station_b": []}
    view_scores_by_label: dict[str, dict[str, list[float]]] = {
        view: {"normal": [], "anomaly": []} for view in views
    }
    split_specs = (("test_normal", 0, "normal"), ("test_anomaly", 1, "anomaly"))
    for split, label, label_name in split_specs:
        names = sorted_product_names(index, split)
        for number, filename in enumerate(names, start=1):
            station_results: list[dict[str, Any]] = []
            for station_position, current_views in enumerate(station_views):
                paths = [index.files[split][view][filename] for view in current_views]
                station_results.append(
                    _infer_station(
                        view_names=current_views,
                        source_paths=paths,
                        models=models,
                        spatial=spatial,
                        manifest=manifest,
                        device=device,
                        streams=streams,
                        parallel_count=parallel_count,
                    )
                )
                station_name = "station_a" if station_position == 0 else "station_b"
                station_forward[station_name].append(
                    station_results[-1]["model_forward_only_ms"]
                )
                station_pipeline[station_name].append(
                    station_results[-1]["model_pipeline_ms"]
                )
                station_end_to_end[station_name].append(station_results[-1]["end_to_end_ms"])

            product_prediction = bool(any(result["station_prediction"] for result in station_results))
            labels.append(label)
            predictions.append(int(product_prediction))
            view_document: dict[str, Any] = {}
            for current_views, station_result in zip(station_views, station_results, strict=True):
                for position, view in enumerate(current_views):
                    score = station_result["view_scores"][position]
                    ratio = station_result["ratios"][position]
                    prediction = station_result["predictions"][position]
                    view_scores_by_label[view][label_name].append(score)
                    view_document[view] = {
                        "view_score": score,
                        "threshold": float(manifest["thresholds"][view]),
                        "threshold_ratio_score": ratio,
                        "is_anomaly": bool(prediction),
                    }
            records.append(
                {
                    "filename": filename,
                    "ground_truth": label_name,
                    "product_is_anomaly": product_prediction,
                    "station_a_model_forward_only_ms": station_results[0][
                        "model_forward_only_ms"
                    ],
                    "station_a_model_pipeline_ms": station_results[0]["model_pipeline_ms"],
                    "station_a_end_to_end_ms": station_results[0]["end_to_end_ms"],
                    "station_b_model_forward_only_ms": station_results[1][
                        "model_forward_only_ms"
                    ],
                    "station_b_model_pipeline_ms": station_results[1]["model_pipeline_ms"],
                    "station_b_end_to_end_ms": station_results[1]["end_to_end_ms"],
                    "views": view_document,
                }
            )
            category = {
                (0, 0): "true-negative",
                (0, 1): "false-positive",
                (1, 0): "false-negative",
                (1, 1): "true-positive",
            }[(label, int(product_prediction))]
            product_dir = result_root / category / Path(filename).stem
            metadata: dict[str, Any] = {
                "filename": filename,
                "ground_truth": label_name,
                "product_is_anomaly": product_prediction,
                "outcome": category,
                "views": {},
            }
            for current_views, station_result in zip(station_views, station_results, strict=True):
                for position, view in enumerate(current_views):
                    prepared = station_result["prepared"][position]
                    score = station_result["view_scores"][position]
                    threshold = float(manifest["thresholds"][view])
                    ratio = station_result["ratios"][position]
                    predicted = bool(station_result["predictions"][position])
                    _copy_original(
                        index.files[split][view][filename],
                        product_dir / f"{view}_original.png",
                    )
                    _save_rgb(
                        product_dir / f"{view}_model_input.png",
                        _tensor_rgb(prepared.tensor),
                    )
                    _save_normalized_heatmap(
                        product_dir / f"{view}_normalized_heatmap.png",
                        prepared.tensor,
                        station_result["normalized_maps"][position],
                        view_score=score,
                        threshold=threshold,
                        ratio_score=ratio,
                        prediction=predicted,
                    )
                    metadata["views"][view] = {
                        "view_score": score,
                        "threshold": threshold,
                        "threshold_ratio_score": ratio,
                        "strict_greater_than_prediction": predicted,
                    }
            write_json(product_dir / "scores.json", metadata)
            print(f"  {split}: {number}/{len(names)} {filename} -> {'anomaly' if product_prediction else 'normal'}")

    matrix_values = save_confusion_matrix(result_root / "confusion_matrix.png", labels, predictions)
    metrics = _classification_metrics(matrix_values, len(labels))
    timing = {
        "definition": {
            "model_forward_only": "GPU input ready through native PatchCore map completion",
            "entire_model_pipeline": "GPU input ready through native map, spatial normalization, aggregation and verdict",
            "end_to_end": "full-frame PNG read through preprocessing, transfer, model pipeline and verdict",
            "excluded": ["camera capture", "physical station travel", "diagnostic image persistence", "durable logging"],
        },
        "station_a": {
            "model_forward_only": _timing_summary(station_forward["station_a"]),
            "entire_model_pipeline": _timing_summary(station_pipeline["station_a"]),
            "end_to_end": _timing_summary(station_end_to_end["station_a"]),
        },
        "station_b": {
            "model_forward_only": _timing_summary(station_forward["station_b"]),
            "entire_model_pipeline": _timing_summary(station_pipeline["station_b"]),
            "end_to_end": _timing_summary(station_end_to_end["station_b"]),
        },
    }
    for view in views:
        view_dir = result_root / view
        view_dir.mkdir(parents=True, exist_ok=True)
        _save_score_plot(
            view_dir / "spatial_view_score_distribution.png",
            manifest["calibration_b_scores"][view],
            view_scores_by_label[view]["normal"],
            view_scores_by_label[view]["anomaly"],
            float(manifest["thresholds"][view]),
            view,
        )
    write_json(result_root / "product_predictions.json", {"records": records})
    write_json(result_root / "metrics.json", metrics)
    write_json(result_root / "inference_time.json", timing)
    (result_root / "accuracy.txt").write_text(
        "\n".join(f"{key}: {value}" for key, value in metrics.items()) + "\n",
        encoding="utf-8",
    )
    (result_root / "inference_time.txt").write_text(
        json.dumps(timing, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return {"metrics": metrics, "timing": timing, "product_count": len(labels)}


def main() -> None:
    args = parse_args()
    artifact_name = validate_safe_name(args.artifact_name, "artifact name")
    artifact_dir = args.artifact_root.expanduser().resolve() / artifact_name
    dataset_root = args.dataset_root.expanduser().resolve()
    result_parent = args.result_root.expanduser().resolve()
    if not artifact_dir.is_dir():
        raise FileNotFoundError(f"PatchCore artifact 폴더가 없습니다: {artifact_dir}")
    manifest = read_json(artifact_dir / "manifest.json")
    validate_v3_manifest(manifest, artifact_name, artifact_dir)
    artifact_sha256 = artifact_directory_sha256(artifact_dir)
    selected_policy = _print_startup_summary(
        artifact_name=artifact_name,
        artifact_sha256=artifact_sha256,
        manifest=manifest,
    )
    runtime_versions = require_supported_versions()
    index = index_v3_dataset(dataset_root, manifest["view_names"], ("test_normal", "test_anomaly"))
    validate_v3_dataset_pngs(index)
    test_dataset_provenance = dataset_provenance(index)

    with GpuProcessLock(0, GPU_LOCK_TIMEOUT_SECONDS, GPU_LOCK_POLL_INTERVAL_SECONDS):
        device = resolve_cuda_device()
        print(f"GPU: {device} ({torch.cuda.get_device_name(device)})")
        models: dict[str, PatchCoreViewModel] = {}
        spatial: dict[str, dict[str, torch.Tensor]] = {}
        for view in manifest["view_names"]:
            parameters = manifest["parameters_by_view"][view]
            model = PatchCoreViewModel(
                backbone=parameters["backbone"],
                layer_numbers=parameters["feature_layers"],
                num_neighbors=parameters["k"],
                pretrained=False,
                distance_chunk_size=parameters["distance_chunk_size"],
            )
            load_patchcore_state(
                model,
                torch.load(artifact_dir / view / "model.pt", map_location="cpu", weights_only=True),
            )
            models[view] = model.to(device).eval()
            payload = torch.load(
                artifact_dir / view / "spatial_calibration.pt", map_location="cpu", weights_only=True
            )
            validate_spatial_payload(payload, manifest["patch_grid_shapes"][view])
            spatial[view] = {
                "center": payload["center"].to(device),
                "denominator": payload["denominator"].to(device),
            }

        first_name = sorted_product_names(index, "test_normal")[0]
        sample_inputs = [
            preprocess_full_frame(
                index.files["test_normal"][view][first_name],
                view=view,
                settings=manifest["preprocessing_by_view"][view],
                resolution=manifest["parameters_by_view"][view]["input_resolution"],
                resize_mode=manifest["parameters_by_view"][view]["resize_mode"],
            ).tensor[None].to(device)
            for view in manifest["view_names"]
        ]
        benchmark = manifest["parallel_benchmark"]
        parallel_count = int(benchmark["selected_parallel_count"])
        memory_safety = verify_saved_parallelism_memory_safety(
            models=[models[view] for view in manifest["view_names"]],
            sample_inputs=sample_inputs,
            parallel_count=parallel_count,
            reserve_mib=int(benchmark["reserve_mib"]),
            warmup_runs=int(benchmark["warmup_runs"]),
        )
        if not memory_safety.memory_safe:
            raise RuntimeError("저장된 병렬도가 현재 VRAM reserve 조건을 통과하지 못했습니다.")
        with create_result_directory(result_parent, artifact_name, OVERWRITE_EXISTING_RESULT) as result_dir:
            summary = evaluate(
                models=models,
                spatial=spatial,
                index=index,
                manifest=manifest,
                result_root=result_dir,
                device=device,
                parallel_count=parallel_count,
            )
            write_json(
                result_dir / "run_information.json",
                {
                    "artifact_sha256": artifact_sha256,
                    "selected_policy": selected_policy,
                    "runtime_library_versions": runtime_versions,
                    "runtime_parallel_memory_safety": memory_safety.to_dict(),
                    "test_dataset_provenance": test_dataset_provenance,
                    "summary": summary,
                },
            )
    print(f"\nPatchCore v3 final test complete: {result_parent / ('result_' + artifact_name)}")


if __name__ == "__main__":
    main()
