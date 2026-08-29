#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PatchCore artifact v3의 위치 정규화와 full-frame dataset 공통 계약.

기존 v2 offline 도구와 artifact를 보존하기 위해 이 모듈은 ``ad_common.py``와
분리되어 있다. v3 construction/evaluation만 이 모듈을 사용한다.
"""

from __future__ import annotations

import hashlib
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import torch
from sklearn.metrics import accuracy_score, confusion_matrix, precision_score, recall_score

from ad_common import (
    PATCHCORE_CAMERA_SERIAL_BY_VIEW,
    PATCHCORE_EXPECTED_VIEWS,
    PATCHCORE_STATION_VIEWS,
    DatasetIndex,
    PatchCoreViewModel,
    read_json,
    validate_safe_name,
)


PATCHCORE_ARTIFACT_FORMAT_VERSION = 3
PREPROCESSING_PIPELINE_VERSION = "mono8-largest-component-padding-v1"
PATCHCORE_V3_PARAMETER_KEYS = {
    "backbone",
    "feature_layers",
    "coreset_ratio",
    "k",
    "input_resolution",
    "resize_mode",
    "construction_batch_size",
    "distance_chunk_size",
    "seed",
}
PATCHCORE_PREPROCESSING_KEYS = {
    "v_threshold",
    "connectivity",
    "remove_disconnected_noise",
    "check_connection",
}
LOGICAL_SPLIT_PATHS: dict[str, tuple[str, ...]] = {
    "training_set": ("training_set",),
    "calibration_set_A": ("calibration_set_A",),
    "calibration_set_B": ("calibration_set_B",),
    "validation_normal": ("validation_set", "normal"),
    "validation_anomaly": ("validation_set", "anomaly"),
    "test_normal": ("test_set", "normal"),
    "test_anomaly": ("test_set", "anomaly"),
}
NORMALIZATION_COMPLEXITY = {
    "std_floor": 0,
    "epsilon": 1,
    "shrinkage": 2,
    "mad": 3,
    "none": 99,
}


@dataclass(frozen=True)
class SpatialCalibration:
    center: np.ndarray
    denominator: np.ndarray
    raw_scale: np.ndarray
    grid_shape: tuple[int, int]
    scale_reference: float
    sample_count: int


@dataclass(frozen=True)
class PreparedFullFrame:
    tensor: torch.Tensor
    crop_1: np.ndarray
    crop_2: np.ndarray
    component_count: int


def _png_files_exact(directory: Path) -> dict[str, Path]:
    if not directory.is_dir():
        raise FileNotFoundError(f"필수 view 폴더가 없습니다: {directory}")
    paths = sorted(
        (path for path in directory.iterdir() if path.is_file() and path.suffix.lower() == ".png"),
        key=lambda path: path.name,
    )
    if not paths:
        raise ValueError(f"PNG가 하나도 없습니다: {directory}")
    folded: dict[str, str] = {}
    result: dict[str, Path] = {}
    for path in paths:
        key = path.name.casefold()
        previous = folded.get(key)
        if previous is not None and previous != path.name:
            raise ValueError(f"대소문자만 다른 PNG 파일명이 공존합니다: {previous}, {path.name}")
        folded[key] = path.name
        result[path.name] = path
    return result


def index_v3_dataset(
    dataset_root: Path,
    view_names: Sequence[str],
    required_splits: Sequence[str],
) -> DatasetIndex:
    """v3 논리 split을 실제 ``validation_set/normal`` 등의 경로로 변환한다."""

    root = dataset_root.expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"DATASET_ROOT가 없습니다: {root}")
    views = tuple(validate_safe_name(str(name), "view name") for name in view_names)
    if tuple(views) != PATCHCORE_EXPECTED_VIEWS:
        raise ValueError(f"v3 view 순서는 {list(PATCHCORE_EXPECTED_VIEWS)}여야 합니다.")
    files: dict[str, dict[str, dict[str, Path]]] = {}
    for logical_split in required_splits:
        relative = LOGICAL_SPLIT_PATHS.get(logical_split)
        if relative is None:
            raise ValueError(f"알 수 없는 v3 dataset split: {logical_split}")
        split_root = root.joinpath(*relative)
        split_views: dict[str, dict[str, Path]] = {}
        reference_names: set[str] | None = None
        reference_view = ""
        for view in views:
            current = _png_files_exact(split_root / view)
            names = set(current)
            if reference_names is None:
                reference_names = names
                reference_view = view
            elif names != reference_names:
                raise ValueError(
                    f"{logical_split}/{view} 파일명 집합이 {reference_view}와 다릅니다. "
                    f"누락={sorted(reference_names - names)[:10]}, "
                    f"추가={sorted(names - reference_names)[:10]}"
                )
            split_views[view] = current
        files[logical_split] = split_views
    return DatasetIndex(root=root, view_names=views, files=files)


def sorted_product_names(index: DatasetIndex, split: str) -> list[str]:
    return sorted(index.files[split][index.view_names[0]])


def read_full_frame_mono8(path: Path) -> np.ndarray:
    try:
        encoded = np.fromfile(str(path), dtype=np.uint8)
    except OSError as exc:
        raise RuntimeError(f"full-frame PNG 읽기 실패: {path}") from exc
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"full-frame PNG decode 실패: {path}")
    if image.dtype != np.uint8 or image.ndim != 2:
        raise ValueError(f"v3 dataset은 8-bit 1-channel full-frame Mono8 PNG여야 합니다: {path}")
    return image


def validate_v3_dataset_pngs(index: DatasetIndex) -> None:
    checked: set[Path] = set()
    for split_views in index.files.values():
        for view_files in split_views.values():
            for path in view_files.values():
                if path not in checked:
                    read_full_frame_mono8(path)
                    checked.add(path)
    print(f"v3 full-frame Mono8 integrity check passed: {len(checked)} files")


def validate_preprocessing_settings(view: str, settings: Mapping[str, Any]) -> None:
    if set(settings) != PATCHCORE_PREPROCESSING_KEYS:
        raise ValueError(f"{view}: preprocessing key가 올바르지 않습니다.")
    if (
        type(settings["v_threshold"]) is not int
        or type(settings["connectivity"]) is not int
        or type(settings["remove_disconnected_noise"]) is not bool
        or type(settings["check_connection"]) is not bool
    ):
        raise ValueError(f"{view}: preprocessing type이 올바르지 않습니다.")
    if not 0 <= int(settings["v_threshold"]) <= 255:
        raise ValueError(f"{view}: v_threshold는 0~255여야 합니다.")
    if settings["connectivity"] != 8:
        raise ValueError(f"{view}: connectivity는 8이어야 합니다.")
    if not settings["remove_disconnected_noise"] or settings["check_connection"]:
        raise ValueError(f"{view}: v3 foreground 안전 정책과 다릅니다.")


def preprocess_full_frame(
    path: Path,
    *,
    view: str,
    settings: Mapping[str, Any],
    resolution: Sequence[int],
    resize_mode: str,
) -> PreparedFullFrame:
    """운영 Mono8 preprocessor와 동일한 crop/padding을 offline에서 수행한다."""

    validate_preprocessing_settings(view, settings)
    if resize_mode != "padding":
        raise ValueError(f"{view}: v3 resize_mode는 padding이어야 합니다.")
    if len(resolution) != 2 or min(map(int, resolution)) < 1:
        raise ValueError(f"{view}: input_resolution이 올바르지 않습니다.")
    image = read_full_frame_mono8(path)
    initial_mask = np.where(image >= int(settings["v_threshold"]), 255, 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        initial_mask,
        connectivity=int(settings["connectivity"]),
    )
    component_count = int(count) - 1
    if component_count == 0:
        raise ValueError(f"{view}: foreground가 없습니다: {path}")
    final_mask = initial_mask
    if bool(settings["remove_disconnected_noise"]) and component_count > 1:
        largest_label = int(np.argmax(stats[1:, cv2.CC_STAT_AREA])) + 1
        final_mask = np.where(labels == largest_label, 255, 0).astype(np.uint8)
    if bool(settings["check_connection"]) and component_count != 1:
        raise ValueError(f"{view}: foreground가 분리되어 있습니다: {path}")
    points = cv2.findNonZero(final_mask)
    if points is None:
        raise ValueError(f"{view}: 정리된 foreground가 비어 있습니다: {path}")
    x, y, width, height = (int(value) for value in cv2.boundingRect(points))
    crop_1 = cv2.bitwise_and(image, image, mask=final_mask)
    crop_2 = crop_1[y : y + height, x : x + width]
    target_h, target_w = int(resolution[0]), int(resolution[1])
    source_h, source_w = crop_2.shape
    scale = min(target_w / source_w, target_h / source_h)
    resized_w = max(1, min(target_w, int(round(source_w * scale))))
    resized_h = max(1, min(target_h, int(round(source_h * scale))))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(crop_2, (resized_w, resized_h), interpolation=interpolation)
    canvas = np.zeros((target_h, target_w), dtype=np.uint8)
    x0 = (target_w - resized_w) // 2
    y0 = (target_h - resized_h) // 2
    canvas[y0 : y0 + resized_h, x0 : x0 + resized_w] = resized
    rgb = np.repeat(canvas[:, :, None], 3, axis=2)
    tensor = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1).float().div_(255.0)
    return PreparedFullFrame(tensor=tensor, crop_1=crop_1, crop_2=crop_2, component_count=component_count)


@torch.inference_mode()
def native_patch_maps(model: PatchCoreViewModel, images: torch.Tensor) -> torch.Tensor:
    """Bilinear upsample 전 최근접 memory-bank distance map을 반환한다."""

    embedding, (grid_h, grid_w) = model._generate_embedding_and_grid(images)
    patch_scores, _ = model.nearest_neighbors(embedding, 1)
    maps = patch_scores.reshape(images.shape[0], grid_h, grid_w)
    if not torch.isfinite(maps).all():
        raise RuntimeError("native PatchCore map에 NaN/Inf가 있습니다.")
    return maps


@torch.inference_mode()
def native_patch_maps_and_legacy_scores(
    model: PatchCoreViewModel, images: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """한 embedding pass에서 native map과 기존 reweighted image score를 반환한다."""

    embedding, (grid_h, grid_w) = model._generate_embedding_and_grid(images)
    patch_scores, locations = model.nearest_neighbors(embedding, 1)
    legacy_scores = model.compute_image_score(
        patch_scores,
        locations,
        embedding,
        images.shape[0],
    )
    maps = patch_scores.reshape(images.shape[0], grid_h, grid_w)
    if not torch.isfinite(maps).all() or not torch.isfinite(legacy_scores).all():
        raise RuntimeError("PatchCore map/legacy score에 NaN/Inf가 있습니다.")
    return maps, legacy_scores


def positive_median(values: np.ndarray, name: str) -> float:
    positive = np.asarray(values, dtype=np.float64)
    positive = positive[np.isfinite(positive) & (positive > 0)]
    if positive.size == 0:
        raise RuntimeError(f"{name}에 유한한 양수 scale이 없습니다.")
    result = float(np.median(positive))
    if not math.isfinite(result) or result <= 0:
        raise RuntimeError(f"{name} reference scale이 안전하지 않습니다.")
    return result


def validate_normalization_config(normalization: Mapping[str, Any], *, allow_none: bool = False) -> None:
    method = str(normalization.get("method", ""))
    expected_keys = {
        "epsilon": {"method", "epsilon_ratio"},
        "std_floor": {"method", "std_floor_ratio"},
        "shrinkage": {"method", "shrinkage_lambda"},
        "mad": {"method", "mad_epsilon_ratio"},
        "none": {"method"},
    }.get(method)
    if expected_keys is None or set(normalization) != expected_keys:
        raise ValueError("normalization config key가 올바르지 않습니다.")
    if method == "none":
        if not allow_none:
            raise ValueError("normalization=none은 baseline 전용입니다.")
        return
    field = next(key for key in expected_keys if key != "method")
    value = float(normalization[field])
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{field}는 유한한 양수여야 합니다.")
    if method == "shrinkage" and value > 1:
        raise ValueError("shrinkage_lambda는 0보다 크고 1 이하여야 합니다.")
    allowed_values = {
        "epsilon": {0.001, 0.01, 0.1},
        "std_floor": {0.05, 0.1, 0.2},
        "shrinkage": {0.05, 0.1, 0.25, 0.5},
        "mad": {0.001, 0.01, 0.1},
    }
    if value not in allowed_values[method]:
        raise ValueError(f"{method} 값이 승인된 후보 집합에 없습니다.")


def validate_aggregation_config(aggregation: Mapping[str, Any]) -> None:
    method = str(aggregation.get("method", ""))
    if method == "percentile":
        if set(aggregation) != {"method", "percentile"}:
            raise ValueError("percentile aggregation key가 올바르지 않습니다.")
        percentile = float(aggregation["percentile"])
        if not math.isfinite(percentile) or not 0 <= percentile <= 100:
            raise ValueError("map percentile은 0~100이어야 합니다.")
        if percentile not in {99.0, 99.5, 99.9, 100.0}:
            raise ValueError("map percentile이 승인된 후보 집합에 없습니다.")
        return
    if method == "top_k_average":
        if set(aggregation) != {"method", "top_k"}:
            raise ValueError("top-k aggregation key가 올바르지 않습니다.")
        if type(aggregation["top_k"]) is not int or int(aggregation["top_k"]) < 1:
            raise ValueError("top_k는 양의 정수여야 합니다.")
        if int(aggregation["top_k"]) not in {1, 3, 5, 10}:
            raise ValueError("top_k가 승인된 후보 집합에 없습니다.")
        return
    raise ValueError("지원하지 않는 aggregation config입니다.")


def build_spatial_calibration(
    calibration_a_maps: np.ndarray,
    normalization: Mapping[str, Any],
) -> SpatialCalibration:
    values = np.asarray(calibration_a_maps, dtype=np.float64)
    if values.ndim != 3 or values.shape[0] < 2 or not np.all(np.isfinite(values)):
        raise ValueError("calibration A map은 finite [N,H,W]이고 N>=2여야 합니다.")
    validate_normalization_config(normalization, allow_none=True)
    method = str(normalization["method"])
    if method in {"epsilon", "std_floor", "shrinkage"}:
        center = values.mean(axis=0)
        raw_scale = values.std(axis=0, ddof=1)
        reference = positive_median(raw_scale, "std")
        if method == "epsilon":
            epsilon = float(normalization["epsilon_ratio"]) * reference
            denominator = raw_scale + epsilon
        elif method == "std_floor":
            floor = float(normalization["std_floor_ratio"]) * reference
            denominator = np.maximum(raw_scale, floor)
        else:
            shrinkage = float(normalization["shrinkage_lambda"])
            if not 0.0 <= shrinkage <= 1.0:
                raise ValueError("shrinkage_lambda는 0~1이어야 합니다.")
            global_variance = float(np.mean(np.square(raw_scale, dtype=np.float64)))
            epsilon_numeric = max(1e-12, 1e-6 * reference)
            denominator = np.sqrt(
                (1.0 - shrinkage) * np.square(raw_scale)
                + shrinkage * global_variance
            ) + epsilon_numeric
    elif method == "mad":
        center = np.median(values, axis=0)
        raw_scale = 1.4826 * np.median(np.abs(values - center[None]), axis=0)
        reference = positive_median(raw_scale, "scaled MAD")
        epsilon = float(normalization["mad_epsilon_ratio"]) * reference
        denominator = raw_scale + epsilon
    elif method == "none":
        center = np.zeros(values.shape[1:], dtype=np.float64)
        raw_scale = np.ones(values.shape[1:], dtype=np.float64)
        reference = 1.0
        denominator = raw_scale.copy()
    else:
        raise ValueError(f"알 수 없는 normalization method: {method}")
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(denominator)):
        raise RuntimeError("spatial calibration에 NaN/Inf가 있습니다.")
    if np.any(denominator <= 0):
        raise RuntimeError("spatial denominator는 모두 양수여야 합니다.")
    return SpatialCalibration(
        center=center.astype(np.float32),
        denominator=denominator.astype(np.float32),
        raw_scale=raw_scale.astype(np.float32),
        grid_shape=(int(values.shape[1]), int(values.shape[2])),
        scale_reference=reference,
        sample_count=int(values.shape[0]),
    )


def normalize_maps_numpy(maps: np.ndarray, calibration: SpatialCalibration) -> np.ndarray:
    values = np.asarray(maps, dtype=np.float32)
    if values.ndim != 3 or tuple(values.shape[1:]) != calibration.grid_shape:
        raise ValueError("raw map shape와 spatial calibration grid가 다릅니다.")
    normalized = (values - calibration.center[None]) / calibration.denominator[None]
    if not np.all(np.isfinite(normalized)):
        raise RuntimeError("normalized map에 NaN/Inf가 있습니다.")
    return normalized


def aggregate_maps_numpy(maps: np.ndarray, aggregation: Mapping[str, Any]) -> np.ndarray:
    values = np.asarray(maps, dtype=np.float32)
    if values.ndim != 3 or not np.all(np.isfinite(values)):
        raise ValueError("aggregation 입력은 finite [N,H,W]여야 합니다.")
    validate_aggregation_config(aggregation)
    flattened = values.reshape(values.shape[0], -1)
    method = str(aggregation["method"])
    if method == "percentile":
        percentile = float(aggregation["percentile"])
        if not 0 <= percentile <= 100:
            raise ValueError("map percentile은 0~100이어야 합니다.")
        return np.percentile(flattened, percentile, axis=1, method="linear")
    if method == "top_k_average":
        top_k = int(aggregation["top_k"])
        if not 1 <= top_k <= flattened.shape[1]:
            raise ValueError("top_k가 patch 수 범위를 벗어났습니다.")
        partitioned = np.partition(flattened, flattened.shape[1] - top_k, axis=1)
        return partitioned[:, -top_k:].mean(axis=1)
    raise ValueError(f"알 수 없는 aggregation method: {method}")


def spatial_view_scores_torch(
    raw_maps: torch.Tensor,
    center: torch.Tensor,
    denominator: torch.Tensor,
    aggregation: Mapping[str, Any],
) -> torch.Tensor:
    """운영 runtime과 golden test가 공유할 v3 view score 수식."""

    if raw_maps.ndim != 3 or center.ndim != 2 or denominator.ndim != 2:
        raise RuntimeError("v3 scoring tensor rank가 올바르지 않습니다.")
    if tuple(raw_maps.shape[1:]) != tuple(center.shape) or center.shape != denominator.shape:
        raise RuntimeError("v3 scoring patch grid shape가 다릅니다.")
    if not torch.isfinite(raw_maps).all() or not torch.isfinite(center).all():
        raise RuntimeError("v3 scoring tensor에 NaN/Inf가 있습니다.")
    if not torch.isfinite(denominator).all() or torch.any(denominator <= 0):
        raise RuntimeError("v3 denominator가 안전하지 않습니다.")
    normalized = (raw_maps - center[None]) / denominator[None]
    flattened = normalized.reshape(normalized.shape[0], -1)
    method = str(aggregation["method"])
    if method == "percentile":
        percentile = float(aggregation["percentile"])
        if percentile == 100.0:
            return flattened.amax(dim=1)
        return torch.quantile(flattened, percentile / 100.0, dim=1, interpolation="linear")
    if method == "top_k_average":
        top_k = int(aggregation["top_k"])
        if not 1 <= top_k <= flattened.shape[1]:
            raise RuntimeError("artifact top_k가 patch 수 범위를 벗어났습니다.")
        return flattened.topk(top_k, dim=1, largest=True, sorted=False).values.mean(dim=1)
    raise RuntimeError(f"지원하지 않는 v3 aggregation method: {method}")


def normalization_candidates() -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for ratio in (0.001, 0.01, 0.1):
        candidates.append({"method": "epsilon", "epsilon_ratio": ratio})
    for ratio in (0.05, 0.1, 0.2):
        candidates.append({"method": "std_floor", "std_floor_ratio": ratio})
    for shrinkage in (0.05, 0.1, 0.25, 0.5):
        candidates.append({"method": "shrinkage", "shrinkage_lambda": shrinkage})
    for ratio in (0.001, 0.01, 0.1):
        candidates.append({"method": "mad", "mad_epsilon_ratio": ratio})
    return candidates


def aggregation_candidates() -> list[dict[str, Any]]:
    candidates = [
        {"method": "percentile", "percentile": value}
        for value in (99.0, 99.5, 99.9, 100.0)
    ]
    candidates.extend(
        {"method": "top_k_average", "top_k": value}
        for value in (1, 3, 5, 10)
    )
    return candidates


def find_product_fpr_thresholds(
    calibration_b_scores_by_view: Mapping[str, np.ndarray],
    target_product_fpr: float,
) -> tuple[float, dict[str, float], float]:
    """공통 percentile 중 가장 민감하면서 4-view OR FPR을 만족하는 값을 찾는다."""

    views = tuple(calibration_b_scores_by_view)
    arrays = [np.asarray(calibration_b_scores_by_view[view], dtype=np.float64) for view in views]
    if not arrays or len({array.size for array in arrays}) != 1:
        raise ValueError("calibration B view score 수가 다릅니다.")
    sample_count = arrays[0].size
    if sample_count < 100:
        raise ValueError("1% product FPR calibration에는 calibration_set_B 제품 100개 이상이 필요합니다.")
    if any(not np.all(np.isfinite(array)) for array in arrays):
        raise ValueError("calibration B score에 NaN/Inf가 있습니다.")
    # 0.01% 간격은 4-view 보정 percentile을 충분히 세밀하게 탐색한다.
    for percentile in np.linspace(90.0, 100.0, 1001):
        thresholds = {
            view: float(np.percentile(array, percentile, method="linear"))
            for view, array in zip(views, arrays, strict=True)
        }
        if any(not math.isfinite(value) or value <= 0 for value in thresholds.values()):
            continue
        predictions = np.zeros(sample_count, dtype=bool)
        for view, array in zip(views, arrays, strict=True):
            predictions |= array > thresholds[view]
        fpr = float(predictions.mean())
        if fpr <= target_product_fpr:
            return float(percentile), thresholds, fpr
    raise RuntimeError("calibration B에서 목표 4-view product FPR을 만족하는 threshold가 없습니다.")


def evaluate_product_scores(
    normal_scores_by_view: Mapping[str, np.ndarray],
    anomaly_scores_by_view: Mapping[str, np.ndarray],
    thresholds: Mapping[str, float],
) -> dict[str, Any]:
    normal_count = len(next(iter(normal_scores_by_view.values())))
    anomaly_count = len(next(iter(anomaly_scores_by_view.values())))
    normal_predictions = np.zeros(normal_count, dtype=bool)
    anomaly_predictions = np.zeros(anomaly_count, dtype=bool)
    for view, threshold in thresholds.items():
        normal_predictions |= np.asarray(normal_scores_by_view[view]) > threshold
        anomaly_predictions |= np.asarray(anomaly_scores_by_view[view]) > threshold
    labels = np.concatenate((np.zeros(normal_count, dtype=np.int64), np.ones(anomaly_count, dtype=np.int64)))
    predictions = np.concatenate((normal_predictions.astype(np.int64), anomaly_predictions.astype(np.int64)))
    tn, fp, fn, tp = (int(value) for value in confusion_matrix(labels, predictions, labels=[0, 1]).ravel())
    fpr = fp / (fp + tn) if fp + tn else 0.0
    return {
        "accuracy": float(accuracy_score(labels, predictions)),
        "recall": float(recall_score(labels, predictions, zero_division=0)),
        "precision": float(precision_score(labels, predictions, zero_division=0)),
        "fpr": float(fpr),
        "tn": tn,
        "fp": fp,
        "fn": fn,
        "tp": tp,
    }


def score_candidate(
    *,
    candidate_id: str,
    normalization: Mapping[str, Any],
    aggregation: Mapping[str, Any],
    calibration_a_maps_by_view: Mapping[str, np.ndarray],
    calibration_b_maps_by_view: Mapping[str, np.ndarray],
    validation_normal_maps_by_view: Mapping[str, np.ndarray],
    validation_anomaly_maps_by_view: Mapping[str, np.ndarray],
    target_product_fpr: float,
    eligible_for_selection: bool,
) -> tuple[dict[str, Any], dict[str, SpatialCalibration], dict[str, np.ndarray]]:
    calibrations: dict[str, SpatialCalibration] = {}
    calibration_b_scores: dict[str, np.ndarray] = {}
    validation_normal_scores: dict[str, np.ndarray] = {}
    validation_anomaly_scores: dict[str, np.ndarray] = {}
    started = time.perf_counter()
    for view in calibration_a_maps_by_view:
        calibration = build_spatial_calibration(calibration_a_maps_by_view[view], normalization)
        calibrations[view] = calibration
        calibration_b_scores[view] = aggregate_maps_numpy(
            normalize_maps_numpy(calibration_b_maps_by_view[view], calibration), aggregation
        )
        validation_normal_scores[view] = aggregate_maps_numpy(
            normalize_maps_numpy(validation_normal_maps_by_view[view], calibration), aggregation
        )
        validation_anomaly_scores[view] = aggregate_maps_numpy(
            normalize_maps_numpy(validation_anomaly_maps_by_view[view], calibration), aggregation
        )
    threshold_percentile, thresholds, calibration_fpr = find_product_fpr_thresholds(
        calibration_b_scores, target_product_fpr
    )
    validation_metrics = evaluate_product_scores(
        validation_normal_scores,
        validation_anomaly_scores,
        thresholds,
    )
    per_product_ms: list[float] = []
    for split_maps in (
        validation_normal_maps_by_view,
        validation_anomaly_maps_by_view,
    ):
        product_count = len(next(iter(split_maps.values())))
        for product_index in range(product_count):
            product_started = time.perf_counter()
            for view in split_maps:
                single = split_maps[view][product_index : product_index + 1]
                aggregate_maps_numpy(
                    normalize_maps_numpy(single, calibrations[view]), aggregation
                )
            per_product_ms.append((time.perf_counter() - product_started) * 1000.0)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    record = {
        "candidate_id": candidate_id,
        "eligible_for_selection": bool(eligible_for_selection),
        "normalization": dict(normalization),
        "aggregation": dict(aggregation),
        "threshold_percentile": threshold_percentile,
        "thresholds": thresholds,
        "calibration_product_fpr": calibration_fpr,
        "validation_metrics": validation_metrics,
        "validation_postprocess_p95_ms": float(
            np.percentile(per_product_ms, 95, method="linear")
        ),
        "offline_postprocess_total_ms": elapsed_ms,
        "normalization_complexity": NORMALIZATION_COMPLEXITY[str(normalization["method"])],
    }
    return record, calibrations, calibration_b_scores


def candidate_sort_key(record: Mapping[str, Any]) -> tuple[float, float, float, int, str]:
    metrics = record["validation_metrics"]
    return (
        -float(metrics["recall"]),
        float(metrics["fpr"]),
        float(record["validation_postprocess_p95_ms"]),
        int(record["normalization_complexity"]),
        str(record["candidate_id"]),
    )


def dataset_provenance(index: DatasetIndex) -> dict[str, Any]:
    document: dict[str, Any] = {}
    for split, split_views in index.files.items():
        digest = hashlib.sha256()
        file_count = 0
        product_count = len(split_views[index.view_names[0]])
        for view in index.view_names:
            for name, path in sorted(split_views[view].items()):
                relative = path.relative_to(index.root).as_posix()
                digest.update(relative.encode("utf-8"))
                digest.update(b"\0")
                with path.open("rb") as stream:
                    while chunk := stream.read(1024 * 1024):
                        digest.update(chunk)
                digest.update(b"\0")
                file_count += 1
        document[split] = {
            "product_count": product_count,
            "file_count": file_count,
            "sha256": digest.hexdigest(),
        }
    return document


def preprocessing_pipeline_document() -> dict[str, Any]:
    return {
        "version": PREPROCESSING_PIPELINE_VERSION,
        "input": "full_frame_mono8_png_uint8",
        "foreground": "mono8_greater_equal_v_threshold",
        "component_selection": "largest_connected_component",
        "crop": "masked_foreground_bounding_rect",
        "resize": "aspect_ratio_preserving_centered_black_padding",
        "downscale_interpolation": "opencv_inter_area",
        "upscale_interpolation": "opencv_inter_linear",
        "padding_value": 0,
        "channel_conversion": "mono8_repeat_to_rgb3",
        "tensor_scaling": "float32_divide_255",
        "model_normalization": "imagenet_mean_std_in_model",
    }


def spatial_calibration_payload(calibration: SpatialCalibration) -> dict[str, Any]:
    return {
        "center": torch.from_numpy(calibration.center.copy()),
        "denominator": torch.from_numpy(calibration.denominator.copy()),
        "raw_scale": torch.from_numpy(calibration.raw_scale.copy()),
        "grid_shape": torch.tensor(calibration.grid_shape, dtype=torch.int64),
        "scale_reference": torch.tensor(calibration.scale_reference, dtype=torch.float64),
        "sample_count": torch.tensor(calibration.sample_count, dtype=torch.int64),
    }


def validate_spatial_payload(payload: Mapping[str, Any], expected_grid: Sequence[int]) -> None:
    required = {"center", "denominator", "raw_scale", "grid_shape", "scale_reference", "sample_count"}
    if not isinstance(payload, Mapping) or set(payload) != required:
        raise RuntimeError("spatial_calibration.pt key가 올바르지 않습니다.")
    center = payload["center"]
    denominator = payload["denominator"]
    raw_scale = payload["raw_scale"]
    if any(not isinstance(value, torch.Tensor) for value in (center, denominator, raw_scale)):
        raise RuntimeError("spatial calibration map이 tensor가 아닙니다.")
    if center.dtype != torch.float32 or denominator.dtype != torch.float32 or raw_scale.dtype != torch.float32:
        raise RuntimeError("spatial calibration map은 float32여야 합니다.")
    if center.ndim != 2 or center.shape != denominator.shape or center.shape != raw_scale.shape:
        raise RuntimeError("spatial calibration map shape가 다릅니다.")
    grid = tuple(int(value) for value in payload["grid_shape"].reshape(-1).tolist())
    if grid != tuple(int(value) for value in expected_grid) or tuple(center.shape) != grid:
        raise RuntimeError("spatial calibration grid가 manifest와 다릅니다.")
    if (
        not torch.isfinite(center).all()
        or not torch.isfinite(denominator).all()
        or not torch.isfinite(raw_scale).all()
    ):
        raise RuntimeError("spatial calibration에 NaN/Inf가 있습니다.")
    if torch.any(denominator <= 0):
        raise RuntimeError("spatial denominator는 모두 양수여야 합니다.")
    if torch.any(raw_scale < 0):
        raise RuntimeError("spatial raw scale은 음수일 수 없습니다.")
    scale_reference = float(payload["scale_reference"].reshape(-1)[0])
    sample_count = int(payload["sample_count"].reshape(-1)[0])
    if not math.isfinite(scale_reference) or scale_reference <= 0 or sample_count < 2:
        raise RuntimeError("spatial calibration metadata가 안전하지 않습니다.")


def validate_v3_manifest(
    manifest: Mapping[str, Any],
    artifact_name: str,
    artifact_dir: Path,
) -> None:
    """Offline construction/evaluation용 엄격한 v3 artifact 검증."""

    if manifest.get("format_version") != PATCHCORE_ARTIFACT_FORMAT_VERSION:
        raise RuntimeError("PatchCore artifact v3만 지원합니다.")
    if manifest.get("algorithm") != "patchcore" or manifest.get("artifact_name") != artifact_name:
        raise RuntimeError("artifact algorithm/name이 다릅니다.")
    views = manifest.get("view_names")
    if not isinstance(views, list) or tuple(views) != PATCHCORE_EXPECTED_VIEWS:
        raise RuntimeError("artifact view 순서가 운영 계약과 다릅니다.")
    if manifest.get("station_views") != PATCHCORE_STATION_VIEWS:
        raise RuntimeError("artifact station_views가 다릅니다.")
    if manifest.get("camera_serial_by_view") != PATCHCORE_CAMERA_SERIAL_BY_VIEW:
        raise RuntimeError("artifact camera serial/view가 다릅니다.")
    pipeline = manifest.get("preprocessing_pipeline")
    if pipeline != preprocessing_pipeline_document():
        raise RuntimeError("artifact preprocessing pipeline version이 다릅니다.")
    parameters_by_view = manifest.get("parameters_by_view")
    preprocessing = manifest.get("preprocessing_by_view")
    thresholds = manifest.get("thresholds")
    grids = manifest.get("patch_grid_shapes")
    bank_shapes = manifest.get("memory_bank_shapes")
    calibration_b_scores = manifest.get("calibration_b_scores")
    for field_name, value in (
        ("parameters_by_view", parameters_by_view),
        ("preprocessing_by_view", preprocessing),
        ("thresholds", thresholds),
        ("patch_grid_shapes", grids),
        ("memory_bank_shapes", bank_shapes),
        ("calibration_b_scores", calibration_b_scores),
    ):
        if not isinstance(value, dict) or set(value) != set(views):
            raise RuntimeError(f"artifact {field_name} view key가 다릅니다.")
    policy = manifest.get("spatial_scoring_policy")
    if not isinstance(policy, dict) or set(policy) != {"normalization", "aggregation", "decision"}:
        raise RuntimeError("artifact spatial_scoring_policy가 올바르지 않습니다.")
    try:
        validate_normalization_config(policy["normalization"])
        validate_aggregation_config(policy["aggregation"])
    except (TypeError, ValueError) as exc:
        raise RuntimeError("artifact spatial scoring option이 올바르지 않습니다.") from exc
    decision = policy["decision"]
    if decision.get("comparison") != "strict_greater_than" or decision.get("view_score_normalization") != "divide_by_threshold":
        raise RuntimeError("artifact v3 decision 정책이 다릅니다.")
    target_fpr = float(decision.get("target_product_fpr", math.nan))
    if not math.isfinite(target_fpr) or target_fpr != 0.01:
        raise RuntimeError("artifact target product FPR은 0.01이어야 합니다.")
    for view in views:
        parameters = parameters_by_view[view]
        if not isinstance(parameters, dict) or set(parameters) != PATCHCORE_V3_PARAMETER_KEYS:
            raise RuntimeError(f"{view}: v3 parameter key가 다릅니다.")
        resolution = parameters["input_resolution"]
        if (
            not isinstance(resolution, (list, tuple))
            or len(resolution) != 2
            or any(type(value) is not int or value < 1 for value in resolution)
        ):
            raise RuntimeError(f"{view}: input_resolution이 올바르지 않습니다.")
        validate_preprocessing_settings(view, preprocessing[view])
        threshold = float(thresholds[view])
        if not math.isfinite(threshold) or threshold <= 0:
            raise RuntimeError(f"{view}: threshold가 안전하지 않습니다.")
        scores = np.asarray(calibration_b_scores[view], dtype=np.float64)
        if scores.size < 100 or not np.all(np.isfinite(scores)):
            raise RuntimeError(f"{view}: calibration B score가 부족하거나 유효하지 않습니다.")
        grid = grids[view]
        if not isinstance(grid, list) or len(grid) != 2 or min(map(int, grid)) < 1:
            raise RuntimeError(f"{view}: patch grid shape가 올바르지 않습니다.")
        if policy["aggregation"]["method"] == "top_k_average" and int(
            policy["aggregation"]["top_k"]
        ) > int(grid[0]) * int(grid[1]):
            raise RuntimeError(f"{view}: top_k가 patch 수보다 큽니다.")
        shape = bank_shapes[view]
        if not isinstance(shape, list) or len(shape) != 2 or min(map(int, shape)) < 1:
            raise RuntimeError(f"{view}: memory bank shape가 올바르지 않습니다.")
    selection = manifest.get("candidate_selection")
    if not isinstance(selection, dict) or not selection.get("selected_candidate_id"):
        raise RuntimeError("artifact candidate selection 기록이 없습니다.")
    candidates = selection.get("candidates")
    if not isinstance(candidates, list):
        raise RuntimeError("artifact candidate 목록이 없습니다.")
    selected_matches = [
        candidate
        for candidate in candidates
        if isinstance(candidate, dict)
        and candidate.get("candidate_id") == selection["selected_candidate_id"]
    ]
    if len(selected_matches) != 1:
        raise RuntimeError("artifact selected candidate 기록이 유일하지 않습니다.")
    selected_candidate = selected_matches[0]
    if (
        selected_candidate.get("status") != "valid"
        or not selected_candidate.get("eligible_for_selection")
        or selected_candidate.get("normalization") != policy["normalization"]
        or selected_candidate.get("aggregation") != policy["aggregation"]
        or selected_candidate.get("thresholds") != thresholds
        or float(selected_candidate.get("threshold_percentile", math.nan))
        != float(decision.get("threshold_percentile", math.nan))
        or float(selected_candidate.get("calibration_product_fpr", math.inf)) > target_fpr
        or float(selected_candidate["validation_metrics"]["fpr"]) > target_fpr
    ):
        raise RuntimeError("artifact selected candidate와 배포 정책이 다릅니다.")
    provenance = manifest.get("dataset_provenance")
    if not isinstance(provenance, dict) or set(provenance) != set(LOGICAL_SPLIT_PATHS) - {"test_normal", "test_anomaly"}:
        raise RuntimeError("artifact construction dataset provenance가 올바르지 않습니다.")
    for split, record in provenance.items():
        if not isinstance(record, dict) or set(record) != {"product_count", "file_count", "sha256"}:
            raise RuntimeError(f"artifact {split} provenance 구조가 올바르지 않습니다.")
        product_count = int(record["product_count"])
        if product_count < 1 or int(record["file_count"]) != product_count * len(views):
            raise RuntimeError(f"artifact {split} provenance count가 올바르지 않습니다.")
        digest = str(record["sha256"])
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
            raise RuntimeError(f"artifact {split} provenance SHA-256이 올바르지 않습니다.")
    versions = manifest.get("library_versions")
    if not isinstance(versions, dict) or not {"torch", "torchvision"} <= set(versions):
        raise RuntimeError("artifact library version 기록이 없습니다.")
    sample_policy = manifest.get("calibration_b_sample_policy")
    if (
        not isinstance(sample_policy, dict)
        or set(sample_policy) != {"minimum", "recommended", "actual"}
        or int(sample_policy["minimum"]) != 100
        or int(sample_policy["recommended"]) != 1000
        or int(sample_policy["actual"]) != len(calibration_b_scores[views[0]])
    ):
        raise RuntimeError("artifact calibration B 표본 정책이 올바르지 않습니다.")
    benchmark = manifest.get("parallel_benchmark")
    if not isinstance(benchmark, dict):
        raise RuntimeError("artifact parallel benchmark가 올바르지 않습니다.")
    try:
        selected_parallel = int(benchmark["selected_parallel_count"])
        reserve_mib = int(benchmark["reserve_mib"])
        warmup_runs = int(benchmark["warmup_runs"])
        benchmark_runs = int(benchmark["benchmark_runs"])
        minimum_speedup = float(benchmark["min_speedup_percent"])
        candidates = benchmark["candidates"]
        all_view_safety = benchmark["all_view_memory_safety"]
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("artifact parallel benchmark 값이 부족합니다.") from exc
    if (
        not 1 <= selected_parallel <= 3
        or reserve_mib < 0
        or warmup_runs < 1
        or benchmark_runs < 1
        or not 0 <= minimum_speedup < 100
        or not isinstance(candidates, list)
        or not isinstance(all_view_safety, dict)
        or not bool(all_view_safety.get("memory_safe"))
    ):
        raise RuntimeError("artifact parallel benchmark 안전 정책이 올바르지 않습니다.")
    root = artifact_dir.expanduser().resolve()
    allowed_files = {Path("manifest.json")}
    for view in views:
        allowed_files.update(
            {
                Path(view) / "model.pt",
                Path(view) / "spatial_calibration.pt",
                Path(view) / "calibration.json",
            }
        )
        calibration = read_json(root / view / "calibration.json")
        if (
            calibration.get("view") != view
            or float(calibration.get("threshold", math.nan)) != float(thresholds[view])
            or calibration.get("calibration_b_scores") != calibration_b_scores[view]
            or calibration.get("memory_bank_shape") != bank_shapes[view]
            or calibration.get("patch_grid_shape") != grids[view]
            or calibration.get("normalization") != policy["normalization"]
            or calibration.get("aggregation") != policy["aggregation"]
        ):
            raise RuntimeError(f"{view}: calibration.json이 manifest와 다릅니다.")
        payload = torch.load(root / view / "spatial_calibration.pt", map_location="cpu", weights_only=True)
        validate_spatial_payload(payload, grids[view])
    paths = tuple(root.rglob("*"))
    if any(path.is_symlink() for path in paths):
        raise RuntimeError("v3 artifact symlink는 허용하지 않습니다.")
    actual_files = {path.relative_to(root) for path in paths if path.is_file()}
    if actual_files != allowed_files:
        raise RuntimeError(
            f"v3 artifact 파일 집합이 다릅니다: missing={sorted(map(str, allowed_files-actual_files))}, "
            f"extra={sorted(map(str, actual_files-allowed_files))}"
        )


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value
