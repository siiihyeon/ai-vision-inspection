#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""다중-view PatchCore/EfficientAD에서 공통으로 사용하는 안전 유틸리티.

이 모듈은 사용자 단독 실행용이 아니다. construction/AD 스크립트가 다음 기능을
공유하도록 모아 둔 파일이다.

* view 우선 데이터 폴더와 동일 제품 파일명 집합 검증
* PNG 로딩, stretch/검은색 letterbox 변환
* 추론 전용 artifact의 원자적 생성 및 안전한 로딩
* 프로세스 사이의 단일 CUDA GPU lock
* CUDA stream 1/2/3개에 대한 peak-memory/속도 실측 및 선택
* 제품 단위 지표, confusion matrix, score 분포, 오분류 이미지 저장
* torchvision ResNet34/EfficientNet-B0 기반 PatchCore 구현
"""

from __future__ import annotations

import contextlib
import hashlib
import json
import math
import os
import re
import shutil
import statistics
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterator, Sequence

import cv2
import matplotlib
import numpy as np
import torch
from sklearn.metrics import accuracy_score, confusion_matrix, precision_score, recall_score
from torch import nn
from torch.nn import functional as F
from torchvision.models import (
    EfficientNet_B0_Weights,
    ResNet34_Weights,
    efficientnet_b0,
    resnet34,
)
from torchvision.models.feature_extraction import create_feature_extractor

# GUI가 없는 실행 환경에서도 PNG plot을 만들 수 있도록 고정한다.
matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402


# EfficientAD/student artifact는 기존 v1 계약을 유지합니다. 운영 Vision Node와
# 공유하는 PatchCore bundle만 별도의 엄격한 v2 계약을 사용합니다.
ARTIFACT_FORMAT_VERSION = 1
PATCHCORE_ARTIFACT_FORMAT_VERSION = 2
PATCHCORE_EXPECTED_VIEWS = ("CAM_A_1", "CAM_A_2", "CAM_A_3", "CAM_B_1")
PATCHCORE_STATION_VIEWS = {
    "station_a": ["CAM_A_1", "CAM_A_2", "CAM_A_3"],
    "station_b": ["CAM_B_1"],
}
PATCHCORE_CAMERA_SERIAL_BY_VIEW = {
    "CAM_A_1": "DA9880512",
    "CAM_A_2": "DA9880516",
    "CAM_A_3": "DA7552836",
    "CAM_B_1": "DA7838410",
}
PATCHCORE_PARAMETER_KEYS = {
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
PATCHCORE_PREPROCESSING_KEYS = {
    "v_threshold",
    "connectivity",
    "remove_disconnected_noise",
    "check_connection",
}
SUPPORTED_RESIZE_MODES = {"padding", "stretch"}
SUPPORTED_SPLITS = ("training_set", "val_set", "test_set_normal", "test_set_anomaly")
SAFE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


@dataclass(frozen=True)
class DatasetIndex:
    """split -> view -> product filename -> PNG path."""

    root: Path
    view_names: tuple[str, ...]
    files: dict[str, dict[str, dict[str, Path]]]


@dataclass(frozen=True)
class ParallelCandidateResult:
    """CUDA stream 후보 하나의 memory 및 반복 시간 측정 결과."""

    parallel_count: int
    memory_safe: bool
    peak_extra_mib: float | None
    available_after_reserve_mib: float
    median_ms: float | None
    p95_ms: float | None
    error: str | None = None


@dataclass(frozen=True)
class ParallelBenchmarkResult:
    """Artifact에 그대로 JSON 저장할 병렬 추론 결정 기록."""

    selected_parallel_count: int
    reserve_mib: int
    warmup_runs: int
    benchmark_runs: int
    min_speedup_percent: float
    candidates: tuple[ParallelCandidateResult, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "selected_parallel_count": self.selected_parallel_count,
            "reserve_mib": self.reserve_mib,
            "warmup_runs": self.warmup_runs,
            "benchmark_runs": self.benchmark_runs,
            "min_speedup_percent": self.min_speedup_percent,
            "candidates": [asdict(candidate) for candidate in self.candidates],
        }


@dataclass(frozen=True)
class ParallelMemorySafetyResult:
    """저장된 CUDA stream 수가 현재 VRAM에서 안전한지 확인한 결과.

    이 검사는 속도를 다시 비교하지 않는다. construction 당시 선택된 stream 수를
    그대로 유지하되, 현재 free VRAM에서 실제 forward peak가 reserve를 넘지 않는지만
    확인하기 위한 실행 시 안전장치다.
    """

    parallel_count: int
    memory_safe: bool
    peak_extra_mib: float | None
    available_after_reserve_mib: float
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def set_reproducible_seed(seed: int) -> None:
    """Python 외 난수원을 고정한다. CUDA benchmark 자체의 시간은 변동 가능하다."""
    import random

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def validate_safe_name(value: str, field_name: str) -> str:
    """폴더 탈출과 이식성이 낮은 문자를 막는 artifact/view 이름 검사."""
    if not value or not SAFE_NAME_PATTERN.fullmatch(value):
        raise ValueError(
            f"{field_name}은 영문/숫자/점/밑줄/하이픈만 사용할 수 있습니다: {value!r}",
        )
    if value in {".", ".."}:
        raise ValueError(f"허용되지 않는 {field_name}: {value!r}")
    return value


def require_supported_versions() -> dict[str, str]:
    """본 코드가 작성·검토된 핵심 라이브러리 버전을 확인한다."""
    import anomalib
    import lightning
    import torchvision

    versions = {
        "anomalib": str(anomalib.__version__),
        "torch": str(torch.__version__),
        "torchvision": str(torchvision.__version__),
        "lightning": str(lightning.__version__),
    }
    if versions["anomalib"] != "2.6.0":
        raise RuntimeError(
            "이 코드는 anomalib==2.6.0 기준입니다. "
            f"현재 버전은 {versions['anomalib']}입니다.",
        )
    if versions["torch"] != "2.12.1+cu132":
        raise RuntimeError(
            "이 코드는 Ubuntu 환경의 torch==2.12.1+cu132 기준입니다. "
            f"현재 버전은 {versions['torch']}입니다.",
        )
    if versions["lightning"] != "2.6.5":
        raise RuntimeError(
            "이 코드는 lightning==2.6.5 환경용입니다. "
            f"현재 버전은 {versions['lightning']}입니다.",
        )
    return versions


def resolve_cuda_device() -> torch.device:
    """현재 프로젝트가 사용할 단일 NVIDIA CUDA GPU를 반환한다."""
    if not torch.cuda.is_available() or torch.cuda.device_count() < 1:
        raise RuntimeError("CUDA를 사용할 수 없습니다. NVIDIA driver/PyTorch CUDA 설치를 확인하세요.")
    if torch.cuda.device_count() == 1:
        print("It only has 1 GPU")
    device = torch.device("cuda:0")
    torch.cuda.set_device(device)
    print(f"CUDA device: {torch.cuda.get_device_name(device)}")
    return device


def synchronize_cuda_stage(device: torch.device, stage_name: str) -> None:
    """비동기 CUDA 오류를 작업 단계 직후 명확한 예외로 전파한다.

    construction은 default stream만 사용하므로 평가처럼 event polling을 할 필요는
    없다. 다만 CUDA kernel 오류가 다음 단계까지 늦게 나타나는 것을 막기 위해
    단계 경계에서 synchronize한다.
    """
    try:
        torch.cuda.synchronize(device)
    except Exception as exc:
        raise RuntimeError(f"CUDA synchronization failed after: {stage_name}") from exc


def _png_files_exact(directory: Path) -> dict[str, Path]:
    """바로 아래 PNG만 읽으며 대소문자까지 정확한 파일명을 key로 사용한다."""
    if not directory.is_dir():
        raise FileNotFoundError(f"필수 view 폴더가 없습니다: {directory}")
    paths = sorted(
        (path for path in directory.iterdir() if path.is_file() and path.suffix.lower() == ".png"),
        key=lambda path: path.name,
    )
    if not paths:
        raise ValueError(f"PNG가 하나도 없습니다: {directory}")

    # Linux에서도 다른 파일 시스템으로 옮길 때 충돌하지 않도록 대소문자만 다른
    # 이름을 오류로 취급한다.
    casefolded: dict[str, str] = {}
    result: dict[str, Path] = {}
    for path in paths:
        folded = path.name.casefold()
        previous = casefolded.get(folded)
        if previous is not None and previous != path.name:
            raise ValueError(f"대소문자만 다른 PNG 파일명이 공존합니다: {previous}, {path.name}")
        casefolded[folded] = path.name
        result[path.name] = path
    return result


def index_multiview_dataset(
    dataset_root: Path,
    view_names: Sequence[str],
    required_splits: Sequence[str] = SUPPORTED_SPLITS,
) -> DatasetIndex:
    """모든 view의 제품 파일명 집합이 정확히 같은지 검사한다."""
    root = dataset_root.expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"DATASET_ROOT가 없습니다: {root}")
    views = tuple(validate_safe_name(name, "view name") for name in view_names)
    if not views or len(set(views)) != len(views):
        raise ValueError("VIEW_NAMES에는 중복 없는 view 이름을 1개 이상 입력해야 합니다.")

    files: dict[str, dict[str, dict[str, Path]]] = {}
    for split in required_splits:
        if split not in SUPPORTED_SPLITS:
            raise ValueError(f"알 수 없는 dataset split: {split}")
        split_views: dict[str, dict[str, Path]] = {}
        reference_names: set[str] | None = None
        reference_view = ""
        for view in views:
            current = _png_files_exact(root / split / view)
            names = set(current)
            if reference_names is None:
                reference_names = names
                reference_view = view
            elif names != reference_names:
                missing = sorted(reference_names - names)
                extra = sorted(names - reference_names)
                raise ValueError(
                    f"{split}/{view}의 제품 파일명 집합이 {reference_view}와 다릅니다. "
                    f"누락={missing[:10]}, 추가={extra[:10]}",
                )
            split_views[view] = current
        files[split] = split_views
    return DatasetIndex(root=root, view_names=views, files=files)


def sorted_product_names(index: DatasetIndex, split: str) -> list[str]:
    """VIEW_NAMES 첫 view를 기준으로 제품 파일명을 정확한 대소문자로 정렬한다."""
    return sorted(index.files[split][index.view_names[0]])


def _validate_mono8_compatible_png(path: Path) -> None:
    """Mono8 또는 세 color 채널 값이 같은 8-bit PNG만 허용합니다."""

    try:
        encoded = np.fromfile(str(path), dtype=np.uint8)
    except OSError as exc:
        raise RuntimeError(f"PNG 읽기 실패: {path}") from exc
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None or image.dtype != np.uint8:
        raise RuntimeError(f"Mono8 호환 8-bit PNG가 아닙니다: {path}")
    if image.ndim == 2:
        return
    if image.ndim != 3 or image.shape[2] not in {3, 4}:
        raise RuntimeError(f"Mono8 호환 channel 구성이 아닙니다: {path}")
    channels = image[:, :, :3]
    if not (
        np.array_equal(channels[:, :, 0], channels[:, :, 1])
        and np.array_equal(channels[:, :, 1], channels[:, :, 2])
    ):
        raise RuntimeError(f"RGB channel 값이 달라 Mono8 데이터가 아닙니다: {path}")


def validate_all_indexed_pngs(
    index: DatasetIndex, *, require_mono8_compatible: bool = False
) -> None:
    """학습/평가를 시작하기 전에 색상 PNG 전체를 실제로 디코딩한다.

    파일 목록만 검사하고 긴 작업을 시작했다가 마지막에 손상 파일을 발견하는 일을
    막기 위한 선행 검사다. 정상 파일은 이후 model input 생성 시 다시 읽는다.
    """
    checked: set[Path] = set()
    for split_views in index.files.values():
        for files in split_views.values():
            for path in files.values():
                if path not in checked:
                    read_png_rgb(path)
                    if require_mono8_compatible:
                        _validate_mono8_compatible_png(path)
                    checked.add(path)
    suffix = " (Mono8 compatible)" if require_mono8_compatible else ""
    print(f"PNG integrity check passed: {len(checked)} files{suffix}")


def read_png_rgb(path: Path) -> np.ndarray:
    """한글/공백이 포함된 Linux 경로를 지원하며 RGB8 배열로 디코딩한다."""
    try:
        encoded = np.fromfile(str(path), dtype=np.uint8)
    except OSError as exc:
        raise RuntimeError(f"PNG 읽기 실패: {path}") from exc
    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError(f"손상되었거나 지원하지 않는 PNG입니다: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def resize_rgb(image_rgb: np.ndarray, resolution: Sequence[int], resize_mode: str) -> np.ndarray:
    """RGB 이미지를 stretch 또는 검은색 letterbox 방식으로 고정 해상도화한다."""
    if len(resolution) != 2:
        raise ValueError("input_resolution은 (height, width) 두 정수여야 합니다.")
    target_h, target_w = (int(resolution[0]), int(resolution[1]))
    if target_h <= 0 or target_w <= 0:
        raise ValueError("input_resolution 값은 양수여야 합니다.")
    if resize_mode not in SUPPORTED_RESIZE_MODES:
        raise ValueError(f"resize_mode은 {sorted(SUPPORTED_RESIZE_MODES)} 중 하나여야 합니다.")

    source_h, source_w = image_rgb.shape[:2]
    if resize_mode == "stretch":
        interpolation = cv2.INTER_AREA if source_h > target_h or source_w > target_w else cv2.INTER_LINEAR
        return cv2.resize(image_rgb, (target_w, target_h), interpolation=interpolation)

    scale = min(target_w / source_w, target_h / source_h)
    resized_w = max(1, min(target_w, int(round(source_w * scale))))
    resized_h = max(1, min(target_h, int(round(source_h * scale))))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(image_rgb, (resized_w, resized_h), interpolation=interpolation)
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
    x0 = (target_w - resized_w) // 2
    y0 = (target_h - resized_h) // 2
    canvas[y0 : y0 + resized_h, x0 : x0 + resized_w] = resized
    return canvas


def load_model_input(path: Path, resolution: Sequence[int], resize_mode: str) -> torch.Tensor:
    """PNG를 [3,H,W] float32 RGB tensor([0,1])로 변환한다."""
    image = resize_rgb(read_png_rgb(path), resolution, resize_mode)
    return torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1).float().div_(255.0)


def write_model_input_png(path: Path, tensor: torch.Tensor) -> None:
    """정규화 전 모델 입력 tensor를 사람이 볼 수 있는 RGB PNG로 저장한다."""
    image = _model_input_rgb(tensor)
    _write_rgb_png(path, image)


def _model_input_rgb(tensor: torch.Tensor) -> np.ndarray:
    value = tensor.detach().cpu()
    if value.ndim == 4 and value.shape[0] == 1:
        value = value[0]
    if value.ndim != 3 or value.shape[0] != 3:
        raise ValueError("model input은 [3,H,W] 또는 [1,3,H,W]여야 합니다.")
    return value.clamp(0, 1).mul(255).round().byte().permute(1, 2, 0).numpy()


def _write_rgb_png(path: Path, image: np.ndarray) -> None:
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("저장할 이미지는 uint8 RGB여야 합니다.")
    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError(f"PNG 인코딩 실패: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded.tofile(str(path))


def validate_percentile_margin(percentile: float, margin: float, epsilon: float) -> None:
    """Threshold 관련 사용자 설정의 허용 범위를 검사한다."""
    if not 0.0 <= float(percentile) <= 100.0:
        raise ValueError("threshold_percentile은 0~100이어야 합니다.")
    if not 0.0 <= float(margin) <= 1.0:
        raise ValueError("customized_margin은 0.0~1.0이어야 합니다.")
    if not math.isfinite(float(epsilon)) or float(epsilon) <= 0:
        raise ValueError("threshold_epsilon은 유한한 양수여야 합니다.")


def percentile_threshold(scores: Sequence[float], percentile: float, epsilon: float) -> float:
    """정상 validation raw score에서 percentile threshold를 계산한다."""
    values = np.asarray(scores, dtype=np.float64)
    if values.size == 0 or not np.all(np.isfinite(values)):
        raise ValueError("Validation raw score가 비어 있거나 NaN/Inf를 포함합니다.")
    threshold = float(np.percentile(values, percentile))
    if threshold <= epsilon:
        raise RuntimeError(
            f"Validation threshold={threshold:.12g}가 epsilon={epsilon:.12g} 이하입니다. "
            "안전한 normalization이 불가능하므로 construction을 중단합니다.",
        )
    return threshold


def normalized_score(raw_score: float, threshold: float, epsilon: float) -> float:
    """View별 threshold가 서로 달라도 경계가 1이 되도록 점수를 정규화한다."""
    if threshold <= epsilon:
        raise RuntimeError("Artifact threshold가 epsilon 이하입니다.")
    value = float(raw_score) / float(threshold)
    if not math.isfinite(value):
        raise RuntimeError("Normalized score가 NaN/Inf입니다.")
    return value


def anomaly_from_normalized(score: float, margin: float) -> bool:
    """경계와 정확히 같은 경우도 anomaly로 보는 <= margin 정책."""
    return bool(score >= 1.0 - margin)


class PatchCoreViewModel(nn.Module):
    """Anomalib PatchCore 점수식을 사용하는 ResNet34/EfficientNet-B0 view 모델.

    torchvision의 명시적인 stage 이름을 사용해 사용자가 정의한 layer 1/2 의미를
    보존한다. ImageNet normalization, 3x3 average pooling, multi-scale feature
    결합, 논문식 kNN reweighting은 Anomalib PatchCore 구현과 동일하게 구성한다.
    """

    def __init__(
        self,
        backbone: str,
        layer_numbers: Sequence[int],
        num_neighbors: int,
        pretrained: bool,
        distance_chunk_size: int = 1024,
    ) -> None:
        super().__init__()
        self.backbone_name = backbone
        self.layer_numbers = tuple(int(value) for value in layer_numbers)
        self.num_neighbors = int(num_neighbors)
        self.distance_chunk_size = int(distance_chunk_size)
        if not self.layer_numbers or len(set(self.layer_numbers)) != len(self.layer_numbers):
            raise ValueError("feature_layers에는 중복 없는 layer 번호가 1개 이상 필요합니다.")
        if self.num_neighbors < 1:
            raise ValueError("k(num_neighbors)는 1 이상이어야 합니다.")
        if self.distance_chunk_size < 1:
            raise ValueError("distance_chunk_size는 1 이상이어야 합니다.")

        if backbone == "resnet34":
            if any(number not in {1, 2, 3, 4} for number in self.layer_numbers):
                raise ValueError("ResNet34 feature layer는 1~4만 지원합니다.")
            weights = ResNet34_Weights.DEFAULT if pretrained else None
            base = resnet34(weights=weights)
            return_nodes = {f"layer{number}": f"layer{number}" for number in self.layer_numbers}
        elif backbone == "efficientnet_b0":
            if any(number not in set(range(1, 8)) for number in self.layer_numbers):
                raise ValueError("EfficientNet-B0 MBConv feature layer는 1~7만 지원합니다.")
            weights = EfficientNet_B0_Weights.DEFAULT if pretrained else None
            base = efficientnet_b0(weights=weights)
            return_nodes = {f"features.{number}": f"layer{number}" for number in self.layer_numbers}
        else:
            raise ValueError("backbone은 'resnet34' 또는 'efficientnet_b0'이어야 합니다.")

        self.feature_extractor = create_feature_extractor(base, return_nodes=return_nodes)
        self.feature_extractor.eval()
        self.feature_pooler = nn.AvgPool2d(kernel_size=3, stride=1, padding=1)
        self.register_buffer("memory_bank", torch.empty((0, 0), dtype=torch.float32))
        self.register_buffer("imagenet_mean", torch.tensor([0.485, 0.456, 0.406])[None, :, None, None])
        self.register_buffer("imagenet_std", torch.tensor([0.229, 0.224, 0.225])[None, :, None, None])

    def train(self, mode: bool = True) -> "PatchCoreViewModel":
        # PatchCore backbone은 construction 중에도 pretrained eval 상태를 유지한다.
        super().train(False)
        self.feature_extractor.eval()
        return self

    @torch.inference_mode()
    def _generate_embedding_and_grid(
        self, images: torch.Tensor
    ) -> tuple[torch.Tensor, tuple[int, int]]:
        normalized = (images - self.imagenet_mean) / self.imagenet_std
        features = self.feature_extractor(normalized)
        ordered = [self.feature_pooler(features[f"layer{number}"]) for number in self.layer_numbers]
        # 사용자가 layer 순서를 바꾸더라도 가장 조밀한 spatial grid를 기준으로 한다.
        reference_size = max((tensor.shape[-2:] for tensor in ordered), key=lambda size: size[0] * size[1])
        resized = [
            tensor if tensor.shape[-2:] == reference_size else F.interpolate(tensor, reference_size, mode="bilinear")
            for tensor in ordered
        ]
        combined = torch.cat(resized, dim=1)
        embedding = combined.permute(0, 2, 3, 1).reshape(-1, combined.shape[1])
        return embedding, (int(reference_size[0]), int(reference_size[1]))

    @torch.inference_mode()
    def generate_embedding(self, images: torch.Tensor) -> torch.Tensor:
        embedding, _ = self._generate_embedding_and_grid(images)
        return embedding

    @staticmethod
    def euclidean_dist(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        x_norm = x.pow(2).sum(dim=-1, keepdim=True)
        y_norm = y.pow(2).sum(dim=-1, keepdim=True)
        result = torch.matmul(x, y.transpose(-2, -1))
        result.mul_(-2).add_(x_norm).add_(y_norm.transpose(-2, -1))
        return result.clamp_min_(0).sqrt_()

    def nearest_neighbors(self, embedding: torch.Tensor, count: int) -> tuple[torch.Tensor, torch.Tensor]:
        if self.memory_bank.numel() == 0:
            raise RuntimeError("PatchCore memory bank가 비어 있습니다.")
        count = min(int(count), int(self.memory_bank.shape[0]))
        all_scores: list[torch.Tensor] = []
        all_locations: list[torch.Tensor] = []
        for start in range(0, embedding.shape[0], self.distance_chunk_size):
            chunk = embedding[start : start + self.distance_chunk_size]
            distances = self.euclidean_dist(chunk, self.memory_bank)
            if count == 1:
                scores, locations = distances.min(dim=1)
            else:
                scores, locations = distances.topk(k=count, largest=False, dim=1)
            all_scores.append(scores)
            all_locations.append(locations)
        return torch.cat(all_scores), torch.cat(all_locations)

    def compute_image_score(
        self,
        patch_scores: torch.Tensor,
        locations: torch.Tensor,
        embedding: torch.Tensor,
        batch_size: int,
    ) -> torch.Tensor:
        patch_scores = patch_scores.reshape(batch_size, -1)
        locations = locations.reshape(batch_size, -1)
        if self.num_neighbors == 1 or self.memory_bank.shape[0] == 1:
            return patch_scores.amax(dim=1)
        patch_count = patch_scores.shape[1]
        max_patches = torch.argmax(patch_scores, dim=1)
        rows = torch.arange(batch_size, device=embedding.device)
        max_features = embedding.reshape(batch_size, patch_count, -1)[rows, max_patches]
        score = patch_scores[rows, max_patches]
        nearest_index = locations[rows, max_patches]
        nearest_sample = self.memory_bank[nearest_index]
        _, support = self.nearest_neighbors(nearest_sample, self.num_neighbors)
        distances = self.euclidean_dist(max_features.unsqueeze(1), self.memory_bank[support])
        weights = (1.0 - F.softmax(distances.squeeze(1), dim=1))[..., 0]
        return weights * score

    @torch.inference_mode()
    def forward(self, images: torch.Tensor) -> torch.Tensor:
        embedding, _ = self._generate_embedding_and_grid(images)
        patch_scores, locations = self.nearest_neighbors(embedding, 1)
        return self.compute_image_score(patch_scores, locations, embedding, images.shape[0])

    @torch.inference_mode()
    def score_and_anomaly_map(
        self, images: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Image score와 최근접 memory-bank patch distance map을 함께 반환한다."""

        embedding, (grid_h, grid_w) = self._generate_embedding_and_grid(images)
        patch_scores, locations = self.nearest_neighbors(embedding, 1)
        image_scores = self.compute_image_score(
            patch_scores,
            locations,
            embedding,
            images.shape[0],
        )
        maps = patch_scores.reshape(images.shape[0], 1, grid_h, grid_w)
        maps = F.interpolate(
            maps,
            size=images.shape[-2:],
            mode="bilinear",
            align_corners=False,
        )
        return image_scores, maps[:, 0]


def patchcore_select_coreset(embeddings: torch.Tensor, ratio: float) -> torch.Tensor:
    """Anomalib 2.6.0의 KCenterGreedy로 PatchCore coreset을 선택한다."""
    if not 0.0 < float(ratio) <= 1.0:
        raise ValueError("coreset_ratio는 0보다 크고 1 이하여야 합니다.")
    if float(ratio) == 1.0:
        return embeddings
    from anomalib.models.components import KCenterGreedy

    sampler = KCenterGreedy(embedding=embeddings, sampling_ratio=float(ratio))
    return sampler.sample_coreset()


def export_patchcore_state(model: PatchCoreViewModel) -> dict[str, Any]:
    """Dynamic memory bank를 일반 state와 분리해 weights_only 로딩 가능하게 저장한다."""
    state = {key: value.detach().cpu() for key, value in model.state_dict().items() if key != "memory_bank"}
    return {"state_dict": state, "memory_bank": model.memory_bank.detach().cpu()}


def load_patchcore_state(model: PatchCoreViewModel, payload: dict[str, Any]) -> None:
    """PatchCore tensor artifact를 검증해 모델에 적용한다."""
    state = payload.get("state_dict")
    bank = payload.get("memory_bank")
    if not isinstance(state, dict) or not isinstance(bank, torch.Tensor) or bank.ndim != 2 or bank.numel() == 0:
        raise RuntimeError("유효하지 않은 PatchCore artifact입니다.")
    incompatible = model.load_state_dict(state, strict=False)
    if incompatible.unexpected_keys or set(incompatible.missing_keys) != {"memory_bank"}:
        raise RuntimeError(
            f"PatchCore state key 불일치: missing={incompatible.missing_keys}, "
            f"unexpected={incompatible.unexpected_keys}",
        )
    model.memory_bank = bank


def create_efficientad_model(
    model_size: str,
    teacher_out_channels: int,
    padding: bool,
    pad_maps: bool,
) -> nn.Module:
    """Anomalib 2.6.0의 순수 PyTorch EfficientAD teacher/student/AE 모델을 만든다."""
    from anomalib.models.image.efficient_ad.torch_model import EfficientAdModel, EfficientAdModelSize

    size = EfficientAdModelSize(model_size)
    return EfficientAdModel(
        teacher_out_channels=int(teacher_out_channels),
        model_size=size,
        padding=bool(padding),
        pad_maps=bool(pad_maps),
    )


def create_efficientad_with_pretrained_teacher(config: dict[str, Any], device: torch.device) -> nn.Module:
    """Anomalib downloader/cache를 이용해 pretrained teacher까지 준비한 모델을 만든다."""
    from anomalib.models import EfficientAd

    wrapper = EfficientAd(
        imagenet_dir=Path(config["imagenette_dir"]),
        teacher_out_channels=int(config["teacher_out_channels"]),
        model_size=str(config["model_size"]),
        lr=float(config["learning_rate"]),
        weight_decay=float(config["weight_decay"]),
        padding=bool(config["padding"]),
        pad_maps=bool(config["pad_maps"]),
        pre_processor=False,
        post_processor=False,
        evaluator=False,
        visualizer=False,
    ).to(device)
    wrapper.prepare_pretrained_model()
    model = wrapper.model
    model.teacher.eval()
    for parameter in model.teacher.parameters():
        parameter.requires_grad_(False)
    return model


def export_efficientad_state(model: nn.Module) -> dict[str, Any]:
    """Teacher/student/AE/mean-std/quantile tensor만 CPU artifact로 내보낸다."""
    return {"state_dict": {key: value.detach().cpu() for key, value in model.state_dict().items()}}


def load_efficientad_state(model: nn.Module, payload: dict[str, Any]) -> None:
    state = payload.get("state_dict")
    if not isinstance(state, dict) or not state:
        raise RuntimeError("유효하지 않은 EfficientAD artifact입니다.")
    model.load_state_dict(state, strict=True)


def model_score_tensor(model: nn.Module, image: torch.Tensor) -> torch.Tensor:
    """PatchCore tensor 또는 Anomalib InferenceBatch에서 image score tensor를 꺼낸다."""
    output = model(image)
    if isinstance(output, torch.Tensor):
        score = output
    elif hasattr(output, "pred_score"):
        score = output.pred_score
    else:
        raise RuntimeError(f"지원하지 않는 model output type: {type(output)!r}")
    return score.reshape(-1)


def _run_stream_workload(
    models: Sequence[nn.Module],
    inputs: Sequence[torch.Tensor],
    parallel_count: int,
    streams: Sequence[torch.cuda.Stream] | None = None,
) -> list[float]:
    """동일 workload 전체를 wave 단위 CUDA stream으로 실행하고 score를 반환한다."""
    if parallel_count < 1:
        raise ValueError("parallel_count는 1 이상이어야 합니다.")
    scores: list[float] = []
    active_streams = list(streams) if streams is not None else [torch.cuda.Stream() for _ in range(parallel_count)]
    if len(active_streams) < parallel_count:
        raise ValueError("제공된 CUDA stream 수가 parallel_count보다 작습니다.")
    with torch.inference_mode():
        for start in range(0, len(models), parallel_count):
            wave_models = models[start : start + parallel_count]
            wave_inputs = inputs[start : start + parallel_count]
            outputs: list[torch.Tensor] = []
            for slot, (model, image) in enumerate(zip(wave_models, wave_inputs, strict=True)):
                with torch.cuda.stream(active_streams[slot]):
                    outputs.append(model_score_tensor(model, image))
            for stream in active_streams[: len(wave_models)]:
                stream.synchronize()
            scores.extend(float(output[0].detach().cpu()) for output in outputs)
    return scores


def _percentile_95(values: Sequence[float]) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), 95))


def benchmark_parallelism(
    models: Sequence[nn.Module],
    sample_inputs: Sequence[torch.Tensor],
    reserve_mib: int = 2048,
    warmup_runs: int = 3,
    benchmark_runs: int = 10,
    min_speedup_percent: float = 5.0,
    maximum_parallel_count: int = 3,
) -> ParallelBenchmarkResult:
    """후보 1/2/3의 실제 memory와 wall time을 측정해 최종 stream 수를 선택한다."""
    if not torch.cuda.is_available():
        raise RuntimeError("병렬 추론 benchmark에는 CUDA가 필요합니다.")
    if len(models) != len(sample_inputs) or not models:
        raise ValueError("models와 sample_inputs는 길이가 같은 비어 있지 않은 목록이어야 합니다.")
    if warmup_runs < 0 or benchmark_runs < 1:
        raise ValueError("warmup_runs>=0, benchmark_runs>=1이어야 합니다.")
    if not 0.0 <= min_speedup_percent < 100.0:
        raise ValueError("min_speedup_percent는 0 이상 100 미만이어야 합니다.")

    workload_size = min(len(models), max(1, int(maximum_parallel_count)))
    workload_models = list(models[:workload_size])
    workload_inputs = list(sample_inputs[:workload_size])
    candidates: list[ParallelCandidateResult] = []
    torch.cuda.synchronize()

    for count in range(1, workload_size + 1):
        error: str | None = None
        peak_extra_mib: float | None = None
        median_ms: float | None = None
        p95_ms: float | None = None
        memory_safe = False
        try:
            candidate_streams = [torch.cuda.Stream() for _ in range(count)]
            torch.cuda.empty_cache()
            for _ in range(warmup_runs):
                _run_stream_workload(workload_models, workload_inputs, count, candidate_streams)
            torch.cuda.synchronize()
            torch.cuda.empty_cache()
            free_bytes, _ = torch.cuda.mem_get_info()
            available_bytes = max(0, free_bytes - reserve_mib * 1024**2)
            base_allocated = torch.cuda.memory_allocated()
            torch.cuda.reset_peak_memory_stats()
            _run_stream_workload(workload_models, workload_inputs, count, candidate_streams)
            torch.cuda.synchronize()
            peak_extra = max(0, torch.cuda.max_memory_allocated() - base_allocated)
            peak_extra_mib = peak_extra / 1024**2
            memory_safe = peak_extra <= available_bytes

            if memory_safe:
                durations: list[float] = []
                for _ in range(benchmark_runs):
                    torch.cuda.synchronize()
                    started = time.perf_counter()
                    _run_stream_workload(workload_models, workload_inputs, count, candidate_streams)
                    torch.cuda.synchronize()
                    durations.append((time.perf_counter() - started) * 1000.0)
                median_ms = float(statistics.median(durations))
                p95_ms = _percentile_95(durations)
        except torch.cuda.OutOfMemoryError as exc:
            error = f"CUDA OOM: {exc}"
            memory_safe = False
            torch.cuda.empty_cache()
        free_now, _ = torch.cuda.mem_get_info()
        available_after_reserve_mib = max(0.0, free_now / 1024**2 - reserve_mib)
        candidates.append(
            ParallelCandidateResult(
                parallel_count=count,
                memory_safe=memory_safe,
                peak_extra_mib=peak_extra_mib,
                available_after_reserve_mib=available_after_reserve_mib,
                median_ms=median_ms,
                p95_ms=p95_ms,
                error=error,
            ),
        )

    first = candidates[0]
    if not first.memory_safe or first.median_ms is None or first.p95_ms is None:
        raise RuntimeError("병렬 개수 1도 VRAM 안전 조건을 통과하지 못했습니다.")

    best = first
    required_factor = 1.0 - min_speedup_percent / 100.0
    for candidate in candidates[1:]:
        if not candidate.memory_safe or candidate.median_ms is None or candidate.p95_ms is None:
            continue
        median_is_faster = candidate.median_ms <= best.median_ms * required_factor
        p95_not_worse = candidate.p95_ms <= best.p95_ms
        if median_is_faster and p95_not_worse:
            best = candidate

    result = ParallelBenchmarkResult(
        selected_parallel_count=best.parallel_count,
        reserve_mib=int(reserve_mib),
        warmup_runs=int(warmup_runs),
        benchmark_runs=int(benchmark_runs),
        min_speedup_percent=float(min_speedup_percent),
        candidates=tuple(candidates),
    )
    print_parallel_benchmark(result)
    return result


def verify_saved_parallelism_memory_safety(
    models: Sequence[nn.Module],
    sample_inputs: Sequence[torch.Tensor],
    parallel_count: int,
    reserve_mib: int = 2048,
    warmup_runs: int = 3,
) -> ParallelMemorySafetyResult:
    """저장된 stream 수의 현재 VRAM 안전성만 실측한다.

    CUDA timing은 GPU clock/온도/Linux scheduler에 따라 흔들릴 수 있으므로
    construction 때 선택한 병렬 수를 runtime speed benchmark로 다시 바꾸지 않는다.
    오직 ``peak_extra <= free_vram - reserve`` 조건만 통과하면 저장된 개수를 쓴다.
    """
    if not torch.cuda.is_available():
        raise RuntimeError("병렬 추론 VRAM 검증에는 CUDA가 필요합니다.")
    if len(models) != len(sample_inputs) or not models:
        raise ValueError("models와 sample_inputs는 길이가 같은 비어 있지 않은 목록이어야 합니다.")
    if not 1 <= int(parallel_count) <= min(3, len(models)):
        raise ValueError("저장된 parallel_count가 view 수/상한 범위를 벗어났습니다.")
    if int(reserve_mib) < 0 or int(warmup_runs) < 0:
        raise ValueError("reserve_mib와 warmup_runs는 0 이상이어야 합니다.")

    peak_extra_mib: float | None = None
    error: str | None = None
    memory_safe = False
    available_after_reserve_mib = 0.0
    try:
        streams = [torch.cuda.Stream() for _ in range(int(parallel_count))]
        torch.cuda.synchronize()
        for _ in range(int(warmup_runs)):
            _run_stream_workload(models, sample_inputs, int(parallel_count), streams)
        torch.cuda.synchronize()
        torch.cuda.empty_cache()

        free_bytes, _ = torch.cuda.mem_get_info()
        available_bytes = max(0, free_bytes - int(reserve_mib) * 1024**2)
        available_after_reserve_mib = available_bytes / 1024**2
        base_allocated = torch.cuda.memory_allocated()
        torch.cuda.reset_peak_memory_stats()
        _run_stream_workload(models, sample_inputs, int(parallel_count), streams)
        torch.cuda.synchronize()
        peak_extra_bytes = max(0, torch.cuda.max_memory_allocated() - base_allocated)
        peak_extra_mib = peak_extra_bytes / 1024**2
        memory_safe = peak_extra_bytes <= available_bytes
    except torch.cuda.OutOfMemoryError as exc:
        error = f"CUDA OOM: {exc}"
        torch.cuda.empty_cache()

    result = ParallelMemorySafetyResult(
        parallel_count=int(parallel_count),
        memory_safe=memory_safe,
        peak_extra_mib=peak_extra_mib,
        available_after_reserve_mib=available_after_reserve_mib,
        error=error,
    )
    print("\n[CUDA saved-parallelism VRAM safety check]")
    print(
        f"  streams={result.parallel_count} | safe={result.memory_safe} | "
        f"peak_extra={result.peak_extra_mib} MiB | "
        f"free_after_reserve={result.available_after_reserve_mib:.1f} MiB"
        + (f" | {result.error}" if result.error else ""),
    )
    return result


def print_parallel_benchmark(result: ParallelBenchmarkResult) -> None:
    """사용자가 선택 과정을 확인할 수 있도록 후보별 결과를 출력한다."""
    print("\n[CUDA parallel inference benchmark]")
    for candidate in result.candidates:
        print(
            f"  streams={candidate.parallel_count} | safe={candidate.memory_safe} | "
            f"peak_extra={candidate.peak_extra_mib} MiB | "
            f"free_after_reserve={candidate.available_after_reserve_mib:.1f} MiB | "
            f"median={candidate.median_ms} ms | p95={candidate.p95_ms} ms"
            + (f" | {candidate.error}" if candidate.error else ""),
        )
    print(f"Concurrent view forwards selected: {result.selected_parallel_count}")


class GpuProcessLock:
    """서로 다른 프로그램 프로세스가 CUDA GPU 0을 동시에 점유하지 못하게 한다.

    동일 프로세스 내부 view별 CUDA stream에는 이 lock을 사용하지 않는다. Linux의
    advisory file lock은 프로세스가 비정상 종료돼도 운영체제가 자동 해제한다.
    """

    def __init__(self, device_index: int = 0, timeout_seconds: int = 300, poll_seconds: float = 1.0) -> None:
        self.device_index = int(device_index)
        self.timeout_seconds = int(timeout_seconds)
        self.poll_seconds = float(poll_seconds)
        self.path = Path(tempfile.gettempdir()) / f"multiview_ad_cuda_{self.device_index}.lock"
        self._file: Any = None

    def __enter__(self) -> "GpuProcessLock":
        import fcntl

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.touch(exist_ok=True)
        self._file = self.path.open("r+b")
        self._file.seek(0, os.SEEK_END)
        if self._file.tell() == 0:
            self._file.write(b"0" * 32)
            self._file.flush()
        deadline = time.monotonic() + self.timeout_seconds
        while True:
            try:
                fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                self._file.seek(0)
                self._file.write(str(os.getpid()).encode("ascii").ljust(32, b" "))
                self._file.truncate(32)
                self._file.flush()
                print(f"GPU lock acquired: CUDA {self.device_index} (PID={os.getpid()})")
                return self
            except OSError:
                if time.monotonic() >= deadline:
                    self._file.close()
                    self._file = None
                    raise TimeoutError(
                        f"{self.timeout_seconds}초 안에 CUDA {self.device_index} GPU lock을 얻지 못했습니다.",
                    )
                time.sleep(self.poll_seconds)

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        import fcntl

        if self._file is not None:
            try:
                fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            finally:
                self._file.close()
                self._file = None
                print(f"GPU lock released: CUDA {self.device_index}")


def _assert_child_path(root: Path, target: Path) -> None:
    root_resolved = root.resolve()
    target_resolved = target.resolve()
    if root_resolved == target_resolved or root_resolved not in target_resolved.parents:
        raise RuntimeError(f"안전하지 않은 출력 경로입니다: {target_resolved}")


@contextlib.contextmanager
def managed_output_directory(
    parent: Path,
    final_name: str,
    overwrite: bool,
) -> Iterator[Path]:
    """임시 폴더에서 완성한 뒤 성공한 경우에만 최종 폴더로 원자적 교체한다."""
    validate_safe_name(final_name, "output folder name")
    root = parent.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    final = root / final_name
    temporary = root / f".{final_name}.building_{os.getpid()}_{time.time_ns()}"
    backup = root / f".{final_name}.backup_{os.getpid()}_{time.time_ns()}"
    _assert_child_path(root, final)
    _assert_child_path(root, temporary)
    _assert_child_path(root, backup)
    if final.exists() and not overwrite:
        raise FileExistsError(f"출력 폴더가 이미 존재합니다: {final}")
    temporary.mkdir(parents=False, exist_ok=False)
    committed = False
    try:
        yield temporary
        if final.exists():
            os.replace(final, backup)
        try:
            os.replace(temporary, final)
            committed = True
        except Exception:
            if backup.exists() and not final.exists():
                os.replace(backup, final)
            raise
        if backup.exists():
            shutil.rmtree(backup)
    finally:
        if temporary.exists():
            shutil.rmtree(temporary)
        if not committed and backup.exists() and not final.exists():
            os.replace(backup, final)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2, sort_keys=True)


def read_json(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"JSON artifact를 읽을 수 없습니다: {path}") from exc


def torch_save_verified(path: Path, payload: dict[str, Any]) -> None:
    """tensor/primitives 전용 파일을 임시 저장한 뒤 weights_only 재로딩 검증한다."""
    temporary = path.with_name(path.name + ".tmp")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        torch.save(payload, temporary)
        loaded = torch.load(temporary, map_location="cpu", weights_only=True)
        if not isinstance(loaded, dict) or loaded.keys() != payload.keys():
            raise RuntimeError(f"저장 검증에 실패했습니다: {path}")
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def artifact_directory_sha256(root: Path) -> str:
    """Vision Node와 동일하게 상대경로와 파일 내용을 합쳐 hash합니다."""

    resolved = root.expanduser().resolve()
    if not resolved.is_dir() or root.is_symlink():
        raise RuntimeError("Artifact 경로는 실제 directory여야 합니다.")
    files = sorted(path for path in resolved.rglob("*") if path.is_file())
    if not files:
        raise RuntimeError("Artifact directory가 비어 있습니다.")
    digest = hashlib.sha256()
    for path in files:
        if path.is_symlink():
            raise RuntimeError(f"Artifact symlink는 허용되지 않습니다: {path}")
        digest.update(path.relative_to(resolved).as_posix().encode("utf-8"))
        digest.update(b"\0")
        try:
            with path.open("rb") as stream:
                while chunk := stream.read(1024 * 1024):
                    digest.update(chunk)
        except OSError as exc:
            raise RuntimeError(f"Artifact 파일 hash 실패: {path}") from exc
        digest.update(b"\0")
    return digest.hexdigest()


def validate_artifact_manifest(
    manifest: dict[str, Any],
    expected_algorithm: str,
    artifact_name: str,
    artifact_dir: Path | None = None,
) -> None:
    """추론 전에 manifest, calibration 및 file set을 방어적으로 검증합니다."""

    if not isinstance(manifest, dict):
        raise RuntimeError("Artifact manifest 최상위 값은 object여야 합니다.")
    expected_format = (
        PATCHCORE_ARTIFACT_FORMAT_VERSION
        if expected_algorithm == "patchcore"
        else ARTIFACT_FORMAT_VERSION
    )
    if manifest.get("format_version") != expected_format:
        raise RuntimeError(
            "지원하지 않는 artifact format_version입니다: "
            f"expected={expected_format}, actual={manifest.get('format_version')}"
        )
    if manifest.get("algorithm") != expected_algorithm:
        raise RuntimeError(
            f"Artifact algorithm 불일치: expected={expected_algorithm}, "
            f"actual={manifest.get('algorithm')}"
        )
    if manifest.get("artifact_name") != artifact_name:
        raise RuntimeError("Artifact 폴더명과 manifest artifact_name이 다릅니다.")

    views = manifest.get("view_names")
    thresholds = manifest.get("thresholds")
    validation_scores = manifest.get("validation_raw_scores")
    benchmark = manifest.get("parallel_benchmark")
    if (
        not isinstance(views, list)
        or not views
        or not all(isinstance(view, str) for view in views)
        or len(set(views)) != len(views)
    ):
        raise RuntimeError("Artifact view_names가 비어 있거나 중복됩니다.")
    for view in views:
        validate_safe_name(str(view), "artifact view name")
    if not isinstance(thresholds, dict):
        raise RuntimeError("Artifact thresholds 구조가 올바르지 않습니다.")
    if not isinstance(validation_scores, dict) or set(validation_scores) != set(views):
        raise RuntimeError("Artifact validation_raw_scores의 view key가 일치하지 않습니다.")
    if set(thresholds) != set(views):
        raise RuntimeError("Artifact thresholds의 view key가 일치하지 않습니다.")
    if not isinstance(benchmark, dict):
        raise RuntimeError("Artifact parallel_benchmark가 없습니다.")
    try:
        selected = int(benchmark["selected_parallel_count"])
        reserve = int(benchmark["reserve_mib"])
        warmups = int(benchmark["warmup_runs"])
        runs = int(benchmark["benchmark_runs"])
        speedup = float(benchmark["min_speedup_percent"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("Artifact parallel_benchmark 값이 올바르지 않습니다.") from exc
    if not 1 <= selected <= min(3, len(views)):
        raise RuntimeError("Artifact selected_parallel_count 범위가 올바르지 않습니다.")
    if reserve < 0 or warmups < 0 or runs < 1 or not 0 <= speedup < 100:
        raise RuntimeError("Artifact parallel benchmark 안전 설정이 올바르지 않습니다.")

    # EfficientAD/student v1 사용자는 기존 공통 검증만 유지합니다.
    if expected_algorithm != "patchcore":
        parameters = manifest.get("parameters")
        if not isinstance(parameters, dict):
            raise RuntimeError("Artifact parameters 구조가 올바르지 않습니다.")
        try:
            epsilon = float(parameters["threshold_epsilon"])
            margin = float(parameters["customized_margin"])
            percentile = float(parameters["threshold_percentile"])
            validate_percentile_margin(percentile, margin, epsilon)
            for view in views:
                threshold = float(thresholds[view])
                scores = np.asarray(validation_scores[view], dtype=np.float64)
                if not math.isfinite(threshold) or threshold <= epsilon:
                    raise RuntimeError(
                        f"Artifact {view} threshold가 안전하지 않습니다: {threshold}"
                    )
                if scores.size == 0 or not np.all(np.isfinite(scores)):
                    raise RuntimeError(
                        f"Artifact {view} validation score가 비어 있거나 NaN/Inf입니다."
                    )
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "Artifact threshold/margin parameter가 올바르지 않습니다."
            ) from exc
        return

    if manifest.get("model_version") != artifact_name:
        raise RuntimeError("PatchCore model_version이 artifact 이름과 다릅니다.")
    if tuple(views) != PATCHCORE_EXPECTED_VIEWS:
        raise RuntimeError(
            f"PatchCore view 순서는 {list(PATCHCORE_EXPECTED_VIEWS)}여야 합니다."
        )
    if manifest.get("station_views") != PATCHCORE_STATION_VIEWS:
        raise RuntimeError("PatchCore station_views 매핑이 운영 계약과 다릅니다.")
    if manifest.get("camera_serial_by_view") != PATCHCORE_CAMERA_SERIAL_BY_VIEW:
        raise RuntimeError("PatchCore camera serial/view 매핑이 운영 계약과 다릅니다.")
    if "parameters" in manifest:
        raise RuntimeError("PatchCore v2는 전역 parameters를 허용하지 않습니다.")
    parameters_by_view = manifest.get("parameters_by_view")
    if not isinstance(parameters_by_view, dict) or set(parameters_by_view) != set(views):
        raise RuntimeError("PatchCore parameters_by_view의 view key가 다릅니다.")

    bank_shapes = manifest.get("memory_bank_shapes")
    preprocessing = manifest.get("preprocessing_by_view")
    if not isinstance(bank_shapes, dict) or set(bank_shapes) != set(views):
        raise RuntimeError("PatchCore memory_bank_shapes의 view key가 다릅니다.")
    if not isinstance(preprocessing, dict) or set(preprocessing) != set(views):
        raise RuntimeError("PatchCore preprocessing_by_view의 view key가 다릅니다.")
    for view in views:
        parameters = parameters_by_view[view]
        if not isinstance(parameters, dict) or set(parameters) != PATCHCORE_PARAMETER_KEYS:
            raise RuntimeError(f"PatchCore {view} parameter key가 v2 운영 계약과 다릅니다.")
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
            raise RuntimeError(f"PatchCore {view} parameter JSON type이 올바르지 않습니다.")
        try:
            backbone = str(parameters["backbone"])
            layers = parameters["feature_layers"]
            coreset_ratio = float(parameters["coreset_ratio"])
            k = int(parameters["k"])
            percentile = float(parameters["threshold_percentile"])
            margin = float(parameters["customized_margin"])
            epsilon = float(parameters["threshold_epsilon"])
            resolution = parameters["input_resolution"]
            batch_size = int(parameters["construction_batch_size"])
            chunk_size = int(parameters["distance_chunk_size"])
            seed = int(parameters["seed"])
            validate_percentile_margin(percentile, margin, epsilon)
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"PatchCore {view} parameter 값이 올바르지 않습니다.") from exc
        if backbone not in {"resnet34", "efficientnet_b0"}:
            raise RuntimeError(f"PatchCore {view} backbone이 올바르지 않습니다.")
        allowed_layers = {1, 2, 3, 4} if backbone == "resnet34" else set(range(1, 8))
        if (
            not isinstance(layers, list)
            or not layers
            or any(type(layer) is not int or layer not in allowed_layers for layer in layers)
            or len(set(layers)) != len(layers)
        ):
            raise RuntimeError(f"PatchCore {view} feature_layers가 올바르지 않습니다.")
        if not 0.0 < coreset_ratio <= 1.0 or k < 1:
            raise RuntimeError(f"PatchCore {view} coreset_ratio/k가 올바르지 않습니다.")
        if parameters.get("resize_mode") != "padding":
            raise RuntimeError(f"PatchCore {view} resize_mode는 padding이어야 합니다.")
        if (
            not isinstance(resolution, (list, tuple))
            or len(resolution) != 2
            or any(type(value) is not int or value < 1 for value in resolution)
        ):
            raise RuntimeError(f"PatchCore {view} input_resolution이 올바르지 않습니다.")
        if batch_size < 1 or chunk_size < 1 or seed < 0:
            raise RuntimeError(f"PatchCore {view} batch/chunk/seed가 올바르지 않습니다.")
        threshold = float(thresholds[view])
        scores = np.asarray(validation_scores[view], dtype=np.float64)
        if not math.isfinite(threshold) or threshold <= epsilon:
            raise RuntimeError(f"Artifact {view} threshold가 안전하지 않습니다: {threshold}")
        if scores.size == 0 or not np.all(np.isfinite(scores)):
            raise RuntimeError(
                f"Artifact {view} validation score가 비어 있거나 NaN/Inf입니다."
            )
        shape = bank_shapes[view]
        if (
            not isinstance(shape, list)
            or len(shape) != 2
            or min(int(value) for value in shape) < 1
        ):
            raise RuntimeError(f"PatchCore {view} memory bank shape가 올바르지 않습니다.")
        settings = preprocessing[view]
        if not isinstance(settings, dict) or set(settings) != PATCHCORE_PREPROCESSING_KEYS:
            raise RuntimeError(f"PatchCore {view} preprocessing key가 올바르지 않습니다.")
        if (
            type(settings["v_threshold"]) is not int
            or type(settings["connectivity"]) is not int
            or type(settings["remove_disconnected_noise"]) is not bool
            or type(settings["check_connection"]) is not bool
        ):
            raise RuntimeError(f"PatchCore {view} preprocessing type이 올바르지 않습니다.")
        if not 0 <= settings["v_threshold"] <= 255:
            raise RuntimeError(f"PatchCore {view} v_threshold 범위가 올바르지 않습니다.")
        if (
            settings["connectivity"] != 8
            or not settings["remove_disconnected_noise"]
            or settings["check_connection"]
        ):
            raise RuntimeError(f"PatchCore {view} 전처리 안전 정책이 올바르지 않습니다.")

    versions = manifest.get("library_versions")
    if not isinstance(versions, dict) or not {"torch", "torchvision"} <= set(versions):
        raise RuntimeError("PatchCore library_versions에 torch/torchvision이 필요합니다.")
    if artifact_dir is None:
        raise RuntimeError("PatchCore v2 검증에는 artifact_dir가 필요합니다.")
    root = artifact_dir.expanduser().resolve()
    if not root.is_dir() or artifact_dir.is_symlink():
        raise RuntimeError("PatchCore artifact_dir가 실제 artifact 폴더가 아닙니다.")

    allowed_files = {Path("manifest.json")}
    for view in views:
        allowed_files.update(
            {Path(view) / "model.pt", Path(view) / "calibration.json"}
        )
        calibration = read_json(root / view / "calibration.json")
        if (
            not isinstance(calibration, dict)
            or calibration.get("view") != view
            or float(calibration.get("threshold", math.nan)) != float(thresholds[view])
            or calibration.get("memory_bank_shape") != bank_shapes[view]
            or calibration.get("validation_raw_scores") != validation_scores[view]
        ):
            raise RuntimeError(f"PatchCore {view} calibration이 manifest와 다릅니다.")
    paths = tuple(root.rglob("*"))
    symlinks = [path for path in paths if path.is_symlink()]
    if symlinks:
        raise RuntimeError(f"PatchCore artifact symlink는 허용되지 않습니다: {symlinks[0]}")
    actual_files = {path.relative_to(root) for path in paths if path.is_file()}
    if actual_files != allowed_files:
        raise RuntimeError(
            "PatchCore artifact file set이 다릅니다: "
            f"missing={sorted(map(str, allowed_files - actual_files))}, "
            f"extra={sorted(map(str, actual_files - allowed_files))}"
        )


def run_product_inference(
    models: Sequence[nn.Module],
    inputs: Sequence[torch.Tensor],
    thresholds: Sequence[float],
    epsilons: Sequence[float],
    margins: Sequence[float],
    parallel_count: int,
    streams: Sequence[torch.cuda.Stream],
    event_timeout_seconds: float,
) -> tuple[list[float], list[float], list[bool], float, list[float]]:
    """모든 view를 계산하되 첫 anomaly가 완료된 시점을 제품 판정 시간으로 기록한다.

    반환값은 raw scores, normalized scores, view predictions, product decision ms,
    view별 model-forward CUDA ms 순서다. Tensor는 호출 전에 GPU로 옮겨져 있어야 한다.

    ``streams``는 제품마다 새로 만들지 않고 전체 평가에서 재사용한다. PatchCore의
    큰 거리 행렬용 CUDA allocator cache가 제품별 stream에 누적되는 것을 막는다.
    CUDA event가 timeout 안에 끝나지 않으면 polling을 무한 반복하지 않고 예외로
    종료해 CUDA 오류/OOM을 사용자가 즉시 확인할 수 있게 한다.
    """
    if not (
        len(models)
        == len(inputs)
        == len(thresholds)
        == len(epsilons)
        == len(margins)
    ):
        raise ValueError("models/inputs/thresholds/epsilons/margins 길이가 다릅니다.")
    if parallel_count < 1 or len(streams) < parallel_count:
        raise ValueError("parallel_count 또는 재사용 CUDA stream 수가 올바르지 않습니다.")
    if not math.isfinite(float(event_timeout_seconds)) or float(event_timeout_seconds) <= 0:
        raise ValueError("event_timeout_seconds는 유한한 양수여야 합니다.")
    active_streams = streams[:parallel_count]
    raw_scores: list[float] = []
    normalized_scores: list[float] = []
    predictions: list[bool] = []
    per_view_ms: list[float] = []
    elapsed_before_wave = 0.0
    decision_ms: float | None = None

    with torch.inference_mode():
        for start in range(0, len(models), parallel_count):
            wave_models = models[start : start + parallel_count]
            wave_inputs = inputs[start : start + parallel_count]
            wave_thresholds = thresholds[start : start + parallel_count]
            wave_epsilons = epsilons[start : start + parallel_count]
            wave_margins = margins[start : start + parallel_count]
            outputs: list[torch.Tensor] = []
            events: list[tuple[torch.cuda.Event, torch.cuda.Event]] = []
            wall_started = time.perf_counter()
            for slot, (model, image) in enumerate(zip(wave_models, wave_inputs, strict=True)):
                begin = torch.cuda.Event(enable_timing=True)
                end = torch.cuda.Event(enable_timing=True)
                with torch.cuda.stream(active_streams[slot]):
                    begin.record()
                    outputs.append(model_score_tensor(model, image))
                    end.record()
                events.append((begin, end))
            # 종료 event를 polling해 stream 순서와 무관하게 실제 wall-clock 완료 시점을
            # 기록한다. 한 stream을 먼저 synchronize해서 다른 stream의 빠른 완료 시점을
            # 놓치는 측정 편향을 피한다.
            pending = set(range(len(wave_models)))
            completion_ms = [0.0] * len(wave_models)
            while pending:
                completed_now = [slot for slot in pending if events[slot][1].query()]
                if not completed_now:
                    waited_seconds = time.perf_counter() - wall_started
                    if waited_seconds >= float(event_timeout_seconds):
                        raise TimeoutError(
                            "CUDA event 완료 대기 시간이 제한을 넘었습니다. "
                            f"timeout={event_timeout_seconds}s, pending_slots={sorted(pending)}. "
                            "CUDA OOM/비동기 오류 또는 driver stall 가능성이 있습니다. "
                            "실행을 종료한 뒤 nvidia-smi와 journalctl -k를 확인하세요.",
                        )
                    # sleep(0)은 사실상 busy polling이 될 수 있다. 1 ms 양보로 CPU
                    # 사용률 폭증을 막으면서 event 완료 시점을 충분히 정밀하게 기록한다.
                    time.sleep(0.001)
                    continue
                observed_ms = (time.perf_counter() - wall_started) * 1000.0
                for slot in completed_now:
                    completion_ms[slot] = observed_ms
                    pending.remove(slot)
            wave_wall_ms = max(completion_ms)

            wave_raw: list[float] = []
            wave_normalized: list[float] = []
            wave_predictions: list[bool] = []
            wave_view_ms: list[float] = []
            wave_anomaly_completion: list[float] = []
            for slot, (output, threshold, epsilon, margin, event_pair) in enumerate(
                zip(
                    outputs,
                    wave_thresholds,
                    wave_epsilons,
                    wave_margins,
                    events,
                    strict=True,
                ),
            ):
                raw = float(output[0].detach().cpu())
                normalized = normalized_score(raw, threshold, epsilon)
                predicted = anomaly_from_normalized(normalized, margin)
                view_ms = float(event_pair[0].elapsed_time(event_pair[1]))
                wave_raw.append(raw)
                wave_normalized.append(normalized)
                wave_predictions.append(predicted)
                wave_view_ms.append(view_ms)
                if predicted:
                    wave_anomaly_completion.append(completion_ms[slot])

            raw_scores.extend(wave_raw)
            normalized_scores.extend(wave_normalized)
            predictions.extend(wave_predictions)
            per_view_ms.extend(wave_view_ms)
            if decision_ms is None and wave_anomaly_completion:
                decision_ms = elapsed_before_wave + min(wave_anomaly_completion)
            elapsed_before_wave += wave_wall_ms

    if decision_ms is None:
        decision_ms = elapsed_before_wave
    return raw_scores, normalized_scores, predictions, decision_ms, per_view_ms


def create_result_directory(result_parent: Path, artifact_name: str, overwrite: bool) -> contextlib.AbstractContextManager[Path]:
    return managed_output_directory(result_parent, f"result_{artifact_name}", overwrite)


def save_score_distribution(
    output_path: Path,
    validation_scores: Sequence[float],
    normal_scores: Sequence[float],
    anomaly_scores: Sequence[float],
    threshold_line: float,
    title: str,
    x_label: str,
) -> None:
    """Validation normal/test normal/test anomaly의 image score 분포를 한 plot에 저장한다."""
    fig, axis = plt.subplots(figsize=(10, 6))
    groups = [
        np.asarray(validation_scores, dtype=np.float64),
        np.asarray(normal_scores, dtype=np.float64),
        np.asarray(anomaly_scores, dtype=np.float64),
    ]
    combined = np.concatenate(groups)
    if combined.size == 0 or not np.all(np.isfinite(combined)):
        raise RuntimeError(f"Score plot에 유효하지 않은 값이 있습니다: {output_path}")
    bin_count = max(10, min(80, int(math.sqrt(combined.size)) * 2))
    low, high = float(combined.min()), float(combined.max())
    if low == high:
        padding = max(abs(low) * 0.01, 1e-9)
        low, high = low - padding, high + padding
    shared_bins = np.linspace(low, high, bin_count + 1)
    axis.hist(groups, bins=shared_bins, alpha=0.45, label=["validation normal", "test normal", "test anomaly"])
    axis.axvline(threshold_line, color="red", linestyle="--", linewidth=2, label=f"decision={threshold_line:.6g}")
    axis.set_title(title)
    axis.set_xlabel(x_label)
    axis.set_ylabel("Image count")
    axis.grid(alpha=0.2)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def save_confusion_matrix(output_path: Path, labels: Sequence[int], predictions: Sequence[int]) -> tuple[int, int, int, int]:
    """제품 단위 confusion matrix를 고정 label 순서 [normal, anomaly]로 저장한다."""
    matrix = confusion_matrix(labels, predictions, labels=[0, 1])
    tn, fp, fn, tp = (int(value) for value in matrix.ravel())
    fig, axis = plt.subplots(figsize=(6, 5))
    image = axis.imshow(matrix, cmap="Blues")
    axis.set_xticks([0, 1], labels=["Pred Normal", "Pred Anomaly"])
    axis.set_yticks([0, 1], labels=["True Normal", "True Anomaly"])
    axis.set_xlabel("Prediction")
    axis.set_ylabel("Ground truth")
    axis.set_title("Product-level Confusion Matrix")
    for row in range(2):
        for column in range(2):
            axis.text(column, row, str(matrix[row, column]), ha="center", va="center")
    fig.colorbar(image, ax=axis)
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return tn, fp, fn, tp


def save_accuracy_report(
    output_path: Path,
    labels: Sequence[int],
    predictions: Sequence[int],
    parameters_by_view: dict[str, dict[str, Any]],
    thresholds: dict[str, float],
    confusion_values: tuple[int, int, int, int],
) -> None:
    """요청된 Accuracy/Recall/Precision/FPR과 판정 설정을 TXT로 저장한다."""
    tn, fp, fn, tp = confusion_values
    fpr = fp / (fp + tn) if fp + tn else 0.0
    lines = [
        f"Accuracy: {accuracy_score(labels, predictions):.10f}",
        f"Recall: {recall_score(labels, predictions, zero_division=0):.10f}",
        f"Precision: {precision_score(labels, predictions, zero_division=0):.10f}",
        f"FPR: {fpr:.10f}",
        f"TN: {tn}",
        f"FP: {fp}",
        f"FN: {fn}",
        f"TP: {tp}",
        "View decision settings:",
    ]
    for view, threshold in thresholds.items():
        parameters = parameters_by_view[view]
        margin = float(parameters["customized_margin"])
        epsilon = float(parameters["threshold_epsilon"])
        lines.append(
            f"  {view}: threshold={threshold:.12g}, margin={margin}, "
            f"normalized_boundary={1.0 - margin}, epsilon={epsilon}"
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_inference_time_report(
    output_path: Path,
    decision_times_ms: Sequence[float],
    per_view_times_ms: dict[str, Sequence[float]],
    parallel_count: int,
) -> None:
    """모델 forward만 포함한 제품 판정 wall-clock과 view별 CUDA 시간을 저장한다."""
    average_product_ms = float(np.mean(decision_times_ms))
    product_fps = 1000.0 / average_product_ms if average_product_ms > 0 else math.inf
    lines = [
        f"Selected concurrent view forwards: {parallel_count}",
        f"Average product decision model-forward wall-clock: {average_product_ms:.6f} ms/product",
        f"Equivalent product throughput: {product_fps:.6f} products/second",
        f"Measured products: {len(decision_times_ms)}",
        "Per-view model-forward CUDA event time:",
    ]
    for view, values in per_view_times_ms.items():
        mean_ms = float(np.mean(values))
        fps = 1000.0 / mean_ms if mean_ms > 0 else math.inf
        lines.append(f"  {view}: {mean_ms:.6f} ms/image, {fps:.6f} images/second, n={len(values)}")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_misclassified_product(
    result_root: Path,
    category: str,
    product_filename: str,
    view_names: Sequence[str],
    models: Sequence[nn.Module],
    display_inputs: Sequence[torch.Tensor],
    inference_inputs: Sequence[torch.Tensor],
    raw_scores: Sequence[float],
    normalized_scores: Sequence[float],
    thresholds: Sequence[float],
    margins: Sequence[float],
    view_predictions: Sequence[bool],
    true_product_label: bool,
) -> None:
    """FP/FN view별 입력, score metadata, 실제 PatchCore heat map을 저장한다."""

    lengths = {
        len(view_names),
        len(models),
        len(display_inputs),
        len(inference_inputs),
        len(raw_scores),
        len(normalized_scores),
        len(thresholds),
        len(margins),
        len(view_predictions),
    }
    if len(lengths) != 1:
        raise ValueError("오분류 진단 입력의 view별 길이가 다릅니다.")
    product_stem = Path(product_filename).stem
    product_dir = result_root / category / product_stem
    metadata: dict[str, Any] = {
        "product_filename": product_filename,
        "category": category,
        "ground_truth": "anomaly" if true_product_label else "normal",
        "views": {},
    }
    for (
        view,
        model,
        display_input,
        inference_input,
        raw_score,
        normalized,
        threshold,
        margin,
        prediction,
    ) in zip(
        view_names,
        models,
        display_inputs,
        inference_inputs,
        raw_scores,
        normalized_scores,
        thresholds,
        margins,
        view_predictions,
        strict=True,
    ):
        if not isinstance(model, PatchCoreViewModel):
            raise TypeError("PatchCore heat map은 PatchCoreViewModel에서만 생성할 수 있습니다.")
        is_correct = bool(prediction) == bool(true_product_label)
        verdict_name = "anomaly" if prediction else "normal"
        score_tag = f"{float(normalized):.6f}"
        prefix = f"{product_stem}_{view}_norm-{score_tag}_{verdict_name}"
        input_name = f"{prefix}_input.png"
        heatmap_name = f"{prefix}_heatmap.png"
        write_model_input_png(product_dir / input_name, display_input)
        repeated_scores, anomaly_maps = model.score_and_anomaly_map(inference_input)
        repeated_raw = float(repeated_scores[0].detach().cpu())
        if not math.isclose(repeated_raw, float(raw_score), rel_tol=1e-5, abs_tol=1e-6):
            raise RuntimeError(
                f"{view}: heat map 재추론 score가 평가 score와 다릅니다: "
                f"evaluation={raw_score}, heatmap={repeated_raw}"
            )
        _save_anomaly_heatmap(
            product_dir / heatmap_name,
            display_input,
            anomaly_maps[0],
            normalized_score=float(normalized),
            raw_score=float(raw_score),
            threshold=float(threshold),
            decision_boundary=1.0 - float(margin),
            predicted_anomaly=bool(prediction),
        )
        metadata["views"][view] = {
            "raw_score": float(raw_score),
            "threshold": float(threshold),
            "normalized_score": float(normalized),
            "normalized_decision_boundary": 1.0 - float(margin),
            "predicted": verdict_name,
            "view_prediction_correct": is_correct,
            "input_image": input_name,
            "anomaly_heatmap": heatmap_name,
        }
    write_json(product_dir / "scores.json", metadata)


def _save_anomaly_heatmap(
    path: Path,
    model_input: torch.Tensor,
    anomaly_map: torch.Tensor,
    *,
    normalized_score: float,
    raw_score: float,
    threshold: float,
    decision_boundary: float,
    predicted_anomaly: bool,
) -> None:
    """Patch distance map을 threshold 기준으로 색상화해 model input 위에 겹친다."""

    image_rgb = _model_input_rgb(model_input)
    values = anomaly_map.detach().float().cpu().numpy()
    if values.ndim != 2 or not np.all(np.isfinite(values)):
        raise RuntimeError("PatchCore anomaly map이 유효한 2차원 finite array가 아닙니다.")
    if values.shape != image_rgb.shape[:2]:
        values = cv2.resize(
            values,
            (image_rgb.shape[1], image_rgb.shape[0]),
            interpolation=cv2.INTER_LINEAR,
        )
    values = cv2.GaussianBlur(values, (0, 0), sigmaX=2.0, sigmaY=2.0)
    normalized_map = values / float(threshold)
    scale = max(float(decision_boundary), 1e-12)
    heat_u8 = np.clip(normalized_map / scale, 0.0, 1.0)
    heat_u8 = np.rint(heat_u8 * 255.0).astype(np.uint8)
    heat_rgb = cv2.cvtColor(cv2.applyColorMap(heat_u8, cv2.COLORMAP_JET), cv2.COLOR_BGR2RGB)
    overlay = cv2.addWeighted(image_rgb, 0.55, heat_rgb, 0.45, 0.0)

    banner_height = 42
    banner = np.zeros((banner_height, overlay.shape[1], 3), dtype=np.uint8)
    verdict = "ANOMALY" if predicted_anomaly else "NORMAL"
    labels = (
        f"norm={normalized_score:.6f}  boundary={decision_boundary:.6f}",
        f"raw={raw_score:.6f}  {verdict}",
    )
    font_scale = max(0.30, min(0.50, overlay.shape[1] / 520.0))
    line_positions = (17, 36)
    for label, y_position in zip(labels, line_positions, strict=True):
        cv2.putText(
            banner,
            label,
            (4, y_position),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    _write_rgb_png(path, np.concatenate((banner, overlay), axis=0))


def evaluate_multiview_models(
    models: Sequence[nn.Module],
    index: DatasetIndex,
    manifest: dict[str, Any],
    result_root: Path,
    device: torch.device,
    parallel_count: int,
    cuda_event_timeout_seconds: float = 60.0,
) -> dict[str, Any]:
    """Artifact 모델 전체를 제품 단위로 평가하고 요청 산출물을 저장한다.

    PNG decode/resize 및 CPU->GPU 복사는 시간 측정 전에 끝낸다. 따라서
    inference_time.txt에는 요청대로 model-forward 구간만 포함된다. 모든 view를
    끝까지 계산하되 제품 판정 시간은 첫 anomaly view가 완료된 시점이다.
    """
    view_names = tuple(str(value) for value in manifest["view_names"])
    if tuple(index.view_names) != view_names or len(models) != len(view_names):
        raise RuntimeError("Dataset view 순서와 artifact model 순서가 다릅니다.")

    parameters_by_view = manifest["parameters_by_view"]
    thresholds_by_view = {
        view: float(manifest["thresholds"][view]) for view in view_names
    }
    thresholds = [thresholds_by_view[view] for view in view_names]
    epsilons = [
        float(parameters_by_view[view]["threshold_epsilon"]) for view in view_names
    ]
    margins = [
        float(parameters_by_view[view]["customized_margin"]) for view in view_names
    ]
    if parallel_count < 1 or parallel_count > min(3, len(view_names)):
        raise ValueError("parallel_count가 artifact view 수 범위를 벗어났습니다.")
    if not math.isfinite(float(cuda_event_timeout_seconds)) or float(cuda_event_timeout_seconds) <= 0:
        raise ValueError("cuda_event_timeout_seconds는 유한한 양수여야 합니다.")

    # 전체 evaluation 동안 같은 stream을 재사용한다. 제품마다 stream을 만들면
    # stream별 CUDA cache가 누적돼 긴 PatchCore 평가에서 VRAM이 고갈될 수 있다.
    persistent_streams = [torch.cuda.Stream() for _ in range(parallel_count)]

    labels: list[int] = []
    product_predictions: list[int] = []
    decision_times_ms: list[float] = []
    per_view_times_ms: dict[str, list[float]] = {view: [] for view in view_names}
    raw_scores: dict[str, dict[str, list[float]]] = {
        view: {"normal": [], "anomaly": []} for view in view_names
    }
    normalized_scores: dict[str, dict[str, list[float]]] = {
        view: {"normal": [], "anomaly": []} for view in view_names
    }
    product_records: list[dict[str, Any]] = []

    split_specs = (("test_set_normal", 0, "normal"), ("test_set_anomaly", 1, "anomaly"))
    for split, ground_truth, distribution_key in split_specs:
        names = sorted_product_names(index, split)
        for product_number, product_filename in enumerate(names, start=1):
            source_paths = [index.files[split][view][product_filename] for view in view_names]
            # 손상 PNG도 이 단계에서 즉시 예외가 나 전체 실행이 중단된다.
            cpu_inputs = [
                load_model_input(
                    path,
                    parameters_by_view[view]["input_resolution"],
                    parameters_by_view[view]["resize_mode"],
                )[None]
                for view, path in zip(view_names, source_paths, strict=True)
            ]
            gpu_inputs = [value.to(device, non_blocking=True) for value in cpu_inputs]
            torch.cuda.synchronize(device)
            product_raw, product_normalized, view_predictions, decision_ms, view_times = (
                run_product_inference(
                    models=models,
                    inputs=gpu_inputs,
                    thresholds=thresholds,
                    epsilons=epsilons,
                    margins=margins,
                    parallel_count=parallel_count,
                    streams=persistent_streams,
                    event_timeout_seconds=cuda_event_timeout_seconds,
                )
            )
            product_prediction = int(any(view_predictions))
            labels.append(ground_truth)
            product_predictions.append(product_prediction)
            decision_times_ms.append(decision_ms)

            view_details: dict[str, Any] = {}
            for view, raw, normalized, prediction, elapsed in zip(
                view_names,
                product_raw,
                product_normalized,
                view_predictions,
                view_times,
                strict=True,
            ):
                raw_scores[view][distribution_key].append(raw)
                normalized_scores[view][distribution_key].append(normalized)
                per_view_times_ms[view].append(elapsed)
                view_details[view] = {
                    "raw_score": raw,
                    "normalized_score": normalized,
                    "is_anomaly": bool(prediction),
                    "forward_cuda_ms": elapsed,
                }

            product_records.append(
                {
                    "filename": product_filename,
                    "ground_truth": "anomaly" if ground_truth else "normal",
                    "product_is_anomaly": bool(product_prediction),
                    "decision_forward_wall_clock_ms": decision_ms,
                    "views": view_details,
                },
            )

            if product_prediction != ground_truth:
                category = "false-positive" if product_prediction else "false-negative"
                save_misclassified_product(
                    result_root=result_root,
                    category=category,
                    product_filename=product_filename,
                    view_names=view_names,
                    models=models,
                    display_inputs=cpu_inputs,
                    inference_inputs=gpu_inputs,
                    raw_scores=product_raw,
                    normalized_scores=product_normalized,
                    thresholds=thresholds,
                    margins=margins,
                    view_predictions=view_predictions,
                    true_product_label=bool(ground_truth),
                )

            del gpu_inputs, cpu_inputs
            print(
                f"  evaluation {split}: {product_number}/{len(names)} | "
                f"{product_filename} -> {'anomaly' if product_prediction else 'normal'}",
            )

    # View별 score plot은 서로 다른 threshold를 섞지 않고 독립적으로 저장한다.
    for view in view_names:
        parameters = parameters_by_view[view]
        epsilon = float(parameters["threshold_epsilon"])
        effective_normalized_boundary = 1.0 - float(parameters["customized_margin"])
        view_dir = result_root / view
        view_dir.mkdir(parents=True, exist_ok=True)
        validation_raw = [float(value) for value in manifest["validation_raw_scores"][view]]
        validation_normalized = [
            normalized_score(value, thresholds_by_view[view], epsilon) for value in validation_raw
        ]
        save_score_distribution(
            view_dir / "raw_score_distribution.png",
            validation_raw,
            raw_scores[view]["normal"],
            raw_scores[view]["anomaly"],
            thresholds_by_view[view] * effective_normalized_boundary,
            f"Raw score distribution - {view}",
            "Raw image-level anomaly score",
        )
        save_score_distribution(
            view_dir / "normalized_score_distribution.png",
            validation_normalized,
            normalized_scores[view]["normal"],
            normalized_scores[view]["anomaly"],
            effective_normalized_boundary,
            f"Normalized score distribution - {view}",
            "Raw score / view threshold",
        )

    confusion_values = save_confusion_matrix(
        result_root / "confusion_matrix.png",
        labels,
        product_predictions,
    )
    save_accuracy_report(
        result_root / "accuracy.txt",
        labels,
        product_predictions,
        parameters_by_view,
        thresholds_by_view,
        confusion_values,
    )
    save_inference_time_report(
        result_root / "inference_time.txt",
        decision_times_ms,
        per_view_times_ms,
        parallel_count,
    )
    write_json(
        result_root / "product_predictions.json",
        {
            "artifact_name": manifest["artifact_name"],
            "algorithm": manifest["algorithm"],
            "parallel_count": parallel_count,
            "records": product_records,
        },
    )
    tn, fp, fn, tp = confusion_values
    return {
        "product_count": len(labels),
        "tn": tn,
        "fp": fp,
        "fn": fn,
        "tp": tp,
        "mean_decision_ms": float(np.mean(decision_times_ms)),
    }
