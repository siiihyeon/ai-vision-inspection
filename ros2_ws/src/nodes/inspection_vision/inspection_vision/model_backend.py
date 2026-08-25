"""Mono8 전처리와 versioned multi-station PatchCore artifact runtime.

Memory-bank construction은 운영 밖에서 수행합니다. Vision Node는 완성된
artifact 디렉터리의 통합 SHA-256, manifest, calibration, tensor state를 모두
검증한 뒤 CUDA 추론만 수행합니다. Threshold와 margin은 오직 manifest에서
읽으며 ROS parameter나 코드 기본값으로 대체하지 않습니다.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import torch
from inspection_common import Verdict
from torch import nn
from torch.nn import functional as F
from torchvision import __version__ as torchvision_version
from torchvision.models import (
    EfficientNet_B0_Weights,
    ResNet34_Weights,
    efficientnet_b0,
    resnet34,
)
from torchvision.models.feature_extraction import create_feature_extractor


ARTIFACT_FORMAT_VERSION = 2
ARTIFACT_RUNTIME = "PYTORCH_PATCHCORE_ARTIFACT"
EXPECTED_STATION_VIEWS = {
    1: ("CAM_A_1", "CAM_A_2", "CAM_A_3"),
    2: ("CAM_B_1",),
}
SAFE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


class ArtifactContractError(RuntimeError):
    """Artifact가 배포·알고리즘 계약을 만족하지 않습니다."""


class PreprocessingFailure(RuntimeError):
    """정상적인 model input을 만들 수 없는 전처리 실패입니다."""

    is_preprocessing_failure = True


@dataclass(frozen=True, slots=True)
class ModelIdentity:
    path: str
    version: str
    sha256: str
    runtime: str = ARTIFACT_RUNTIME
    library_compatibility_warnings: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class StationInferenceResult:
    verdict: int
    score: float
    model_version: str
    model_sha256: str
    view_verdicts: tuple[int, ...]
    view_scores: tuple[float, ...]
    view_raw_scores: tuple[float, ...] = ()
    view_names: tuple[str, ...] = ()
    diagnostic_paths: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class PreprocessingSettings:
    v_threshold: int
    connectivity: int
    remove_disconnected_noise: bool
    check_connection: bool

    @classmethod
    def from_payload(cls, view: str, payload: Any) -> "PreprocessingSettings":
        if not isinstance(payload, dict):
            raise ArtifactContractError(f"{view}: preprocessing config must be an object")
        required = {
            "v_threshold",
            "connectivity",
            "remove_disconnected_noise",
            "check_connection",
        }
        if set(payload) != required:
            raise ArtifactContractError(
                f"{view}: preprocessing keys differ: expected={sorted(required)}"
            )
        if (
            type(payload["v_threshold"]) is not int
            or type(payload["connectivity"]) is not int
            or type(payload["remove_disconnected_noise"]) is not bool
            or type(payload["check_connection"]) is not bool
        ):
            raise ArtifactContractError(
                f"{view}: preprocessing values have invalid JSON types"
            )
        settings = cls(
            v_threshold=int(payload["v_threshold"]),
            connectivity=int(payload["connectivity"]),
            remove_disconnected_noise=bool(payload["remove_disconnected_noise"]),
            check_connection=bool(payload["check_connection"]),
        )
        if not 0 <= settings.v_threshold <= 255:
            raise ArtifactContractError(f"{view}: v_threshold must be between 0 and 255")
        if settings.connectivity != 8:
            raise ArtifactContractError(f"{view}: connectivity must be 8")
        if not settings.remove_disconnected_noise:
            raise ArtifactContractError(
                f"{view}: remove_disconnected_noise must remain enabled"
            )
        if settings.check_connection:
            raise ArtifactContractError(f"{view}: check_connection must remain disabled")
        return settings


@dataclass(frozen=True, slots=True)
class PreparedView:
    source_path: Path
    view_name: str
    tensor: torch.Tensor
    crop_1: np.ndarray
    crop_2: np.ndarray
    original_component_count: int


def _read_json_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ArtifactContractError(f"cannot read JSON: {path}") from exc
    if not isinstance(payload, dict):
        raise ArtifactContractError(f"JSON root must be an object: {path}")
    return payload


def artifact_directory_sha256(root: Path) -> str:
    """정렬된 상대경로와 파일 내용을 함께 hash하는 artifact 지문입니다."""

    resolved = root.expanduser().resolve()
    if not resolved.is_dir() or root.is_symlink():
        raise ArtifactContractError("artifact path must be a real directory")
    files = sorted(path for path in resolved.rglob("*") if path.is_file())
    if not files:
        raise ArtifactContractError("artifact directory is empty")
    digest = hashlib.sha256()
    for path in files:
        if path.is_symlink():
            raise ArtifactContractError(f"artifact symlink is not allowed: {path}")
        digest.update(path.relative_to(resolved).as_posix().encode("utf-8"))
        digest.update(b"\0")
        try:
            with path.open("rb") as stream:
                while chunk := stream.read(1024 * 1024):
                    digest.update(chunk)
        except OSError as exc:
            raise ArtifactContractError(f"cannot hash artifact file: {path}") from exc
        digest.update(b"\0")
    return digest.hexdigest()


def inspect_model_artifact(
    path: Path,
    version: str,
    expected_sha256: str,
) -> ModelIdentity:
    if not path.is_absolute() or not path.is_dir():
        raise ValueError("model artifact path must be an existing absolute directory")
    if not version:
        raise ValueError("model version is required")
    expected = expected_sha256.lower()
    if len(expected) != 64 or any(character not in "0123456789abcdef" for character in expected):
        raise ValueError("model artifact SHA-256 must be 64 lowercase hexadecimal characters")
    actual = artifact_directory_sha256(path)
    if actual != expected:
        raise ValueError("model artifact SHA-256 differs from configured identity")
    return ModelIdentity(str(path.resolve()), version, actual)


def _write_png_atomic(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(".png", image, [cv2.IMWRITE_PNG_COMPRESSION, 3])
    if not ok:
        raise RuntimeError(f"diagnostic PNG encoding failed: {path}")
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.part")
    try:
        with temporary.open("xb") as stream:
            stream.write(encoded.tobytes())
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


class Mono8PatchCorePreprocessor:
    """HSV-V와 동치인 Mono8 threshold 후 가장 큰 component를 crop합니다."""

    def __init__(
        self,
        *,
        serial_to_view: Mapping[str, str],
        settings_by_view: Mapping[str, PreprocessingSettings],
        input_resolution: Sequence[int],
        resize_mode: str,
        diagnostic_root: Path,
    ) -> None:
        self.serial_to_view = dict(serial_to_view)
        self.settings_by_view = dict(settings_by_view)
        if set(self.serial_to_view.values()) != set(self.settings_by_view):
            raise ArtifactContractError("camera serial/view preprocessing mapping differs")
        if len(input_resolution) != 2:
            raise ArtifactContractError("input_resolution must contain height and width")
        self.input_resolution = (int(input_resolution[0]), int(input_resolution[1]))
        if min(self.input_resolution) < 1:
            raise ArtifactContractError("input_resolution must be positive")
        if resize_mode != "padding":
            raise ArtifactContractError("production preprocessing requires padding resize mode")
        self.resize_mode = resize_mode
        self.diagnostic_root = diagnostic_root.expanduser().resolve()

    def load(self, path: Path) -> PreparedView:
        resolved = path.expanduser().resolve()
        serial = resolved.stem
        view = self.serial_to_view.get(serial)
        if view is None:
            raise PreprocessingFailure(f"camera serial is not mapped to an artifact view: {serial}")
        try:
            encoded = np.fromfile(str(resolved), dtype=np.uint8)
        except OSError as exc:
            raise RuntimeError(f"canonical PNG read failed: {resolved}") from exc
        image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f"canonical PNG decode failed: {resolved}")
        if image.dtype != np.uint8 or image.ndim != 2:
            raise PreprocessingFailure("PatchCore input must be an 8-bit Mono8 PNG")

        settings = self.settings_by_view[view]
        initial_mask = np.where(image >= settings.v_threshold, 255, 0).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            initial_mask,
            connectivity=settings.connectivity,
        )
        component_count = int(count) - 1
        if component_count == 0:
            black = np.zeros_like(image)
            self._save_arrays(resolved, view, "preprocessing_failure", black, black)
            raise PreprocessingFailure(f"{view}: no foreground was detected")

        final_mask = initial_mask
        if settings.remove_disconnected_noise and component_count > 1:
            largest_label = int(np.argmax(stats[1:, cv2.CC_STAT_AREA])) + 1
            final_mask = np.where(labels == largest_label, 255, 0).astype(np.uint8)
        if settings.check_connection and component_count != 1:
            crop_1 = cv2.bitwise_and(image, image, mask=final_mask)
            self._save_arrays(resolved, view, "preprocessing_failure", crop_1, crop_1)
            raise PreprocessingFailure(f"{view}: foreground is disconnected")

        points = cv2.findNonZero(final_mask)
        if points is None:
            black = np.zeros_like(image)
            self._save_arrays(resolved, view, "preprocessing_failure", black, black)
            raise PreprocessingFailure(f"{view}: cleaned foreground is empty")
        x, y, width, height = (int(value) for value in cv2.boundingRect(points))
        crop_1 = cv2.bitwise_and(image, image, mask=final_mask)
        crop_2 = crop_1[y : y + height, x : x + width]
        resized = self._resize_with_black_padding(crop_2)
        rgb = np.repeat(resized[:, :, None], 3, axis=2)
        tensor = (
            torch.from_numpy(np.ascontiguousarray(rgb))
            .permute(2, 0, 1)
            .float()
            .div_(255.0)
        )
        return PreparedView(
            source_path=resolved,
            view_name=view,
            tensor=tensor,
            crop_1=crop_1,
            crop_2=crop_2,
            original_component_count=component_count,
        )

    def _resize_with_black_padding(self, image: np.ndarray) -> np.ndarray:
        target_h, target_w = self.input_resolution
        source_h, source_w = image.shape
        scale = min(target_w / source_w, target_h / source_h)
        resized_w = max(1, min(target_w, int(round(source_w * scale))))
        resized_h = max(1, min(target_h, int(round(source_h * scale))))
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(image, (resized_w, resized_h), interpolation=interpolation)
        canvas = np.zeros((target_h, target_w), dtype=np.uint8)
        x0 = (target_w - resized_w) // 2
        y0 = (target_h - resized_h) // 2
        canvas[y0 : y0 + resized_h, x0 : x0 + resized_w] = resized
        return canvas

    def save_diagnostic(self, prepared: PreparedView, reason: str) -> tuple[str, str]:
        return self._save_arrays(
            prepared.source_path,
            prepared.view_name,
            reason,
            prepared.crop_1,
            prepared.crop_2,
        )

    def _save_arrays(
        self,
        source: Path,
        view: str,
        reason: str,
        crop_1: np.ndarray,
        crop_2: np.ndarray,
    ) -> tuple[str, str]:
        safe_reason = re.sub(r"[^A-Za-z0-9_.-]+", "_", reason)
        frame_batch = source.parent.name if SAFE_NAME_PATTERN.fullmatch(source.parent.name) else "unknown_batch"
        output = self.diagnostic_root / safe_reason / frame_batch / view
        first = output / "crop_1.png"
        second = output / "crop_2.png"
        _write_png_atomic(first, crop_1)
        _write_png_atomic(second, crop_2)
        return str(first), str(second)


class PatchCoreViewModel(nn.Module):
    """참고 알고리즘과 동일한 torchvision 기반 PatchCore image score 모델."""

    def __init__(
        self,
        backbone: str,
        layer_numbers: Sequence[int],
        num_neighbors: int,
        pretrained: bool,
        distance_chunk_size: int,
    ) -> None:
        super().__init__()
        self.layer_numbers = tuple(int(value) for value in layer_numbers)
        self.num_neighbors = int(num_neighbors)
        self.distance_chunk_size = int(distance_chunk_size)
        if not self.layer_numbers or len(set(self.layer_numbers)) != len(self.layer_numbers):
            raise ArtifactContractError("feature_layers must be non-empty and unique")
        if self.num_neighbors < 1 or self.distance_chunk_size < 1:
            raise ArtifactContractError("PatchCore neighbor/chunk settings must be positive")
        if backbone == "resnet34":
            if any(number not in {1, 2, 3, 4} for number in self.layer_numbers):
                raise ArtifactContractError("ResNet34 feature layer must be between 1 and 4")
            base = resnet34(weights=ResNet34_Weights.DEFAULT if pretrained else None)
            return_nodes = {f"layer{number}": f"layer{number}" for number in self.layer_numbers}
        elif backbone == "efficientnet_b0":
            if any(number not in set(range(1, 8)) for number in self.layer_numbers):
                raise ArtifactContractError("EfficientNet-B0 feature layer must be between 1 and 7")
            base = efficientnet_b0(
                weights=EfficientNet_B0_Weights.DEFAULT if pretrained else None
            )
            return_nodes = {
                f"features.{number}": f"layer{number}" for number in self.layer_numbers
            }
        else:
            raise ArtifactContractError("unsupported PatchCore backbone")
        self.feature_extractor = create_feature_extractor(base, return_nodes=return_nodes)
        self.feature_extractor.eval()
        self.feature_pooler = nn.AvgPool2d(kernel_size=3, stride=1, padding=1)
        self.register_buffer("memory_bank", torch.empty((0, 0), dtype=torch.float32))
        self.register_buffer(
            "imagenet_mean", torch.tensor([0.485, 0.456, 0.406])[None, :, None, None]
        )
        self.register_buffer(
            "imagenet_std", torch.tensor([0.229, 0.224, 0.225])[None, :, None, None]
        )

    def train(self, mode: bool = True) -> "PatchCoreViewModel":
        super().train(False)
        self.feature_extractor.eval()
        return self

    @torch.inference_mode()
    def generate_embedding(self, images: torch.Tensor) -> torch.Tensor:
        normalized = (images - self.imagenet_mean) / self.imagenet_std
        features = self.feature_extractor(normalized)
        ordered = [
            self.feature_pooler(features[f"layer{number}"])
            for number in self.layer_numbers
        ]
        reference_size = max(
            (tensor.shape[-2:] for tensor in ordered),
            key=lambda size: size[0] * size[1],
        )
        resized = [
            tensor
            if tensor.shape[-2:] == reference_size
            else F.interpolate(tensor, reference_size, mode="bilinear")
            for tensor in ordered
        ]
        combined = torch.cat(resized, dim=1)
        return combined.permute(0, 2, 3, 1).reshape(-1, combined.shape[1])

    @staticmethod
    def euclidean_dist(x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        x_norm = x.pow(2).sum(dim=-1, keepdim=True)
        y_norm = y.pow(2).sum(dim=-1, keepdim=True)
        result = torch.matmul(x, y.transpose(-2, -1))
        result.mul_(-2).add_(x_norm).add_(y_norm.transpose(-2, -1))
        return result.clamp_min_(0).sqrt_()

    def nearest_neighbors(
        self, embedding: torch.Tensor, count: int
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if self.memory_bank.numel() == 0:
            raise RuntimeError("PatchCore memory bank is empty")
        count = min(int(count), int(self.memory_bank.shape[0]))
        scores: list[torch.Tensor] = []
        locations: list[torch.Tensor] = []
        for start in range(0, embedding.shape[0], self.distance_chunk_size):
            distances = self.euclidean_dist(
                embedding[start : start + self.distance_chunk_size], self.memory_bank
            )
            if count == 1:
                chunk_scores, chunk_locations = distances.min(dim=1)
            else:
                chunk_scores, chunk_locations = distances.topk(
                    k=count, largest=False, dim=1
                )
            scores.append(chunk_scores)
            locations.append(chunk_locations)
        return torch.cat(scores), torch.cat(locations)

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
        distances = self.euclidean_dist(
            max_features.unsqueeze(1), self.memory_bank[support]
        )
        weights = (1.0 - F.softmax(distances.squeeze(1), dim=1))[..., 0]
        return weights * score

    @torch.inference_mode()
    def forward(self, images: torch.Tensor) -> torch.Tensor:
        embedding = self.generate_embedding(images)
        patch_scores, locations = self.nearest_neighbors(embedding, 1)
        return self.compute_image_score(
            patch_scores, locations, embedding, images.shape[0]
        )


def _load_patchcore_state(model: PatchCoreViewModel, path: Path) -> tuple[int, int]:
    try:
        payload = torch.load(path, map_location="cpu", weights_only=True)
    except Exception as exc:
        raise ArtifactContractError(f"cannot load tensor artifact: {path}") from exc
    if not isinstance(payload, dict):
        raise ArtifactContractError(f"tensor artifact root is invalid: {path}")
    state = payload.get("state_dict")
    bank = payload.get("memory_bank")
    if (
        not isinstance(state, dict)
        or not isinstance(bank, torch.Tensor)
        or bank.ndim != 2
        or bank.numel() == 0
        or bank.dtype != torch.float32
        or not bool(torch.isfinite(bank).all())
    ):
        raise ArtifactContractError(f"PatchCore state/memory bank is invalid: {path}")
    incompatible = model.load_state_dict(state, strict=False)
    if incompatible.unexpected_keys or set(incompatible.missing_keys) != {"memory_bank"}:
        raise ArtifactContractError(
            f"PatchCore state keys differ: missing={incompatible.missing_keys}, "
            f"unexpected={incompatible.unexpected_keys}"
        )
    model.memory_bank = bank
    return int(bank.shape[0]), int(bank.shape[1])


def _base_version(value: str) -> str:
    return str(value).split("+", 1)[0]


class PatchCoreArtifactModel:
    """네 view의 독립 memory bank를 한 versioned artifact로 로드합니다."""

    def __init__(
        self,
        *,
        identity: ModelIdentity,
        manifest: dict[str, Any],
        station_views: Mapping[int, tuple[str, ...]],
        models: Mapping[str, PatchCoreViewModel],
        preprocessor: Mono8PatchCorePreprocessor,
        device: torch.device,
        parallel_count: int,
    ) -> None:
        self.identity = identity
        self.manifest = manifest
        self.station_views = dict(station_views)
        self.models = dict(models)
        self.preprocessor = preprocessor
        self.device = device
        self.parallel_count = int(parallel_count)
        self._streams = [torch.cuda.Stream(device=device) for _ in range(self.parallel_count)]

    @classmethod
    def load(
        cls,
        *,
        artifact_path: Path,
        expected_version: str,
        expected_sha256: str,
        serial_to_view: Mapping[str, str],
        diagnostic_root: Path,
        warmup_runs: int,
        cuda_required: bool,
    ) -> "PatchCoreArtifactModel":
        if not cuda_required:
            raise ArtifactContractError("production PatchCore requires CUDA; CPU fallback is forbidden")
        if not torch.cuda.is_available() or torch.cuda.device_count() < 1:
            raise ArtifactContractError("CUDA device 0 is unavailable")
        if warmup_runs < 1:
            raise ArtifactContractError("model warmup_runs must be positive")
        identity = inspect_model_artifact(
            artifact_path, expected_version, expected_sha256
        )
        root = artifact_path.resolve()
        manifest = _read_json_object(root / "manifest.json")
        station_views = cls._validate_manifest(
            root,
            manifest,
            expected_version=expected_version,
            serial_to_view=serial_to_view,
        )
        warnings = cls._library_compatibility_warnings(manifest)
        identity = ModelIdentity(
            identity.path,
            identity.version,
            identity.sha256,
            identity.runtime,
            warnings,
        )
        parameters = manifest["parameters"]
        device = torch.device("cuda:0")
        torch.cuda.set_device(device)
        models: dict[str, PatchCoreViewModel] = {}
        try:
            for view in manifest["view_names"]:
                model = PatchCoreViewModel(
                    backbone=str(parameters["backbone"]),
                    layer_numbers=parameters["feature_layers"],
                    num_neighbors=int(parameters["k"]),
                    pretrained=False,
                    distance_chunk_size=int(parameters["distance_chunk_size"]),
                )
                actual_shape = _load_patchcore_state(
                    model, root / view / "model.pt"
                )
                expected_shape = tuple(
                    int(value)
                    for value in manifest["memory_bank_shapes"][view]
                )
                if actual_shape != expected_shape:
                    raise ArtifactContractError(
                        f"{view}: memory bank shape differs from manifest"
                    )
                models[view] = model.to(device).eval()
        except Exception:
            models.clear()
            torch.cuda.empty_cache()
            raise

        preprocessing = {
            view: PreprocessingSettings.from_payload(
                view, manifest["preprocessing_by_view"][view]
            )
            for view in manifest["view_names"]
        }
        preprocessor = Mono8PatchCorePreprocessor(
            serial_to_view=serial_to_view,
            settings_by_view=preprocessing,
            input_resolution=parameters["input_resolution"],
            resize_mode=str(parameters["resize_mode"]),
            diagnostic_root=diagnostic_root,
        )
        benchmark = manifest["parallel_benchmark"]
        parallel_count = int(benchmark["selected_parallel_count"])
        instance = cls(
            identity=identity,
            manifest=manifest,
            station_views=station_views,
            models=models,
            preprocessor=preprocessor,
            device=device,
            parallel_count=parallel_count,
        )
        try:
            instance._warmup_and_verify_memory(
                warmup_runs=warmup_runs,
                reserve_mib=int(benchmark["reserve_mib"]),
            )
        except Exception:
            instance.close()
            raise
        return instance

    @staticmethod
    def _validate_manifest(
        root: Path,
        manifest: dict[str, Any],
        *,
        expected_version: str,
        serial_to_view: Mapping[str, str],
    ) -> dict[int, tuple[str, ...]]:
        if manifest.get("format_version") != ARTIFACT_FORMAT_VERSION:
            raise ArtifactContractError("unsupported PatchCore artifact format_version")
        if manifest.get("algorithm") != "patchcore":
            raise ArtifactContractError("artifact algorithm must be patchcore")
        artifact_name = str(manifest.get("artifact_name", ""))
        model_version = str(manifest.get("model_version", artifact_name))
        if not artifact_name or model_version != expected_version:
            raise ArtifactContractError("artifact model_version differs from configuration")
        view_names = manifest.get("view_names")
        expected_views = tuple(
            view for station_id in (1, 2) for view in EXPECTED_STATION_VIEWS[station_id]
        )
        if not isinstance(view_names, list) or tuple(view_names) != expected_views:
            raise ArtifactContractError(
                f"artifact view order must be exactly {list(expected_views)}"
            )
        if set(serial_to_view.values()) != set(expected_views) or len(serial_to_view) != 4:
            raise ArtifactContractError("runtime serial/view mapping must cover four views")
        manifest_station_views = manifest.get("station_views")
        expected_station_document = {
            "station_a": list(EXPECTED_STATION_VIEWS[1]),
            "station_b": list(EXPECTED_STATION_VIEWS[2]),
        }
        if manifest_station_views != expected_station_document:
            raise ArtifactContractError("artifact station_views mapping is invalid")
        camera_serial_by_view = manifest.get("camera_serial_by_view")
        expected_serial_by_view = {view: serial for serial, view in serial_to_view.items()}
        if camera_serial_by_view != expected_serial_by_view:
            raise ArtifactContractError("artifact camera serial/view mapping differs from runtime")

        parameters = manifest.get("parameters")
        if not isinstance(parameters, dict):
            raise ArtifactContractError("artifact parameters are missing")
        required_parameters = {
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
            "view_names",
        }
        if set(parameters) != required_parameters:
            raise ArtifactContractError("artifact PatchCore parameter keys differ")
        if list(parameters["view_names"]) != list(view_names):
            raise ArtifactContractError("parameters.view_names differs from manifest")
        if parameters["resize_mode"] != "padding":
            raise ArtifactContractError("artifact resize_mode must be padding")
        resolution = parameters["input_resolution"]
        if not isinstance(resolution, list) or len(resolution) != 2 or min(map(int, resolution)) < 1:
            raise ArtifactContractError("artifact input_resolution is invalid")
        epsilon = float(parameters["threshold_epsilon"])
        margin = float(parameters["customized_margin"])
        if not math.isfinite(epsilon) or epsilon <= 0:
            raise ArtifactContractError("artifact threshold_epsilon is invalid")
        if not math.isfinite(margin) or not 0 <= margin <= 1:
            raise ArtifactContractError("artifact customized_margin is invalid")

        keyed_fields = (
            "thresholds",
            "memory_bank_shapes",
            "validation_raw_scores",
            "preprocessing_by_view",
        )
        for field in keyed_fields:
            value = manifest.get(field)
            if not isinstance(value, dict) or set(value) != set(view_names):
                raise ArtifactContractError(f"artifact {field} view keys differ")
        for view in view_names:
            threshold = float(manifest["thresholds"][view])
            if not math.isfinite(threshold) or threshold <= epsilon:
                raise ArtifactContractError(f"{view}: threshold is invalid")
            shape = manifest["memory_bank_shapes"][view]
            if not isinstance(shape, list) or len(shape) != 2 or min(map(int, shape)) < 1:
                raise ArtifactContractError(f"{view}: memory bank shape is invalid")
            scores = np.asarray(manifest["validation_raw_scores"][view], dtype=np.float64)
            if scores.size == 0 or not np.all(np.isfinite(scores)):
                raise ArtifactContractError(f"{view}: validation scores are invalid")
            PreprocessingSettings.from_payload(
                view, manifest["preprocessing_by_view"][view]
            )
            calibration = _read_json_object(root / view / "calibration.json")
            if (
                calibration.get("view") != view
                or float(calibration.get("threshold", math.nan)) != threshold
                or calibration.get("memory_bank_shape") != shape
                or calibration.get("validation_raw_scores")
                != manifest["validation_raw_scores"][view]
            ):
                raise ArtifactContractError(f"{view}: calibration differs from manifest")

        benchmark = manifest.get("parallel_benchmark")
        if not isinstance(benchmark, dict):
            raise ArtifactContractError("artifact parallel_benchmark is missing")
        selected = int(benchmark.get("selected_parallel_count", 0))
        reserve = int(benchmark.get("reserve_mib", -1))
        if not 1 <= selected <= 3 or reserve < 0:
            raise ArtifactContractError("artifact parallel benchmark policy is invalid")

        allowed_files = {Path("manifest.json")}
        for view in view_names:
            allowed_files.add(Path(view) / "model.pt")
            allowed_files.add(Path(view) / "calibration.json")
        actual_files = {
            path.relative_to(root) for path in root.rglob("*") if path.is_file()
        }
        if actual_files != allowed_files:
            raise ArtifactContractError(
                "artifact file set differs from manifest contract: "
                f"missing={sorted(map(str, allowed_files - actual_files))}, "
                f"extra={sorted(map(str, actual_files - allowed_files))}"
            )
        return dict(EXPECTED_STATION_VIEWS)

    @staticmethod
    def _library_compatibility_warnings(manifest: dict[str, Any]) -> tuple[str, ...]:
        versions = manifest.get("library_versions")
        if not isinstance(versions, dict):
            raise ArtifactContractError("artifact library_versions are missing")
        artifact_torch = str(versions.get("torch", ""))
        artifact_vision = str(versions.get("torchvision", ""))
        if _base_version(artifact_torch) != _base_version(torch.__version__):
            raise ArtifactContractError(
                f"torch base version differs: artifact={artifact_torch}, runtime={torch.__version__}"
            )
        if _base_version(artifact_vision) != _base_version(torchvision_version):
            raise ArtifactContractError(
                "torchvision base version differs: "
                f"artifact={artifact_vision}, runtime={torchvision_version}"
            )
        warnings: list[str] = []
        if artifact_torch != str(torch.__version__):
            warnings.append(
                f"torch build suffix differs: artifact={artifact_torch}, runtime={torch.__version__}"
            )
        if artifact_vision != str(torchvision_version):
            warnings.append(
                "torchvision build suffix differs: "
                f"artifact={artifact_vision}, runtime={torchvision_version}"
            )
        return tuple(warnings)

    def load_image(self, path: Path) -> PreparedView:
        return self.preprocessor.load(path)

    @torch.inference_mode()
    def infer(self, images: tuple[PreparedView, ...]) -> StationInferenceResult:
        views = tuple(image.view_name for image in images)
        if views == self.station_views[1]:
            station_id = 1
        elif views == self.station_views[2]:
            station_id = 2
        else:
            raise ArtifactContractError(f"inference view order is invalid: {views}")
        expected = self.station_views[station_id]
        raw_scores = self._run_raw(expected, images)
        parameters = self.manifest["parameters"]
        epsilon = float(parameters["threshold_epsilon"])
        margin = float(parameters["customized_margin"])
        normalized: list[float] = []
        view_verdicts: list[int] = []
        diagnostics: list[str] = []
        for view, raw, prepared in zip(expected, raw_scores, images, strict=True):
            threshold = float(self.manifest["thresholds"][view])
            if threshold <= epsilon:
                raise ArtifactContractError(f"{view}: unsafe artifact threshold")
            score = float(raw) / threshold
            if not math.isfinite(score):
                raise RuntimeError(f"{view}: normalized score is not finite")
            is_ng = score >= 1.0 - margin
            normalized.append(score)
            view_verdicts.append(int(Verdict.NG if is_ng else Verdict.PASS))
            if is_ng:
                diagnostics.extend(self.preprocessor.save_diagnostic(prepared, "ng"))
        verdict = int(Verdict.NG if int(Verdict.NG) in view_verdicts else Verdict.PASS)
        return StationInferenceResult(
            verdict=verdict,
            score=max(normalized),
            model_version=self.identity.version,
            model_sha256=self.identity.sha256,
            view_verdicts=tuple(view_verdicts),
            view_scores=tuple(normalized),
            view_raw_scores=tuple(raw_scores),
            view_names=views,
            diagnostic_paths=tuple(diagnostics),
        )

    def _run_raw(
        self,
        views: Sequence[str],
        images: Sequence[PreparedView],
    ) -> list[float]:
        parallel = min(self.parallel_count, len(views))
        raw_scores: list[float] = []
        for start in range(0, len(views), parallel):
            wave_views = views[start : start + parallel]
            wave_images = images[start : start + parallel]
            outputs: list[torch.Tensor] = []
            for slot, (view, prepared) in enumerate(
                zip(wave_views, wave_images, strict=True)
            ):
                with torch.cuda.stream(self._streams[slot]):
                    tensor = prepared.tensor[None].to(self.device, non_blocking=True)
                    outputs.append(self.models[view](tensor))
            for stream in self._streams[: len(wave_views)]:
                stream.synchronize()
            raw_scores.extend(float(output[0].detach().cpu()) for output in outputs)
        return raw_scores

    def _warmup_and_verify_memory(self, *, warmup_runs: int, reserve_mib: int) -> None:
        height, width = (int(value) for value in self.manifest["parameters"]["input_resolution"])
        prepared = tuple(
            PreparedView(
                source_path=Path(f"/{view}.png"),
                view_name=view,
                tensor=torch.zeros((3, height, width), dtype=torch.float32),
                crop_1=np.zeros((1, 1), dtype=np.uint8),
                crop_2=np.zeros((1, 1), dtype=np.uint8),
                original_component_count=1,
            )
            for view in self.station_views[1]
        )
        for _ in range(warmup_runs):
            self._run_raw(self.station_views[1], prepared)
        torch.cuda.synchronize(self.device)
        torch.cuda.empty_cache()
        free_bytes, _ = torch.cuda.mem_get_info(self.device)
        available = max(0, free_bytes - reserve_mib * 1024**2)
        base_allocated = torch.cuda.memory_allocated(self.device)
        torch.cuda.reset_peak_memory_stats(self.device)
        self._run_raw(self.station_views[1], prepared)
        torch.cuda.synchronize(self.device)
        peak_extra = max(
            0, torch.cuda.max_memory_allocated(self.device) - base_allocated
        )
        if peak_extra > available:
            raise ArtifactContractError(
                "artifact parallel_count is unsafe for current free VRAM reserve"
            )

    def handle_cuda_oom(self) -> None:
        try:
            torch.cuda.synchronize(self.device)
        except Exception:
            pass
        torch.cuda.empty_cache()

    def close(self) -> None:
        self.models.clear()
        self._streams.clear()
        torch.cuda.empty_cache()


class FakeStationModel:
    """실제 artifact 없이 ROS pipeline 계약을 검증하는 sim 전용 모델."""

    identity = ModelIdentity("", "fake-v0", "0" * 64, "FAKE")

    def infer(self, images: tuple[object, ...]) -> StationInferenceResult:
        if len(images) not in {1, 3}:
            raise ValueError("station batch must contain exactly 1 or 3 views")
        view_scores = tuple(0.1 for _ in images)
        view_verdicts = tuple(int(Verdict.PASS) for _ in images)
        return StationInferenceResult(
            verdict=int(Verdict.PASS),
            score=max(view_scores),
            model_version=self.identity.version,
            model_sha256=self.identity.sha256,
            view_verdicts=view_verdicts,
            view_scores=view_scores,
        )
