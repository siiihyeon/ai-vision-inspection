"""추론 알고리즘을 Vision orchestration에서 분리하는 plugin 계약."""

from __future__ import annotations

import hashlib
import importlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


@dataclass(frozen=True, slots=True)
class StationInference:
    verdict: int
    score: float
    model_version: str

    def validate(self) -> None:
        if self.verdict not in {1, 2}:
            raise ValueError("station verdict must be PASS(1) or NG(2)")
        if not math.isfinite(self.score):
            raise ValueError("station inference score must be finite")
        if not self.model_version:
            raise ValueError("model_version is required")


class ModelAdapter(Protocol):
    @property
    def model_version(self) -> str: ...

    def warmup(self) -> None:
        """실제 입력과 같은 device/runtime 경로를 최소 1회 실행합니다."""

    def infer(self, images_rgb: tuple[object, ...]) -> StationInference:
        """정렬된 station 카메라 RGB 배열을 하나의 station 결과로 결합합니다."""

    def close(self) -> None:
        """GPU/런타임 자원을 해제합니다."""


class SimulationModel:
    """알고리즘 정확도 시험에 사용하면 안 되는 sim wiring 전용 모델."""

    model_version = "simulation-wiring-only-v1"

    def warmup(self) -> None:
        return None

    def infer(self, images_rgb: tuple[object, ...]) -> StationInference:
        if not images_rgb:
            raise ValueError("simulation inference requires at least one image")
        return StationInference(verdict=1, score=1.0, model_version=self.model_version)

    def close(self) -> None:
        return None


def load_rgb_png_with_opencv(path: Path) -> object:
    """canonical RGB PNG를 OpenCV BGR decode 후 RGB ndarray로 반환합니다."""

    try:
        cv2 = importlib.import_module("cv2")
    except ImportError as exc:
        raise RuntimeError("OpenCV Python binding is required to load inference images") from exc
    image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError(f"OpenCV could not decode PNG: {path}")
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return image_rgb


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_plugin_model(
    *,
    factory_reference: str,
    model_path: Path,
    expected_sha256: str,
    device: str,
) -> ModelAdapter:
    """`package.module:factory`를 로드하고 artifact digest를 먼저 검증합니다."""

    if ":" not in factory_reference:
        raise ValueError("model factory must use 'package.module:factory' format")
    if not model_path.is_absolute() or not model_path.is_file():
        raise ValueError("model_path must be an existing absolute file")
    if len(expected_sha256) != 64 or sha256_file(model_path) != expected_sha256:
        raise ValueError("model artifact SHA-256 mismatch")
    module_name, factory_name = factory_reference.rsplit(":", 1)
    module = importlib.import_module(module_name)
    factory = getattr(module, factory_name, None)
    if not callable(factory):
        raise TypeError("model factory reference is not callable")
    model = factory(model_path=model_path, device=device)
    for attribute in ("warmup", "infer", "close", "model_version"):
        if not hasattr(model, attribute):
            raise TypeError(f"model adapter is missing {attribute}")
    return model
