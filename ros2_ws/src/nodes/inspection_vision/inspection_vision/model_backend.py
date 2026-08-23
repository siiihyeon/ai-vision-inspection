"""나중에 실제 전처리/출력 decoder를 주입하는 TorchScript 계약."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from inspection_common import Verdict


@dataclass(frozen=True, slots=True)
class ModelIdentity:
    path: str
    version: str
    sha256: str
    runtime: str = "PYTORCH_TORCHSCRIPT"


@dataclass(frozen=True, slots=True)
class StationInferenceResult:
    verdict: int
    score: float
    model_version: str
    model_sha256: str
    view_verdicts: tuple[int, ...]
    view_scores: tuple[float, ...]


class StationModelBackend(Protocol):
    identity: ModelIdentity

    def infer(self, images: tuple[object, ...]) -> StationInferenceResult:
        """Station A는 3개 view, B는 1개 view를 한 batch로 처리합니다."""


def inspect_model_artifact(path: Path, version: str, expected_sha256: str) -> ModelIdentity:
    if not path.is_absolute() or not path.is_file():
        raise ValueError("model path must be an existing absolute file")
    if not version:
        raise ValueError("model version is required")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if len(expected_sha256) != 64 or digest != expected_sha256.lower():
        raise ValueError("model SHA-256 differs from configured identity")
    return ModelIdentity(str(path), version, digest)


class FakeStationModel:
    """모델 계약과 A=3/B=1 batch 흐름을 검증하는 sim 전용 모델."""

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


class UnconfiguredTorchScriptModel:
    """입력 크기·normalization·decoder가 주입되기 전 hardware fail-closed 경계."""

    def __init__(self, identity: ModelIdentity) -> None:
        self.identity = identity

    def infer(self, _images: tuple[object, ...]) -> StationInferenceResult:
        raise NotImplementedError(
            "TorchScript preprocessing/output decoder contract is not configured"
        )
