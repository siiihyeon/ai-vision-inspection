"""정상 프로그램 종료 시 CSV·성능 summary와 다음 실행 timeout 제안을 생성합니다."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

from .storage import LogRepository


@dataclass(frozen=True, slots=True)
class SessionReportResult:
    csv_path: Path
    summary_path: Path
    tuning_path: Path | None


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _nearest_rank(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(quantile * len(ordered)) - 1))
    return ordered[index]


def _atomic_json(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def generate_session_report(
    repository: LogRepository,
    *,
    session_id: str,
    report_root: Path,
    timeout_tuning_path: Path,
    auto_apply_timeouts: bool,
    minimum_samples: int,
    safety_factor: float,
) -> SessionReportResult:
    """정답 label 기반 지표 없이 승인된 운영 지표만 기록합니다."""

    session_root = report_root / session_id
    session_root.mkdir(parents=True, exist_ok=True)
    csv_path = session_root / "product_inference_results.csv"
    rows = repository.session_report_rows(session_id)
    fieldnames = [
        "fifo_sequence",
        "product_id",
        "final_verdict",
        "final_reason",
        "station_a_terminal_kind",
        "station_a_verdict",
        "station_a_error_code",
        "station_a_model_forward_ms",
        "station_a_enqueue_to_result_ms",
        "station_b_terminal_kind",
        "station_b_verdict",
        "station_b_error_code",
        "station_b_model_forward_ms",
        "station_b_enqueue_to_result_ms",
        "product_total_ms",
    ]
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(fieldnames)
        writer.writerows(rows)

    a_samples, a_version, a_sha, a_fingerprint = (
        repository.station_timeout_samples(session_id, 1)
    )
    b_samples, b_version, b_sha, b_fingerprint = (
        repository.station_timeout_samples(session_id, 2)
    )
    forward_a = [float(row[7]) for row in rows if row[7] is not None]
    forward_b = [float(row[12]) for row in rows if row[12] is not None]
    product_totals = [float(row[14]) for row in rows if row[14] is not None]
    capture_candidates = repository.capture_timeout_samples(session_id)
    gpu = repository.gpu_metrics(session_id)
    station_summary: dict[str, dict[str, object]] = {}
    tuning_values: dict[str, int] = {}
    for station_name, samples, version, sha, fingerprint in (
        ("station_a", a_samples, a_version, a_sha, a_fingerprint),
        ("station_b", b_samples, b_version, b_sha, b_fingerprint),
    ):
        p999 = _nearest_rank(samples, 0.999)
        eligible = (
            len(samples) >= minimum_samples
            and bool(version)
            and bool(sha)
            and bool(fingerprint)
        )
        candidate = math.ceil(p999 * safety_factor) if eligible and p999 else None
        station_summary[station_name] = {
            "sample_count": len(samples),
            "enqueue_to_result_p99_9_ms": p999,
            "safety_factor": safety_factor,
            "recommended_total_timeout_ms": candidate,
            "eligible_for_next_run": eligible,
        }
        if candidate is not None:
            tuning_values[station_name] = candidate

    same_identity = all(
        value
        for value in (
            a_version,
            a_sha,
            a_fingerprint,
            b_version,
            b_sha,
            b_fingerprint,
        )
    ) and all(
        left == right
        for left, right in (
            (a_version, b_version),
            (a_sha, b_sha),
            (a_fingerprint, b_fingerprint),
        )
    )
    gpu_document = None
    if gpu is not None:
        gpu_document = {
            "normal_shutdown": bool(gpu[0]),
            "sample_count": int(gpu[1]),
            "mean_gpu_utilization_pct": float(gpu[2]),
            "mean_vram_used_mib": float(gpu[3]),
            "peak_vram_used_mib": float(gpu[4]),
            "total_vram_mib": float(gpu[5]),
        }
    summary = {
        "schema_version": 1,
        "session_id": session_id,
        "normal_shutdown": True,
        "product_count": len(rows),
        "model_forward_ms": {
            "station_a_mean": _mean(forward_a),
            "station_b_mean": _mean(forward_b),
        },
        "product_total_ms": {
            "mean": _mean(product_totals),
            "p99_9": _nearest_rank(product_totals, 0.999),
        },
        "capture_timeout_candidate_ms": {
            "sample_count": len(capture_candidates),
            "mean": _mean(capture_candidates),
            "formula": "mean acquisition * 2 + save overhead + validation",
        },
        "inference_timeout_tuning": station_summary,
        "gpu": gpu_document,
        "excluded_metrics": [
            "confusion_matrix",
            "accuracy",
            "precision",
            "recall",
            "f1",
        ],
    }
    summary_path = session_root / "session_summary.json"
    _atomic_json(summary_path, summary)

    written_tuning_path: Path | None = None
    if (
        auto_apply_timeouts
        and set(tuning_values) == {"station_a", "station_b"}
        and same_identity
    ):
        _atomic_json(
            timeout_tuning_path,
            {
                "schema_version": 1,
                "source_session_id": session_id,
                "model_version": a_version,
                "model_sha256": a_sha,
                "config_fingerprint": a_fingerprint,
                "minimum_samples": minimum_samples,
                "safety_factor": safety_factor,
                "timeouts_ms": tuning_values,
            },
        )
        written_tuning_path = timeout_tuning_path
    return SessionReportResult(csv_path, summary_path, written_tuning_path)
