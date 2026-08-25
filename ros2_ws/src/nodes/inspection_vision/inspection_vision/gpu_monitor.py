"""NVML이 있을 때만 동작하는 session GPU/VRAM sampler."""

from __future__ import annotations

import threading
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class GpuSnapshot:
    available: bool
    sample_count: int
    mean_utilization_pct: float
    mean_vram_used_mib: float
    peak_vram_used_mib: float
    total_vram_mib: float
    reason: str = ""


class GpuMonitor:
    def __init__(self, *, interval_seconds: float = 0.2, device_index: int = 0) -> None:
        if interval_seconds <= 0:
            raise ValueError("GPU sampling interval must be positive")
        self._interval = interval_seconds
        self._device_index = device_index
        self._stop = threading.Event()
        self._guard = threading.Lock()
        self._thread: threading.Thread | None = None
        self._utilization_sum = 0.0
        self._vram_sum = 0.0
        self._vram_peak = 0.0
        self._total_vram = 0.0
        self._samples = 0
        self._reason = "not started"

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="vision-gpu-monitor", daemon=True
        )
        self._thread.start()

    def stop(self) -> GpuSnapshot:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._interval * 2))
            self._thread = None
        return self.snapshot()

    def snapshot(self) -> GpuSnapshot:
        with self._guard:
            count = self._samples
            return GpuSnapshot(
                available=count > 0,
                sample_count=count,
                mean_utilization_pct=(self._utilization_sum / count if count else 0.0),
                mean_vram_used_mib=(self._vram_sum / count if count else 0.0),
                peak_vram_used_mib=self._vram_peak,
                total_vram_mib=self._total_vram,
                reason="" if count else self._reason,
            )

    def _run(self) -> None:
        try:
            import pynvml  # type: ignore[import-not-found]

            pynvml.nvmlInit()
            handle = pynvml.nvmlDeviceGetHandleByIndex(self._device_index)
        except Exception as exc:
            with self._guard:
                self._reason = f"NVML unavailable: {type(exc).__name__}"
            return
        try:
            while not self._stop.wait(self._interval):
                utilization = pynvml.nvmlDeviceGetUtilizationRates(handle)
                memory = pynvml.nvmlDeviceGetMemoryInfo(handle)
                used_mib = float(memory.used) / (1024 * 1024)
                with self._guard:
                    self._utilization_sum += float(utilization.gpu)
                    self._vram_sum += used_mib
                    self._vram_peak = max(self._vram_peak, used_mib)
                    self._total_vram = float(memory.total) / (1024 * 1024)
                    self._samples += 1
        finally:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass
