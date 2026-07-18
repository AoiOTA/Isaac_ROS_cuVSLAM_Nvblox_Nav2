"""Adaptive wall-time sampling backed by Isaac Sim's official recorders."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import statistics
import time
from typing import Any

from isaacsim.benchmark.services.datarecorders.app_frametime import (
    AppFrametimeRecorder,
)
from isaacsim.benchmark.services.datarecorders.cpu_continuous import (
    CPUContinuousRecorder,
)
from isaacsim.benchmark.services.datarecorders.hardware import HardwareSpecRecorder
from isaacsim.benchmark.services.datarecorders.interface import InputContext
from isaacsim.benchmark.services.datarecorders.memory import MemoryRecorder
from isaacsim.benchmark.services.datarecorders.physics_frametime import (
    PhysicsFrametimeRecorder,
)


@dataclass(frozen=True)
class AdaptiveSamplingConfig:
    minimum_warmup_s: float
    maximum_warmup_s: float
    stability_window_s: float
    stable_windows_required: int
    maximum_mean_change_ratio: float
    maximum_coefficient_of_variation: float
    minimum_sample_s: float
    maximum_sample_s: float

    def validate(self) -> None:
        if min(
            self.minimum_warmup_s,
            self.maximum_warmup_s,
            self.stability_window_s,
            self.minimum_sample_s,
            self.maximum_sample_s,
        ) <= 0.0:
            raise ValueError("performance durations must be positive")
        if self.maximum_warmup_s < self.minimum_warmup_s:
            raise ValueError("maximum warmup must be at least the minimum")
        if self.maximum_sample_s < self.minimum_sample_s:
            raise ValueError("maximum sample duration must be at least the minimum")
        if self.stable_windows_required < 1:
            raise ValueError("stable_windows_required must be positive")
        if not 0.0 < self.maximum_mean_change_ratio <= 1.0:
            raise ValueError("maximum_mean_change_ratio must be in (0, 1]")
        if not 0.0 < self.maximum_coefficient_of_variation <= 1.0:
            raise ValueError("maximum_coefficient_of_variation must be in (0, 1]")


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    rank = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower, upper = math.floor(rank), math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def sample_stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "sample_count": 0,
            "mean": 0.0,
            "stdev": 0.0,
            "minimum": 0.0,
            "maximum": 0.0,
            "p50": 0.0,
            "p95": 0.0,
            "p99": 0.0,
        }
    return {
        "sample_count": len(values),
        "mean": statistics.fmean(values),
        "stdev": statistics.stdev(values) if len(values) > 1 else 0.0,
        "minimum": min(values),
        "maximum": max(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
    }


class AdaptiveOfficialBenchmark:
    """Start official recorders only after workload and frame times stabilize."""

    def __init__(
        self,
        config: AdaptiveSamplingConfig,
        *,
        camera_profile: str,
        start_file: Path | None,
    ) -> None:
        config.validate()
        self.config = config
        self.camera_profile = camera_profile
        self.start_file = start_file
        self.phase = "waiting_for_workload" if start_file is not None else "warmup"
        self.created_monotonic = time.monotonic()
        self.workload_ready_monotonic: float | None = None
        self.workload_ready_unix_s: float | None = None
        self.sample_started_monotonic: float | None = None
        self.sample_started_unix_s: float | None = None
        self.sample_ended_monotonic: float | None = None
        self.sample_ended_unix_s: float | None = None
        self.warmup_stability_reached = False
        self.sample_stability_reached = False
        self.stop_reason = "not_started"
        self.warmup_windows: list[dict[str, float | int | bool]] = []
        self.sample_windows: list[dict[str, float | int | bool]] = []
        self._previous_window_mean: float | None = None
        self._stable_streak = 0
        self._last_window_sample_index = 0
        self._next_window_monotonic: float | None = None
        self._probe: AppFrametimeRecorder | None = None
        self._app: AppFrametimeRecorder | None = None
        self._physics: PhysicsFrametimeRecorder | None = None
        self._cpu: CPUContinuousRecorder | None = None
        self._measurements: list[dict[str, object]] = []
        if self.phase == "warmup":
            self._start_warmup()

    def _context(self, phase: str) -> InputContext:
        return InputContext(
            artifact_prefix=f"kujiale_jackal_{self.camera_profile}",
            kit_version="Isaac Sim 6.0.1",
            phase=phase,
        )

    def _start_warmup(self) -> None:
        now = time.monotonic()
        self.phase = "warmup"
        self.workload_ready_monotonic = now
        self.workload_ready_unix_s = time.time()
        self._probe = AppFrametimeRecorder(self._context("adaptive_warmup"))
        self._probe.start_collecting()
        self._reset_windows(now)

    def _reset_windows(self, now: float) -> None:
        self._previous_window_mean = None
        self._stable_streak = 0
        self._last_window_sample_index = 0
        self._next_window_monotonic = now + self.config.stability_window_s

    def _evaluate_window(
        self,
        samples: list[float],
        elapsed_s: float,
        destination: list[dict[str, float | int | bool]],
    ) -> None:
        new_samples = samples[self._last_window_sample_index :]
        self._last_window_sample_index = len(samples)
        if len(new_samples) < 2:
            self._stable_streak = 0
            destination.append(
                {
                    "elapsed_s": elapsed_s,
                    "sample_count": len(new_samples),
                    "mean_ms": 0.0,
                    "coefficient_of_variation": math.inf,
                    "mean_change_ratio": math.inf,
                    "stable": False,
                }
            )
            return
        mean = statistics.fmean(new_samples)
        stdev = statistics.stdev(new_samples)
        coefficient = stdev / mean if mean > 0.0 else math.inf
        change = (
            abs(mean - self._previous_window_mean) / self._previous_window_mean
            if self._previous_window_mean not in (None, 0.0)
            else math.inf
        )
        stable = (
            coefficient <= self.config.maximum_coefficient_of_variation
            and change <= self.config.maximum_mean_change_ratio
        )
        self._stable_streak = self._stable_streak + 1 if stable else 0
        destination.append(
            {
                "elapsed_s": elapsed_s,
                "sample_count": len(new_samples),
                "mean_ms": mean,
                "coefficient_of_variation": coefficient,
                "mean_change_ratio": change,
                "stable": stable,
                "stable_streak": self._stable_streak,
            }
        )
        self._previous_window_mean = mean

    def _start_sample(self, now: float, stable: bool) -> None:
        if self._probe is not None:
            self._probe.stop_collecting()
            self._probe = None
        self.warmup_stability_reached = stable
        self.phase = "sampling"
        self.sample_started_monotonic = now
        self.sample_started_unix_s = time.time()
        context = self._context("benchmark")
        self._app = AppFrametimeRecorder(context)
        self._physics = PhysicsFrametimeRecorder(context)
        self._cpu = CPUContinuousRecorder(context)
        for recorder in (self._app, self._physics, self._cpu):
            recorder.start_collecting()
        self._reset_windows(now)

    @staticmethod
    def _measurement_dict(measurement: object) -> dict[str, object]:
        result = {"name": str(getattr(measurement, "name", ""))}
        for field in ("value", "bvalue", "unit", "type"):
            if hasattr(measurement, field):
                result[field] = getattr(measurement, field)
        return result

    def _finish_sample(self, now: float, reason: str, stable: bool) -> None:
        self.sample_ended_monotonic = now
        self.sample_ended_unix_s = time.time()
        self.sample_stability_reached = stable
        self.stop_reason = reason
        for recorder in (self._app, self._physics, self._cpu):
            if recorder is not None:
                recorder.stop_collecting()
        recorders: list[object] = [
            recorder
            for recorder in (self._app, self._physics, self._cpu)
            if recorder is not None
        ]
        recorders.extend((MemoryRecorder(), HardwareSpecRecorder(self._context("benchmark"))))
        self._measurements = [
            self._measurement_dict(measurement)
            for recorder in recorders
            for measurement in recorder.get_data().measurements
        ]
        self.phase = "completed"

    def tick(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if self.phase == "waiting_for_workload":
            if self.start_file is not None and self.start_file.exists():
                self._start_warmup()
            return
        if self.phase == "warmup":
            assert self.workload_ready_monotonic is not None
            assert self._probe is not None
            elapsed = now - self.workload_ready_monotonic
            if self._next_window_monotonic is not None and now >= self._next_window_monotonic:
                self._evaluate_window(self._probe.samples, elapsed, self.warmup_windows)
                self._next_window_monotonic += self.config.stability_window_s
            stable = (
                elapsed >= self.config.minimum_warmup_s
                and self._stable_streak >= self.config.stable_windows_required
            )
            if stable or elapsed >= self.config.maximum_warmup_s:
                self._start_sample(now, stable)
            return
        if self.phase == "sampling":
            assert self.sample_started_monotonic is not None
            assert self._app is not None
            elapsed = now - self.sample_started_monotonic
            if self._next_window_monotonic is not None and now >= self._next_window_monotonic:
                self._evaluate_window(self._app.samples, elapsed, self.sample_windows)
                self._next_window_monotonic += self.config.stability_window_s
            stable = (
                elapsed >= self.config.minimum_sample_s
                and self._stable_streak >= self.config.stable_windows_required
            )
            if stable:
                self._finish_sample(now, "stable_after_minimum_wall_time", True)
            elif elapsed >= self.config.maximum_sample_s:
                self._finish_sample(now, "maximum_wall_time_reached", False)

    @property
    def completed(self) -> bool:
        return self.phase == "completed" and self.stop_reason in {
            "stable_after_minimum_wall_time",
            "maximum_wall_time_reached",
        }

    def stop_incomplete(self) -> None:
        if self.phase == "warmup" and self._probe is not None:
            self._probe.stop_collecting()
            self._probe = None
        elif self.phase == "sampling":
            self._finish_sample(time.monotonic(), "interrupted_before_completion", False)

    def report(self) -> dict[str, object]:
        app_samples = list(self._app.samples) if self._app is not None else []
        physics_samples = (
            list(self._physics.samples) if self._physics is not None else []
        )
        warmup_duration = (
            (self.sample_started_monotonic - self.workload_ready_monotonic)
            if self.sample_started_monotonic is not None
            and self.workload_ready_monotonic is not None
            else None
        )
        sample_duration = (
            (self.sample_ended_monotonic - self.sample_started_monotonic)
            if self.sample_ended_monotonic is not None
            and self.sample_started_monotonic is not None
            else None
        )
        return {
            "method": "adaptive_wall_time_official_isaac_recorders",
            "official_recorder_source": "isaacsim.benchmark.services 6.0.1",
            "camera_profile": self.camera_profile,
            "fixed_frame_count": None,
            "fixed_kpi_thresholds": None,
            "completed": self.completed,
            "phase": self.phase,
            "parameters": asdict(self.config),
            "workload_ready_unix_s": self.workload_ready_unix_s,
            "warmup": {
                "duration_s": warmup_duration,
                "stability_reached": self.warmup_stability_reached,
                "windows": self.warmup_windows,
            },
            "sample": {
                "duration_s": sample_duration,
                "start_unix_s": self.sample_started_unix_s,
                "end_unix_s": self.sample_ended_unix_s,
                "stop_reason": self.stop_reason,
                "stability_reached": self.sample_stability_reached,
                "windows": self.sample_windows,
                "app_update_frametime_ms": sample_stats(app_samples),
                "physics_frametime_ms": sample_stats(physics_samples),
                "official_measurements": self._measurements,
            },
        }
