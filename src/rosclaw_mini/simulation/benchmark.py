"""Repeatable headless benchmark for simulation-only grasp strategy studies."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from statistics import mean, pstdev
from typing import Iterable, Sequence

from rosclaw_mini.runtime import build_sim_runtime
from rosclaw_mini.simulation.pipeline import SimulationGraspPipeline
from rosclaw_mini.simulation.strategy import GraspStrategyVersion


@dataclass(frozen=True)
class SimulationBenchmarkRun:
    scene: str
    seed: int
    task: str
    strategy: str
    success: bool
    message: str
    localization_error_m: float | None
    simulated_motion_time_s: float
    tcp_path_length_m: float
    joint_total_motion_rad: float
    smoothness_delta_rad: float
    reobserve_count: int
    collision_events: int
    workspace_rejections: int


@dataclass(frozen=True)
class SimulationBenchmarkSummary:
    strategy: str
    runs: int
    successes: int
    success_rate: float
    mean_motion_time_s: float
    std_motion_time_s: float
    mean_tcp_path_length_m: float
    mean_joint_total_motion_rad: float
    mean_smoothness_delta_rad: float
    mean_localization_error_m: float | None
    mean_reobservations: float
    failure_reasons: dict[str, int]


def _metrics(runtime, result) -> tuple[float, float, float, float]:
    adapter = runtime.adapter
    records = adapter.world.motion_records
    motion_time = sum(record.waypoint_count / 30.0 for record in records)
    path = 0.0
    if records:
        previous = records[0].target_tcp_m
        for record in records[1:]:
            path += math.dist(previous, record.target_tcp_m)
            previous = record.target_tcp_m
    waypoints = adapter.executed_waypoints
    total_joint_motion = sum(
        sum(abs(next_value - current_value) for current_value, next_value in zip(first, second, strict=True))
        for first, second in zip(waypoints, waypoints[1:])
    )
    deltas = [
        tuple(next_value - current_value for current_value, next_value in zip(first, second, strict=True))
        for first, second in zip(waypoints, waypoints[1:])
    ]
    smoothness = sum(
        sum(abs(next_value - current_value) for current_value, next_value in zip(first, second, strict=True))
        for first, second in zip(deltas, deltas[1:])
    )
    return motion_time, path, total_joint_motion, smoothness


def run_headless_benchmark(
    *,
    scenes: Sequence[str] = (
        "standard",
        "multi_object",
        "randomized",
        "noisy",
        "depth_holes",
        "boundary_reject",
    ),
    seeds: Sequence[int] = (0, 1, 2),
    strategies: Sequence[GraspStrategyVersion] = (
        GraspStrategyVersion.BASELINE_V1,
        GraspStrategyVersion.EFFICIENT_V2,
        GraspStrategyVersion.HUMANLIKE_V3,
    ),
    task: str | None = None,
) -> tuple[SimulationBenchmarkRun, ...]:
    """Run matched scene/seed comparisons without hardware, GUI or network."""

    runs: list[SimulationBenchmarkRun] = []
    for scene in scenes:
        for seed in seeds:
            selected_task = task or _default_task_for_scene(scene, seed)
            for strategy in strategies:
                runtime = build_sim_runtime(scene_name=scene, seed=seed)
                try:
                    result = SimulationGraspPipeline(
                        runtime,
                        runtime.adapter.world,
                    ).run(
                        selected_task,
                        strategy_version=strategy,
                        allow_one_reobserve=(strategy is GraspStrategyVersion.HUMANLIKE_V3),
                    )
                    motion_time, path, joint_motion, smoothness = _metrics(runtime, result)
                    runs.append(
                        SimulationBenchmarkRun(
                            scene=scene,
                            seed=seed,
                            task=selected_task,
                            strategy=strategy.value,
                            success=result.success,
                            message=result.message,
                            localization_error_m=result.localization_error_m,
                            simulated_motion_time_s=motion_time,
                            tcp_path_length_m=path,
                            joint_total_motion_rad=joint_motion,
                            smoothness_delta_rad=smoothness,
                            reobserve_count=result.reobserve_count,
                            collision_events=len(runtime.adapter.world.collision_events),
                            workspace_rejections=int("工作空间" in result.message or "Safety/Gateway" in result.message),
                        )
                    )
                finally:
                    runtime.shutdown()
    return tuple(runs)


def _default_task_for_scene(scene: str, seed: int) -> str:
    """Cycle shapes in the multi-object scene while retaining matched seeds."""

    if scene == "multi_object":
        return "抓取蓝色盒子" if seed % 2 == 0 else "抓取绿色圆柱体"
    return "抓取红色立方体"


def summarize_runs(runs: Iterable[SimulationBenchmarkRun]) -> tuple[SimulationBenchmarkSummary, ...]:
    grouped: dict[str, list[SimulationBenchmarkRun]] = defaultdict(list)
    for run in runs:
        grouped[run.strategy].append(run)
    summaries = []
    for strategy, values in sorted(grouped.items()):
        failures = Counter(run.message for run in values if not run.success)
        localization = [run.localization_error_m for run in values if run.localization_error_m is not None]
        timings = [run.simulated_motion_time_s for run in values]
        summaries.append(
            SimulationBenchmarkSummary(
                strategy=strategy,
                runs=len(values),
                successes=sum(run.success for run in values),
                success_rate=sum(run.success for run in values) / len(values),
                mean_motion_time_s=mean(timings),
                std_motion_time_s=pstdev(timings),
                mean_tcp_path_length_m=mean(run.tcp_path_length_m for run in values),
                mean_joint_total_motion_rad=mean(run.joint_total_motion_rad for run in values),
                mean_smoothness_delta_rad=mean(run.smoothness_delta_rad for run in values),
                mean_localization_error_m=mean(localization) if localization else None,
                mean_reobservations=mean(run.reobserve_count for run in values),
                failure_reasons=dict(failures),
            )
        )
    return tuple(summaries)


def save_benchmark_results(
    output_path: Path,
    runs: Sequence[SimulationBenchmarkRun],
) -> Path:
    """Save only compact JSON/Markdown metrics; never save real calibration."""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    summaries = summarize_runs(runs)
    output_path.write_text(
        json.dumps(
            {
                "simulation_only": True,
                "runs": [asdict(run) for run in runs],
                "summary": [asdict(summary) for summary in summaries],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    markdown_path = output_path.with_suffix(".md")
    lines = [
        "# Simulation benchmark result",
        "",
        "This file records headless **simulation-only** measurements, not real-arm performance.",
        "",
        "| strategy | success | motion time mean ± std (s) | TCP path (m) | joint motion (rad) | localization error (m) | reobserve |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ]
    for item in summaries:
        localization = "n/a" if item.mean_localization_error_m is None else f"{item.mean_localization_error_m:.4f}"
        lines.append(
            f"| {item.strategy} | {item.successes}/{item.runs} ({item.success_rate:.0%}) | "
            f"{item.mean_motion_time_s:.3f} ± {item.std_motion_time_s:.3f} | "
            f"{item.mean_tcp_path_length_m:.3f} | {item.mean_joint_total_motion_rad:.3f} | "
            f"{localization} | {item.mean_reobservations:.2f} |"
        )
    markdown_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path
