"""Immutable visual grasp plans, fail-closed preview, and Mock execution.

This module never reads a camera or robot by itself.  Perception produces a
``BasePositionEstimate``; planning turns it into fixed Commands; preview runs
the existing Gateway/Safety Checker for every step.  Actual hardware use is
intentionally not exposed here yet.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from threading import Event, Lock
from typing import Any
from uuid import uuid4

from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.gateway.command.gateway import preflight_command, run_command
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.vision.exceptions import GraspPlanningError
from rosclaw_mini.vision.localization import BasePositionEstimate


PositionValidator = Callable[[float, float, float], tuple[float, float, float]]


def _finite(value: object, *, label: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise GraspPlanningError(f"{label}必须是有限数值。")
    return float(value)


def _finite_triplet(values, *, label: str) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)):
        raise GraspPlanningError(f"{label}必须是三个有限数值。")
    try:
        result = tuple(values)
    except TypeError as error:
        raise GraspPlanningError(
            f"{label}必须是三个有限数值。"
        ) from error
    if len(result) != 3:
        raise GraspPlanningError(f"{label}必须是三个有限数值。")
    return tuple(
        _finite(value, label=f"{label}[{index}]")
        for index, value in enumerate(result)
    )  # type: ignore[return-value]


@dataclass(frozen=True)
class GraspPlanningConfig:
    pre_grasp_height_m: float = 0.08
    approach_tcp_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lift_height_m: float = 0.08
    maximum_cartesian_step_m: float = 0.12
    maximum_localization_uncertainty_m: float = 0.03
    maximum_data_age_seconds: float = 30.0
    allowed_localization_qualities: tuple[str, ...] = ("good", "usable")

    def __post_init__(self) -> None:
        for field_name in (
            "pre_grasp_height_m",
            "lift_height_m",
            "maximum_cartesian_step_m",
            "maximum_localization_uncertainty_m",
            "maximum_data_age_seconds",
        ):
            value = _finite(getattr(self, field_name), label=field_name)
            if value <= 0.0:
                raise GraspPlanningError(f"{field_name}必须大于 0。")
            object.__setattr__(self, field_name, value)
        object.__setattr__(
            self,
            "approach_tcp_offset_m",
            _finite_triplet(
                self.approach_tcp_offset_m,
                label="approach_tcp_offset_m",
            ),
        )
        qualities = tuple(self.allowed_localization_qualities)
        if not qualities or any(
            not isinstance(value, str) or not value.strip() for value in qualities
        ):
            raise GraspPlanningError(
                "allowed_localization_qualities 不能为空。"
            )
        object.__setattr__(self, "allowed_localization_qualities", qualities)


@dataclass(frozen=True)
class GraspPlanStep:
    stage: str
    command: Command

    def to_dict(self) -> dict[str, Any]:
        return {
            "stage": self.stage,
            "skill_name": self.command.skill_name,
            "params": dict(self.command.params),
        }


@dataclass(frozen=True)
class GraspPlan:
    plan_id: str
    object_name: str
    observation_id: str
    target_base_point_m: tuple[float, float, float]
    source_frame: int
    source_timestamp_ms: float
    calibration_sha256: str
    localization_quality: str
    localization_uncertainty_m: float
    created_at: str
    steps: tuple[GraspPlanStep, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "plan_id": self.plan_id,
            "object_name": self.object_name,
            "observation_id": self.observation_id,
            "target_base_point_m": list(self.target_base_point_m),
            "units": "m",
            "source_frame": self.source_frame,
            "source_timestamp_ms": self.source_timestamp_ms,
            "calibration_sha256": self.calibration_sha256,
            "localization_quality": self.localization_quality,
            "localization_uncertainty_m": self.localization_uncertainty_m,
            "created_at": self.created_at,
            "steps": [step.to_dict() for step in self.steps],
        }


@dataclass(frozen=True)
class GraspPlanPreview:
    plan: GraspPlan
    is_safe: bool
    checks: tuple[tuple[str, bool, str], ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "plan": self.plan.to_dict(),
            "is_safe": self.is_safe,
            "checks": [
                {"stage": stage, "is_safe": safe, "reason": reason}
                for stage, safe, reason in self.checks
            ],
            "execution_enabled": False,
        }


def _command(plan_id: str, stage: str, skill_name: str, params: dict) -> Command:
    return Command(
        command_id=f"{plan_id}:{stage}",
        skill_name=skill_name,
        params=params,
        source="vision_grasp_plan",
    )


def _move_command(
    plan_id: str,
    stage: str,
    position: tuple[float, float, float],
) -> Command:
    return _command(
        plan_id,
        stage,
        "move_arm",
        {"x": position[0], "y": position[1], "z": position[2]},
    )


def validate_estimate_for_grasp(
    estimate: BasePositionEstimate,
    config: GraspPlanningConfig,
    *,
    now_timestamp_ms: float,
) -> None:
    now_ms = _finite(now_timestamp_ms, label="now_timestamp_ms")
    source_ms = _finite(
        estimate.source_timestamp_ms,
        label="source_timestamp_ms",
    )
    age_seconds = (now_ms - source_ms) / 1000.0
    if age_seconds < -1.0:
        raise GraspPlanningError("视觉帧时间戳来自未来，时钟域不可信。")
    if age_seconds > config.maximum_data_age_seconds:
        raise GraspPlanningError(
            f"视觉定位已过期：{age_seconds:.3f} s，允许上限 "
            f"{config.maximum_data_age_seconds:.3f} s。"
        )
    if estimate.localization_quality not in config.allowed_localization_qualities:
        raise GraspPlanningError(
            "定位质量不允许生成抓取计划："
            f"{estimate.localization_quality}。"
        )
    uncertainty = _finite(
        estimate.localization_uncertainty_m,
        label="localization_uncertainty_m",
    )
    if uncertainty > config.maximum_localization_uncertainty_m:
        raise GraspPlanningError(
            f"定位不确定度 {uncertainty:.6f} m 超过上限 "
            f"{config.maximum_localization_uncertainty_m:.6f} m。"
        )
    if not estimate.calibration_active:
        raise GraspPlanningError("eye-to-hand 外参未激活。")
    validation_rmse = estimate.validation_rmse_m
    validation_max = estimate.validation_max_error_m
    if validation_rmse is None or validation_max is None:
        raise GraspPlanningError("eye-to-hand 外参缺少独立验证误差。")
    if validation_rmse > estimate.activation_max_rmse_m:
        raise GraspPlanningError("eye-to-hand 外参验证 RMSE 超限。")
    if validation_max > estimate.activation_max_error_m:
        raise GraspPlanningError("eye-to-hand 外参验证最大误差超限。")


def build_grasp_plan(
    estimate: BasePositionEstimate,
    config: GraspPlanningConfig = GraspPlanningConfig(),
    *,
    now_timestamp_ms: float,
    plan_id: str | None = None,
) -> GraspPlan:
    validate_estimate_for_grasp(
        estimate,
        config,
        now_timestamp_ms=now_timestamp_ms,
    )
    base = _finite_triplet(estimate.base_point_m, label="base_point_m")
    offset = config.approach_tcp_offset_m
    target = tuple(base[index] + offset[index] for index in range(3))
    pre_grasp = (target[0], target[1], target[2] + config.pre_grasp_height_m)
    lift = (target[0], target[1], target[2] + config.lift_height_m)
    for label, start, end in (
        ("pre_grasp → approach", pre_grasp, target),
        ("approach → lift", target, lift),
    ):
        distance = math.dist(start, end)
        if distance > config.maximum_cartesian_step_m:
            raise GraspPlanningError(
                f"{label} 步长 {distance:.6f} m 超过上限 "
                f"{config.maximum_cartesian_step_m:.6f} m。"
            )
    identifier = plan_id or str(uuid4())
    steps = (
        GraspPlanStep(
            "open_gripper",
            _command(identifier, "open_gripper", "open_gripper", {}),
        ),
        GraspPlanStep(
            "pre_grasp",
            _move_command(identifier, "pre_grasp", pre_grasp),
        ),
        GraspPlanStep(
            "approach",
            _move_command(identifier, "approach", target),
        ),
        GraspPlanStep(
            "close_gripper",
            _command(identifier, "close_gripper", "close_gripper", {}),
        ),
        GraspPlanStep(
            "lift",
            _move_command(identifier, "lift", lift),
        ),
    )
    return GraspPlan(
        plan_id=identifier,
        object_name=estimate.object_name,
        observation_id=estimate.observation_id,
        target_base_point_m=target,
        source_frame=estimate.source_frame,
        source_timestamp_ms=estimate.source_timestamp_ms,
        calibration_sha256=estimate.calibration_sha256,
        localization_quality=estimate.localization_quality,
        localization_uncertainty_m=estimate.localization_uncertainty_m,
        created_at=datetime.now(timezone.utc).isoformat(),
        steps=steps,
    )


def preview_grasp_plan(
    plan: GraspPlan,
    skills: dict[str, SkillDefinition],
    *,
    position_validator: PositionValidator | None = None,
) -> GraspPlanPreview:
    checks = []
    for step in plan.steps:
        failure = preflight_command(step.command, skills)
        if failure is not None:
            checks.append((step.stage, False, failure.message))
            continue
        if step.command.skill_name == "move_arm" and position_validator is not None:
            params = step.command.params
            try:
                position_validator(params["x"], params["y"], params["z"])
            except Exception as error:
                checks.append((step.stage, False, str(error)))
                continue
        checks.append((step.stage, True, "Gateway/Safety/Workspace 预检通过"))
    return GraspPlanPreview(
        plan=plan,
        is_safe=all(safe for _stage, safe, _reason in checks),
        checks=tuple(checks),
    )


class GraspPlanExecutor:
    """Sequential executor intended for Mock acceptance before real integration."""

    def __init__(
        self,
        skills: dict[str, SkillDefinition],
        *,
        position_validator: PositionValidator | None = None,
    ) -> None:
        self._skills = skills
        self._position_validator = position_validator
        self._lock = Lock()
        self._active_stop_event: Event | None = None

    def execute(self, plan: GraspPlan, *, command_id: str) -> ExecutionResult:
        with self._lock:
            if self._active_stop_event is not None:
                return ExecutionResult(
                    command_id=command_id,
                    skill_name="grasp_object",
                    success=False,
                    message="已有抓取计划正在执行。",
                )
            stop_event = Event()
            self._active_stop_event = stop_event
        try:
            preview = preview_grasp_plan(
                plan,
                self._skills,
                position_validator=self._position_validator,
            )
            if not preview.is_safe:
                failures = "; ".join(
                    f"{stage}: {reason}"
                    for stage, safe, reason in preview.checks
                    if not safe
                )
                return ExecutionResult(
                    command_id=command_id,
                    skill_name="grasp_object",
                    success=False,
                    message=f"抓取计划预检失败：{failures}",
                )
            if stop_event.is_set():
                return self._interrupted(command_id, "第一条动作前")
            for step in plan.steps:
                if stop_event.is_set():
                    return self._interrupted(command_id, step.stage)
                result = run_command(step.command, self._skills)
                if stop_event.is_set():
                    return self._interrupted(command_id, step.stage)
                if not result.success:
                    return ExecutionResult(
                        command_id=command_id,
                        skill_name="grasp_object",
                        success=False,
                        message=(
                            f"抓取计划在 {step.stage} 失败："
                            f"{result.message}；已停止后续步骤。"
                        ),
                    )
            return ExecutionResult(
                command_id=command_id,
                skill_name="grasp_object",
                success=True,
                message=f"抓取计划 {plan.plan_id} 已完成。",
            )
        finally:
            with self._lock:
                if self._active_stop_event is stop_event:
                    self._active_stop_event = None

    def request_stop(self, command: Command) -> ExecutionResult:
        with self._lock:
            stop_event = self._active_stop_event
            if stop_event is not None:
                stop_event.set()
        return run_command(command, self._skills)

    @staticmethod
    def _interrupted(command_id: str, stage: str) -> ExecutionResult:
        return ExecutionResult(
            command_id=command_id,
            skill_name="grasp_object",
            success=False,
            message=f"抓取计划在 {stage} 被 stop 中断，未执行后续步骤。",
        )
