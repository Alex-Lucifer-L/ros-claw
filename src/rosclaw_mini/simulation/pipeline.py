"""Headless natural-language → RGB-D → grasp-plan → sim-arm experiment loop."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from uuid import uuid4

from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.runtime import ArmRuntime
from rosclaw_mini.simulation.calibration import SimulationEyeToHandCalibration
from rosclaw_mini.simulation.camera import VirtualRGBDCamera
from rosclaw_mini.simulation.perception import SimulatedColorVLM
from rosclaw_mini.simulation.strategy import (
    FakeGraspStrategyLLM,
    GraspStrategyVersion,
    SimulatedGraspStrategy,
)
from rosclaw_mini.simulation.world import SimulationWorld
from rosclaw_mini.vision.grasp_planning import (
    GraspPlan,
    GraspPlanPreview,
    GraspPlanningConfig,
    build_grasp_plan,
    preview_grasp_plan,
)
from rosclaw_mini.vision.localization import RealSenseLocalizationService


@dataclass(frozen=True)
class SimulatedGraspResult:
    success: bool
    task: str
    strategy: SimulatedGraspStrategy
    plan: GraspPlan | None
    preview: GraspPlanPreview | None
    stages: tuple[str, ...]
    reobserve_count: int
    message: str
    localization_error_m: float | None


class SimulationGraspPipeline:
    """Uses the repository's normal visual localization and Gateway checks.

    ``SimulationWorld`` ground truth is used only after a run to compute the
    localization error and physical grasp result.  The plan receives pixels,
    depth and the explicit simulation-only transform, not an object position.
    """

    def __init__(
        self,
        runtime: ArmRuntime,
        world: SimulationWorld,
        *,
        vlm: SimulatedColorVLM | None = None,
        strategy_llm: FakeGraspStrategyLLM | None = None,
    ) -> None:
        self.runtime = runtime
        self.world = world
        self._vlm = vlm or SimulatedColorVLM()
        self._strategy_llm = strategy_llm or FakeGraspStrategyLLM()
        self._calibration = SimulationEyeToHandCalibration(world.scene.camera)

    def _localize(self, task: str):
        service = RealSenseLocalizationService(
            client=self._vlm,
            camera_factory=lambda: VirtualRGBDCamera(self.world),
        )
        return service.locate(task)

    def _nearest_world_object(self, estimate_point: tuple[float, float, float]):
        # This method is evaluator-only.  It chooses which object was actually
        # grasped after image-only localization; it never replaces estimate data.
        return min(
            self.world.objects,
            key=lambda item: math.dist(item.grasp_point_m, estimate_point),
        )

    def _execute_step(self, command: Command) -> ExecutionResult:
        if command.skill_name == "stop":
            return self.runtime.controller.request_stop(command)
        submitted = self.runtime.controller.submit(command)
        if not submitted:
            active = self.runtime.controller.active_command_id
            return ExecutionResult(
                command.command_id,
                command.skill_name,
                False,
                f"ExecutionController 正忙，当前 command_id={active}。",
            )
        result = self.runtime.controller.wait(timeout=10.0)
        if result is None:
            return ExecutionResult(command.command_id, command.skill_name, False, "仿真命令等待超时。")
        return result

    def run(
        self,
        task: str,
        *,
        strategy_version: GraspStrategyVersion = GraspStrategyVersion.BASELINE_V1,
        allow_one_reobserve: bool = False,
    ) -> SimulatedGraspResult:
        stages: list[str] = ["observe"]
        reobserve_count = 0
        plan: GraspPlan | None = None
        preview: GraspPlanPreview | None = None
        try:
            localized = self._localize(task)
            base_estimate = self._calibration.transform_position_estimate(localized.position)
            evaluated_object = self._nearest_world_object(base_estimate.base_point_m)
            strategy = self._strategy_llm.choose(
                task=task,
                target_object=evaluated_object.name,
                version=strategy_version,
            )
            config = GraspPlanningConfig(
                pre_grasp_height_m=strategy.pre_grasp_height_m,
                lift_height_m=strategy.lift_height_m,
            )
            plan = build_grasp_plan(
                base_estimate,
                config,
                now_timestamp_ms=time.time() * 1000.0,
                plan_id=f"sim-{uuid4()}",
            )
            plan = replace(plan, object_name=evaluated_object.name)
            preview = preview_grasp_plan(
                plan,
                self.runtime.skills,
                position_validator=lambda x, y, z: self.world.workspace.validate_position(x, y, z),
            )
            localization_error = math.dist(
                base_estimate.base_point_m,
                evaluated_object.grasp_point_m,
            )
        except Exception as error:
            fallback = self._strategy_llm.choose(
                task=task or "invalid",
                target_object="unknown",
                version=strategy_version,
            )
            return SimulatedGraspResult(False, task, fallback, None, None, tuple(stages), 0, f"observe/localize/plan 失败：{error}", None)

        if not preview.is_safe:
            reasons = "; ".join(f"{stage}: {reason}" for stage, safe, reason in preview.checks if not safe)
            return SimulatedGraspResult(False, task, strategy, plan, preview, tuple(stages), 0, f"Safety/Gateway 预检失败：{reasons}", localization_error)

        for step in plan.steps:
            if step.stage == "approach":
                stages.append("orient")
            stages.append(step.stage)
            result = self._execute_step(step.command)
            if not result.success:
                return SimulatedGraspResult(False, task, strategy, plan, preview, tuple(stages), reobserve_count, f"{step.stage} 失败：{result.message}", localization_error)

        stages.append("verify")
        success, message = self.world.verify_lift(
            evaluated_object.name,
            minimum_lift_m=strategy.lift_height_m * 0.75,
        )
        if not success and (allow_one_reobserve or strategy.need_reobserve):
            # A closed gripper failure is not blindly repeated.  The one
            # permitted correction is a fresh image observation, recorded here.
            reobserve_count = 1
            stages.append("reobserve")
            try:
                self._localize(task)
            except Exception as error:
                message += f"；重新观察失败：{error}"
        return SimulatedGraspResult(success, task, strategy, plan, preview, tuple(stages), reobserve_count, message, localization_error)
