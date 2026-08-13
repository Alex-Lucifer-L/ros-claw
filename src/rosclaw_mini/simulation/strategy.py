"""Versioned high-level grasp strategies used by offline prompt experiments."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import math


class GraspStrategyVersion(str, Enum):
    BASELINE_V1 = "baseline_v1"
    EFFICIENT_V2 = "efficient_v2"
    HUMANLIKE_V3 = "humanlike_v3"


@dataclass(frozen=True)
class SimulatedGraspStrategy:
    version: GraspStrategyVersion
    target_object: str
    grasp_style: str
    approach_direction: str
    preferred_gripper_yaw: float
    pre_grasp_height_m: float
    lift_height_m: float
    need_reobserve: bool
    confidence: float
    failure_reason: str | None = None

    def to_json(self) -> str:
        return json.dumps(
            {
                "target_object": self.target_object,
                "grasp_style": self.grasp_style,
                "approach_direction": self.approach_direction,
                "preferred_gripper_yaw": self.preferred_gripper_yaw,
                "pre_grasp_strategy": "vertical_clearance",
                "need_reobserve": self.need_reobserve,
                "confidence": self.confidence,
                "failure_reason": self.failure_reason,
            },
            ensure_ascii=False,
        )


class FakeGraspStrategyLLM:
    """A structured offline stand-in; it never produces joint commands."""

    def choose(
        self,
        *,
        task: str,
        target_object: str,
        version: GraspStrategyVersion,
    ) -> SimulatedGraspStrategy:
        if not isinstance(task, str) or not task.strip():
            raise ValueError("仿真抓取任务不能为空。")
        if not isinstance(target_object, str) or not target_object.strip():
            raise ValueError("仿真抓取目标不能为空。")
        if version is GraspStrategyVersion.BASELINE_V1:
            return SimulatedGraspStrategy(version, target_object, "parallel_top_down", "top_down", 0.0, 0.08, 0.08, False, 0.70)
        if version is GraspStrategyVersion.EFFICIENT_V2:
            return SimulatedGraspStrategy(version, target_object, "parallel_top_down", "top_down", 0.0, 0.06, 0.07, False, 0.80)
        return SimulatedGraspStrategy(version, target_object, "shape_aware_top_down", "top_down", 0.0, 0.07, 0.08, True, 0.86)
