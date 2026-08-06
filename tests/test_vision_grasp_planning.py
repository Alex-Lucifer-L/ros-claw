from __future__ import annotations

from threading import Event

import pytest

from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.vision.exceptions import GraspPlanningError
from rosclaw_mini.vision.grasp_planning import (
    GraspPlanExecutor,
    GraspPlanningConfig,
    build_grasp_plan,
    preview_grasp_plan,
)
from rosclaw_mini.vision.localization import BasePositionEstimate


def base_estimate(**overrides):
    values = {
        "observation_id": "obs-1",
        "object_name": "red box",
        "camera_point_m": (0.0, 0.0, 0.3),
        "base_point_m": (0.3, 0.2, 0.1),
        "camera_frame": "camera",
        "base_frame": "base",
        "source_frame": 7,
        "source_timestamp_ms": 1000.0,
        "localization_quality": "good",
        "localization_uncertainty_m": 0.004,
        "calibration_sha256": "a" * 64,
        "calibration_created_at": "2026-08-05T00:00:00+00:00",
        "fit_rmse_m": 0.01,
        "fit_max_error_m": 0.02,
        "validation_rmse_m": 0.02,
        "validation_max_error_m": 0.03,
        "calibration_active": True,
        "activation_max_rmse_m": 0.025,
        "activation_max_error_m": 0.04,
    }
    values.update(overrides)
    return BasePositionEstimate(**values)


def workspace():
    return WorkspaceLimits(
        x=AxisLimits(0.1, 0.6),
        y=AxisLimits(-0.3, 0.3),
        z=AxisLimits(0.05, 0.4),
    )


class RecordingMockArm(MockArmAdapter):
    def __init__(self, *, move_duration_seconds=0.0):
        super().__init__(move_duration_seconds=move_duration_seconds)
        self.events = []

    def move_to(self, x, y, z):
        self.events.append(("move", (x, y, z)))
        return super().move_to(x, y, z)

    def open_gripper(self):
        self.events.append(("gripper", "open"))
        return super().open_gripper()

    def close_gripper(self):
        self.events.append(("gripper", "close"))
        return super().close_gripper()


def plan():
    return build_grasp_plan(
        base_estimate(),
        now_timestamp_ms=1000.0,
        plan_id="plan-1",
    )


def test_builds_fixed_pregrasp_approach_close_and_lift_commands():
    result = plan()

    assert [step.stage for step in result.steps] == [
        "open_gripper",
        "pre_grasp",
        "approach",
        "close_gripper",
        "lift",
    ]
    assert result.steps[1].command.params == {"x": 0.3, "y": 0.2, "z": 0.18}
    assert result.steps[2].command.params == {"x": 0.3, "y": 0.2, "z": 0.1}
    assert result.steps[4].command.params == {"x": 0.3, "y": 0.2, "z": 0.18}
    assert all(step.command.source == "vision_grasp_plan" for step in result.steps)


@pytest.mark.parametrize(
    ("overrides", "now_ms", "message"),
    (
        ({}, 32000.0, "已过期"),
        ({"localization_uncertainty_m": 0.04}, 1000.0, "不确定度"),
        ({"calibration_active": False}, 1000.0, "未激活"),
        ({"validation_rmse_m": None}, 1000.0, "独立验证"),
    ),
)
def test_rejects_stale_uncertain_or_uncertified_perception(overrides, now_ms, message):
    with pytest.raises(GraspPlanningError, match=message):
        build_grasp_plan(
            base_estimate(**overrides),
            now_timestamp_ms=now_ms,
        )


def test_preview_runs_gateway_safety_and_exact_workspace_for_every_step():
    adapter = RecordingMockArm()
    skills = build_arm_skills(adapter, workspace_limits=workspace())
    checked_positions = []

    def exact_validator(x, y, z):
        checked_positions.append((x, y, z))
        if z == pytest.approx(0.1):
            raise ValueError("exact workspace hole")
        return x, y, z

    preview = preview_grasp_plan(
        plan(),
        skills,
        position_validator=exact_validator,
    )

    assert preview.is_safe is False
    assert len(preview.checks) == 5
    assert len(checked_positions) == 3
    assert any(
        stage == "approach" and not safe and "workspace hole" in reason
        for stage, safe, reason in preview.checks
    )
    assert adapter.events == []


def test_mock_execution_runs_fixed_steps_in_order():
    adapter = RecordingMockArm()
    adapter.connect()
    limits = workspace()
    skills = build_arm_skills(adapter, workspace_limits=limits)
    executor = GraspPlanExecutor(skills, position_validator=limits.validate_position)

    result = executor.execute(plan(), command_id="grasp-1")

    assert result.success is True
    assert adapter.events == [
        ("gripper", "open"),
        ("move", (0.3, 0.2, 0.18)),
        ("move", (0.3, 0.2, 0.1)),
        ("gripper", "close"),
        ("move", (0.3, 0.2, 0.18)),
    ]
    assert adapter.position == pytest.approx((0.3, 0.2, 0.18))
    assert adapter.gripper_is_open is False


def test_stop_interrupts_current_mock_step_and_skips_remaining_steps():
    adapter = RecordingMockArm(move_duration_seconds=2.0)
    adapter.connect()
    limits = workspace()
    skills = build_arm_skills(adapter, workspace_limits=limits)
    executor = GraspPlanExecutor(skills, position_validator=limits.validate_position)

    def runner(command):
        if command.skill_name == "stop":
            return executor.request_stop(command)
        return executor.execute(plan(), command_id=command.command_id)

    controller = ExecutionController(runner)
    command = Command("grasp-1", "grasp_object", {}, "test")
    assert controller.submit(command) is True
    assert adapter.wait_until_moving(timeout=1.0)

    stop_result = controller.request_stop(Command("stop-1", "stop", {}, "test"))
    result = controller.wait(timeout=1.0)

    assert stop_result.success is True
    assert result is not None and result.success is False
    assert "stop 中断" in result.message
    assert adapter.events == [
        ("gripper", "open"),
        ("move", (0.3, 0.2, 0.18)),
    ]


def test_preflight_failure_has_zero_mock_actions():
    adapter = RecordingMockArm()
    adapter.connect()
    limits = workspace()
    skills = build_arm_skills(adapter, workspace_limits=limits)
    executor = GraspPlanExecutor(
        skills,
        position_validator=lambda *_position: (_ for _ in ()).throw(
            ValueError("blocked target")
        ),
    )

    result = executor.execute(plan(), command_id="grasp-1")

    assert result.success is False
    assert "预检失败" in result.message
    assert adapter.events == []
