from dataclasses import replace

import pytest

from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.safety.limits import (
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    AxisLimits,
    WorkspaceLimits,
)
from rosclaw_mini.skills.arm_skills import (
    build_arm_skills,
    build_so100_plus_right_follower_arm_skills,
)
from rosclaw_mini.skills.base import SkillDefinition


adapter = MockArmAdapter()
skills = build_arm_skills(
    adapter,
    workspace_limits=WorkspaceLimits(
        x=AxisLimits(0.0, 1.0),
        y=AxisLimits(-1.0, 1.0),
        z=AxisLimits(0.0, 1.0),
    ),
)


def make_command(skill_name="move_arm", params=None):
    return Command(
        command_id="cmd-001",
        skill_name=skill_name,
        params=params if params is not None else {"x": 0.5, "y": 0.4, "z": 0.3},
        source="user",
    )


def test_run_valid_command():
    result = run_command(make_command(), skills)

    assert result.success is True
    assert result.skill_name == "move_arm"
    assert adapter.position == (0.5, 0.4, 0.3)


def test_gateway_rejects_disable_torque_by_default():
    adapter.connect()

    result = run_command(
        make_command(skill_name="disable_torque", params={}),
        skills,
    )

    assert result.success is False
    assert result.message == "技能未启用: disable_torque"
    assert adapter.torque_enabled is True


def test_run_command_uses_registered_handler():
    def custom_handler(command):
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="custom handler called",
        )

    custom_skill = SkillDefinition(
        skill_name="custom_skill",
        description="test custom handler dispatch",
        risk_level="low",
        enabled=True,
        params_schema={},
        handler=custom_handler,
    )

    result = run_command(
        make_command(skill_name="custom_skill", params={}),
        {"custom_skill": custom_skill},
    )

    assert result.success is True
    assert result.message == "custom handler called"


def test_reject_unsafe_command():
    result = run_command(
        make_command(params={"x": 2.0, "y": 0.4, "z": 0.3}),
        skills,
    )
    assert result.success is False
    assert "x" in result.message


def test_right_follower_gateway_uses_formal_workspace_boundaries():
    right_adapter = MockArmAdapter()
    right_skills = build_so100_plus_right_follower_arm_skills(right_adapter)

    accepted = run_command(
        make_command(
            params={
                "x": 0.3135714232672181,
                "y": -0.041185494280163625,
                "z": 0.17932848288990053,
            }
        ),
        right_skills,
    )
    rejected = run_command(
        make_command(
            params={
                "x": 0.313,
                "y": -0.041185494280163625,
                "z": 0.17932848288990053,
            }
        ),
        right_skills,
    )

    assert accepted.success is True
    assert right_adapter.position == (
        0.3135714232672181,
        -0.041185494280163625,
        0.17932848288990053,
    )
    assert rejected.success is False
    assert "x" in rejected.message


def test_work_initial_and_transition_envelope_cannot_bypass_move_workspace():
    right_adapter = MockArmAdapter()
    right_skills = build_so100_plus_right_follower_arm_skills(right_adapter)

    work_initial_endpoint = run_command(
        make_command(
            params={
                "x": 0.3035714232672181,
                "y": -0.0011854942801636243,
                "z": 0.17932848288990053,
            }
        ),
        right_skills,
    )
    transition_envelope_only = run_command(
        make_command(
            params={
                "x": 0.20,
                "y": -0.0011854942801636243,
                "z": 0.10,
            }
        ),
        right_skills,
    )

    assert work_initial_endpoint.success is False
    assert "x" in work_initial_endpoint.message
    assert transition_envelope_only.success is False
    assert "x" in transition_envelope_only.message
    assert right_adapter.position is None


def test_move_relative_rejects_final_target_before_adapter_motion():
    class CountingMockArmAdapter(MockArmAdapter):
        def __init__(self):
            super().__init__()
            self.move_calls = 0

        def move_to(self, x, y, z):
            self.move_calls += 1
            super().move_to(x, y, z)

    workspace = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS
    relative_adapter = CountingMockArmAdapter()
    relative_adapter.position = (
        workspace.x.maximum - 0.001,
        0.0,
        0.22,
    )
    right_skills = build_so100_plus_right_follower_arm_skills(
        relative_adapter
    )

    result = run_command(
        make_command(
            "move_relative",
            {"dx": 0.01, "dy": 0.0, "dz": 0.0},
        ),
        right_skills,
    )

    assert result.success is False
    assert relative_adapter.move_calls == 0
    assert "当前 TCP=" in result.message
    assert "请求位移 dx/dy/dz=" in result.message
    assert "最终目标=" in result.message
    assert "x=" in result.message
    assert "超出允许范围" in result.message


def test_move_relative_zero_displacement_is_rejected_before_adapter_motion():
    class CountingMockArmAdapter(MockArmAdapter):
        def __init__(self):
            super().__init__()
            self.move_calls = 0

        def move_to(self, x, y, z):
            self.move_calls += 1
            super().move_to(x, y, z)

    relative_adapter = CountingMockArmAdapter()
    relative_adapter.position = (0.35, -0.01, 0.24)
    relative_skills = build_so100_plus_right_follower_arm_skills(
        relative_adapter
    )

    result = run_command(
        make_command(
            "move_relative",
            {"dx": 0.0, "dy": 0.0, "dz": 0.0},
        ),
        relative_skills,
    )

    assert result.success is False
    assert relative_adapter.move_calls == 0
    assert "dx/dy/dz 不能全部为 0" in result.message


def test_mock_consecutive_relative_moves_use_previous_completed_position():
    relative_adapter = MockArmAdapter()
    relative_skills = build_arm_skills(
        relative_adapter,
        workspace_limits=WorkspaceLimits(
            x=AxisLimits(-1.0, 1.0),
            y=AxisLimits(-1.0, 1.0),
            z=AxisLimits(-1.0, 1.0),
        ),
    )
    absolute_result = run_command(
        make_command(
            "move_arm",
            {"x": 0.35, "y": -0.01, "z": 0.24},
        ),
        relative_skills,
    )
    first = run_command(
        make_command(
            "move_relative",
            {"dx": 0.0, "dy": 0.0, "dz": 0.02},
        ),
        relative_skills,
    )
    second = run_command(
        make_command(
            "move_relative",
            {"dx": 0.01, "dy": 0.0, "dz": 0.0},
        ),
        relative_skills,
    )

    assert absolute_result.success is True
    assert first.success is True
    assert second.success is True
    assert relative_adapter.position == pytest.approx((0.36, -0.01, 0.26))


def test_unfold_and_fold_reject_all_user_supplied_path_parameters():
    class StubSession:
        def __init__(self):
            self.calls = []

        def _handle(self, command):
            self.calls.append(command.skill_name)
            return ExecutionResult(
                command_id=command.command_id,
                skill_name=command.skill_name,
                success=True,
                message="called",
            )

        move_arm = _handle
        move_relative = _handle
        open_gripper = _handle
        close_gripper = _handle
        stop = _handle
        unfold_arm = _handle
        fold_arm = _handle

    session = StubSession()
    right_skills = build_so100_plus_right_follower_arm_skills(
        MockArmAdapter(),
        session=session,
    )

    accepted_unfold = run_command(
        make_command("unfold_arm", {}),
        right_skills,
    )
    rejected_unfold = run_command(
        make_command("unfold_arm", {"storage_escape": [0, 0, 0]}),
        right_skills,
    )
    rejected_fold = run_command(
        make_command("fold_arm", {"speed": 1.0}),
        right_skills,
    )

    assert accepted_unfold.success is True
    assert rejected_unfold.success is False
    assert "不允许额外参数" in rejected_unfold.message
    assert rejected_fold.success is False
    assert "不允许额外参数" in rejected_fold.message
    assert session.calls == ["unfold_arm"]


def test_reject_unknown_skill():
    result = run_command(make_command("destroy_arm", {}), skills)
    assert result.success is False
    assert result.message == "技能不存在: destroy_arm"


def test_reject_disabled_skill():
    disabled_skills = dict(skills)
    disabled_skills["move_arm"] = replace(skills["move_arm"], enabled=False)
    result = run_command(make_command(), disabled_skills)
    assert result.success is False
    assert result.message == "技能未启用: move_arm"


def test_run_command_handles_handler_exception():
    def failing_handler(command):
        raise RuntimeError("mock arm disconnected")

    failing_skill = SkillDefinition(
        skill_name="failing_skill",
        description="test handler exception",
        risk_level="low",
        enabled=True,
        params_schema={},
        handler=failing_handler,
    )

    result = run_command(
        make_command(
            skill_name="failing_skill",
            params={},
        ),
        {"failing_skill": failing_skill},
    )

    assert result.success is False
    assert result.skill_name == "failing_skill"
    assert result.message == "技能执行失败: mock arm disconnected"
