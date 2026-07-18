from dataclasses import replace

from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
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
