from dataclasses import replace

from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.skills.registry import BUILTIN_SKILLS


def make_command(skill_name="move_arm", params=None):
    return Command(
        command_id="cmd-001",
        skill_name=skill_name,
        params=params if params is not None else {"x": 0.5, "y": 0.4, "z": 0.3},
        source="user",
    )


def test_run_valid_command():
    result = run_command(make_command(), BUILTIN_SKILLS)
    assert result.success is True
    assert result.skill_name == "move_arm"


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
    result = run_command(make_command(params={"x": 2.0, "y": 0.4, "z": 0.3}), BUILTIN_SKILLS)
    assert result.success is False
    assert "x" in result.message


def test_reject_unknown_skill():
    result = run_command(make_command("destroy_arm", {}), BUILTIN_SKILLS)
    assert result.success is False
    assert result.message == "技能不存在: destroy_arm"


def test_reject_disabled_skill():
    skills = dict(BUILTIN_SKILLS)
    skills["move_arm"] = replace(BUILTIN_SKILLS["move_arm"], enabled=False)
    result = run_command(make_command(), skills)
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
