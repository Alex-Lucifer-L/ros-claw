from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.skills.base import ParamSpec, SkillDefinition


def dummy_handler(command: Command) -> ExecutionResult:
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="test handler called",
    )


def test_skill_definition_holds_parameter_schema():
    skill = SkillDefinition(
        skill_name="test_skill",
        description="test",
        risk_level="low",
        enabled=True,
        params_schema={"value": ParamSpec((int,), min_value=0, max_value=1)},
        handler=dummy_handler,
    )
    assert skill.params_schema["value"].accepted_types == (int,)
    assert skill.handler is dummy_handler
    assert skill.allow_extra_params is False
