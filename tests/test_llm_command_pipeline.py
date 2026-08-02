from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.llm.command_generator import CommandGenerator
from rosclaw_mini.llm.fake_client import FakeLLMClient
from rosclaw_mini.llm.prompt_builder import build_command_prompt
from rosclaw_mini.skills.base import SkillDefinition


def unused_handler(command: Command) -> ExecutionResult:
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=True,
        message="测试处理器",
    )


def test_llm_command_pipeline() -> None:
    skills = {
        "open_gripper": SkillDefinition(
            skill_name="open_gripper",
            description="打开机械爪",
            risk_level="low",
            enabled=True,
            params_schema={},
            handler=unused_handler,
        ),
        "disabled_skill": SkillDefinition(
            skill_name="disabled_skill",
            description="禁用的测试技能",
            risk_level="high",
            enabled=False,
            params_schema={},
            handler=unused_handler,
        ),
    }

    prompt = build_command_prompt(
        user_input="请帮我打开夹爪",
        skills=skills,
    )

    assert "请帮我打开夹爪" in prompt
    assert "open_gripper" in prompt
    assert "disabled_skill" not in prompt

    client = FakeLLMClient(
        response='{"skill_name": "open_gripper", "params": {}}'
    )

    generator = CommandGenerator(
        client=client,
        skills=skills,
    )

    command = generator.generate("请帮我打开夹爪")

    assert command.command_id != ""
    assert command.skill_name == "open_gripper"
    assert command.params == {}
    assert command.source == "user"