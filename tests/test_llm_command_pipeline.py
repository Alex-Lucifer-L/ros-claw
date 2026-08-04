from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.llm.command_generator import CommandGenerator
from rosclaw_mini.llm.client import LLMClientError
from rosclaw_mini.llm.fake_client import FakeLLMClient
from rosclaw_mini.llm.prompt_builder import build_command_prompt
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.main import run_llm_command_loop
from rosclaw_mini.runtime import build_mock_runtime



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



def test_llm_command_runs_through_existing_execution_chain() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)

    client = FakeLLMClient(
        response='{"skill_name": "open_gripper", "params": {}}'
    )

    generator = CommandGenerator(
        client=client,
        skills=runtime.skills,
    )

    inputs = iter(
        (
            "请帮我打开夹爪",
            "exit",
        )
    )

    run_llm_command_loop(
        runtime,
        generator,
        input_func=lambda _prompt: next(inputs),
        output_func=lambda _message: None,
    )

    result = runtime.controller.wait(timeout=1.0)

    assert result is not None
    assert result.success is True
    assert result.skill_name == "open_gripper"
    assert runtime.adapter.gripper_is_open is True

    runtime.shutdown()


def test_prompt_distinguishes_absolute_and_relative_motion() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)

    prompt = build_command_prompt(
        user_input="向上移动2厘米",
        skills=runtime.skills,
    )

    assert "move_arm" in prompt
    assert "绝对目标" in prompt
    assert "基座系 x/y/z 目标" in prompt
    assert "move_relative" in prompt
    assert "参数：dx, dy, dz" in prompt
    assert "向上移动2厘米" in prompt
    assert "1厘米=0.01米" in prompt
    assert '"dz":0.03' in prompt
    assert "不得猜测当前 TCP" in prompt
    assert "+X=向前/伸出/远离底座" in prompt
    assert "向左=+Y，向右=-Y" in prompt
    assert "明确轴方向优先" in prompt
    assert "向右移动3cm" in prompt
    assert '"dy":-0.03' in prompt
    assert "unsupported_action" in prompt

    runtime.shutdown()


def test_prompt_maps_state_revalidation_without_implying_motion() -> None:
    skills = {
        skill_name: SkillDefinition(
            skill_name=skill_name,
            description=description,
            risk_level=risk_level,
            enabled=True,
            params_schema={},
            handler=unused_handler,
        )
        for skill_name, description, risk_level in (
            ("unfold_arm", "从 REST 展开到 WORK", "high"),
            ("fold_arm", "从 WORK 收纳到 REST", "high"),
            (
                "revalidate_state",
                "只读取真实反馈并重新认证会话状态，不产生运动",
                "low",
            ),
        )
    }

    prompt = build_command_prompt(
        user_input="请重新读取并认证当前真实状态",
        skills=skills,
    )

    assert "REST → WORK" in prompt
    assert "使用 unfold_arm" in prompt
    assert "WORK → REST" in prompt
    assert "使用 fold_arm" in prompt
    assert "UNVERIFIED 后重新认证" in prompt
    assert "使用 revalidate_state" in prompt
    assert "只读取反馈" in prompt
    assert "不会移动机械臂" in prompt
    assert "不等于“移动回 WORK”" in prompt
    assert "不得用它绕过状态门禁" in prompt
    assert "回到或恢复 WORK 状态" not in prompt
    assert "状态不明确" in prompt
    assert "unsupported_action" in prompt

    generator = CommandGenerator(
        client=FakeLLMClient(
            response=(
                '{"skill_name":"revalidate_state","params":{}}'
            )
        ),
        skills=skills,
    )
    command = generator.generate("请重新读取并认证当前真实状态")
    assert command.skill_name == "revalidate_state"
    assert command.params == {}


def test_fake_llm_generates_two_centimeter_upward_relative_command() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    generator = CommandGenerator(
        client=FakeLLMClient(
            response=(
                '{"skill_name":"move_relative",'
                '"params":{"dx":0.0,"dy":0.0,"dz":0.02}}'
            )
        ),
        skills=runtime.skills,
    )

    command = generator.generate("向上移动2厘米")

    assert command.skill_name == "move_relative"
    assert command.params == {"dx": 0.0, "dy": 0.0, "dz": 0.02}

    runtime.shutdown()


def test_generator_sends_directional_language_to_llm_with_semantic_context(
) -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)

    class RecordingClient:
        prompt: str | None = None

        def generate(self, prompt: str) -> str:
            self.prompt = prompt
            return (
                '{"skill_name":"move_relative",'
                '"params":{"dx":0.0,"dy":-0.03,"dz":0.0}}'
            )

    client = RecordingClient()
    generator = CommandGenerator(client=client, skills=runtime.skills)
    command = generator.generate("请把夹爪向右边移动3厘米")

    assert client.prompt is not None
    assert "请把夹爪向右边移动3厘米" in client.prompt
    assert "向左=+Y，向右=-Y" in client.prompt
    assert command.skill_name == "move_relative"
    assert command.params == {"dx": 0.0, "dy": -0.03, "dz": 0.0}

    runtime.shutdown()


def test_missing_distance_stays_in_existing_validator_flow() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    generator = CommandGenerator(
        client=FakeLLMClient(
            response='{"skill_name":"move_relative","params":{}}'
        ),
        skills=runtime.skills,
    )

    command = generator.generate("向右")
    result = run_command(command, runtime.skills)

    assert command.skill_name == "move_relative"
    assert result.success is False
    assert result.message == "缺少必需参数: dx"

    runtime.shutdown()


def test_unsupported_semantics_stay_in_existing_gateway_flow() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    generator = CommandGenerator(
        client=FakeLLMClient(
            response='{"skill_name":"unsupported_action","params":{}}'
        ),
        skills=runtime.skills,
    )

    command = generator.generate("向左转3度")
    result = run_command(command, runtime.skills)

    assert result.success is False
    assert result.message == "技能不存在: unsupported_action"

    runtime.shutdown()


def test_generator_accepts_explicit_stop_intent() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    generator = CommandGenerator(
        client=FakeLLMClient(
            response='{"skill_name":"stop","params":{}}'
        ),
        skills=runtime.skills,
    )

    command = generator.generate("请停止机械臂")

    assert command.skill_name == "stop"
    runtime.shutdown()


def test_llm_loop_reports_client_error_and_allows_next_input() -> None:
    runtime = build_mock_runtime(move_duration_seconds=0.0)

    class RecoveringLLMClient:
        def __init__(self) -> None:
            self.calls = 0

        def generate(self, _prompt: str) -> str:
            self.calls += 1
            if self.calls == 1:
                raise LLMClientError("测试连接失败")
            return '{"skill_name": "open_gripper", "params": {}}'

    client = RecoveringLLMClient()
    generator = CommandGenerator(client=client, skills=runtime.skills)
    outputs: list[str] = []
    input_count = 0

    def input_command(_prompt: str) -> str:
        nonlocal input_count
        input_count += 1
        if input_count <= 2:
            return "请打开夹爪"
        assert runtime.controller.wait(timeout=1.0) is not None
        return "exit"

    run_llm_command_loop(
        runtime,
        generator,
        input_func=input_command,
        output_func=outputs.append,
    )
    result = runtime.controller.wait(timeout=1.0)

    assert client.calls == 2
    assert any("LLM 调用失败: 测试连接失败" in message for message in outputs)
    assert result is not None
    assert result.success is True
    assert result.skill_name == "open_gripper"
    assert runtime.adapter.gripper_is_open is True

    runtime.shutdown()
