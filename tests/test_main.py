from threading import Event

import pytest

import rosclaw_mini.main as main_module
from rosclaw_mini.main import (
    build_llm_client_from_environment,
    build_parser,
    build_runtime_from_args,
    dispatch_command,
    main,
    run_json_command_loop,
)
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.arm.so100_plus_session import ArmSessionState
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.llm.fake_client import FakeLLMClient
from rosclaw_mini.rag.document import KnowledgeChunk, RetrievedChunk
from rosclaw_mini.runtime import ArmRuntimeShutdownError, build_mock_runtime


class FakeRuntime:
    def __init__(self) -> None:
        self.shutdown_calls = 0
        self.shutdown_error = None
        self.current_tcp_position_m = None
        self.move_arm_disabled_reason = None
        self.torque_disabled_on_shutdown = False

    def shutdown(self) -> None:
        self.shutdown_calls += 1
        if self.shutdown_error is not None:
            raise self.shutdown_error


def test_runtime_from_default_arguments_builds_mock_without_devices():
    args = build_parser().parse_args([])

    runtime = build_runtime_from_args(args)

    assert isinstance(runtime.adapter, MockArmAdapter)
    assert runtime.adapter.is_connected is True

    runtime.shutdown()


def test_parser_defaults_to_json_input_mode():
    args = build_parser().parse_args([])

    assert args.input_mode == "json"


def test_llm_environment_creates_client_with_api_key():
    received = {}
    expected_client = FakeLLMClient(response="{}")

    def client_builder(**kwargs):
        received.update(kwargs)
        return expected_client

    client = build_llm_client_from_environment(
        environ={
            "ROSCLAW_LLM_BASE_URL": " http://localhost:11434/v1 ",
            "ROSCLAW_LLM_MODEL": " qwen-test ",
            "ROSCLAW_LLM_API_KEY": " placeholder-key ",
        },
        client_builder=client_builder,
    )

    assert client is expected_client
    assert received == {
        "base_url": "http://localhost:11434/v1",
        "model": "qwen-test",
        "api_key": "placeholder-key",
    }


def test_llm_environment_allows_missing_api_key():
    received = {}

    def client_builder(**kwargs):
        received.update(kwargs)
        return FakeLLMClient(response="{}")

    build_llm_client_from_environment(
        environ={
            "ROSCLAW_LLM_BASE_URL": "http://localhost:11434/v1",
            "ROSCLAW_LLM_MODEL": "local-model",
        },
        client_builder=client_builder,
    )

    assert received["api_key"] is None


def test_runtime_from_real_arguments_builds_robot_config_without_devices(
    monkeypatch,
):
    args = build_parser().parse_args(
        [
            "--backend",
            "so100_plus",
            "--port",
            "/path-not-accessed/robot",
            "--calibration-dir",
            "/path-not-accessed/calibration",
            "--follower-name",
            "right",
            "--acknowledge-so100-plus-risk",
        ]
    )
    runtime = FakeRuntime()
    received_config = None
    received_risk = None

    def fake_build_so100_plus_runtime(config, *, risk_acknowledged):
        nonlocal received_config, received_risk
        received_config = config
        received_risk = risk_acknowledged
        return runtime

    monkeypatch.setattr(
        main_module,
        "build_so100_plus_runtime",
        fake_build_so100_plus_runtime,
    )

    result = build_runtime_from_args(args)

    assert result is runtime
    assert str(received_config.port) == "/path-not-accessed/robot"
    assert str(received_config.calibration_dir) == (
        "/path-not-accessed/calibration"
    )
    assert received_config.follower_name == "right"
    assert received_risk is True


def test_main_defaults_to_mock_and_shuts_down_on_exit():
    runtime = FakeRuntime()
    received_backend = None
    outputs: list[str] = []

    def runtime_builder(args):
        nonlocal received_backend
        received_backend = args.backend
        return runtime

    exit_code = main(
        [],
        input_func=lambda _prompt: "exit",
        output_func=outputs.append,
        runtime_builder=runtime_builder,
    )

    assert exit_code == 0
    assert received_backend == "mock"
    assert runtime.shutdown_calls == 1
    assert "当前后端: mock" in outputs


def test_main_default_json_mode_does_not_create_llm_client():
    runtime = FakeRuntime()

    def forbidden_client_builder(**_kwargs):
        raise AssertionError("默认 JSON 模式不应创建 LLM 客户端")

    exit_code = main(
        [],
        input_func=lambda _prompt: "exit",
        output_func=lambda _message: None,
        runtime_builder=lambda _args: runtime,
        llm_client_builder=forbidden_client_builder,
        environ={},
    )

    assert exit_code == 0
    assert runtime.shutdown_calls == 1


def test_main_llm_mode_rejects_missing_configuration_before_runtime():
    runtime_builder_called = False
    outputs: list[str] = []

    def forbidden_runtime_builder(_args):
        nonlocal runtime_builder_called
        runtime_builder_called = True
        raise AssertionError("LLM 配置无效时不应创建 Runtime")

    exit_code = main(
        ["--input-mode", "llm"],
        output_func=outputs.append,
        runtime_builder=forbidden_runtime_builder,
        environ={},
    )

    assert exit_code != 0
    assert runtime_builder_called is False
    assert any("ROSCLAW_LLM_BASE_URL" in message for message in outputs)
    assert any("ROSCLAW_LLM_MODEL" in message for message in outputs)


def test_main_llm_mode_runs_natural_language_through_mock_runtime():
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    client = FakeLLMClient(
        response='{"skill_name": "open_gripper", "params": {}}'
    )
    outputs: list[str] = []
    input_count = 0

    def input_command(_prompt):
        nonlocal input_count
        input_count += 1
        if input_count == 1:
            return "请打开夹爪"
        if input_count == 2:
            assert runtime.controller.wait(timeout=1.0) is not None
            return "result"
        return "exit"

    exit_code = main(
        ["--input-mode", "llm"],
        input_func=input_command,
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
        llm_client_builder=lambda **_kwargs: client,
        environ={
            "ROSCLAW_LLM_BASE_URL": "http://localhost:11434/v1",
            "ROSCLAW_LLM_MODEL": "local-model",
        },
    )

    assert exit_code == 0
    assert runtime.adapter.gripper_is_open is True
    assert any("open_gripper" in message for message in outputs)


def test_main_real_backend_mode_does_not_add_per_command_confirmation():
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    client = FakeLLMClient(
        response=(
            '{"skill_name":"move_arm","params":'
            '{"x":0.2,"y":0.1,"z":0.3}}'
        )
    )
    prompts: list[str] = []
    inputs = iter(
        (
            "移动到 x=0.2, y=0.1, z=0.3 米",
            "result",
            "exit",
        )
    )

    def fake_input(prompt):
        prompts.append(prompt)
        if len(prompts) == 2:
            assert runtime.controller.wait(timeout=1.0) is not None
        return next(inputs)

    exit_code = main(
        [
            "--input-mode",
            "llm",
            "--disable-rag",
            "--backend",
            "so100_plus",
            "--acknowledge-so100-plus-risk",
        ],
        input_func=fake_input,
        output_func=lambda _message: None,
        runtime_builder=lambda _args: runtime,
        llm_client_builder=lambda **_kwargs: client,
        environ={
            "ROSCLAW_LLM_BASE_URL": "http://localhost:11434/v1",
            "ROSCLAW_LLM_MODEL": "local-model",
        },
    )

    assert exit_code == 0
    assert runtime.adapter.position == (0.2, 0.1, 0.3)
    assert not any(prompt.startswith("确认执行") for prompt in prompts)


def test_main_llm_mode_initializes_rag_once_and_uses_its_context():
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    prompts: list[str] = []
    builder_calls: list[dict] = []

    class RecordingClient:
        def generate(self, prompt: str) -> str:
            prompts.append(prompt)
            return '{"skill_name":"open_gripper","params":{}}'

    class FakeContextProvider:
        def retrieve(self, _query: str):
            return [
                RetrievedChunk(
                    chunk=KnowledgeChunk(
                        chunk_id="gripper-0",
                        document_id="gripper",
                        chunk_index=0,
                        content="open_gripper 无参数。",
                        metadata={"section": "Open"},
                    ),
                    score=1.0,
                )
            ]

    provider = FakeContextProvider()

    def rag_builder(**kwargs):
        builder_calls.append(kwargs)
        return provider

    input_count = 0

    def input_command(_prompt):
        nonlocal input_count
        input_count += 1
        if input_count == 1:
            return "请打开夹爪"
        assert runtime.controller.wait(timeout=1.0) is not None
        return "exit"

    exit_code = main(
        ["--input-mode", "llm", "--rag-top-k", "2"],
        input_func=input_command,
        output_func=lambda _message: None,
        runtime_builder=lambda _args: runtime,
        llm_client_builder=lambda **_kwargs: RecordingClient(),
        rag_context_provider_builder=rag_builder,
        environ={
            "ROSCLAW_LLM_BASE_URL": "http://localhost:11434/v1",
            "ROSCLAW_LLM_MODEL": "local-model",
        },
    )

    assert exit_code == 0
    assert len(builder_calls) == 1
    assert builder_calls[0]["top_k"] == 2
    assert len(prompts) == 1
    assert "[SOURCE: gripper#Open]" in prompts[0]
    assert "backend=mock; session_state=NOT_APPLICABLE" in prompts[0]


def test_main_rag_initialization_failure_falls_back_to_base_prompt():
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    outputs: list[str] = []
    prompts: list[str] = []

    class RecordingClient:
        def generate(self, prompt: str) -> str:
            prompts.append(prompt)
            return '{"skill_name":"open_gripper","params":{}}'

    def broken_rag_builder(**_kwargs):
        raise ValueError("broken metadata")

    input_count = 0

    def input_command(_prompt):
        nonlocal input_count
        input_count += 1
        if input_count == 1:
            return "请打开夹爪"
        assert runtime.controller.wait(timeout=1.0) is not None
        return "exit"

    exit_code = main(
        ["--input-mode", "llm"],
        input_func=input_command,
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
        llm_client_builder=lambda **_kwargs: RecordingClient(),
        rag_context_provider_builder=broken_rag_builder,
        environ={
            "ROSCLAW_LLM_BASE_URL": "http://localhost:11434/v1",
            "ROSCLAW_LLM_MODEL": "local-model",
        },
    )

    assert exit_code == 0
    assert any(
        "RAG 初始化失败" in message and "broken metadata" in message
        for message in outputs
    )
    assert len(prompts) == 1
    assert "PROJECT_KNOWLEDGE" not in prompts[0]


def test_main_rejects_real_backend_without_risk_acknowledgement():
    runtime_builder_called = False

    def forbidden_runtime_builder(args):
        nonlocal runtime_builder_called
        runtime_builder_called = True
        raise AssertionError("未确认风险时不应装配真机")

    with pytest.raises(SystemExit) as error:
        main(
            ["--backend", "so100_plus"],
            input_func=lambda _prompt: "exit",
            runtime_builder=forbidden_runtime_builder,
        )

    assert error.value.code == 2
    assert runtime_builder_called is False


def test_main_passes_explicit_real_configuration_without_accessing_devices():
    runtime = FakeRuntime()
    received = None

    def runtime_builder(args):
        nonlocal received
        received = args
        return runtime

    exit_code = main(
        [
            "--backend",
            "so100_plus",
            "--port",
            "/path-not-accessed/robot",
            "--calibration-dir",
            "/path-not-accessed/calibration",
            "--follower-name",
            "right",
            "--acknowledge-so100-plus-risk",
        ],
        input_func=lambda _prompt: "exit",
        output_func=lambda _message: None,
        runtime_builder=runtime_builder,
    )

    assert exit_code == 0
    assert received.backend == "so100_plus"
    assert str(received.port) == "/path-not-accessed/robot"
    assert received.acknowledge_so100_plus_risk is True
    assert runtime.shutdown_calls == 1


def test_main_shuts_down_after_ctrl_c():
    runtime = FakeRuntime()
    outputs: list[str] = []

    def interrupt(_prompt):
        raise KeyboardInterrupt

    exit_code = main(
        [],
        input_func=interrupt,
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
    )

    assert exit_code == 130
    assert runtime.shutdown_calls == 1
    assert any("Ctrl+C" in message for message in outputs)


def test_main_does_not_report_safe_completion_when_shutdown_fails():
    runtime = FakeRuntime()
    runtime.shutdown_error = ArmRuntimeShutdownError("后台线程超时，未断开")
    outputs: list[str] = []

    exit_code = main(
        [],
        input_func=lambda _prompt: "exit",
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
    )

    assert exit_code == 1
    assert runtime.shutdown_calls == 1
    assert any("停止或断开失败" in message for message in outputs)
    assert not any("机械臂已停止" in message for message in outputs)


def test_main_reports_startup_tcp_and_disabled_move_arm_reason():
    runtime = FakeRuntime()
    runtime.current_tcp_position_m = (0.208819920, -0.021115885, 0.002416365)
    runtime.move_arm_disabled_reason = "move_arm 已失败关闭：启动 TCP 在工作空间外。"
    outputs: list[str] = []

    exit_code = main(
        [
            "--backend",
            "so100_plus",
            "--acknowledge-so100-plus-risk",
        ],
        input_func=lambda _prompt: "exit",
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
    )

    assert exit_code == 0
    assert "启动 TCP (m): 0.208820, -0.021116, 0.002416" in outputs
    assert runtime.move_arm_disabled_reason in outputs
    assert runtime.shutdown_calls == 1


def test_main_reports_rest_session_without_treating_it_as_startup_failure():
    runtime = FakeRuntime()
    runtime.session_state = ArmSessionState.REST
    runtime.exit_pose_warning = None
    runtime.torque_disabled_on_shutdown = True
    outputs: list[str] = []

    exit_code = main(
        [
            "--backend",
            "so100_plus",
            "--acknowledge-so100-plus-risk",
        ],
        input_func=lambda _prompt: "exit",
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
    )

    assert exit_code == 0
    assert "SO-100 Plus 会话状态: REST" in outputs
    assert any("可执行 unfold_arm" in message for message in outputs)
    assert not any("启动失败" in message for message in outputs)
    assert "机械臂已停止，力矩已关闭，后端连接已断开。" in outputs


def test_main_warns_when_exit_does_not_start_from_rest():
    runtime = FakeRuntime()
    runtime.session_state = ArmSessionState.UNVERIFIED
    runtime.exit_pose_warning = (
        "退出提示：当前会话状态为 UNVERIFIED，"
        "不会自动展开或收纳，将只停止、等待并断开。"
    )
    outputs: list[str] = []

    inputs = iter(("exit", "emergency_exit"))
    exit_code = main(
        [
            "--backend",
            "so100_plus",
            "--acknowledge-so100-plus-risk",
        ],
        input_func=lambda _prompt: next(inputs),
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
    )

    assert exit_code == 0
    assert any("普通 exit 已拒绝" in message for message in outputs)
    assert any("紧急退出" in message for message in outputs)
    assert runtime.shutdown_calls == 1


def test_dispatch_submit_failure_never_reports_submitted():
    class BusyController:
        active_command_id = "already-running"

        def submit(self, _command):
            return False

    class BusyRuntime:
        controller = BusyController()

    message = dispatch_command(
        BusyRuntime(),
        Command(
            command_id="rejected",
            skill_name="move_arm",
            params={"x": 0.1, "y": 0.2, "z": 0.3},
            source="test",
        ),
    )

    assert "未提交" in message
    assert "already-running" in message
    assert "已提交" not in message


def test_work_state_blocks_normal_exit_until_explicit_emergency_exit():
    class WorkRuntime:
        session_state = ArmSessionState.WORK

    outputs: list[str] = []
    inputs = iter(("exit", "emergency_exit"))

    emergency = run_json_command_loop(
        WorkRuntime(),
        input_func=lambda _prompt: next(inputs),
        output_func=outputs.append,
    )

    assert emergency is True
    assert any("当前 SO-100 Plus 会话状态为 WORK" in message for message in outputs)
    assert any("先执行 fold_arm" in message for message in outputs)
    assert any("机械臂可能没有回到认证 REST" in message for message in outputs)


def test_main_emergency_exit_uses_runtime_emergency_shutdown():
    class EmergencyRuntime(FakeRuntime):
        session_state = ArmSessionState.WORK
        exit_pose_warning = "普通退出警告不应覆盖紧急提示"

        def __init__(self):
            super().__init__()
            self.emergency_shutdown_calls = 0

        def emergency_shutdown(self):
            self.emergency_shutdown_calls += 1

    runtime = EmergencyRuntime()
    outputs: list[str] = []

    exit_code = main(
        [
            "--backend",
            "so100_plus",
            "--acknowledge-so100-plus-risk",
        ],
        input_func=lambda _prompt: "emergency_exit",
        output_func=outputs.append,
        runtime_builder=lambda _args: runtime,
    )

    assert exit_code == 0
    assert runtime.emergency_shutdown_calls == 1
    assert runtime.shutdown_calls == 0
    assert any("机械臂可能没有回到认证 REST" in message for message in outputs)
    assert runtime.exit_pose_warning not in outputs


def test_json_loop_runs_existing_gateway_skill_chain_with_mock():
    runtime = build_mock_runtime(move_duration_seconds=0.0)
    commands = iter(
        (
            '{"skill_name": "open_gripper", "params": {}}',
            "exit",
        )
    )

    run_json_command_loop(
        runtime,
        input_func=lambda _prompt: next(commands),
        output_func=lambda _message: None,
    )
    result = runtime.controller.wait(timeout=1.0)

    assert result is not None
    assert result.success is True
    assert runtime.adapter.gripper_is_open is True

    runtime.shutdown()


def test_json_loop_stop_interrupts_running_move_through_request_stop():
    runtime = build_mock_runtime(move_duration_seconds=2.0)
    outputs: list[str] = []
    stop_commands = []
    original_request_stop = runtime.controller.request_stop
    input_count = 0

    def recording_request_stop(command):
        stop_commands.append(command)
        return original_request_stop(command)

    runtime.controller.request_stop = recording_request_stop

    def input_command(_prompt):
        nonlocal input_count
        input_count += 1
        if input_count == 1:
            return (
                '{"skill_name": "move_arm", '
                '"params": {"x": 0.5, "y": 0.4, "z": 0.3}}'
            )
        if input_count == 2:
            assert runtime.adapter.wait_until_moving(timeout=0.5) is True
            return '{"skill_name": "stop", "params": {}}'
        return "exit"

    run_json_command_loop(
        runtime,
        input_func=input_command,
        output_func=outputs.append,
    )
    move_result = runtime.controller.wait(timeout=0.5)

    assert len(stop_commands) == 1
    assert stop_commands[0].skill_name == "stop"
    assert move_result is not None
    assert move_result.success is False
    assert move_result.skill_name == "move_arm"
    assert "停止" in move_result.message
    assert runtime.adapter.position is None
    assert runtime.adapter.is_stopped is True
    assert any("停止命令执行结果" in message for message in outputs)

    runtime.shutdown()


@pytest.mark.parametrize("skill_name", ("unfold_arm", "fold_arm"))
def test_json_loop_stop_interrupts_background_session_action(skill_name):
    action_started = Event()
    stop_requested = Event()

    def runner(command):
        if command.skill_name == "stop":
            stop_requested.set()
            return ExecutionResult(
                command_id=command.command_id,
                skill_name=command.skill_name,
                success=True,
                message="已请求停止",
            )
        action_started.set()
        stop_requested.wait(timeout=1.0)
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"{skill_name} 被 stop 中断",
        )

    controller = ExecutionController(runner)

    class ControllerRuntime:
        pass

    runtime = ControllerRuntime()
    runtime.controller = controller
    request_stop_calls = []
    original_request_stop = controller.request_stop

    def recording_request_stop(command):
        request_stop_calls.append(command)
        return original_request_stop(command)

    controller.request_stop = recording_request_stop
    inputs = iter(
        (
            f'{{"skill_name": "{skill_name}", "params": {{}}}}',
            '{"skill_name": "stop", "params": {}}',
            "exit",
        )
    )

    def input_command(_prompt):
        command = next(inputs)
        if '"skill_name": "stop"' in command:
            assert action_started.wait(timeout=0.5) is True
        if command == "exit":
            assert controller.wait(timeout=1.0) is not None
        return command

    run_json_command_loop(
        runtime,
        input_func=input_command,
        output_func=lambda _message: None,
    )
    action_result = controller.wait(timeout=1.0)

    assert len(request_stop_calls) == 1
    assert request_stop_calls[0].skill_name == "stop"
    assert action_result is not None
    assert action_result.success is False
    assert action_result.skill_name == skill_name
    assert "stop 中断" in action_result.message
