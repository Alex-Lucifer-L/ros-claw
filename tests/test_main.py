import pytest

import rosclaw_mini.main as main_module
from rosclaw_mini.main import (
    build_parser,
    build_runtime_from_args,
    main,
    run_json_command_loop,
)
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.runtime import build_mock_runtime


class FakeRuntime:
    def __init__(self) -> None:
        self.shutdown_calls = 0
        self.current_tcp_position_m = None
        self.move_arm_disabled_reason = None

    def shutdown(self) -> None:
        self.shutdown_calls += 1


def test_runtime_from_default_arguments_builds_mock_without_devices():
    args = build_parser().parse_args([])

    runtime = build_runtime_from_args(args)

    assert isinstance(runtime.adapter, MockArmAdapter)
    assert runtime.adapter.is_connected is True

    runtime.shutdown()


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
