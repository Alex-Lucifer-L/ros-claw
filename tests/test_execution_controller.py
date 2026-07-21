from threading import Event

from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.execution.controller import ExecutionController


def make_command(
    skill_name: str = "move_arm",
    command_id: str = "cmd-001",
) -> Command:
    return Command(
        command_id=command_id,
        skill_name=skill_name,
        params={"x": 0.5, "y": 0.4, "z": 0.3}
        if skill_name == "move_arm"
        else {},
        source="user",
    )


def test_submit_runs_command_in_background():
    motion_started = Event()
    allow_motion_finish = Event()

    def fake_runner(command: Command) -> ExecutionResult:
        motion_started.set()
        allow_motion_finish.wait(timeout=1.0)

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="动作完成",
        )

    controller = ExecutionController(fake_runner)

    accepted = controller.submit(make_command())

    assert accepted is True
    assert motion_started.wait(timeout=1.0) is True
    assert controller.is_running() is True

    allow_motion_finish.set()
    result = controller.wait(timeout=1.0)

    assert result is not None
    assert result.success is True
    assert result.message == "动作完成"
    assert controller.is_running() is False
    assert controller.last_result() == result

def test_submit_rejects_second_command_while_running():
    motion_started = Event()
    allow_motion_finish = Event()

    def fake_runner(command: Command) -> ExecutionResult:
        motion_started.set()
        allow_motion_finish.wait(timeout=1.0)

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="动作完成",
        )

    controller = ExecutionController(fake_runner)

    first_accepted = controller.submit(
        make_command(command_id="cmd-001")
    )

    assert motion_started.wait(timeout=1.0) is True

    second_accepted = controller.submit(
        make_command(command_id="cmd-002")
    )

    assert first_accepted is True
    assert second_accepted is False

    allow_motion_finish.set()
    controller.wait(timeout=1.0)


def test_request_stop_runs_while_background_command_is_active():
    motion_started = Event()
    allow_motion_finish = Event()
    stop_called = Event()

    def fake_runner(command: Command) -> ExecutionResult:
        if command.skill_name == "stop":
            stop_called.set()
            allow_motion_finish.set()

            return ExecutionResult(
                command_id=command.command_id,
                skill_name=command.skill_name,
                success=True,
                message="停止成功",
            )

        motion_started.set()
        allow_motion_finish.wait(timeout=1.0)

        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message="后台动作结束",
        )

    controller = ExecutionController(fake_runner)

    controller.submit(make_command())

    assert motion_started.wait(timeout=1.0) is True
    assert controller.is_running() is True

    stop_result = controller.request_stop(
        make_command(
            skill_name="stop",
            command_id="cmd-stop",
        )
    )

    assert stop_result.success is True
    assert stop_called.is_set() is True

    background_result = controller.wait(timeout=1.0)

    assert background_result is not None
    assert controller.is_running() is False