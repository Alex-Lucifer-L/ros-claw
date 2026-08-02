from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_skills import build_arm_skills


def build_test_system(move_duration_seconds: float):
    """
    创建测试需要的 Adapter、Skills 和 Controller。
    """
    adapter = MockArmAdapter(
        move_duration_seconds=move_duration_seconds
    )

    workspace = WorkspaceLimits(
        x=AxisLimits(-1.0, 1.0),
        y=AxisLimits(-1.0, 1.0),
        z=AxisLimits(-1.0, 1.0),
    )

    skills = build_arm_skills(
        adapter,
        workspace_limits=workspace,
    )

    def execute_command(command):
        return run_command(command, skills)

    controller = ExecutionController(execute_command)

    return adapter, controller


def make_move_command(
    command_id: str,
    x: float = 0.5,
    y: float = 0.4,
    z: float = 0.3,
) -> Command:
    return Command(
        command_id=command_id,
        skill_name="move_arm",
        params={
            "x": x,
            "y": y,
            "z": z,
        },
        source="user",
    )


def make_stop_command(command_id: str) -> Command:
    return Command(
        command_id=command_id,
        skill_name="stop",
        params={},
        source="user",
    )


def make_relative_command(
    command_id: str,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.02,
) -> Command:
    return Command(
        command_id=command_id,
        skill_name="move_relative",
        params={"dx": dx, "dy": dy, "dz": dz},
        source="user",
    )


def test_move_completes_normally_without_stop():
    """
    没有发送 stop 时，运动应该正常完成。
    """
    adapter, controller = build_test_system(
        move_duration_seconds=0.05
    )

    move_command = make_move_command("move-normal")

    accepted = controller.submit(move_command)
    result = controller.wait(timeout=1.0)

    assert accepted is True
    assert result is not None
    assert result.success is True
    assert result.skill_name == "move_arm"

    assert adapter.position == (0.5, 0.4, 0.3)
    assert adapter.is_stopped is False
    assert controller.is_running() is False
    assert controller.last_result() == result


def test_stop_interrupts_running_move():
    """
    运动过程中发送 stop，运动应该提前结束并返回失败结果。
    """
    adapter, controller = build_test_system(
        move_duration_seconds=2.0
    )

    move_command = make_move_command("move-interrupted")
    stop_command = make_stop_command("stop-running")

    accepted = controller.submit(move_command)

    assert accepted is True
    assert adapter.wait_until_moving(timeout=0.5) is True

    stop_result = controller.request_stop(stop_command)
    move_result = controller.wait(timeout=0.5)

    assert stop_result.success is True
    assert stop_result.skill_name == "stop"

    # 模拟运动原本需要2秒，却在0.5秒内产生结果，
    # 说明 stop 确实提前中断了运动。
    assert move_result is not None
    assert move_result.success is False
    assert move_result.skill_name == "move_arm"
    assert "停止" in move_result.message

    # 被中断后不能把目标位置记录为已经到达。
    assert adapter.position is None
    assert adapter.is_stopped is True

    assert controller.is_running() is False
    assert controller.last_result() == move_result


def test_stop_interrupts_relative_move_through_existing_controller_path():
    adapter, controller = build_test_system(move_duration_seconds=2.0)
    adapter.position = (0.35, -0.01, 0.24)

    assert controller.submit(make_relative_command("relative-stop")) is True
    assert adapter.wait_until_moving(timeout=0.5) is True

    stop_result = controller.request_stop(make_stop_command("stop-relative"))
    move_result = controller.wait(timeout=0.5)

    assert stop_result.success is True
    assert move_result is not None
    assert move_result.success is False
    assert move_result.skill_name == "move_relative"
    assert "停止" in move_result.message
    assert adapter.position == (0.35, -0.01, 0.24)
    assert adapter.is_stopped is True


def test_next_move_succeeds_after_interrupted_move():
    """
    第一次运动被中断后，Event 应该被正确恢复，
    第二次运动应该仍然可以正常完成。
    """
    adapter, controller = build_test_system(
        move_duration_seconds=0.2
    )

    first_move = make_move_command(
        command_id="move-first",
        x=0.1,
        y=0.2,
        z=0.3,
    )

    stop_command = make_stop_command("stop-first")

    assert controller.submit(first_move) is True
    assert adapter.wait_until_moving(timeout=0.5) is True

    stop_result = controller.request_stop(stop_command)
    first_result = controller.wait(timeout=0.5)

    assert stop_result.success is True
    assert first_result is not None
    assert first_result.success is False
    assert adapter.position is None

    second_move = make_move_command(
        command_id="move-second",
        x=0.6,
        y=0.7,
        z=0.8,
    )

    assert controller.submit(second_move) is True

    second_result = controller.wait(timeout=1.0)

    assert second_result is not None
    assert second_result.success is True

    assert adapter.position == (0.6, 0.7, 0.8)
    assert adapter.is_stopped is False
    assert controller.last_result() == second_result


def test_stop_while_idle_does_not_interrupt_next_move():
    """
    机械臂空闲时发送 stop，不应该留下停止信号，
    导致下一次正常运动被错误中断。
    """
    adapter, controller = build_test_system(
        move_duration_seconds=0.05
    )

    stop_command = make_stop_command("stop-idle")

    stop_result = controller.request_stop(stop_command)

    assert stop_result.success is True
    assert controller.is_running() is False

    move_command = make_move_command("move-after-idle-stop")

    assert controller.submit(move_command) is True

    move_result = controller.wait(timeout=1.0)

    assert move_result is not None
    assert move_result.success is True

    assert adapter.position == (0.5, 0.4, 0.3)
    assert adapter.is_stopped is False
