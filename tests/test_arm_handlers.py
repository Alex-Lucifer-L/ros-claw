from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_handler import ArmHandlers


TEST_WORKSPACE = WorkspaceLimits(
    x=AxisLimits(0.0, 1.0),
    y=AxisLimits(-1.0, 1.0),
    z=AxisLimits(0.0, 1.0),
)


def test_move_arm_handler_calls_adapter():
    adapter = MockArmAdapter()
    handlers = ArmHandlers(adapter)

    command = Command(
        command_id="cmd-001",
        skill_name="move_arm",
        params={
            "x": 0.5,
            "y": 0.4,
            "z": 0.3,
        },
        source="user",
    )

    result = handlers.move_arm(command)

    assert adapter.position == (0.5, 0.4, 0.3)
    assert result.success is True
    assert result.command_id == "cmd-001"
    assert result.skill_name == "move_arm"
    assert result.message == "夹爪 TCP 已移动到位置: 0.5, 0.4, 0.3"


def test_move_relative_uses_current_tcp_and_reports_absolute_target():
    adapter = MockArmAdapter()
    adapter.position = (0.35, -0.01, 0.24)
    handlers = ArmHandlers(adapter, workspace_limits=TEST_WORKSPACE)
    command = Command(
        command_id="cmd-relative",
        skill_name="move_relative",
        params={"dx": 0.0, "dy": 0.0, "dz": 0.02},
        source="user",
    )

    result = handlers.move_relative(command)

    assert adapter.position == (0.35, -0.01, 0.26)
    assert result.success is True
    assert result.skill_name == "move_relative"
    assert "dx/dy/dz=(0.0, 0.0, 0.02)" in result.message
    assert "最终位置 (0.35, -0.01, 0.26)" in result.message


def test_move_relative_reads_tcp_when_handler_actually_executes():
    adapter = MockArmAdapter()
    adapter.position = (0.1, 0.1, 0.1)
    handlers = ArmHandlers(adapter, workspace_limits=TEST_WORKSPACE)
    command = Command(
        command_id="cmd-relative-late-read",
        skill_name="move_relative",
        params={"dx": 0.1, "dy": 0.0, "dz": 0.0},
        source="user",
    )

    adapter.position = (0.4, 0.2, 0.3)
    result = handlers.move_relative(command)

    assert result.success is True
    assert adapter.position == (0.5, 0.2, 0.3)


def make_command(command_id: str, skill_name: str) -> Command:
    return Command(
        command_id=command_id,
        skill_name=skill_name,
        params={},
        source="user",
    )


def test_open_gripper_handler_calls_adapter():
    adapter = MockArmAdapter()
    handlers = ArmHandlers(adapter)

    result = handlers.open_gripper(make_command("cmd-002", "open_gripper"))

    assert adapter.gripper_is_open is True
    assert result.success is True
    assert result.message == "机械臂夹爪已打开"


def test_close_gripper_handler_calls_adapter():
    adapter = MockArmAdapter()
    handlers = ArmHandlers(adapter)

    result = handlers.close_gripper(make_command("cmd-003", "close_gripper"))

    assert adapter.gripper_is_open is False
    assert result.success is True
    assert result.message == "机械臂夹爪已关闭"


def test_stop_handler_calls_adapter():
    adapter = MockArmAdapter()
    handlers = ArmHandlers(adapter)

    result = handlers.stop(make_command("cmd-004", "stop"))

    assert adapter.is_stopped is True
    assert result.success is True
    assert result.message == "机械臂已停止所有动作"


def test_disable_torque_handler_calls_adapter():
    adapter = MockArmAdapter()
    adapter.connect()
    handlers = ArmHandlers(adapter)

    result = handlers.disable_torque(
        make_command("cmd-005", "disable_torque")
    )

    assert adapter.torque_enabled is False
    assert result.success is True
    assert result.message == "机械臂力矩已关闭"
