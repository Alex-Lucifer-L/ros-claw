from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.skills.arm_handler import ArmHandlers


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
    assert result.message == "机械臂已移动到位置: 0.5, 0.4, 0.3"


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
