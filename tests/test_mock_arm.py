from rosclaw_mini.arm.mock_arm import (
    close_gripper,
    move_arm,
    open_gripper,
    stop,
)
from rosclaw_mini.command_schema.commands import Command


def make_command(command_id: str, skill_name: str, params: dict | None = None) -> Command:
    return Command(
        command_id=command_id,
        skill_name=skill_name,
        params=params if params is not None else {},
        source="user",
    )


def test_move_arm():
    command = make_command(
        "cmd-001",
        "move_arm",
        {"x": 0.5, "y": 0.4, "z": 0.3},
    )

    result = move_arm(command)

    assert result.command_id == "cmd-001"
    assert result.skill_name == "move_arm"
    assert result.success is True
    assert result.message == "机械臂已移动到位置: {'x': 0.5, 'y': 0.4, 'z': 0.3}"


def test_open_gripper():
    result = open_gripper(make_command("cmd-002", "open_gripper"))

    assert result.command_id == "cmd-002"
    assert result.skill_name == "open_gripper"
    assert result.success is True
    assert result.message == "机械臂夹爪已打开"


def test_close_gripper():
    result = close_gripper(make_command("cmd-003", "close_gripper"))

    assert result.command_id == "cmd-003"
    assert result.skill_name == "close_gripper"
    assert result.success is True
    assert result.message == "机械臂夹爪已关闭"


def test_stop():
    result = stop(make_command("cmd-004", "stop"))

    assert result.command_id == "cmd-004"
    assert result.skill_name == "stop"
    assert result.success is True
    assert result.message == "机械臂已停止所有动作"
