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