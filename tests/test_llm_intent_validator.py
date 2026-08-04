import pytest

from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.llm.intent_validator import (
    CommandIntentValidationError,
    validate_command_intent,
)


def _relative(dx: float, dy: float, dz: float) -> Command:
    return Command(
        command_id="intent-test",
        skill_name="move_relative",
        params={"dx": dx, "dy": dy, "dz": dz},
        source="user",
    )


@pytest.mark.parametrize(
    ("user_input", "params"),
    (
        ("向右3cm", (0.0, -0.03, 0.0)),
        ("向右3cm并向下2cm", (0.0, -0.03, -0.02)),
        ("向前20毫米", (0.02, 0.0, 0.0)),
        ("向左0.5米", (0.0, 0.5, 0.0)),
        ("沿 +Z 移动10mm", (0.0, 0.0, 0.01)),
    ),
)
def test_explicit_direction_and_distance_match_generated_command(
    user_input,
    params,
):
    validate_command_intent(user_input, _relative(*params))


@pytest.mark.parametrize(
    ("user_input", "command", "expected_reason"),
    (
        ("向右3cm", _relative(0.03, 0.0, 0.0), "dy 应为 -0.03"),
        ("向下2cm", _relative(0.0, 0.0, 0.02), "dz 应为 -0.02"),
        ("向右并向下2cm", _relative(0.0, -0.03, -0.02), "每个明确方向"),
        ("向", _relative(0.01, 0.0, 0.0), "缺少可验证"),
        ("往那边", _relative(0.01, 0.0, 0.0), "缺少可验证"),
        ("移动一下", _relative(0.01, 0.0, 0.0), "缺少可验证"),
    ),
)
def test_conflicting_or_ambiguous_motion_is_rejected(
    user_input,
    command,
    expected_reason,
):
    with pytest.raises(CommandIntentValidationError) as error:
        validate_command_intent(user_input, command)

    message = str(error.value)
    assert repr(user_input) in message
    assert repr(command.params) in message
    assert expected_reason in message


def test_explicit_relative_intent_cannot_be_changed_to_absolute_move():
    command = Command(
        command_id="wrong-skill",
        skill_name="move_arm",
        params={"x": 0.0, "y": -0.03, "z": 0.0},
        source="user",
    )

    with pytest.raises(CommandIntentValidationError, match="move_relative"):
        validate_command_intent("向右3cm", command)


def test_control_command_does_not_enter_direction_validation():
    command = Command(
        command_id="stop",
        skill_name="stop",
        params={},
        source="user",
    )

    validate_command_intent("stop", command)

