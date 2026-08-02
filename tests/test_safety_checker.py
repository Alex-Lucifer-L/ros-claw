import math

import pytest

from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.safety.checker import check_command
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.skills.base import ParamSpec, SkillDefinition


TEST_WORKSPACE = WorkspaceLimits(
    x=AxisLimits(0.0, 1.0),
    y=AxisLimits(-1.0, 1.0),
    z=AxisLimits(0.0, 1.0),
)
skills = build_arm_skills(MockArmAdapter(), workspace_limits=TEST_WORKSPACE)


def make_command(skill_name: str, params: dict) -> Command:
    return Command(
        command_id="cmd-test-001",
        skill_name=skill_name,
        params=params,
        source="user",
    )


def test_accept_valid_move_arm_command():
    command = make_command(
        "move_arm",
        {"x": 0.5, "y": 0.4, "z": 0.3},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is True
    assert result.risk_level == "medium"


def test_accept_value_equal_to_configured_minimum():
    command = make_command(
        "move_arm",
        {"x": 0, "y": 0.4, "z": 0.3},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is True


def test_reject_value_below_minimum():
    command = make_command(
        "move_arm",
        {"x": -0.1, "y": 0.4, "z": 0.3},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is False
    assert "x" in result.reason


def test_accept_negative_y_when_workspace_allows_it():
    command = make_command(
        "move_arm",
        {"x": 0.5, "y": -0.4, "z": 0.3},
    )

    result = check_command(command, skills["move_arm"])

    assert result.is_safe is True


def test_accept_value_equal_to_inclusive_maximum():
    command = make_command(
        "move_arm",
        {"x": 1, "y": 0.4, "z": 0.3},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is True


def test_reject_value_above_maximum():
    command = make_command(
        "move_arm",
        {"x": 1.1, "y": 0.4, "z": 0.3},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is False
    assert "x" in result.reason


def test_reject_missing_required_parameter():
    command = make_command(
        "move_arm",
        {"x": 0.5, "y": 0.4},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is False
    assert "z" in result.reason


def test_reject_wrong_parameter_type():
    command = make_command(
        "move_arm",
        {"x": "0.5", "y": 0.4, "z": 0.3},
    )
    skill = skills["move_arm"]

    result = check_command(command, skill)

    assert result.is_safe is False
    assert "x" in result.reason


def test_reject_non_finite_numeric_parameter():
    command = make_command(
        "move_arm",
        {"x": math.nan, "y": 0.4, "z": 0.3},
    )

    result = check_command(command, skills["move_arm"])

    assert result.is_safe is False
    assert "有限数值" in result.reason


@pytest.mark.parametrize(
    "invalid_value",
    (math.nan, math.inf, -math.inf),
)
def test_move_relative_rejects_non_finite_displacement(invalid_value):
    command = make_command(
        "move_relative",
        {"dx": 0.0, "dy": invalid_value, "dz": 0.02},
    )

    result = check_command(command, skills["move_relative"])

    assert result.is_safe is False
    assert "dy" in result.reason
    assert "有限数值" in result.reason


def test_accept_skill_without_parameters():
    command = make_command("open_gripper", {})
    skill = skills["open_gripper"]

    result = check_command(command, skill)

    assert result.is_safe is True
    assert result.risk_level == "low"


def test_generic_checker_supports_different_boundaries():
    skill = SkillDefinition(
        skill_name="test_skill",
        description="测试不同的开闭区间",
        risk_level="medium",
        enabled=True,
        params_schema={
            "value": ParamSpec(
                accepted_types=(int, float),
                min_value=0,
                max_value=10,
                min_inclusive=True,
                max_inclusive=False,
            )
        },
        handler=skills["stop"].handler,
    )

    minimum_command = make_command(
        "test_skill",
        {"value": 0},
    )
    maximum_command = make_command(
        "test_skill",
        {"value": 10},
    )

    minimum_result = check_command(minimum_command, skill)
    maximum_result = check_command(maximum_command, skill)

    assert minimum_result.is_safe is True
    assert maximum_result.is_safe is False
    assert "value" in maximum_result.reason
