from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.skills.validator import validate_skill_params


skills = build_arm_skills(
    MockArmAdapter(),
    workspace_limits=WorkspaceLimits(
        x=AxisLimits(0.0, 1.0),
        y=AxisLimits(-1.0, 1.0),
        z=AxisLimits(0.0, 1.0),
    ),
)


def test_accept_valid_move_arm_params():
    skill = skills["move_arm"]

    is_valid, message = validate_skill_params(
        skill,
        {"x": 0.5, "y": 0.4, "z": 0.3},
    )

    assert is_valid is True
    assert message == "参数验证通过"


def test_reject_missing_required_param():
    skill = skills["move_arm"]

    is_valid, message = validate_skill_params(
        skill,
        {"x": 0.5, "y": 0.4},
    )

    assert is_valid is False
    assert message == "缺少必需参数: z"


def test_reject_wrong_param_type():
    skill = skills["move_arm"]

    is_valid, message = validate_skill_params(
        skill,
        {"x": "0.5", "y": 0.4, "z": 0.3},
    )

    assert is_valid is False
    assert "参数 x 的类型不正确" in message


def test_reject_bool_as_number():
    skill = skills["move_arm"]

    is_valid, message = validate_skill_params(
        skill,
        {"x": True, "y": 0.4, "z": 0.3},
    )

    assert is_valid is False
    assert "参数 x 的类型不正确" in message


def test_reject_extra_param():
    skill = skills["move_arm"]

    is_valid, message = validate_skill_params(
        skill,
        {
            "x": 0.5,
            "y": 0.4,
            "z": 0.3,
            "speed": 0.8,
        },
    )

    assert is_valid is False
    assert message == "不允许额外参数: speed"


def test_validate_gripper_params():
    skill = skills["open_gripper"]

    valid_result, valid_message = validate_skill_params(skill, {})

    assert valid_result is True
    assert valid_message == "参数验证通过"

    invalid_result, invalid_message = validate_skill_params(
        skill,
        {"force": 0.5},
    )

    assert invalid_result is False
    assert invalid_message == "不允许额外参数: force"
