from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.safety.limits import (
    AxisLimits,
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    WorkspaceLimits,
)
from rosclaw_mini.skills.arm_skills import (
    build_arm_skills,
    build_so100_plus_right_follower_arm_skills,
)
from rosclaw_mini.skills.registry import find_skill


skills = build_arm_skills(
    MockArmAdapter(),
    workspace_limits=WorkspaceLimits(
        x=AxisLimits(0.0, 1.0),
        y=AxisLimits(-1.0, 1.0),
        z=AxisLimits(0.0, 1.0),
    ),
)


def test_find_existing_skills():
    assert find_skill("move_arm", skills).skill_name == "move_arm"
    assert find_skill("open_gripper", skills).skill_name == "open_gripper"
    assert find_skill("disable_torque", skills).skill_name == "disable_torque"


def test_find_unknown_skill():
    assert find_skill("destroy_arm", skills) is None


def test_move_skill_is_disabled_without_explicit_workspace_limits():
    skills_without_limits = build_arm_skills(MockArmAdapter())

    assert skills_without_limits["move_arm"].enabled is False
    assert skills_without_limits["open_gripper"].enabled is True
    assert skills_without_limits["disable_torque"].enabled is False
    assert skills_without_limits["disable_torque"].risk_level == "high"


def test_right_follower_skills_use_registered_formal_workspace():
    right_skills = build_so100_plus_right_follower_arm_skills(
        MockArmAdapter()
    )
    move = right_skills["move_arm"]
    workspace = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS

    assert move.enabled is True
    assert move.params_schema["x"].min_value == workspace.x.minimum
    assert move.params_schema["x"].max_value == workspace.x.maximum
    assert move.params_schema["y"].min_value == workspace.y.minimum
    assert move.params_schema["y"].max_value == workspace.y.maximum
    assert move.params_schema["z"].min_value == workspace.z.minimum
    assert move.params_schema["z"].max_value == workspace.z.maximum
