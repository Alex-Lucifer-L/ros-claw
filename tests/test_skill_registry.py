from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.skills.arm_skills import build_arm_skills
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


def test_find_unknown_skill():
    assert find_skill("destroy_arm", skills) is None


def test_move_skill_is_disabled_without_explicit_workspace_limits():
    skills_without_limits = build_arm_skills(MockArmAdapter())

    assert skills_without_limits["move_arm"].enabled is False
    assert skills_without_limits["open_gripper"].enabled is True
