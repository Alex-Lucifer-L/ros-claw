from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.skills.registry import find_skill


skills = build_arm_skills(MockArmAdapter())


def test_find_existing_skills():
    assert find_skill("move_arm", skills).skill_name == "move_arm"
    assert find_skill("open_gripper", skills).skill_name == "open_gripper"


def test_find_unknown_skill():
    assert find_skill("destroy_arm", skills) is None
