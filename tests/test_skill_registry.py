from rosclaw_mini.skills.registry import BUILTIN_SKILLS, find_skill


def test_find_existing_skills():
    assert find_skill("move_arm", BUILTIN_SKILLS).skill_name == "move_arm"
    assert find_skill("open_gripper", BUILTIN_SKILLS).skill_name == "open_gripper"


def test_find_unknown_skill():
    assert find_skill("destroy_arm", BUILTIN_SKILLS) is None
