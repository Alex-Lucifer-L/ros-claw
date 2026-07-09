from rosclaw_mini.skills.registry import (
    BUILTIN_SKILLS,
    find_skillInfo_by_skill_name,
)


def show_result(title, result):
    print(f"\n=== {title} ===")
    print(result)


# 1. 查找存在的技能：move_arm
skill_1 = find_skillInfo_by_skill_name(BUILTIN_SKILLS, "move_arm")
show_result("查找 move_arm", skill_1)


# 2. 查找存在的技能：open_gripper
skill_2 = find_skillInfo_by_skill_name(BUILTIN_SKILLS, "open_gripper")
show_result("查找 open_gripper", skill_2)


# 3. 查找不存在的技能：destroy_arm
skill_3 = find_skillInfo_by_skill_name(BUILTIN_SKILLS, "destroy_arm")
show_result("查找 destroy_arm", skill_3)


# 简单断言
assert skill_1 is not None
assert skill_1.skill_name == "move_arm"

assert skill_2 is not None
assert skill_2.skill_name == "open_gripper"

assert skill_3 is None

print("\n所有 Skill Registry 测试通过")