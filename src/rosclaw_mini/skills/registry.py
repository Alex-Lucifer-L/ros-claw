###此文件定义了内置技能的注册表，并提供了一个函数来根据技能名称查找技能信息。
from rosclaw_mini.skills.base import   SkillDefinition
from rosclaw_mini.skills.arm_skills import move_arm_skill, open_gripper_skill, close_gripper_skill, stop_skill


BUILTIN_SKILLS = {
    "move_arm": move_arm_skill,
    "open_gripper": open_gripper_skill,
    "close_gripper": close_gripper_skill,
    "stop": stop_skill
}

def find_skill(skill_name: str, skills: dict[str, SkillDefinition]) -> SkillDefinition | None:
    """
    根据技能名称在技能注册表中查找对应的技能定义。
    如果找到匹配的技能定义，则返回该技能定义对象；如果未找到，则返回 None。
    """
    return skills.get(skill_name, None)
