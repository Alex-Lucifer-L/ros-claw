###此文件定义了内置技能的注册表，并提供了一个函数来根据技能名称查找技能信息。
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.skills.base import   SkillDefinition

BUILTIN_SKILLS = {
    "move_arm": move_arm_skill,
    "open_gripper": open_gripper_skill,
    "close_gripper": close_gripper_skill,
    "stop": stop_skill
}

def find_Skill(command: Command,BUILTIN_SKILLS: dict[str, SkillDefinition]) -> SkillDefinition | None:
    """
    根据 Command 对象中的技能名称在技能注册表中查找对应的技能定义。
    如果找到匹配的技能定义，则返回该技能定义对象；如果未找到，则返回 None。
    """
    return BUILTIN_SKILLS.get(command.skill_name)