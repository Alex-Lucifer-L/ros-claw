###此文件定义了内置技能的注册表，并提供了一个函数来根据技能名称查找技能信息。
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.skills.base import   SkillDefinition

BUILTIN_SKILLS = {
    "move_arm": move_arm_skill,
    "open_gripper": open_gripper_skill,
    "close_gripper": close_gripper_skill,
    "stop": stop_skill
}

