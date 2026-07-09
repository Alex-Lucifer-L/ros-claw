from rosclaw_mini.command_schema.commands import SkillInfo

BUILTIN_SKILLS = [
    SkillInfo(
        skill_name="move_arm",
        description="移动机械臂到指定的 x、y、z 位置",
        risk_level="medium",
        enabled=True,
    ),
    SkillInfo(
        skill_name="open_gripper",
        description="打开机械臂夹爪",
        risk_level="low",
        enabled=True,
    ),
    SkillInfo(
        skill_name="close_gripper",
        description="关闭机械臂夹爪",
        risk_level="low",
        enabled=True,
    ),
    SkillInfo(
        skill_name="stop",
        description="停止当前机械臂动作",
        risk_level="low",
        enabled=True,
    ),
]

def find_skillInfo_by_skill_name(skills:list[SkillInfo], skill_name:str)-> SkillInfo | None:
    for skill in skills:
        if skill.skill_name == skill_name:
            return skill
    return None