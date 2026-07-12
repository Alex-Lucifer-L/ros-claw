# 这个文件定义了机械臂技能的参数规范和技能定义，包括移动机械臂、打开和关闭机械爪以及停止机械臂动作的技能。

from rosclaw_mini.skills.base import SkillDefinition,ParamSpec

move_arm_skill = SkillDefinition(
    skill_name="move_arm",
    description="移动机械臂到指定目标位置",
    risk_level="medium",
    enabled=True,
    params_schema={
        "x": ParamSpec(
            accepted_types=(int, float),
            min_value=0,
            max_value=1,
            min_inclusive=False,
            max_inclusive=True
        ),
        "y": ParamSpec(
            accepted_types=(int, float),
            min_value=0,
            max_value=1,
            min_inclusive=False,
            max_inclusive=True
        ),
        "z": ParamSpec(
            accepted_types=(int, float),
            min_value=0,
            max_value=1,
            min_inclusive=False,
            max_inclusive=True
        ),
    },
)

open_gripper_skill = SkillDefinition(
    skill_name="open_gripper",
    description="打开机械爪",
    risk_level="low",
    enabled=True,
    params_schema={},
)

close_gripper_skill = SkillDefinition(
    skill_name="close_gripper",
    description="关闭机械爪",
    risk_level="low",
    enabled=True,
    params_schema={},
)

stop_skill = SkillDefinition(
    skill_name="stop",
    description="停止机械臂的所有动作",
    risk_level="low",
    enabled=True,
    params_schema={},
)       
