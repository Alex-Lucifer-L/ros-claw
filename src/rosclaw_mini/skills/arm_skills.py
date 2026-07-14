# 这个文件定义了机械臂技能的参数规范和技能定义，包括移动机械臂、打开和关闭机械爪以及停止机械臂动作的技能。

from rosclaw_mini.skills.base import SkillDefinition,ParamSpec
from rosclaw_mini.arm.mock_arm import move_arm, open_gripper, close_gripper, stop

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
    handler=move_arm,  # 这里可以指定一个处理函数来执行移动机械臂的操作
)

open_gripper_skill = SkillDefinition(
    skill_name="open_gripper",
    description="打开机械爪",
    risk_level="low",
    enabled=True,
    params_schema={},
    handler=open_gripper
)

close_gripper_skill = SkillDefinition(
    skill_name="close_gripper",
    description="关闭机械爪",
    risk_level="low",
    enabled=True,
    params_schema={},
    handler=close_gripper
)

stop_skill = SkillDefinition(
    skill_name="stop",
    description="停止机械臂的所有动作",
    risk_level="low",
    enabled=True,
    params_schema={},
    handler=stop
)
