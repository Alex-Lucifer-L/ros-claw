# 这个文件定义了机械臂技能的参数规范和技能定义，包括移动机械臂、打开和关闭机械爪以及停止机械臂动作的技能。

from rosclaw_mini.skills.base import SkillDefinition,ParamSpec
from rosclaw_mini.skills.arm_handler import ArmHandlers
from rosclaw_mini.arm.base import ArmAdapter



def build_arm_skills(adapter: ArmAdapter) -> dict[str, SkillDefinition]:
    """
    构建机械臂技能的注册表。
    这个函数接收一个机械臂适配器（Adapter）实例，并返回一个包含所有机械臂技能定义的字典。
    每个技能定义包括技能名称、描述、风险等级、是否启用、参数规范以及对应的处理函数。

    """
    arm_handlers = ArmHandlers(adapter)
    
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
        handler=arm_handlers.move_arm,  # 这里可以指定一个处理函数来执行移动机械臂的操作
    )

    open_gripper_skill = SkillDefinition(
        skill_name="open_gripper",
        description="打开机械爪",
        risk_level="low",
        enabled=True,
        params_schema={},
        handler=arm_handlers.open_gripper
    )

    close_gripper_skill = SkillDefinition(
        skill_name="close_gripper",
        description="关闭机械爪",
        risk_level="low",
        enabled=True,
        params_schema={},
        handler=arm_handlers.close_gripper
    )

    stop_skill = SkillDefinition(
        skill_name="stop",
        description="停止机械臂的所有动作",
        risk_level="low",
        enabled=True,
        params_schema={},
        handler=arm_handlers.stop
    )

    return {
        "move_arm": move_arm_skill,
        "open_gripper": open_gripper_skill,
        "close_gripper": close_gripper_skill,
        "stop": stop_skill,
    }