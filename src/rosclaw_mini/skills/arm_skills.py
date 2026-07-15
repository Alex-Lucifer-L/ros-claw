# 这个文件定义了机械臂技能的参数规范和技能定义，包括移动机械臂、打开和关闭机械爪以及停止机械臂动作的技能。

from rosclaw_mini.skills.base import ParamSpec, SkillDefinition
from rosclaw_mini.skills.arm_handler import ArmHandlers
from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits


def _position_param_spec(axis: AxisLimits | None) -> ParamSpec:
    return ParamSpec(
        accepted_types=(int, float),
        min_value=axis.minimum if axis is not None else None,
        max_value=axis.maximum if axis is not None else None,
        min_inclusive=True,
        max_inclusive=True,
    )


def build_arm_skills(
    adapter: ArmAdapter,
    workspace_limits: WorkspaceLimits | None = None,
) -> dict[str, SkillDefinition]:
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
        # 未提供经过确认的工作空间时失败关闭，避免把示例范围用于实机。
        enabled=workspace_limits is not None,
        params_schema={
            "x": _position_param_spec(
                workspace_limits.x if workspace_limits is not None else None
            ),
            "y": _position_param_spec(
                workspace_limits.y if workspace_limits is not None else None
            ),
            "z": _position_param_spec(
                workspace_limits.z if workspace_limits is not None else None
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
