# 这个文件定义了机械臂技能的参数规范和技能定义，包括移动机械臂、打开和关闭机械爪以及停止机械臂动作的技能。

from dataclasses import replace

from rosclaw_mini.skills.base import ParamSpec, SkillDefinition
from rosclaw_mini.skills.arm_handler import ArmHandlers
from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.so100_plus_session import SO100PlusArmSession
from rosclaw_mini.safety.limits import (
    AxisLimits,
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    WorkspaceLimits,
)


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
        description="移动夹爪工具中心点到指定绝对位置",
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

    disable_torque_skill = SkillDefinition(
        skill_name="disable_torque",
        description="关闭机械臂全部关节力矩，使机械臂变软",
        risk_level="high",
        enabled=False,
        params_schema={},
        handler=arm_handlers.disable_torque,
    )

    return {
        "move_arm": move_arm_skill,
        "open_gripper": open_gripper_skill,
        "close_gripper": close_gripper_skill,
        "stop": stop_skill,
        "disable_torque": disable_torque_skill,
    }


def build_so100_plus_right_follower_arm_skills(
    adapter: ArmAdapter,
    *,
    session: SO100PlusArmSession | None = None,
) -> dict[str, SkillDefinition]:
    """用已登记的 right_follower 工作空间构建真机 Skill。"""

    skills = build_arm_skills(
        adapter,
        workspace_limits=SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    )
    if session is None:
        return skills
    return bind_so100_plus_arm_session(skills, session)


def bind_so100_plus_arm_session(
    skills: dict[str, SkillDefinition],
    session: SO100PlusArmSession,
) -> dict[str, SkillDefinition]:
    """把动态会话门禁绑定到现有 right_follower Skill 注册表。"""

    required = ("move_arm", "open_gripper", "close_gripper", "stop")
    missing = tuple(name for name in required if name not in skills)
    if missing:
        raise RuntimeError(
            "right_follower Skill 注册表缺少："
            + ", ".join(missing)
            + "。"
        )
    skills = dict(skills)
    skills["move_arm"] = replace(
        skills["move_arm"],
        handler=session.move_arm,
    )
    skills["open_gripper"] = replace(
        skills["open_gripper"],
        handler=session.open_gripper,
    )
    skills["close_gripper"] = replace(
        skills["close_gripper"],
        handler=session.close_gripper,
    )
    skills["stop"] = replace(
        skills["stop"],
        handler=session.stop,
    )
    skills["unfold_arm"] = SkillDefinition(
        skill_name="unfold_arm",
        description=(
            "沿已认证的 follower_rest → storage_escape → "
            "JoyCon 工作初始姿态路径展开机械臂"
        ),
        risk_level="high",
        enabled=True,
        params_schema={},
        handler=session.unfold_arm,
    )
    skills["fold_arm"] = SkillDefinition(
        skill_name="fold_arm",
        description=(
            "先返回 JoyCon 工作初始姿态，再沿已认证反向路径"
            "收纳到 follower_rest"
        ),
        risk_level="high",
        enabled=True,
        params_schema={},
        handler=session.fold_arm,
    )
    return skills
