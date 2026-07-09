from rosclaw_mini.command_schema.commands import Command, SkillInfo
from rosclaw_mini.skills.registry import BUILTIN_SKILLS
from rosclaw_mini.gateway.command.gateway import run_command


def show_result(title, result):
    print(f"\n=== {title} ===")
    print(result)


# 1. 正常 move_arm：应该成功
cmd_1 = Command(
    command_id="cmd-001",
    skill_name="move_arm",
    params={
        "x": 0.5,
        "y": 0.4,
        "z": 0.3,
    },
    source="user"
)

result_1 = run_command(cmd_1, BUILTIN_SKILLS)
show_result("正常 move_arm", result_1)


# 2. move_arm 参数越界：应该被 Safety Checker 拦截，执行失败
cmd_2 = Command(
    command_id="cmd-002",
    skill_name="move_arm",
    params={
        "x": 2.0,
        "y": 0.4,
        "z": 0.3,
    },
    source="user"
)

result_2 = run_command(cmd_2, BUILTIN_SKILLS)
show_result("move_arm 参数越界", result_2)


# 3. 技能不存在：应该被 Skill Registry 拦截，执行失败
cmd_3 = Command(
    command_id="cmd-003",
    skill_name="destroy_arm",
    params={},
    source="user"
)

result_3 = run_command(cmd_3, BUILTIN_SKILLS)
show_result("技能不存在 destroy_arm", result_3)


# 4. 技能存在但未启用：应该执行失败
disabled_skills = [
    SkillInfo(
        skill_name="move_arm",
        description="移动机械臂到指定的 x、y、z 位置",
        risk_level="medium",
        enabled=False,
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

cmd_4 = Command(
    command_id="cmd-004",
    skill_name="move_arm",
    params={
        "x": 0.5,
        "y": 0.4,
        "z": 0.3,
    },
    source="user"
)

result_4 = run_command(cmd_4, disabled_skills)
show_result("move_arm 技能未启用", result_4)


# 简单断言
assert result_1.success is True

assert result_2.success is False
assert "Invalid x" in result_2.message or "UnsafeCommand" in result_2.message

assert result_3.success is False
assert result_3.message == "技能不存在"

assert result_4.success is False
assert result_4.message == "技能未启用"

print("\n所有 Command Gateway 测试通过")