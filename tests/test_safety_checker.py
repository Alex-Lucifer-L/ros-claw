from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.safety.checker import check_command


def show_result(title, result):
    print(f"\n=== {title} ===")
    print(result)


# 1. move_arm 正常参数：应该安全
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

result_1 = check_command(cmd_1)
show_result("move_arm 正常参数", result_1)


# 2. move_arm 参数越界：应该不安全
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

result_2 = check_command(cmd_2)
show_result("move_arm x 越界", result_2)


# 3. move_arm 缺少 z：应该不安全
cmd_3 = Command(
    command_id="cmd-003",
    skill_name="move_arm",
    params={
        "x": 0.5,
        "y": 0.4,
    },
    source="user"
)

result_3 = check_command(cmd_3)
show_result("move_arm 缺少 z", result_3)


# 4. open_gripper 不需要参数：应该安全
cmd_4 = Command(
    command_id="cmd-004",
    skill_name="open_gripper",
    params={},
    source="user"
)

result_4 = check_command(cmd_4)
show_result("open_gripper 空参数", result_4)


# 简单断言：如果结果不符合预期，程序会报错
assert result_1.is_safe is True
assert result_2.is_safe is False
assert result_3.is_safe is False
assert result_4.is_safe is True

print("\n所有 Safety Checker 测试通过")