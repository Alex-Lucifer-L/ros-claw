from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.arm.mock_arm import execute_command


def show_result(title, result):
    print(f"\n=== {title} ===")
    print(result)


# 1. 测试 move_arm：应该执行成功
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

result_1 = execute_command(cmd_1)
show_result("move_arm 执行测试", result_1)


# 2. 测试 open_gripper：应该执行成功
cmd_2 = Command(
    command_id="cmd-002",
    skill_name="open_gripper",
    params={},
    source="user"
)

result_2 = execute_command(cmd_2)
show_result("open_gripper 执行测试", result_2)


# 3. 测试未知技能：应该执行失败
cmd_3 = Command(
    command_id="cmd-003",
    skill_name="destroy_arm",
    params={},
    source="user"
)

result_3 = execute_command(cmd_3)
show_result("未知技能执行测试", result_3)


# 简单断言
assert result_1.success is True
assert result_1.skill_name == "move_arm"

assert result_2.success is True
assert result_2.skill_name == "open_gripper"

assert result_3.success is False
assert result_3.skill_name == "destroy_arm"

print("\n所有 MockArm 测试通过")