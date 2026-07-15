from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.llm.command_parser import parse_json_command
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
import uuid
import json

def main():
    adapter = MockArmAdapter()
    # 这里只供 Mock 演示使用，不代表真实 SO-100 Plus 的安全工作空间。
    mock_workspace = WorkspaceLimits(
        x=AxisLimits(-1.0, 1.0),
        y=AxisLimits(-1.0, 1.0),
        z=AxisLimits(-1.0, 1.0),
    )
    skills = build_arm_skills(adapter, workspace_limits=mock_workspace)
    while True:
        command=input("请输入指令：")
        if command=="exit":
            break
        cmd_id=str(uuid.uuid4())
        try:
            cmd = parse_json_command(command, cmd_id)
        except json.JSONDecodeError:
            print("LLM 输入的内容不是合法 JSON")
            continue
        except ValueError:
            print("JSON 合法，但 Command 数据结构不合法")
            continue

        execute=run_command(cmd, skills)
        print(execute)

if __name__ == "__main__":
    main()

