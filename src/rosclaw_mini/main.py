from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.llm.command_parser import parse_json_command
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.safety.limits import AxisLimits, WorkspaceLimits
from rosclaw_mini.execution.controller import ExecutionController
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
    def execute_command(command):
        return run_command(command, skills)
    controller = ExecutionController(execute_command)
    while True:
        command=input("请输入指令：")
        if command=="exit":
            if controller.is_running():
                print("当前有命令正在执行，请等待其完成或使用 stop 命令停止它。")
                continue
            print("退出程序。")
            break
        if command=="result":
           if controller.is_running():
               print("命令正在执行中，请等待其完成。")
           else:
               result = controller.last_result()
               if result is not None:
                   print(f"上一次命令执行结果: {result}")
               else:
                   print("没有上一次命令的执行结果。")
           continue
        cmd_id=str(uuid.uuid4())
        try:
            cmd = parse_json_command(command, cmd_id)
        except json.JSONDecodeError:
            print("LLM 输入的内容不是合法 JSON")
            continue
        except ValueError:
            print("JSON 合法，但 Command 数据结构不合法")
            continue

        if cmd.skill_name == "stop":
            stop_result = controller.request_stop(cmd)
            print(f"停止命令执行结果: {stop_result}")
            continue
        accepted = controller.submit(cmd)
        if not accepted:
            print("当前有命令正在执行，请等待其完成或使用 stop 命令停止它。")
            continue
        else:
            print(f"命令 {cmd_id} 已提交，正在后台执行。")


if __name__ == "__main__":
    main()

