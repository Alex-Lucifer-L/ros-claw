from rosclaw_mini.skills.registry import BUILTIN_SKILLS
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.llm.command_parser import parse_json_command
import uuid
import json

def main():
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

        execute=run_command(cmd, BUILTIN_SKILLS)
        print(execute)

if __name__ == "__main__":
    main()


