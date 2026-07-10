from rosclaw_mini.skills.registry import BUILTIN_SKILLS
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.llm.command_parser import parse_command
import uuid

def main():
    command=input("请输入指令：")
    cmd_id=str(uuid.uuid4())
    cmd_1 = parse_command(command, cmd_id)
    execute=run_command(cmd_1, BUILTIN_SKILLS)
    print(execute)

if __name__ == "__main__":
    main()


