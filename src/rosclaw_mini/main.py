from rosclaw_mini.command_schema.commands import Command, SkillInfo
from rosclaw_mini.skills.registry import BUILTIN_SKILLS
from rosclaw_mini.gateway.command.gateway import run_command

def main():
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
    execute=run_command(cmd_1, BUILTIN_SKILLS)
    print(execute)

if __name__ == "__main__":
    main()


