from rosclaw_mini.command_schema.commands import Command
import json

def parse_command(command_str: str, command_id: str) -> Command:
    command_str = command_str.strip()

    if command_str == "打开夹爪":
        return Command(
            command_id=command_id,
            skill_name="open_gripper",
            params={},
            source="user"
        )

    if command_str == "关闭夹爪":
        return Command( 
            command_id=command_id,
            skill_name="close_gripper",
            params={},
            source="user"
        )

    if command_str == "停止":
        return Command(
            command_id=command_id,
            skill_name="stop",
            params={},
            source="user"
        )

    return Command(
        command_id=command_id,
        skill_name="unknown",
        params={},
        source="user"
    )

def parse_json_command(command_json: str,command_id: str) -> Command:
    command=json.loads(command_json)
    return Command(
        command_id=command_id,
        skill_name=command["skill_name"],
        params=command["params"],
        source="user"
    )

