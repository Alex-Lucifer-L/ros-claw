import json

from rosclaw_mini.llm.command_parser import parse_json_command


def test_parse_json_command():
    command = parse_json_command('{"skill_name": "open_gripper", "params": {}}', "cmd-001")
    assert command.command_id == "cmd-001"
    assert command.skill_name == "open_gripper"
    assert command.params == {}


def test_parse_json_command_rejects_invalid_json():
    try:
        parse_json_command("not json", "cmd-002")
    except json.JSONDecodeError:
        pass
    else:
        raise AssertionError("invalid JSON should raise JSONDecodeError")


def test_parse_json_command_rejects_invalid_shape():
    try:
        parse_json_command('{"skill_name": "stop"}', "cmd-003")
    except ValueError:
        pass
    else:
        raise AssertionError("invalid command shape should raise ValueError")
