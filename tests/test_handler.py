from rosclaw_mini.llm.command_validator import validate_command_data


def test_accept_valid_command_data():
    assert validate_command_data({"skill_name": "move_arm", "params": {"x": 0.5}}) is True


def test_reject_invalid_command_data():
    assert validate_command_data(None) is False
    assert validate_command_data({"params": {}}) is False
    assert validate_command_data({"skill_name": "", "params": {}}) is False
    assert validate_command_data({"skill_name": "stop", "params": []}) is False
