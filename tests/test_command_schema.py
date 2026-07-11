from rosclaw_mini.command_schema.commands import Command, ExecutionResult, SafetyResult


def test_command_models_store_values():
    command = Command("cmd-001", "stop", {}, "user")
    safety = SafetyResult("cmd-001", True, "low", "safe")
    execution = ExecutionResult("cmd-001", "stop", True, "stopped")

    assert command.skill_name == "stop"
    assert safety.is_safe is True
    assert execution.success is True
