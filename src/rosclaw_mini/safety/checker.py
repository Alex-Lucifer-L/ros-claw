import math
from numbers import Real

from rosclaw_mini.command_schema.commands import Command, SafetyResult
from rosclaw_mini.skills.base import SkillDefinition


def _unsafe(command: Command, reason: str) -> SafetyResult:
    return SafetyResult(
        command_id=command.command_id,
        is_safe=False,
        risk_level="high",
        reason=f"UnsafeCommand, {reason}",
    )


def check_command(command: Command, skill: SkillDefinition) -> SafetyResult:
    """检查命令结构、有限数值和 Skill 中显式配置的边界。"""

    if not isinstance(command.skill_name, str) or not command.skill_name.strip():
        return _unsafe(command, f"No skill name: {command.skill_name}")
    if command.skill_name != skill.skill_name:
        return _unsafe(
            command,
            f"Skill name mismatch: {command.skill_name} != {skill.skill_name}",
        )
    if not isinstance(command.params, dict):
        return _unsafe(command, f"Invalid params: {command.params}")

    params = command.params
    if not skill.allow_extra_params:
        for param_name in params:
            if param_name not in skill.params_schema:
                return _unsafe(command, f"Unexpected parameter: {param_name}")

    for param_name, param_spec in skill.params_schema.items():
        if param_name not in params:
            if param_spec.required:
                return _unsafe(command, f"Missing parameter: {param_name}")
            continue

        param_value = params[param_name]
        if type(param_value) not in param_spec.accepted_types:
            return _unsafe(command, f"Invalid parameter type: {param_name}")
        if (
            isinstance(param_value, Real)
            and not isinstance(param_value, bool)
            and not math.isfinite(float(param_value))
        ):
            return _unsafe(command, f"Parameter {param_name} 必须是有限数值")

        if param_spec.min_value is not None:
            below_minimum = (
                param_value < param_spec.min_value
                if param_spec.min_inclusive
                else param_value <= param_spec.min_value
            )
            if below_minimum:
                operator = (
                    "less than"
                    if param_spec.min_inclusive
                    else "less than or equal to"
                )
                return _unsafe(
                    command,
                    f"Parameter {param_name} value {param_value} is {operator} "
                    f"minimum allowed value {param_spec.min_value}",
                )

        if param_spec.max_value is not None:
            above_maximum = (
                param_value > param_spec.max_value
                if param_spec.max_inclusive
                else param_value >= param_spec.max_value
            )
            if above_maximum:
                operator = (
                    "greater than"
                    if param_spec.max_inclusive
                    else "greater than or equal to"
                )
                return _unsafe(
                    command,
                    f"Parameter {param_name} value {param_value} is {operator} "
                    f"maximum allowed value {param_spec.max_value}",
                )

    return SafetyResult(
        command_id=command.command_id,
        is_safe=True,
        risk_level=skill.risk_level,
        reason="Command is safe",
    )
