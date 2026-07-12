from rosclaw_mini.command_schema.commands import Command,SafetyResult
from rosclaw_mini.skills.base import SkillDefinition

def check_command(command: Command, skill: SkillDefinition) -> SafetyResult:
    """
    检查一个机械臂命令的安全性

    """
    if command.skill_name == None or command.skill_name == "":
        return SafetyResult(
            command_id=command.command_id,
            is_safe=False,
            risk_level="high",
            reason=f"UnsafeCommand, No skill name: {command.skill_name}"
        )
    
    if not isinstance(command.params, dict):
        return SafetyResult(
            command_id=command.command_id,
            is_safe=False,
            risk_level="high",
            reason=f"UnsafeCommand, Invalid params: {command.params}"
        )
    
    params=command.params
    for param_name, param_spec in skill.params_schema.items():
        if param_name not in params:
            if param_spec.required:
                return SafetyResult(
                    command_id=command.command_id,
                    is_safe=False,
                    risk_level="high",
                    reason=f"UnsafeCommand, Missing parameter: {param_name}",
                )
            continue

        param_value = params[param_name]
        if type(param_value) not in param_spec.accepted_types:
            return SafetyResult(
                command_id=command.command_id,
                is_safe=False,
                risk_level="high",
                reason=f"UnsafeCommand, Invalid parameter type: {param_name}",
            )
        if param_name in params:
            if param_spec.min_value is not None:
                if param_spec.min_inclusive:
                    if param_value < param_spec.min_value:
                        return SafetyResult(
                            command_id=command.command_id,
                            is_safe=False,
                            risk_level="high",
                            reason=f"UnsafeCommand, Parameter {param_name} value {param_value} is less than minimum allowed value {param_spec.min_value}"
                        )
                else:
                    if param_value <= param_spec.min_value:
                        return SafetyResult(
                            command_id=command.command_id,
                            is_safe=False,
                            risk_level="high",
                            reason=f"UnsafeCommand, Parameter {param_name} value {param_value} is less than or equal to minimum allowed value {param_spec.min_value}"
                        )
            if param_spec.max_value is not None:
                if param_spec.max_inclusive:
                    if param_value > param_spec.max_value:
                        return SafetyResult(
                            command_id=command.command_id,
                            is_safe=False,
                            risk_level="high",
                            reason=f"UnsafeCommand, Parameter {param_name} value {param_value} is greater than maximum allowed value {param_spec.max_value}"
                        )
                else:
                    if param_value >= param_spec.max_value:
                        return SafetyResult(
                            command_id=command.command_id,
                            is_safe=False,
                            risk_level="high",
                            reason=f"UnsafeCommand, Parameter {param_name} value {param_value} is greater than or equal to maximum allowed value {param_spec.max_value}"
                        )


    
    return SafetyResult(
        command_id=command.command_id,
        is_safe=True,
        risk_level=skill.risk_level,
        reason="Command is safe"
    )
         

    
