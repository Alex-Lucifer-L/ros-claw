#此文件用于处理机器人命令的执行逻辑，包括技能查找、参数验证和安全检查。

from rosclaw_mini.safety.checker import check_command
from rosclaw_mini.skills.registry import find_skill
from rosclaw_mini.command_schema.commands import Command,ExecutionResult
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.skills.validator import validate_skill_params


def preflight_command(
    command: Command,
    skills: dict[str, SkillDefinition],
) -> ExecutionResult | None:
    """只运行现有 Skill Validator 和 Safety Checker，不执行动作。"""

    skill = find_skill(command.skill_name, skills)
    if skill is None:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能不存在: {command.skill_name}",
        )
    if not skill.enabled:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能未启用: {skill.skill_name}",
        )

    params_valid, params_message = validate_skill_params(skill, command.params)
    if not params_valid:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=params_message,
        )

    check_result = check_command(command, skill)
    if not check_result.is_safe:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=check_result.reason,
        )
    return None


def run_command(command:Command, skills: dict[str, SkillDefinition])-> ExecutionResult:
    """
    运行机器人运行命令
    """

    preflight_failure = preflight_command(command, skills)
    if preflight_failure is not None:
        return preflight_failure

    # preflight 已确认 Skill 存在且启用。
    skill = skills[command.skill_name]
    
    try:
        return skill.handler(command)
    except Exception as error:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能执行失败: {str(error)}"
        )

    

