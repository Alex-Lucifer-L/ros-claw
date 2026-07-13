#此文件用于处理机器人命令的执行逻辑，包括技能查找、参数验证和安全检查。

from rosclaw_mini.safety.checker import check_command
from rosclaw_mini.skills.registry import find_skill
from rosclaw_mini.command_schema.commands import Command,ExecutionResult
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.skills.validator import validate_skill_params

def run_command(command:Command, skills: dict[str, SkillDefinition])-> ExecutionResult:
    """
    运行机器人运行命令
    """

    skill = find_skill(command.skill_name, skills)  
    #如果技能不存在，则返回错误结果
    if skill is None:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能不存在: {command.skill_name}"
        )
    
    #判断技能是否启用
    
    if not skill.enabled:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能未启用: {skill.skill_name}"
        )
    
    #验证命令参数是否符合技能定义的要求
    params_valid, params_message = validate_skill_params(skill, command.params)
    if not params_valid:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=params_message
        )

    #检查命令的安全性
    check_result = check_command(command, skill)
    if not check_result.is_safe:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=check_result.reason
        )
    

    return skill.handler(command)

    

        