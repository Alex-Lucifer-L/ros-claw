#此文件定义了网关命令的运行逻辑，包括技能查找、命令安全检查和命令执行。

from rosclaw_mini.safety.checker import check_command
from rosclaw_mini.skills.registry import find_skill
from rosclaw_mini.arm.mock_arm import execute_command
from rosclaw_mini.command_schema.commands import Command,ExecutionResult
from rosclaw_mini.skills.base import SkillDefinition

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
    
    #检查命令的安全性
    check_result = check_command(command)
    if not check_result.is_safe:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=check_result.reason
        )
    


    return execute_command(command)


    

        