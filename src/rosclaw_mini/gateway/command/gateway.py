from rosclaw_mini.safety.checker import check_command
from rosclaw_mini.skills.registry import find_skillInfo_by_skill_name
from rosclaw_mini.arm.mock_arm import execute_command
from rosclaw_mini.command_schema.commands import Command,ExecutionResult, SkillInfo

def run_command(command:Command, skills:list[SkillInfo])-> ExecutionResult:
    """
    运行机器人运行命令
    """
    check_result = check_command(command)
    if not check_result.is_safe:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=check_result.reason
        )
    
    skill=find_skillInfo_by_skill_name(skills, command.skill_name)

    if skill is None:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能不存在: {command.skill_name}"
        )
    
    
    if not skill.enabled:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=f"技能未启用: {skill.skill_name}"
        )
    

    return execute_command(command)


    

        