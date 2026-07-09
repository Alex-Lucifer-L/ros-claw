from rosclaw_mini.command_schema.commands import Command, ExecutionResult



def execute_command(command:Command)-> ExecutionResult:
    if command.skill_name == "move_arm":
        x=command.params.get("x")
        y=command.params.get("y")
        z=command.params.get("z")
        return ExecutionResult(
            command_id=command.command_id,
            skill_name="move_arm",
            success=True,
            message=f"已移动机械臂到指定位置{x},{y},{z}",
        )
    
    if command.skill_name == "open_gripper":
        return ExecutionResult(
            command_id=command.command_id,
            skill_name="open_gripper",
            success=True,
            message="已打开夹爪",
        )
    
    if command.skill_name == "close_gripper":
        return ExecutionResult(
            command_id=command.command_id,
            skill_name="close_gripper",
            success=True,
            message="已关闭夹爪",
        )
    
    if command.skill_name == "stop":
        return ExecutionResult(
            command_id=command.command_id,
            skill_name="stop",
            success=True,
            message="已停止",
        )
    
    return ExecutionResult(
        command_id=command.command_id,
        skill_name=command.skill_name,
        success=False,
        message="未知技能",
    )

