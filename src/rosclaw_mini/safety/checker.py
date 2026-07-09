from rosclaw_mini.command_schema.commands import Command,SafetyResult

def check_command(command: Command) -> SafetyResult:
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
            reason=f"UnsafeCommand, No params: {command.params}"
        )
    
    if command.skill_name == "move_arm":
        x=command.params.get("x")
        y=command.params.get("y")
        z=command.params.get("z")
        if x==None or y==None or z==None:
            return SafetyResult(
                command_id=command.command_id,
                is_safe=False,
                risk_level="high",
                reason=f"UnsafeCommand, Invalid params: {command.params}"
            )

        
        if not isinstance(command.params.get("x"), (int,float)) or not isinstance(command.params.get("y"), (int,float)) or not isinstance(command.params.get("z"), (int,float)):    
            return SafetyResult(
                command_id=command.command_id,
                is_safe=False,
                risk_level="high",
                reason=f"UnsafeCommand, Invalid params: {command.params}"
            )

        is_safe=lambda x: x>0 and x<=1
        if not is_safe(command.params.get("x")):
            return SafetyResult(
                command_id=command.command_id,
                is_safe=False,
                risk_level="high",
                reason=f"UnsafeCommand, Invalid x: {command.params.get('x')}"
            )
        if not is_safe(command.params.get("y")):
            return SafetyResult(
                command_id=command.command_id,
                is_safe=False,
                risk_level="high",
                reason=f"UnsafeCommand, Invalid y: {command.params.get('y')}"
            )
        if not is_safe(command.params.get("z")):
            return SafetyResult(
                command_id=command.command_id,
                is_safe=False,
                risk_level="high",
                reason=f"UnsafeCommand, Invalid z: {command.params.get('z')}"
            )
        
        return SafetyResult(
            command_id=command.command_id,
            is_safe=True,
            risk_level="low",
            reason=f"SafeCommand, No risk"
        )

    if command.skill_name == "open_gripper":
        return SafetyResult(
            command_id=command.command_id,
            is_safe=True,
            risk_level="low",
            reason=f"SafeCommand, No risk"
        )
    if command.skill_name == "close_gripper":
        return SafetyResult(
            command_id=command.command_id,
            is_safe=True,
            risk_level="low",
            reason=f"SafeCommand, No risk"
        )   
    if command.skill_name == "stop":
        return SafetyResult(
            command_id=command.command_id,
            is_safe=True,
            risk_level="low",
            reason=f"SafeCommand, No risk"
        )
    
    
    return SafetyResult(
        command_id=command.command_id,
        is_safe=False,
        risk_level="high",
        reason=f"UnsafeCommand, Unknown skill: {command.skill_name}"
    )
         

    
