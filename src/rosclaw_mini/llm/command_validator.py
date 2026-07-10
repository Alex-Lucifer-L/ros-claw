def validate_command_data(command_data)-> bool:
    if not isinstance(command_data, dict):
        return False
    
    if "skill_name" not in command_data:
        return False
    
    if not isinstance(command_data.get("skill_name"), str):
        return False
    
    if command_data.get("skill_name")=="":
        return False
    
    if "params" not in command_data:
        return False
    
    if not isinstance(command_data.get("params"), dict):
        return False
    
    return True
    
    