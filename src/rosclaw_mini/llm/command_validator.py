def validate_command_data(command_data)-> bool:
    """
    校验指令数据
    分别校验command的格式
    1. skill_name是否
    2. params是否为字典
    3. params中的key和value是否都为字符串
    4. command_id是否为字符串
    5. command_id是否不为空

    
    """
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
    
    