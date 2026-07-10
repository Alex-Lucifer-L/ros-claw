def validate_command_data(command_data)-> bool:
    """
    验证 JSON 解析后的命令数据结构：
    1. command_data 必须是字典
    2. skill_name 必须存在且为非空字符串
    3. params 必须存在且为字典
    """     
    if not isinstance(command_data, dict):
        return False
    
    if "skill_name" not in command_data:
        return False
    
    if not isinstance(command_data.get("skill_name"), str):
        return False
    
    if command_data.get("skill_name").strip()=="":
        return False
    
    if "params" not in command_data:
        return False
    
    if not isinstance(command_data.get("params"), dict):
        return False
    
    return True
    
    