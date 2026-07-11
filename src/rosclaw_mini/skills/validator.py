# 此文件定义了技能参数验证的逻辑，用于检查命令中的参数是否符合技能定义的要求。
from rosclaw_mini.skills.base import SkillDefinition


def validate_skill_params(skill: SkillDefinition, params: dict) -> tuple[bool, str]:
    # params 是command中的参数字典，skill是技能定义对象
    if not isinstance(params, dict):
        return False, "params 必须是字典"

    for param_name, param_spec in skill.params_schema.items():
        if param_spec.required and param_name not in params:
            return False, f"缺少必需参数: {param_name}"

        if param_name in params:
            param_value = params[param_name]
            if type(param_value) not in param_spec.accepted_types:
                return False, f"参数 {param_name} 的类型不正确，期望类型为 {param_spec.accepted_types}，但收到类型为 {type(param_value)}"

    if skill.params_schema=={} and params:
        return False, "此命令不允许任何参数，但提供了参数"

    if not skill.allow_extra_params:
        for extra_param in params.keys():
            if extra_param not in skill.params_schema:
                return False, f"不允许额外参数: {extra_param}"
    return True, "参数验证通过"