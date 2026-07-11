
from rosclaw_mini.skills.base import SkillDefinition

def validate_skill_params(skill: SkillDefinition, params: dict) -> tuple[bool, str]:
    #params是command中的参数字典，skill是技能定义对象
    for param_name, param_spec in skill.params_schema.items():
        if param_spec.required and param_name not in params:
            return False, f"缺少必需参数: {param_name}"

        if param_name in params:
            param_value = params[param_name]
            if not isinstance(param_value, param_spec.accepted_types):
                return False, f"参数 {param_name} 的类型不正确，期望类型为 {param_spec.accepted_types}，但收到类型为 {type(param_value)}"

        if not skill.allow_extra_params:
            for extra_param in params.keys():
                if extra_param not in skill.params_schema:
                    return False, f"不允许额外参数: {extra_param}"
    return True, "参数验证通过"