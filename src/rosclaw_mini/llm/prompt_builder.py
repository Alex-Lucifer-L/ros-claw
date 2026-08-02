from rosclaw_mini.skills.base import SkillDefinition


def build_command_prompt(
    user_input: str,
    skills: dict[str, SkillDefinition],
) -> str:
    skill_lines: list[str] = []

    for skill in skills.values():
        if not skill.enabled:
            continue

        if skill.params_schema:
            parameter_names = ", ".join(skill.params_schema)
            parameter_description = f"参数：{parameter_names}"
        else:
            parameter_description = "无参数"

        skill_lines.append(
            f"- {skill.skill_name}: "
            f"{skill.description}；"
            f"{parameter_description}"
        )

    available_skills = "\n".join(skill_lines)

    return (
        "你是一个机械臂命令转换器。\n"
        "请把用户的自然语言指令转换成一个 JSON 对象。\n"
        "只能使用下面列出的技能。\n"
        "只能输出 JSON，不要输出解释或 Markdown。\n"
        '输出格式：{"skill_name": "技能名称", "params": {}}\n\n'
        f"可用技能：\n{available_skills}\n\n"
        f"用户指令：\n{user_input}"
    )