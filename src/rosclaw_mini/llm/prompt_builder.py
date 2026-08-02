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
    move_arm_enabled = bool(
        skills.get("move_arm") and skills["move_arm"].enabled
    )
    move_relative_enabled = bool(
        skills.get("move_relative") and skills["move_relative"].enabled
    )
    movement_rules: list[str] = []
    if move_arm_enabled:
        movement_rules.append(
            "- “移动到 x/y/z”是基座坐标系绝对位置，使用 move_arm；"
            "x/y/z 单位为米。"
        )
    if move_relative_enabled:
        movement_rules.extend(
            (
                "- “向上移动2厘米”“沿 +X 移动1厘米”或“在当前位置"
                "基础上移动”是相对位移，必须使用 move_relative。",
                "- move_relative 只输出 dx/dy/dz 位移量，单位为米；"
                "厘米必须除以 100 换算成米。例如向上2厘米输出 "
                "dx=0.0、dy=0.0、dz=0.02。",
                "- LLM 不需要也不得猜测当前 TCP；当前 TCP 会在命令"
                "真正执行时由机械臂反馈读取。",
                "- dx/dy/dz 沿基座坐标系：正值分别为 +X、+Y、+Z，"
                "其中 +Z 是向上；负值为相反方向。",
                "- 如果用户只说左、右、前、后等未定义方向，不要擅自"
                "猜测它对应哪个基座坐标轴。",
            )
        )
    movement_guidance = (
        "\n移动语义规则：\n" + "\n".join(movement_rules) + "\n"
        if movement_rules
        else ""
    )

    return (
        "你是一个机械臂命令转换器。\n"
        "请把用户的自然语言指令转换成一个 JSON 对象。\n"
        "只能使用下面列出的技能。\n"
        "只能输出 JSON，不要输出解释或 Markdown。\n"
        '输出格式：{"skill_name": "技能名称", "params": {}}\n\n'
        f"可用技能：\n{available_skills}\n\n"
        f"{movement_guidance}\n"
        f"用户指令：\n{user_input}"
    )
