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
        movement_rules.extend(
            (
                "- 绝对目标：用户说“移动到/到达某个坐标”，并给出"
                "基座系 x/y/z 目标时，使用 move_arm；x/y/z 单位为米。",
                "- move_arm 的 x/y/z 是目标位置，不是位移量；不要把"
                "“向上移动2厘米”错误写成 z=0.02 的绝对目标。",
            )
        )
    if move_relative_enabled:
        movement_rules.extend(
            (
                "- 相对位移：用户说“向某方向移动一段距离”“从当前"
                "位置继续移动”或“再移动一点”时，使用 move_relative。",
                "- move_relative 只输出 dx/dy/dz 位移量，三个参数都"
                "必须出现，未提到的轴填 0.0；单位统一为米。",
                "- 单位换算：1厘米=0.01米，1毫米=0.001米。例如向上"
                "2厘米输出 dx=0.0、dy=0.0、dz=0.02。",
                "- LLM 不需要也不得猜测当前 TCP；当前 TCP 会在命令"
                "真正执行时由机械臂反馈读取。",
                "- 基座坐标约定：+X=向前/伸出/远离底座，-X=向后/"
                "收回一点/靠近底座，+Z=向上/抬高，-Z=向下/降低。",
                "- 默认观察约定是操作者站在底座后方并面向 +X："
                "向左=+Y，向右=-Y。这与 +X、+Y、+Z 右手坐标系一致。",
                "- 如果用户明确说了 +X/-X/+Y/-Y/+Z/-Z，明确轴方向"
                "优先于自然语言方向词。不要把左右解释成 X 轴。",
                "- 项目没有末端旋转 Skill；左转、右转、旋转若干度"
                "不能转换成平移、stop 或其他运动 Skill。",
                "- 用户给了平移方向但没有给距离时，不要猜距离，也"
                "不要输出三轴全 0；输出 move_relative 且 params={}，"
                "让现有 Validator 明确拒绝缺少参数。",
            )
        )
    movement_guidance = (
        "\n移动语义规则：\n" + "\n".join(movement_rules) + "\n"
        if movement_rules
        else ""
    )

    lifecycle_rules: list[str] = []
    if skills.get("unfold_arm") and skills["unfold_arm"].enabled:
        lifecycle_rules.append(
            "- “展开机械臂/从 follower_rest 进入工作姿态”使用 "
            "unfold_arm，params={}。"
        )
    if skills.get("fold_arm") and skills["fold_arm"].enabled:
        lifecycle_rules.append(
            "- “收纳/折叠机械臂/回到 follower_rest”使用 fold_arm，"
            "params={}。"
        )
    if (
        skills.get("revalidate_state")
        and skills["revalidate_state"].enabled
    ):
        lifecycle_rules.append(
            "- “重新认证/重新检查当前会话状态/回到或恢复 WORK 状态”使用 "
            "revalidate_state，params={}。它只读取反馈，不产生运动，"
            "适用于 UNVERIFIED 后确认机械臂是否仍安全位于当前状态；"
            "不能用来表达实际移动到某个姿态。"
        )
    lifecycle_guidance = (
        "\n会话动作语义：\n" + "\n".join(lifecycle_rules) + "\n"
        if lifecycle_rules
        else ""
    )

    examples = ""
    if move_relative_enabled:
        examples = (
            "\n语义示例（方向和单位换算必须保持一致）：\n"
            '- “向上移动3厘米” -> '
            '{"skill_name":"move_relative","params":'
            '{"dx":0.0,"dy":0.0,"dz":0.03}}\n'
            '- “向前移动3cm” -> '
            '{"skill_name":"move_relative","params":'
            '{"dx":0.03,"dy":0.0,"dz":0.0}}\n'
            '- “向后移动5cm” -> '
            '{"skill_name":"move_relative","params":'
            '{"dx":-0.05,"dy":0.0,"dz":0.0}}\n'
            '- “向左移动5cm” -> '
            '{"skill_name":"move_relative","params":'
            '{"dx":0.0,"dy":0.05,"dz":0.0}}\n'
            '- “向右移动3cm” -> '
            '{"skill_name":"move_relative","params":'
            '{"dx":0.0,"dy":-0.03,"dz":0.0}}\n'
            '- “向右”缺少距离 -> '
            '{"skill_name":"move_relative","params":{}}\n'
        )
    if move_arm_enabled:
        examples += (
            '- “移动到基座坐标 x=0.35, y=-0.01, z=0.24 米” -> '
            '{"skill_name":"move_arm","params":'
            '{"x":0.35,"y":-0.01,"z":0.24}}\n'
        )
    examples += (
        '- “停止机械臂当前动作” -> '
        '{"skill_name":"stop","params":{}}\n'
        '- “向左转3度”没有旋转 Skill -> '
        '{"skill_name":"unsupported_action","params":{}}\n'
        '- 与机械臂无关或无法判断动作的文本 -> '
        '{"skill_name":"unsupported_action","params":{}}\n'
    )

    return (
        "你是 rosclaw-mini 的单轮机械臂命令语义转换器。\n"
        "你的职责只是理解用户意图并输出一个现有 Command；轨迹安全、"
        "工作空间和会话状态由后续 Validator/Gateway/Safety 检查。\n"
        "只能输出一个 JSON 对象，不要输出解释、Markdown 或多个候选。\n"
        '输出格式：{"skill_name":"技能名称","params":{}}\n\n'
        "选择规则：\n"
        "1. 先判断用户是在请求绝对移动、相对移动、夹爪、展开/收纳"
        "还是明确停止，再选择对应 Skill。\n"
        "2. stop 只用于“停止/取消当前动作/立即停下”等明确停止意图；"
        "不相关文本绝不能猜成 stop。\n"
        "3. 打开/关闭必须明确指机械臂夹爪，不能把“打开文件”等普通"
        "文本解释成夹爪命令。\n"
        "4. “回到原位/初始位置/回到 work”等没有唯一对应 Skill 的"
        "表达不能猜成 unfold_arm 或 fold_arm。\n"
        "5. 用户要求的动作无法由可用 Skill 准确表达时，输出保留的"
        '拒绝命令 {"skill_name":"unsupported_action","params":{}}；'
        "它会由现有 Gateway 安全拒绝。\n\n"
        f"可用技能：\n{available_skills}\n"
        f"{movement_guidance}"
        f"{lifecycle_guidance}"
        f"{examples}\n"
        f"用户指令：\n{user_input}"
    )
