"""校验自然语言方向与 LLM 生成的相对运动参数是否一致。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import re

from rosclaw_mini.command_schema.commands import Command


class CommandIntentValidationError(ValueError):
    """LLM 生成的 Command 与用户明确表达的运动意图冲突。"""


@dataclass(frozen=True)
class DirectionIntent:
    """用户文本中一个带距离的明确基座系方向位移。"""

    phrase: str
    axis: str
    displacement_m: float


_DIRECTION_TO_AXIS_SIGN = {
    "前": ("dx", 1.0),
    "后": ("dx", -1.0),
    "左": ("dy", 1.0),
    "右": ("dy", -1.0),
    "上": ("dz", 1.0),
    "下": ("dz", -1.0),
}
_UNIT_TO_METERS = {
    "毫米": 0.001,
    "mm": 0.001,
    "厘米": 0.01,
    "cm": 0.01,
    "米": 1.0,
    "m": 1.0,
}
_DIRECTION_MENTION_PATTERN = re.compile(r"(?:向|往)\s*([前后左右上下])")
_DIRECTION_DISTANCE_PATTERN = re.compile(
    r"(?:向|往)\s*(?P<direction>[前后左右上下])"
    r"(?:边|方)?\s*(?:移动|移|走|伸|抬|升|降|降低)?\s*"
    r"(?P<distance>(?:\d+(?:\.\d*)?|\.\d+))\s*"
    r"(?P<unit>毫米|厘米|mm|cm|米|m)",
    re.IGNORECASE,
)
_AXIS_MENTION_PATTERN = re.compile(r"[+-]\s*[xyz](?:轴|方向)?", re.IGNORECASE)
_AXIS_DISTANCE_PATTERN = re.compile(
    r"(?P<sign>[+-])\s*(?P<axis>[xyz])(?:轴|方向)?\s*"
    r"(?:移动|移)?\s*(?P<distance>(?:\d+(?:\.\d*)?|\.\d+))\s*"
    r"(?P<unit>毫米|厘米|mm|cm|米|m)",
    re.IGNORECASE,
)
_AMBIGUOUS_MOVEMENT_PATTERNS = (
    re.compile(r"^(?:向|往)$"),
    re.compile(r"^(?:向|往)(?:那边|那里|这边|这里)(?:移动|挪动|走)?(?:一下)?$"),
    re.compile(r"^(?:移动|挪动|动|走)(?:一下|一点|一点点)?$"),
)
_CONTROL_SKILLS = {"stop"}


def _normalized_text(text: str) -> str:
    return re.sub(r"[\s，,。.!！?？；;]", "", text).lower()


def _intent_error(
    user_input: str,
    command: Command,
    reason: str,
) -> CommandIntentValidationError:
    return CommandIntentValidationError(
        "自然语言与 LLM 命令语义不一致："
        f"用户语义={user_input!r}；生成参数={command.params!r}；"
        f"冲突原因={reason}"
    )


def _parse_direction_intents(user_input: str) -> tuple[DirectionIntent, ...]:
    intents: list[DirectionIntent] = []
    for match in _DIRECTION_DISTANCE_PATTERN.finditer(user_input):
        axis, sign = _DIRECTION_TO_AXIS_SIGN[match.group("direction")]
        unit = match.group("unit").lower()
        distance_m = float(match.group("distance")) * _UNIT_TO_METERS[unit]
        intents.append(
            DirectionIntent(
                phrase=match.group(0),
                axis=axis,
                displacement_m=sign * distance_m,
            )
        )
    for match in _AXIS_DISTANCE_PATTERN.finditer(user_input):
        axis = f"d{match.group('axis').lower()}"
        sign = 1.0 if match.group("sign") == "+" else -1.0
        unit = match.group("unit").lower()
        distance_m = float(match.group("distance")) * _UNIT_TO_METERS[unit]
        intents.append(
            DirectionIntent(
                phrase=match.group(0),
                axis=axis,
                displacement_m=sign * distance_m,
            )
        )
    return tuple(intents)


def validate_command_intent(user_input: str, command: Command) -> None:
    """失败关闭地复核明确方向、距离、轴和符号。

    这里只判断 LLM 是否忠实理解原始文本，不判断目标工作空间、IK、
    轨迹碰撞或会话状态；这些仍由原有安全执行链负责。
    """

    if command.skill_name in _CONTROL_SKILLS:
        return

    normalized = _normalized_text(user_input)
    if any(pattern.fullmatch(normalized) for pattern in _AMBIGUOUS_MOVEMENT_PATTERNS):
        raise _intent_error(
            user_input,
            command,
            "运动要求缺少可验证的明确方向或数值距离",
        )

    direction_mentions = tuple(_DIRECTION_MENTION_PATTERN.finditer(user_input))
    axis_mentions = tuple(_AXIS_MENTION_PATTERN.finditer(user_input))
    intents = _parse_direction_intents(user_input)
    explicit_mention_count = len(direction_mentions) + len(axis_mentions)

    if explicit_mention_count == 0:
        if command.skill_name == "move_relative":
            raise _intent_error(
                user_input,
                command,
                "move_relative 缺少可验证的明确方向和数值距离",
            )
        return

    if len(intents) != explicit_mention_count:
        raise _intent_error(
            user_input,
            command,
            "每个明确方向都必须带有独立的数值距离和 mm/cm/m 单位",
        )
    if command.skill_name != "move_relative":
        raise _intent_error(
            user_input,
            command,
            "带方向和距离的相对运动必须生成 move_relative",
        )

    expected = {"dx": 0.0, "dy": 0.0, "dz": 0.0}
    for intent in intents:
        expected[intent.axis] += intent.displacement_m

    actual: dict[str, float] = {}
    for axis in expected:
        value = command.params.get(axis)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise _intent_error(
                user_input,
                command,
                f"{axis} 不是合法数值",
            )
        value = float(value)
        if not math.isfinite(value):
            raise _intent_error(
                user_input,
                command,
                f"{axis} 不是有限数值",
            )
        actual[axis] = value

    conflicts = []
    for axis, expected_value in expected.items():
        actual_value = actual[axis]
        if not math.isclose(
            actual_value,
            expected_value,
            rel_tol=1e-9,
            abs_tol=1e-9,
        ):
            conflicts.append(
                f"{axis} 应为 {expected_value:.9g} m，"
                f"实际为 {actual_value:.9g} m"
            )
    if conflicts:
        semantic_summary = ", ".join(
            f"{intent.phrase!r}→{intent.axis}={intent.displacement_m:.9g}m"
            for intent in intents
        )
        raise _intent_error(
            user_input,
            command,
            f"识别到 {semantic_summary}；" + "；".join(conflicts),
        )

