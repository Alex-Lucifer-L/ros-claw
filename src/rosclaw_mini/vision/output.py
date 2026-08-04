"""Terminal renderers for SceneObservation."""

from __future__ import annotations

import json

from rosclaw_mini.vision.schemas import SceneObservation


_LOCATION_LABELS = {
    "left": "左侧",
    "center": "中央",
    "right": "右侧",
    "upper_left": "左上",
    "upper_center": "上方中央",
    "upper_right": "右上",
    "lower_left": "左下",
    "lower_center": "下方中央",
    "lower_right": "右下",
    "unknown": "未知",
}


def format_observation_json(observation: SceneObservation) -> str:
    return json.dumps(observation.to_dict(), ensure_ascii=False, indent=2)


def format_observation_text(observation: SceneObservation) -> str:
    lines = [
        f"场景：{observation.scene_description}",
        f"来源：{observation.source}",
        f"模型：{observation.model}",
        "物体：",
    ]
    if not observation.objects:
        lines.append("- 未识别到可靠物体")
    for item in observation.objects:
        details = [item.name]
        if item.category:
            details.append(f"类别={item.category}")
        if item.color:
            details.append(f"颜色={item.color}")
        details.append(
            f"画面位置={_LOCATION_LABELS.get(item.location_in_image, item.location_in_image)}"
        )
        if item.confidence is not None:
            details.append(f"置信度={item.confidence:.2f}")
        lines.append("- " + "，".join(details))
    lines.append("警告：")
    if observation.warnings:
        lines.extend(f"- {warning}" for warning in observation.warnings)
    else:
        lines.append("- 无")
    return "\n".join(lines)

