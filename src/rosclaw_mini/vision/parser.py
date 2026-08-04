"""把视觉模型文本严格解析为 SceneObservation。"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import re
from typing import Any
from uuid import uuid4

from rosclaw_mini.vision.exceptions import (
    SceneObservationValidationError,
    VLMResponseParseError,
)
from rosclaw_mini.vision.schemas import (
    LOCATION_IN_IMAGE_VALUES,
    SceneObject,
    SceneObservation,
)


_CODE_FENCE_PATTERN = re.compile(
    r"^\s*```(?:json)?\s*(?P<body>.*?)\s*```\s*$",
    re.IGNORECASE | re.DOTALL,
)
_DATA_URL_PATTERN = re.compile(
    r"data:image/[^;\s]+;base64,[A-Za-z0-9+/=]+",
    re.IGNORECASE,
)
_FORBIDDEN_COORDINATE_KEYS = frozenset(
    {
        "x",
        "y",
        "z",
        "robot_x",
        "robot_y",
        "robot_z",
        "arm_x",
        "arm_y",
        "arm_z",
        "base_x",
        "base_y",
        "base_z",
        "tcp_x",
        "tcp_y",
        "tcp_z",
        "world_x",
        "world_y",
        "world_z",
        "position_3d",
        "coordinates_3d",
        "robot_coordinates",
        "base_coordinates",
        "tcp_position",
    }
)


def _safe_excerpt(raw_response: str, *, limit: int = 300) -> str:
    redacted = _DATA_URL_PATTERN.sub("<base64-image-redacted>", raw_response)
    compact = " ".join(redacted.split())
    return compact[:limit]


def _strip_json_fence(raw_response: str) -> str:
    text = raw_response.strip().lstrip("\ufeff")
    match = _CODE_FENCE_PATTERN.fullmatch(text)
    return match.group("body").strip() if match else text


def _reject_robot_coordinates(value: Any, *, path: str = "$.") -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            normalized = str(key).strip().lower()
            if normalized in _FORBIDDEN_COORDINATE_KEYS:
                raise SceneObservationValidationError(
                    "SceneObservation 禁止包含机械臂/基座三维坐标字段："
                    f"{path}{key}"
                )
            _reject_robot_coordinates(child, path=f"{path}{key}.")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_robot_coordinates(child, path=f"{path}[{index}].")


def _optional_string(value: Any, *, field_name: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise SceneObservationValidationError(
            f"{field_name} 必须是字符串或 null。"
        )
    stripped = value.strip()
    return stripped or None


def _parse_confidence(value: Any, *, object_index: int) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SceneObservationValidationError(
            f"objects[{object_index}].confidence 必须是 0 到 1 的数值或 null。"
        )
    confidence = float(value)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise SceneObservationValidationError(
            f"objects[{object_index}].confidence 超出 [0, 1]。"
        )
    return confidence


def _parse_bounding_box(
    value: Any,
    *,
    object_index: int,
) -> tuple[float, float, float, float] | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 4:
        raise SceneObservationValidationError(
            f"objects[{object_index}].bounding_box 必须是四个归一化数值或 null。"
        )
    result: list[float] = []
    for coordinate in value:
        if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)):
            raise SceneObservationValidationError(
                f"objects[{object_index}].bounding_box 包含非数值。"
            )
        coordinate = float(coordinate)
        if not math.isfinite(coordinate) or not 0.0 <= coordinate <= 1.0:
            raise SceneObservationValidationError(
                f"objects[{object_index}].bounding_box 必须位于 [0, 1]。"
            )
        result.append(coordinate)
    x_min, y_min, x_max, y_max = result
    if x_min > x_max or y_min > y_max:
        raise SceneObservationValidationError(
            f"objects[{object_index}].bounding_box 最小值不能大于最大值。"
        )
    return x_min, y_min, x_max, y_max


class SceneObservationParser:
    """严格解析 VLM JSON；不猜测或修补错误字段。"""

    def parse(
        self,
        raw_response: str,
        *,
        source: str,
        model: str,
        include_raw_response: bool = False,
    ) -> SceneObservation:
        if not isinstance(raw_response, str) or not raw_response.strip():
            raise VLMResponseParseError("视觉模型返回了空响应。")
        try:
            payload = json.loads(_strip_json_fence(raw_response))
        except json.JSONDecodeError as error:
            raise VLMResponseParseError(
                "视觉模型返回的内容不是有效 JSON；响应摘要："
                f"{_safe_excerpt(raw_response)!r}"
            ) from error
        if not isinstance(payload, dict):
            raise SceneObservationValidationError(
                "SceneObservation 顶层必须是 JSON 对象。"
            )

        _reject_robot_coordinates(payload)
        scene_description = payload.get("scene_description")
        if not isinstance(scene_description, str) or not scene_description.strip():
            raise SceneObservationValidationError(
                "scene_description 必须是非空字符串。"
            )
        objects_payload = payload.get("objects")
        if not isinstance(objects_payload, list):
            raise SceneObservationValidationError("objects 必须是列表。")
        warnings_payload = payload.get("warnings", [])
        if not isinstance(warnings_payload, list) or not all(
            isinstance(item, str) for item in warnings_payload
        ):
            raise SceneObservationValidationError("warnings 必须是字符串列表。")

        objects: list[SceneObject] = []
        for index, item in enumerate(objects_payload):
            if not isinstance(item, dict):
                raise SceneObservationValidationError(
                    f"objects[{index}] 必须是 JSON 对象。"
                )
            name = item.get("name")
            if not isinstance(name, str) or not name.strip():
                raise SceneObservationValidationError(
                    f"objects[{index}].name 必须是非空字符串。"
                )
            location = item.get("location_in_image", "unknown")
            if location not in LOCATION_IN_IMAGE_VALUES:
                raise SceneObservationValidationError(
                    f"objects[{index}].location_in_image 非法：{location!r}。"
                )
            attributes = item.get("attributes", {})
            if not isinstance(attributes, dict):
                raise SceneObservationValidationError(
                    f"objects[{index}].attributes 必须是 JSON 对象。"
                )
            objects.append(
                SceneObject(
                    name=name.strip(),
                    category=_optional_string(
                        item.get("category"),
                        field_name=f"objects[{index}].category",
                    ),
                    color=_optional_string(
                        item.get("color"),
                        field_name=f"objects[{index}].color",
                    ),
                    location_in_image=location,
                    confidence=_parse_confidence(
                        item.get("confidence"),
                        object_index=index,
                    ),
                    attributes=dict(attributes),
                    bounding_box=_parse_bounding_box(
                        item.get("bounding_box"),
                        object_index=index,
                    ),
                )
            )

        return SceneObservation(
            observation_id=str(uuid4()),
            timestamp=datetime.now(timezone.utc).isoformat(),
            scene_description=scene_description.strip(),
            objects=tuple(objects),
            warnings=tuple(item.strip() for item in warnings_payload if item.strip()),
            source=str(source),
            model=str(model),
            raw_response=raw_response if include_raw_response else None,
        )

