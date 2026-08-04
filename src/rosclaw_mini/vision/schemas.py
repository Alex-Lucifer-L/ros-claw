"""与机械臂坐标完全隔离的结构化场景观察对象。"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


LOCATION_IN_IMAGE_VALUES = frozenset(
    {
        "left",
        "center",
        "right",
        "upper_left",
        "upper_center",
        "upper_right",
        "lower_left",
        "lower_center",
        "lower_right",
        "unknown",
    }
)


@dataclass(frozen=True)
class SceneObject:
    name: str
    location_in_image: str
    category: str | None = None
    color: str | None = None
    confidence: float | None = None
    attributes: dict[str, Any] = field(default_factory=dict)
    bounding_box: tuple[float, float, float, float] | None = None

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        if self.bounding_box is not None:
            result["bounding_box"] = list(self.bounding_box)
        return result


@dataclass(frozen=True)
class SceneObservation:
    observation_id: str
    timestamp: str
    scene_description: str
    objects: tuple[SceneObject, ...]
    warnings: tuple[str, ...]
    source: str
    model: str
    raw_response: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "observation_id": self.observation_id,
            "timestamp": self.timestamp,
            "scene_description": self.scene_description,
            "objects": [item.to_dict() for item in self.objects],
            "warnings": list(self.warnings),
            "source": self.source,
            "model": self.model,
            "raw_response": self.raw_response,
        }
