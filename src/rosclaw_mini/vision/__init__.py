"""RosClaw Mini V2.0 只读视觉观察模块。"""

from rosclaw_mini.vision.schemas import SceneObject, SceneObservation
from rosclaw_mini.vision.service import VisionService

__all__ = ["SceneObject", "SceneObservation", "VisionService"]
