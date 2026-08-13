"""Offline VLM substitute that finds coloured objects from rendered RGB pixels.

It intentionally receives JPEG bytes rather than :class:`SimulationWorld`, so
the normal simulation path cannot leak object ground truth into localization.
"""

from __future__ import annotations

import json
from typing import Any

import cv2
import numpy as np

from rosclaw_mini.vision.vlm_client import VLMClient


class SimulatedColorVLM:
    """A deterministic image-only target selector for offline experiments."""

    model = "sim-color-grounding-v1"

    _COLOR_RULES = {
        "red": ("red", lambda image: (image[..., 0] > 140) & (image[..., 1] < 115) & (image[..., 2] < 115)),
        "红": ("red", lambda image: (image[..., 0] > 140) & (image[..., 1] < 115) & (image[..., 2] < 115)),
        "blue": ("blue", lambda image: (image[..., 2] > 130) & (image[..., 0] < 120)),
        "蓝": ("blue", lambda image: (image[..., 2] > 130) & (image[..., 0] < 120)),
        "green": ("green", lambda image: (image[..., 1] > 105) & (image[..., 0] < 120)),
        "绿": ("green", lambda image: (image[..., 1] > 105) & (image[..., 0] < 120)),
    }

    def generate(
        self,
        *,
        image_bytes: bytes,
        mime_type: str,
        prompt: str,
    ) -> str:
        if mime_type not in {"image/jpeg", "image/png"}:
            raise ValueError("仿真 VLM 只接受 JPEG 或 PNG。")
        encoded = np.frombuffer(image_bytes, dtype=np.uint8)
        bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("仿真 VLM 无法解码图像。")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        lowered = prompt.lower()
        selected_name = "simulated_object"
        mask: np.ndarray | None = None
        for token, (name, rule) in self._COLOR_RULES.items():
            if token in lowered:
                selected_name = f"{name}_object"
                mask = rule(rgb)
                break
        if mask is None:
            # Do not consult scene truth.  Any saturated rendered colour is a
            # visible candidate; background intentionally fails this mask.
            mask = (rgb.max(axis=2) - rgb.min(axis=2)) > 55
        ys, xs = np.nonzero(mask)
        if len(xs) < 16:
            return json.dumps(
                {
                    "scene_description": "simulation target not confidently visible",
                    "objects": [],
                    "warnings": ["目标颜色未在 RGB 图像中检测到。"],
                },
                ensure_ascii=False,
            )
        x_min, x_max = int(xs.min()), int(xs.max()) + 1
        y_min, y_max = int(ys.min()), int(ys.max()) + 1
        height, width = rgb.shape[:2]
        return json.dumps(
            {
                "scene_description": "headless simulation RGB observation",
                "objects": [
                    {
                        "name": selected_name,
                        "category": "simulated_object",
                        "color": selected_name.split("_")[0],
                        "location_in_image": "center",
                        "confidence": 0.95,
                        "attributes": {"simulation_only": True},
                        "bounding_box": None,
                        "bounding_box_pixels": [x_min, y_min, x_max, y_max],
                    }
                ],
                "warnings": [
                    "由离线颜色分割模拟 VLM；不代表真实模型识别性能。"
                ],
            },
            ensure_ascii=False,
        )


def simulated_vlm_from_response(payload: dict[str, Any]) -> VLMClient:
    """Testing helper returning a VLM-shaped static response without network."""

    class _StaticVLM:
        model = "sim-static-vlm-test"

        def generate(self, *, image_bytes: bytes, mime_type: str, prompt: str) -> str:
            del image_bytes, mime_type, prompt
            return json.dumps(payload, ensure_ascii=False)

    return _StaticVLM()  # type: ignore[return-value]
