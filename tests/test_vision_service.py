from __future__ import annotations

import json
from pathlib import Path

from rosclaw_mini.vision.image import EncodedImage
from rosclaw_mini.vision.prompt import DEFAULT_VISION_QUESTION
from rosclaw_mini.vision.service import VisionService


class FakeCamera:
    def __init__(self, index, events):
        self.index = index
        self.events = events

    def __enter__(self):
        self.events.append(("open", self.index))
        return self

    def capture_frame(self):
        self.events.append(("capture", self.index))
        return "camera-frame"

    def __exit__(self, *args):
        self.events.append(("close", self.index))


class FakeImageProcessor:
    def __init__(self, events):
        self.events = events

    def load(self, path):
        self.events.append(("load", path))
        return "local-frame"

    def save(self, frame, path):
        self.events.append(("save", frame, path))

    def prepare(self, frame, *, max_width):
        self.events.append(("prepare", frame, max_width))
        return EncodedImage(b"jpeg", "image/jpeg", 320, 240)


class FakeVLMClient:
    model = "fake-vl"

    def __init__(self, events):
        self.events = events

    def generate(self, *, image_bytes, mime_type, prompt):
        self.events.append(("vlm", image_bytes, mime_type, prompt))
        return json.dumps(
            {
                "scene_description": "桌面上有方块。",
                "objects": [{"name": "block", "location_in_image": "left"}],
                "warnings": [],
            }
        )


def make_service(events):
    return VisionService(
        client=FakeVLMClient(events),
        camera_index=3,
        max_width=640,
        camera_factory=lambda index: FakeCamera(index, events),
        image_processor=FakeImageProcessor(events),
    )


def test_service_releases_camera_before_calling_vlm_and_uses_default_question():
    events = []
    observation = make_service(events).observe()
    assert [event[0] for event in events] == [
        "open",
        "capture",
        "close",
        "prepare",
        "vlm",
    ]
    assert DEFAULT_VISION_QUESTION in events[-1][3]
    assert observation.source == "camera:3"


def test_service_passes_user_visual_question():
    events = []
    make_service(events).observe(question="红色方块在哪里？")
    assert "红色方块在哪里？" in events[-1][3]


def test_local_image_mode_never_opens_camera(tmp_path: Path):
    events = []
    image_path = tmp_path / "fixture.jpg"
    observation = make_service(events).observe(image_path=image_path)
    assert [event[0] for event in events] == ["load", "prepare", "vlm"]
    assert observation.source == f"image:{image_path}"


def test_camera_frame_is_saved_only_when_explicitly_requested(tmp_path: Path):
    events = []
    output = tmp_path / "capture.jpg"
    make_service(events).observe(save_frame_path=output)
    assert ("save", "camera-frame", output) in events

