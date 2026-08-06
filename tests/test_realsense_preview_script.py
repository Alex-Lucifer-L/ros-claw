from __future__ import annotations

import numpy as np

from rosclaw_mini.vision.realsense import ColorIntrinsics, RealSenseFrame
from scripts.preview_realsense_camera import main


class FakeAdapter:
    def __init__(self, serial, **kwargs):
        self.serial = serial
        self.kwargs = kwargs
        self.closed = False

    def __enter__(self):
        return self

    def capture_frame(self):
        return RealSenseFrame(
            rgb=np.zeros((4, 6, 3), dtype=np.uint8),
            aligned_depth=np.full((4, 6), 400, dtype=np.uint16),
            color_intrinsics=ColorIntrinsics(
                width=6,
                height=4,
                fx=5.0,
                fy=5.0,
                ppx=3.0,
                ppy=2.0,
                distortion_model="none",
                coefficients=(0.0,) * 5,
            ),
            depth_scale_m_per_unit=0.001,
            source=f"realsense:{self.serial}",
            frame_number=1,
            timestamp_ms=10.0,
        )

    def __exit__(self, *_args):
        self.closed = True


class FakeCV2:
    WINDOW_NORMAL = 0
    COLORMAP_TURBO = 1
    FONT_HERSHEY_SIMPLEX = 2

    def __init__(self):
        self.events = []

    def namedWindow(self, name, mode):
        self.events.append(("window", name, mode))

    def applyColorMap(self, gray, _mapping):
        return np.repeat(gray[..., None], 3, axis=2)

    def imshow(self, name, image):
        self.events.append(("show", name, image.shape))

    def putText(self, image, text, origin, *_args):
        self.events.append(("text", text, origin, image.shape))

    def waitKey(self, _delay):
        return ord("q")

    def destroyWindow(self, name):
        self.events.append(("destroy", name))


def test_preview_requires_acknowledgement_without_opening_camera():
    opened = []
    outputs = []
    assert (
        main(
            ["--serial", "serial-1"],
            adapter_factory=lambda *args, **kwargs: opened.append((args, kwargs)),
            cv2_module=FakeCV2(),
            output_func=outputs.append,
        )
        == 2
    )
    assert opened == []
    assert "显式确认" in outputs[0]


def test_preview_shows_rgb_and_depth_then_closes_on_q():
    cv2 = FakeCV2()
    adapters = []

    def factory(*args, **kwargs):
        adapter = FakeAdapter(*args, **kwargs)
        adapters.append(adapter)
        return adapter

    assert (
        main(
            [
                "--serial",
                "serial-1",
                "--acknowledge-camera-preview",
            ],
            adapter_factory=factory,
            cv2_module=cv2,
        )
        == 0
    )
    assert ("show", cv2.events[1][1], (4, 12, 3)) == cv2.events[1]
    assert cv2.events[-1][0] == "destroy"
    assert adapters[0].closed


def test_preview_draws_target_crosshair_and_depth_text():
    cv2 = FakeCV2()

    assert (
        main(
            [
                "--serial",
                "serial-1",
                "--width",
                "6",
                "--height",
                "4",
                "--target-pixel",
                "3",
                "2",
                "--target-depth-m",
                "0.4",
                "--acknowledge-camera-preview",
            ],
            adapter_factory=FakeAdapter,
            cv2_module=cv2,
        )
        == 0
    )
    text_event = next(event for event in cv2.events if event[0] == "text")
    assert "desired depth=0.400 m" in text_event[1]
    assert "current=0.400 m" in text_event[1]


def test_preview_rejects_target_outside_frame_without_opening_camera():
    opened = []
    outputs = []

    assert (
        main(
            [
                "--serial",
                "serial-1",
                "--width",
                "6",
                "--height",
                "4",
                "--target-pixel",
                "6",
                "2",
                "--acknowledge-camera-preview",
            ],
            adapter_factory=lambda *args, **kwargs: opened.append((args, kwargs)),
            cv2_module=FakeCV2(),
            output_func=outputs.append,
        )
        == 2
    )
    assert opened == []
    assert "target-pixel" in outputs[-1]
