from __future__ import annotations

import pytest

from rosclaw_mini.vision.camera import CameraAdapter
from rosclaw_mini.vision.exceptions import CameraOpenError, FrameCaptureError


class FakeCapture:
    def __init__(self, *, opened=True, read_result=(True, "frame")):
        self.opened = opened
        self.read_result = read_result
        self.released = False

    def isOpened(self):
        return self.opened

    def read(self):
        if isinstance(self.read_result, Exception):
            raise self.read_result
        return self.read_result

    def release(self):
        self.released = True


def test_camera_adapter_captures_one_frame_and_releases():
    capture = FakeCapture()
    with CameraAdapter(2, capture_factory=lambda index: capture) as camera:
        assert camera.capture_frame() == "frame"
        assert not capture.released
    assert capture.released


def test_camera_adapter_rejects_camera_that_cannot_open_and_releases():
    capture = FakeCapture(opened=False)
    camera = CameraAdapter(0, capture_factory=lambda index: capture)
    with pytest.raises(CameraOpenError, match="无法打开摄像头 0"):
        camera.open()
    assert capture.released


def test_camera_adapter_maps_failed_read():
    capture = FakeCapture(read_result=(False, None))
    with CameraAdapter(0, capture_factory=lambda index: capture) as camera:
        with pytest.raises(FrameCaptureError, match="未成功读取图像"):
            camera.capture_frame()
    assert capture.released


def test_camera_adapter_releases_when_context_body_raises():
    capture = FakeCapture()
    with pytest.raises(RuntimeError, match="boom"):
        with CameraAdapter(0, capture_factory=lambda index: capture):
            raise RuntimeError("boom")
    assert capture.released

