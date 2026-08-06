from __future__ import annotations

import numpy as np
import pytest

from rosclaw_mini.vision.exceptions import (
    RealSenseDeviceError,
    RealSenseFrameError,
)
from rosclaw_mini.vision.realsense import (
    RealSenseCameraAdapter,
    frame_number_gaps,
)


class FakeNativeIntrinsics:
    width = 4
    height = 3
    fx = 100.0
    fy = 101.0
    ppx = 1.5
    ppy = 1.0
    model = "distortion.none"
    coeffs = [0.0] * 5


class FakeVideoProfile:
    def as_video_stream_profile(self):
        return self

    def get_intrinsics(self):
        return FakeNativeIntrinsics()


class FakeDepthSensor:
    def get_depth_scale(self):
        return 0.001


class FakeDevice:
    def __init__(self, serial="serial-1"):
        self.serial = serial

    def get_info(self, _key):
        return self.serial

    def first_depth_sensor(self):
        return FakeDepthSensor()


class FakeProfile:
    def __init__(self, serial="serial-1"):
        self.device = FakeDevice(serial)

    def get_device(self):
        return self.device

    def get_stream(self, _stream):
        return FakeVideoProfile()


class FakeFrame:
    def __init__(self, data, *, number=12, timestamp=345.0):
        self.data = data
        self.number = number
        self.timestamp = timestamp

    def get_data(self):
        return self.data

    def get_frame_number(self):
        return self.number

    def get_timestamp(self):
        return self.timestamp


class FakeFrames:
    def __init__(self, *, color=True, depth=True):
        self.color = (
            FakeFrame(np.full((3, 4, 3), 7, dtype=np.uint8))
            if color
            else None
        )
        self.depth = (
            FakeFrame(np.full((3, 4), 800, dtype=np.uint16))
            if depth
            else None
        )

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class FakeConfig:
    def __init__(self):
        self.device = None
        self.streams = []

    def enable_device(self, serial):
        self.device = serial

    def enable_stream(self, *args):
        self.streams.append(args)


class FakeAlign:
    def __init__(self, stream, events):
        self.stream = stream
        self.events = events

    def process(self, frames):
        self.events.append("align")
        return frames


class FakePipeline:
    def __init__(
        self,
        events,
        *,
        frames=None,
        serial="serial-1",
        wait_error=None,
    ):
        self.events = events
        self.frames = frames or FakeFrames()
        self.serial = serial
        self.wait_error = wait_error
        self.config = None

    def start(self, config):
        self.events.append("start")
        self.config = config
        return FakeProfile(self.serial)

    def wait_for_frames(self, timeout_ms):
        self.events.append(("wait", timeout_ms))
        if self.wait_error is not None:
            raise self.wait_error
        return self.frames

    def stop(self):
        self.events.append("stop")


class FakeRS:
    class stream:
        color = "color"
        depth = "depth"

    class format:
        rgb8 = "rgb8"
        z16 = "z16"

    class camera_info:
        serial_number = "serial_number"

    def __init__(self, events):
        self.events = events
        self.configs = []

    def config(self):
        config = FakeConfig()
        self.configs.append(config)
        return config

    def align(self, stream):
        return FakeAlign(stream, self.events)


def build_adapter(*, frames=None, serial="serial-1", wait_error=None):
    events = []
    rs = FakeRS(events)
    pipeline = FakePipeline(
        events,
        frames=frames,
        serial=serial,
        wait_error=wait_error,
    )
    adapter = RealSenseCameraAdapter(
        "serial-1",
        width=4,
        height=3,
        fps=30,
        timeout_ms=1234,
        rs_module=rs,
        numpy_module=np,
        pipeline_factory=lambda _rs: pipeline,
    )
    return adapter, pipeline, rs, events


def test_realsense_adapter_selects_serial_aligns_and_returns_runtime_metadata():
    adapter, pipeline, rs, events = build_adapter()

    with adapter:
        frame = adapter.capture_frame()

    assert pipeline.config.device == "serial-1"
    assert pipeline.config.streams == [
        ("color", 4, 3, "rgb8", 30),
        ("depth", 4, 3, "z16", 30),
    ]
    assert events == ["start", ("wait", 1234), "align", "stop"]
    assert frame.rgb.shape == (3, 4, 3)
    assert frame.aligned_depth.shape == (3, 4)
    assert frame.color_intrinsics.fx == 100.0
    assert frame.depth_scale_m_per_unit == pytest.approx(0.001)
    assert frame.source == "realsense:serial-1"
    assert frame.frame_number == 12
    assert frame.timestamp_ms == pytest.approx(345.0)
    assert not adapter.is_open


def test_realsense_adapter_stops_pipeline_if_identity_check_fails():
    adapter, _pipeline, _rs, events = build_adapter(serial="wrong")

    with pytest.raises(RealSenseDeviceError, match="身份不匹配"):
        adapter.open()

    assert events == ["start", "stop"]
    assert not adapter.is_open


def test_realsense_adapter_maps_timeout_and_context_still_closes():
    adapter, _pipeline, _rs, events = build_adapter(
        wait_error=RuntimeError("timeout")
    )

    with adapter:
        with pytest.raises(RealSenseFrameError, match="超时或失败"):
            adapter.capture_frame()

    assert events == ["start", ("wait", 1234), "stop"]


@pytest.mark.parametrize(
    ("frames", "message"),
    [
        (FakeFrames(color=False), "缺少 Color"),
        (FakeFrames(depth=False), "缺少 Depth"),
    ],
)
def test_realsense_adapter_rejects_missing_stream(frames, message):
    adapter, _pipeline, _rs, events = build_adapter(frames=frames)

    with adapter:
        with pytest.raises(RealSenseFrameError, match=message):
            adapter.capture_frame()

    assert events[-1] == "stop"


def test_frame_number_gaps_counts_only_forward_missing_frames():
    assert frame_number_gaps((10, 11, 14, 15)) == 2
    assert frame_number_gaps(()) == 0
    with pytest.raises(ValueError, match="非负整数"):
        frame_number_gaps((1, -1))
