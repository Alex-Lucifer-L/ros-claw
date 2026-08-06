from __future__ import annotations

import json

import numpy as np

from rosclaw_mini.vision.realsense import ColorIntrinsics, RealSenseFrame
from scripts.check_realsense_camera import main


class FakeAdapter:
    instances = []

    def __init__(self, serial, **kwargs):
        self.serial = serial
        self.kwargs = kwargs
        self.closed = False
        self.count = 0
        self.instances.append(self)

    def __enter__(self):
        return self

    def capture_frame(self):
        self.count += 1
        return RealSenseFrame(
            rgb=np.zeros((3, 4, 3), dtype=np.uint8),
            aligned_depth=np.ones((3, 4), dtype=np.uint16),
            color_intrinsics=ColorIntrinsics(
                width=4,
                height=3,
                fx=100.0,
                fy=100.0,
                ppx=1.5,
                ppy=1.0,
                distortion_model="distortion.none",
                coefficients=(0.0,) * 5,
            ),
            depth_scale_m_per_unit=0.001,
            source=f"realsense:{self.serial}",
            frame_number=10 + self.count,
            timestamp_ms=1000.0 + self.count,
        )

    def __exit__(self, *_args):
        self.closed = True


def test_readonly_diagnostic_uses_configured_serial_and_closes():
    FakeAdapter.instances.clear()
    outputs = []

    code = main(
        [
            "--serial",
            "serial-1",
            "--width",
            "4",
            "--height",
            "3",
            "--frames",
            "3",
            "--warmup-frames",
            "0",
        ],
        adapter_factory=FakeAdapter,
        output_func=outputs.append,
    )

    assert code == 0
    adapter = FakeAdapter.instances[0]
    assert adapter.serial == "serial-1"
    assert adapter.kwargs["width"] == 4
    assert adapter.closed
    payload = json.loads(outputs[0])
    assert payload["valid_frames"] == 3
    assert payload["frame_number_gaps"] == 0
    assert payload["accepted"] is True
    assert payload["color_intrinsics"]["fx"] == 100.0
