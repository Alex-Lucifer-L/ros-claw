from __future__ import annotations

import json
from types import SimpleNamespace

from rosclaw_mini.vision.eye_to_hand import (
    EyeToHandCalibration,
    write_eye_to_hand_calibration,
)
from rosclaw_mini.vision.localization import PositionEstimate
from scripts.locate_realsense_object import main


class FakeObservation:
    def to_dict(self):
        return {"observation_id": "obs-1"}


class FakeService:
    def __init__(self, *, client, camera_factory):
        self.client = client
        self.camera_factory = camera_factory

    def locate(self, question):
        assert question == "locate red cap"
        return SimpleNamespace(
            observation=FakeObservation(),
            position=PositionEstimate(
                observation_id="obs-1",
                object_name="red cap",
                bounding_box=(0.4, 0.4, 0.6, 0.6),
                center_pixel=(320, 240),
                depth_m=0.3,
                camera_point_m=(0.1, 0.2, 0.3),
                valid_depth_ratio=0.9,
                uncertainty_m=0.004,
                quality="good",
                source="realsense:serial-1",
                source_frame=7,
                source_timestamp_ms=1234.5,
            ),
        )


def calibration(*, active=True, serial="serial-1"):
    return EyeToHandCalibration(
        camera_serial=serial,
        width=640,
        height=480,
        camera_frame="realsense_color_optical_frame",
        base_frame="so100_plus_base",
        units="m",
        dataset_sha256="a" * 64,
        method="test",
        base_from_camera=(
            (1.0, 0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0, 2.0),
            (0.0, 0.0, 1.0, 3.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        fit_point_count=6,
        validation_point_count=3,
        fit_rmse_m=0.01,
        fit_max_error_m=0.02,
        validation_rmse_m=0.015,
        validation_max_error_m=0.025,
        per_point_error_m=(("fit_001", 0.01),),
        activation_max_rmse_m=0.025,
        activation_max_error_m=0.04,
        active=active,
        activation_message="ok" if active else "inactive",
        created_at="2026-08-05T00:00:00+00:00",
    )


def run_main(arguments, outputs):
    return main(
        arguments,
        environ={"DASHSCOPE_API_KEY": "test-key"},
        client_builder=lambda **_kwargs: object(),
        camera_builder=lambda *_args, **_kwargs: object(),
        service_builder=FakeService,
        output_func=outputs.append,
    )


def test_camera_only_output_remains_compatible():
    outputs = []

    code = run_main(
        ["--serial", "serial-1", "--question", "locate red cap"],
        outputs,
    )

    assert code == 0
    payload = json.loads(outputs[-1])
    assert payload["position_estimate"]["camera_point_m"] == [0.1, 0.2, 0.3]
    assert "base_position_estimate" not in payload


def test_active_identity_bound_calibration_adds_base_position(tmp_path):
    path = tmp_path / "base_T_camera.json"
    write_eye_to_hand_calibration(calibration(), path)
    outputs = []

    code = run_main(
        [
            "--serial",
            "serial-1",
            "--question",
            "locate red cap",
            "--eye-to-hand-calibration",
            str(path),
        ],
        outputs,
    )

    assert code == 0
    payload = json.loads(outputs[-1])
    base = payload["base_position_estimate"]
    assert base["base_point_m"] == [1.1, 2.2, 3.3]
    assert base["source_frame"] == 7
    assert base["validation_rmse_m"] == 0.015
    assert base["calibration_sha256"] == calibration().calibration_sha256


def test_inactive_or_wrong_device_calibration_fails_closed(tmp_path):
    for name, value in (
        ("inactive", calibration(active=False)),
        ("wrong-device", calibration(serial="another-camera")),
    ):
        path = tmp_path / f"{name}.json"
        write_eye_to_hand_calibration(value, path)
        outputs = []

        code = run_main(
            [
                "--serial",
                "serial-1",
                "--question",
                "locate red cap",
                "--eye-to-hand-calibration",
                str(path),
            ],
            outputs,
        )

        assert code == 1
        assert "RealSense 目标定位失败" in outputs[-1]
