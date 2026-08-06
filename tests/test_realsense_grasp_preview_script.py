from __future__ import annotations

import json
from types import SimpleNamespace

from rosclaw_mini.vision.eye_to_hand import (
    EyeToHandCalibration,
    write_eye_to_hand_calibration,
)
from rosclaw_mini.vision.localization import PositionEstimate
from scripts.preview_realsense_grasp_plan import main


class FakeObservation:
    pass


class FakeService:
    def __init__(self, *, client, camera_factory):
        pass

    def locate(self, _question):
        return SimpleNamespace(
            observation=FakeObservation(),
            position=PositionEstimate(
                observation_id="obs-1",
                object_name="red box",
                bounding_box=(0.4, 0.4, 0.6, 0.6),
                center_pixel=(320, 240),
                depth_m=0.3,
                camera_point_m=(0.3, 0.2, 0.1),
                valid_depth_ratio=0.9,
                uncertainty_m=0.004,
                quality="good",
                source="realsense:serial-1",
                source_frame=7,
                source_timestamp_ms=1000.0,
            ),
        )


def calibration():
    return EyeToHandCalibration(
        camera_serial="serial-1",
        width=640,
        height=480,
        camera_frame="camera",
        base_frame="base",
        units="m",
        dataset_sha256="a" * 64,
        method="test",
        base_from_camera=(
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        fit_point_count=6,
        validation_point_count=3,
        fit_rmse_m=0.01,
        fit_max_error_m=0.02,
        validation_rmse_m=0.02,
        validation_max_error_m=0.03,
        per_point_error_m=(("validation_1", 0.02),),
        activation_max_rmse_m=0.025,
        activation_max_error_m=0.04,
        active=True,
        activation_message="ok",
        created_at="2026-08-05T00:00:00+00:00",
    )


class FakeWorkspace:
    class Aabb:
        from rosclaw_mini.safety.limits import AxisLimits

        x = AxisLimits(0.0, 1.0)
        y = AxisLimits(0.0, 1.0)
        z = AxisLimits(0.0, 1.0)

        def validate_position(self, x, y, z):
            return x, y, z

    endpoint_aabb = Aabb()

    def validate_position(self, x, y, z):
        return x, y, z


def test_preview_cli_requires_upload_acknowledgement(tmp_path):
    outputs = []
    code = main(
        [
            "--serial",
            "serial-1",
            "--question",
            "red box",
            "--eye-to-hand-calibration",
            str(tmp_path / "unused.json"),
        ],
        environ={"DASHSCOPE_API_KEY": "test-key"},
        output_func=outputs.append,
    )
    assert code == 2
    assert "显式确认" in outputs[-1]


def test_preview_cli_outputs_safe_plan_without_runtime_or_motion(tmp_path):
    path = tmp_path / "calibration.json"
    write_eye_to_hand_calibration(calibration(), path)
    outputs = []

    code = main(
        [
            "--serial",
            "serial-1",
            "--question",
            "red box",
            "--eye-to-hand-calibration",
            str(path),
            "--acknowledge-camera-cloud-upload",
        ],
        environ={"DASHSCOPE_API_KEY": "test-key"},
        client_builder=lambda **_kwargs: object(),
        camera_builder=lambda *_args, **_kwargs: object(),
        service_builder=FakeService,
        workspace_loader=FakeWorkspace,
        now_ms=lambda: 1000.0,
        output_func=outputs.append,
    )

    assert code == 0
    payload = json.loads(outputs[0])
    assert payload["is_safe"] is True
    assert payload["execution_enabled"] is False
    assert [step["stage"] for step in payload["plan"]["steps"]] == [
        "open_gripper",
        "pre_grasp",
        "approach",
        "close_gripper",
        "lift",
    ]
    assert "未创建 Runtime" in outputs[-1]
