from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from rosclaw_mini.arm.so100_plus_session import SO100PlusPoseSnapshot
from rosclaw_mini.vision.exceptions import EyeToHandCalibrationError
from rosclaw_mini.vision.eye_to_hand import load_eye_to_hand_dataset
from scripts.capture_realsense_eye_to_hand_point import (
    CapturedEyeToHandPoint,
    capture_synchronized_eye_to_hand_point,
    main,
    validate_readonly_point_snapshot,
)


def snapshot(
    *,
    joint_radians=(0.0,) * 6,
    torque_enabled=(0,) * 7,
    tcp_position_m=(0.35, 0.0, 0.22),
):
    return SO100PlusPoseSnapshot(
        driver_degrees=(0.0,) * 6,
        joint_radians=joint_radians,
        tcp_position_m=tcp_position_m,
        gripper_driver_degrees=20.0,
        torque_enabled=torque_enabled,
    )


class FakeLocalizationService:
    def __init__(self, events):
        self.events = events

    def locate(self, question):
        self.events.append(("locate", question))
        return SimpleNamespace(
            observation=SimpleNamespace(observation_id="obs-1"),
            position=SimpleNamespace(
                camera_point_m=(0.05, 0.02, 0.42),
                source_frame=17,
                uncertainty_m=0.006,
            ),
        )


def test_synchronized_point_capture_reads_pose_before_and_after_localization():
    events = []
    poses = iter(
        (
            snapshot(),
            snapshot(joint_radians=(0.001,) + (0.0,) * 5),
        )
    )

    def pose_reader():
        events.append("read_pose")
        return next(poses)

    def pose_validator(value):
        events.append(("validate", value.joint_radians))

    captured = capture_synchronized_eye_to_hand_point(
        question="定位红色标定点",
        localization_service=FakeLocalizationService(events),
        pose_reader=pose_reader,
        pose_validator=pose_validator,
    )

    assert events == [
        "read_pose",
        ("validate", (0.0,) * 6),
        ("locate", "定位红色标定点"),
        "read_pose",
        ("validate", (0.001,) + (0.0,) * 5),
    ]
    assert captured.camera_point_m == (0.05, 0.02, 0.42)
    assert captured.base_point_m == (0.35, 0.0, 0.22)
    assert captured.observation_id == "obs-1"
    assert captured.source_frame == 17


def test_synchronized_point_capture_rejects_joint_drift():
    poses = iter(
        (
            snapshot(),
            snapshot(joint_radians=(math.radians(1.1),) + (0.0,) * 5),
        )
    )
    with pytest.raises(EyeToHandCalibrationError, match="最大关节漂移"):
        capture_synchronized_eye_to_hand_point(
            question="定位标定点",
            localization_service=FakeLocalizationService([]),
            pose_reader=lambda: next(poses),
            pose_validator=lambda _snapshot: None,
            max_joint_drift_degrees=1.0,
        )


def test_readonly_point_snapshot_requires_all_torque_disabled():
    with pytest.raises(EyeToHandCalibrationError, match="力矩全部关闭"):
        validate_readonly_point_snapshot(
            snapshot(torque_enabled=(0, 0, 1, 0, 0, 0, 0))
        )


def test_readonly_point_snapshot_accepts_finite_manual_pose_outside_motion_limits():
    manually_taught = snapshot(
        joint_radians=(0.0, -3.32, 2.8, 0.0, 0.0, 1.57),
        tcp_position_m=(0.19, -0.03, -0.01),
    )

    validate_readonly_point_snapshot(manually_taught)


def test_readonly_point_snapshot_rejects_nonfinite_feedback():
    with pytest.raises(EyeToHandCalibrationError, match="关节反馈"):
        validate_readonly_point_snapshot(
            snapshot(joint_radians=(0.0, float("nan"), 0.0, 0.0, 0.0, 0.0))
        )


def cli_args(dataset: Path):
    return [
        "--dataset",
        str(dataset),
        "--serial",
        "serial-1",
        "--question",
        "定位红色标定点",
        "--port",
        "/fake/lerobot_right",
        "--calibration-dir",
        "/fake/calibration",
        "--follower-name",
        "right",
    ]


def test_capture_cli_requires_both_readonly_and_cloud_acknowledgements(tmp_path):
    calls = []
    output = []
    assert main(cli_args(tmp_path / "points.json"), output_func=output.append) == 2
    assert "只读" in output[-1]

    assert (
        main(
            [
                *cli_args(tmp_path / "points.json"),
                "--acknowledge-readonly-arm-capture",
            ],
            environ={"ROSCLAW_LLM_API_KEY": "not-a-real-key"},
            point_capture=lambda *_args, **_kwargs: calls.append("capture"),
            output_func=output.append,
        )
        == 2
    )
    assert "上传" in output[-1]
    assert calls == []


def test_capture_cli_with_fakes_appends_dataset_without_hardware_or_network(tmp_path):
    dataset_path = tmp_path / "points.json"
    events = []

    def client_builder(**kwargs):
        events.append(("client", kwargs["model"]))
        return object()

    def point_capture(args, *, client):
        events.append(("capture", args.serial, client is not None))
        return CapturedEyeToHandPoint(
            camera_point_m=(0.05, 0.02, 0.42),
            base_point_m=(0.35, 0.0, 0.22),
            observation_id="obs-1",
            source_frame=17,
            uncertainty_m=0.006,
        )

    outputs = []
    result = main(
        [
            *cli_args(dataset_path),
            "--acknowledge-readonly-arm-capture",
            "--acknowledge-camera-cloud-upload",
        ],
        environ={"ROSCLAW_LLM_API_KEY": "not-a-real-key"},
        client_builder=client_builder,
        point_capture=point_capture,
        output_func=outputs.append,
    )

    assert result == 0
    assert events == [("client", "qwen-vl-plus"), ("capture", "serial-1", True)]
    dataset = load_eye_to_hand_dataset(dataset_path)
    assert dataset.points[0].camera_point_m == (0.05, 0.02, 0.42)
    assert dataset.points[0].base_point_m == (0.35, 0.0, 0.22)
    assert "没有发送电机写入" in outputs[-1]
