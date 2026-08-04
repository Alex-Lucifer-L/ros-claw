from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from rosclaw_mini.arm.so100_plus_session import SO100PlusPoseSnapshot
from rosclaw_mini.vision.calibration import (
    CameraCalibrationIdentity,
    CameraIntrinsicCalibration,
    SO100_PLUS_WRIST_CHECKERBOARD,
)
from rosclaw_mini.vision.exceptions import HandEyeCalibrationError
from rosclaw_mini.vision.hand_eye import CheckerboardPoseEstimate
from scripts.calibrate_so100_plus_hand_eye import build_parser as build_solver_parser
from scripts.collect_so100_plus_hand_eye_samples import (
    build_parser as build_collector_parser,
    capture_synchronized_hand_eye_sample,
    validate_readonly_hand_eye_snapshot,
    validate_readonly_hand_eye_torque_state,
)


def intrinsics():
    return CameraIntrinsicCalibration(
        camera_identity=CameraCalibrationIdentity(
            device="/dev/v4l/by-id/fake-wrist-camera",
            vendor_id="0c58",
            product_id="637a",
            serial="fake",
            width=640,
            height=480,
            pixel_format="YUYV",
        ),
        checkerboard=SO100_PLUS_WRIST_CHECKERBOARD,
        camera_matrix=(
            (520.0, 0.0, 320.0),
            (0.0, 520.0, 240.0),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        rms_reprojection_error_px=0.2,
        per_view_reprojection_errors_px=(0.2,),
        accepted_images=("view.jpg",),
        rejected_images=(),
        created_at="2026-08-05T00:00:00+00:00",
    )


def snapshot(*, joint_offset=0.0, torque=(0,) * 7):
    joints = (joint_offset, 0.0, 0.0, 0.0, 0.0, 0.0)
    return SO100PlusPoseSnapshot(
        driver_degrees=(0.0,) * 6,
        joint_radians=joints,
        tcp_position_m=(0.3, 0.0, 0.2),
        gripper_driver_degrees=0.0,
        torque_enabled=torque,
    )


class FakeTrajectoryValidator:
    def __init__(self):
        self.calls = []

    def gripper_driver_degrees_to_qpos(self, degrees):
        self.calls.append(("gripper", degrees))
        return 0.0

    def verify_collision_free_pose(self, joints, kinematics, *, gripper_qpos):
        self.calls.append(("pose", tuple(joints), kinematics, gripper_qpos))


class FakeKinematics:
    def __init__(self, events=None):
        self.events = events if events is not None else []

    def forward_transform(self, joints):
        self.events.append(("fk", tuple(joints)))
        matrix = np.eye(4)
        matrix[:3, 3] = (0.3, 0.0, 0.2)
        return matrix


class FakeCamera:
    def __init__(self, events):
        self.events = events

    def capture_frame(self):
        self.events.append(("capture",))
        return np.zeros((480, 640, 3), dtype=np.uint8)


class FakeProcessor:
    def __init__(self, events):
        self.events = events

    def save(self, frame, path):
        self.events.append(("save", path, frame.copy()))


def test_readonly_snapshot_requires_torque_off_and_runs_static_pose_check():
    validator = FakeTrajectoryValidator()
    kinematics = FakeKinematics()
    validate_readonly_hand_eye_snapshot(snapshot(), kinematics, validator)
    assert validator.calls[0] == ("gripper", 0.0)
    assert validator.calls[1][0] == "pose"

    with pytest.raises(HandEyeCalibrationError, match="力矩全部关闭"):
        validate_readonly_hand_eye_snapshot(
            snapshot(torque=(0, 0, 1, 0, 0, 0, 0)),
            kinematics,
            FakeTrajectoryValidator(),
        )


def test_readonly_startup_allows_storage_pose_but_still_requires_torque_off():
    storage_snapshot = SO100PlusPoseSnapshot(
        driver_degrees=(0.0,) * 6,
        joint_radians=(0.0, -3.37, 3.0, 0.0, 0.0, 1.57),
        tcp_position_m=(0.19, -0.03, -0.01),
        gripper_driver_degrees=0.0,
        torque_enabled=(0,) * 7,
    )

    validate_readonly_hand_eye_torque_state(storage_snapshot)

    with pytest.raises(HandEyeCalibrationError, match="shoulder_pitch_joint"):
        validate_readonly_hand_eye_snapshot(
            storage_snapshot,
            FakeKinematics(),
            FakeTrajectoryValidator(),
        )
    with pytest.raises(HandEyeCalibrationError, match="力矩全部关闭"):
        validate_readonly_hand_eye_torque_state(
            SO100PlusPoseSnapshot(
                **{
                    **storage_snapshot.__dict__,
                    "torque_enabled": (0, 0, 0, 1, 0, 0, 0),
                }
            )
        )


def test_synchronized_sample_is_read_capture_read_then_save(tmp_path: Path):
    events = []
    snapshots = iter((snapshot(), snapshot(joint_offset=0.001)))

    def pose_reader():
        events.append(("read_pose",))
        return next(snapshots)

    def pose_validator(value):
        events.append(("validate_pose", value.joint_radians))

    estimate = CheckerboardPoseEstimate(
        camera_from_target=tuple(tuple(row) for row in np.eye(4)),
        reprojection_error_px=0.3,
    )
    sample = capture_synchronized_hand_eye_sample(
        sample_index=1,
        camera=FakeCamera(events),
        pose_reader=pose_reader,
        pose_validator=pose_validator,
        kinematics=FakeKinematics(events),
        intrinsics=intrinsics(),
        image_path=tmp_path / "sample.jpg",
        image_processor=FakeProcessor(events),
        pose_estimator=lambda frame, calibration, **kwargs: estimate,
    )

    assert [event[0] for event in events] == [
        "read_pose",
        "validate_pose",
        "capture",
        "read_pose",
        "validate_pose",
        "fk",
        "save",
    ]
    assert sample.sample_id == "sample_001"
    assert sample.joint_radians == snapshot().joint_radians
    assert sample.reprojection_error_px == pytest.approx(0.3)


def test_synchronized_sample_rejects_drift_before_fk_or_save(tmp_path: Path):
    events = []
    snapshots = iter((snapshot(), snapshot(joint_offset=0.1)))

    with pytest.raises(HandEyeCalibrationError, match="没有保持静止"):
        capture_synchronized_hand_eye_sample(
            sample_index=1,
            camera=FakeCamera(events),
            pose_reader=lambda: next(snapshots),
            pose_validator=lambda value: None,
            kinematics=FakeKinematics(events),
            intrinsics=intrinsics(),
            image_path=tmp_path / "sample.jpg",
            image_processor=FakeProcessor(events),
            pose_estimator=lambda frame, calibration, **kwargs: (
                CheckerboardPoseEstimate(
                    camera_from_target=tuple(
                        tuple(row) for row in np.eye(4)
                    ),
                    reprojection_error_px=0.3,
                )
            ),
        )
    assert not any(event[0] in {"fk", "save"} for event in events)


def test_hand_eye_script_defaults_and_explicit_readonly_acknowledgement():
    collector = build_collector_parser().parse_args(
        [
            "--port", "/dev/lerobot_right",
            "--calibration-dir", "/tmp/calibration",
            "--follower-name", "right",
            "--camera-device", "/dev/v4l/by-id/fake-camera",
            "--camera-intrinsics", "/tmp/intrinsics.json",
            "--output-dir", "/tmp/hand-eye",
        ]
    )
    assert collector.count == 15
    assert collector.pixel_format == "YUYV"
    assert collector.acknowledge_readonly_hand_eye_capture is False

    solver = build_solver_parser().parse_args(
        [
            "--dataset", "/tmp/dataset.json",
            "--intrinsics", "/tmp/intrinsics.json",
            "--output", "/tmp/hand-eye.json",
        ]
    )
    assert solver.minimum_samples == 10
