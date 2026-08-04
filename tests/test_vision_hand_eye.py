from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

import rosclaw_mini.vision.hand_eye as hand_eye_module
from rosclaw_mini.vision.calibration import (
    CameraCalibrationIdentity,
    CameraIntrinsicCalibration,
    SO100_PLUS_WRIST_CHECKERBOARD,
)
from rosclaw_mini.vision.exceptions import HandEyeCalibrationError
from rosclaw_mini.vision.hand_eye import (
    HandEyeDataset,
    HandEyeSample,
    SO100_PLUS_HAND_EYE_KINEMATICS_MODEL,
    SO100_PLUS_HAND_EYE_REFERENCE_FRAME,
    estimate_checkerboard_pose,
    load_hand_eye_dataset,
    solve_hand_eye_calibration,
    write_hand_eye_dataset,
)


def make_transform(rotation_vector, translation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = cv2.Rodrigues(
        np.asarray(rotation_vector, dtype=float)
    )[0]
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def matrix_tuple(matrix):
    return tuple(tuple(float(value) for value in row) for row in matrix)


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
        accepted_images=("intrinsic.jpg",),
        rejected_images=(),
        created_at="2026-08-05T00:00:00+00:00",
    )


def synthetic_dataset():
    camera_calibration = intrinsics()
    tcp_from_camera = make_transform(
        (0.18, -0.11, 0.07),
        (0.025, -0.018, 0.061),
    )
    base_from_target = make_transform(
        (-0.08, 0.12, 0.04),
        (0.42, 0.03, 0.08),
    )
    samples = []
    for index in range(12):
        base_from_tcp = make_transform(
            (
                -0.30 + index * 0.055,
                0.20 * np.sin(index * 0.71),
                -0.18 * np.cos(index * 0.43),
            ),
            (
                0.25 + 0.012 * index,
                -0.04 + 0.008 * (index % 4),
                0.18 + 0.006 * (index % 5),
            ),
        )
        base_from_camera = base_from_tcp @ tcp_from_camera
        camera_from_target = np.linalg.inv(base_from_camera) @ base_from_target
        samples.append(
            HandEyeSample(
                sample_id=f"sample_{index + 1:03d}",
                captured_at="2026-08-05T00:00:00+00:00",
                image_path=f"sample_{index + 1:03d}.jpg",
                joint_radians=(0.0,) * 6,
                base_from_tcp=matrix_tuple(base_from_tcp),
                camera_from_target=matrix_tuple(camera_from_target),
                reprojection_error_px=0.2,
            )
        )
    dataset = HandEyeDataset(
        camera_device=camera_calibration.camera_identity.device,
        intrinsics_sha256=camera_calibration.calibration_sha256,
        robot_port="/dev/lerobot_right",
        robot_calibration_filename="right_follower.json",
        robot_calibration_sha256="a" * 64,
        reference_frame=SO100_PLUS_HAND_EYE_REFERENCE_FRAME,
        kinematics_model=SO100_PLUS_HAND_EYE_KINEMATICS_MODEL,
        tcp_offset_m=(0.10127, -0.00690, 0.00118),
        samples=tuple(samples),
        created_at=datetime.now(timezone.utc).isoformat(),
    )
    return camera_calibration, dataset, tcp_from_camera, base_from_target


def test_hand_eye_solver_recovers_known_tcp_t_camera():
    camera_calibration, dataset, expected_tcp_camera, expected_base_target = (
        synthetic_dataset()
    )

    result = solve_hand_eye_calibration(dataset, camera_calibration)

    assert np.asarray(result.tcp_from_camera) == pytest.approx(
        expected_tcp_camera,
        abs=1e-6,
    )
    assert np.asarray(result.base_from_target) == pytest.approx(
        expected_base_target,
        abs=1e-6,
    )
    assert result.translation_rms_m < 1e-8
    assert result.rotation_rms_degrees < 1e-5
    assert result.sample_count == 12
    assert result.checkerboard_flipped_sample_ids == ()


def test_hand_eye_solver_resolves_symmetric_board_half_turns():
    camera_calibration, dataset, expected_tcp_camera, expected_base_target = (
        synthetic_dataset()
    )
    checkerboard = camera_calibration.checkerboard
    half_turn = np.eye(4)
    half_turn[:3, :3] = np.diag((-1.0, -1.0, 1.0))
    half_turn[:3, 3] = (
        (checkerboard.inner_columns - 1) * checkerboard.square_size_m,
        (checkerboard.inner_rows - 1) * checkerboard.square_size_m,
        0.0,
    )
    flipped_indices = {3, 7, 10}
    flipped_samples = tuple(
        HandEyeSample(
            **{
                **sample.__dict__,
                "camera_from_target": matrix_tuple(
                    np.asarray(sample.camera_from_target) @ half_turn
                    if index in flipped_indices
                    else np.asarray(sample.camera_from_target)
                ),
            }
        )
        for index, sample in enumerate(dataset.samples)
    )
    ambiguous_dataset = HandEyeDataset(
        **{
            **dataset.__dict__,
            "samples": flipped_samples,
        }
    )

    result = solve_hand_eye_calibration(
        ambiguous_dataset,
        camera_calibration,
    )

    assert np.asarray(result.tcp_from_camera) == pytest.approx(
        expected_tcp_camera,
        abs=1e-6,
    )
    assert np.asarray(result.base_from_target) == pytest.approx(
        expected_base_target,
        abs=1e-6,
    )
    assert result.checkerboard_flipped_sample_ids == (
        "sample_004",
        "sample_008",
        "sample_011",
    )


def test_hand_eye_solver_rejects_insufficient_or_single_axis_motion():
    camera_calibration, dataset, _tcp_camera, _base_target = synthetic_dataset()
    with pytest.raises(HandEyeCalibrationError, match="至少需要 10 组"):
        solve_hand_eye_calibration(
            HandEyeDataset(
                **{
                    **dataset.__dict__,
                    "samples": dataset.samples[:3],
                }
            ),
            camera_calibration,
        )

    same_rotation_samples = tuple(
        HandEyeSample(
            **{
                **sample.__dict__,
                "base_from_tcp": matrix_tuple(
                    make_transform((0.0, 0.0, 0.0), (index * 0.01, 0.0, 0.0))
                ),
            }
        )
        for index, sample in enumerate(dataset.samples)
    )
    degenerate = HandEyeDataset(
        **{
            **dataset.__dict__,
            "samples": same_rotation_samples,
        }
    )
    with pytest.raises(HandEyeCalibrationError, match="旋转变化"):
        solve_hand_eye_calibration(degenerate, camera_calibration)


def test_hand_eye_dataset_round_trip_and_tamper_detection(tmp_path: Path):
    _camera_calibration, dataset, _tcp_camera, _base_target = synthetic_dataset()
    output = tmp_path / "dataset.json"
    write_hand_eye_dataset(dataset, output)
    assert load_hand_eye_dataset(output) == dataset

    payload = json.loads(output.read_text(encoding="utf-8"))
    payload["samples"][0]["joint_radians"][0] = 0.5
    output.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(HandEyeCalibrationError, match="SHA-256 不匹配"):
        load_hand_eye_dataset(output)


class FakePoseCV2:
    def solvePnP(self, object_points, corners, matrix, distortion):
        self.object_points = object_points
        self.corners = corners
        return (
            True,
            np.zeros((3, 1), dtype=float),
            np.asarray(((0.1,), (0.2,), (0.5,)), dtype=float),
        )

    def Rodrigues(self, _rotation_vector):
        return np.eye(3), None

    def projectPoints(
        self, object_points, rotation, translation, matrix, distortion
    ):
        return self.corners.copy(), None


def test_checkerboard_pose_uses_metric_board_and_returns_camera_t_target(
    monkeypatch,
):
    corners = np.zeros((42, 1, 2), dtype=np.float32)
    monkeypatch.setattr(
        hand_eye_module,
        "detect_checkerboard_corners",
        lambda frame, spec, **kwargs: corners,
    )
    fake_cv2 = FakePoseCV2()

    estimate = estimate_checkerboard_pose(
        np.zeros((480, 640, 3), dtype=np.uint8),
        intrinsics(),
        cv2_module=fake_cv2,
    )

    transform = np.asarray(estimate.camera_from_target)
    assert transform[:3, :3] == pytest.approx(np.eye(3))
    assert transform[:3, 3] == pytest.approx((0.1, 0.2, 0.5))
    assert estimate.reprojection_error_px == pytest.approx(0.0)
    assert fake_cv2.object_points[1, 0] == pytest.approx(0.024)
