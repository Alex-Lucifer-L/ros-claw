from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np
import pytest

from rosclaw_mini.vision.exceptions import EyeToHandCalibrationError
from rosclaw_mini.vision.eye_to_hand import (
    EYE_TO_HAND_BASE_FRAME,
    EYE_TO_HAND_CAMERA_FRAME,
    EYE_TO_HAND_UNITS,
    EyeToHandCalibration,
    EyeToHandDataset,
    EyeToHandPointPair,
    load_eye_to_hand_calibration,
    load_eye_to_hand_dataset,
    solve_eye_to_hand_calibration,
    transform_camera_point_to_base,
    write_eye_to_hand_calibration,
    write_eye_to_hand_dataset,
)


def transform():
    angle = math.radians(30.0)
    rotation = np.asarray(
        (
            (math.cos(angle), -math.sin(angle), 0.0),
            (math.sin(angle), math.cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = (0.25, -0.08, 0.04)
    return result


CAMERA_POINTS = (
    (0.00, 0.00, 0.45),
    (0.12, 0.00, 0.46),
    (0.00, 0.11, 0.44),
    (0.12, 0.11, 0.47),
    (0.04, 0.03, 0.58),
    (0.10, 0.07, 0.54),
    (0.02, 0.09, 0.51),
    (0.08, 0.01, 0.49),
    (0.06, 0.12, 0.56),
    (0.14, 0.05, 0.52),
)


def dataset(*, corrupt_index=None):
    expected = transform()
    points = []
    for index, camera_point in enumerate(CAMERA_POINTS):
        base_point = (
            expected[:3, :3] @ np.asarray(camera_point) + expected[:3, 3]
        )
        if index == corrupt_index:
            base_point = base_point + (0.12, -0.08, 0.05)
        points.append(
            EyeToHandPointPair(
                point_id=f"point_{index + 1:03d}",
                captured_at="2026-08-05T00:00:00+00:00",
                camera_point_m=tuple(camera_point),
                base_point_m=tuple(base_point),
                split="fit" if index < 8 else "validation",
            )
        )
    return EyeToHandDataset(
        camera_serial="serial-1",
        width=640,
        height=480,
        camera_frame=EYE_TO_HAND_CAMERA_FRAME,
        base_frame=EYE_TO_HAND_BASE_FRAME,
        units=EYE_TO_HAND_UNITS,
        points=tuple(points),
        created_at=datetime.now(timezone.utc).isoformat(),
    )


def test_eye_to_hand_solver_recovers_rigid_transform_and_holdout_metrics():
    result = solve_eye_to_hand_calibration(dataset())

    assert np.asarray(result.base_from_camera) == pytest.approx(
        transform(), abs=1e-10
    )
    assert result.fit_point_count == 8
    assert result.validation_point_count == 2
    assert result.fit_rmse_m < 1e-10
    assert result.validation_rmse_m is not None
    assert result.validation_rmse_m < 1e-10
    assert result.active
    rotation = np.asarray(result.base_from_camera)[:3, :3]
    assert rotation.T @ rotation == pytest.approx(np.eye(3), abs=1e-10)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_eye_to_hand_rejects_collinear_or_insufficient_points():
    source = dataset()
    with pytest.raises(EyeToHandCalibrationError, match="至少需要 6"):
        solve_eye_to_hand_calibration(
            EyeToHandDataset(**{**source.__dict__, "points": source.points[:3]})
        )

    line_points = tuple(
        EyeToHandPointPair(
            point_id=f"line_{index}",
            captured_at="now",
            camera_point_m=(index * 0.03, 0.0, 0.5),
            base_point_m=(index * 0.03 + 0.2, 0.0, 0.6),
        )
        for index in range(6)
    )
    with pytest.raises(EyeToHandCalibrationError, match="共线"):
        solve_eye_to_hand_calibration(
            EyeToHandDataset(**{**source.__dict__, "points": line_points})
        )


def test_excessive_error_produces_inactive_calibration_and_cannot_transform():
    result = solve_eye_to_hand_calibration(
        dataset(corrupt_index=2),
        activation_max_rmse_m=0.005,
        activation_max_error_m=0.01,
    )

    assert not result.active
    assert "超过激活阈值" in result.activation_message
    with pytest.raises(EyeToHandCalibrationError, match="未激活"):
        transform_camera_point_to_base((0.0, 0.0, 0.5), result)


def test_dataset_and_calibration_round_trip_hash_and_identity_binding(tmp_path: Path):
    source = dataset()
    dataset_path = tmp_path / "point_pairs.json"
    write_eye_to_hand_dataset(source, dataset_path)
    assert load_eye_to_hand_dataset(dataset_path) == source

    calibration = solve_eye_to_hand_calibration(source)
    calibration_path = tmp_path / "eye_to_hand.json"
    write_eye_to_hand_calibration(calibration, calibration_path)
    loaded = load_eye_to_hand_calibration(
        calibration_path,
        expected_camera_serial="serial-1",
        expected_width=640,
        expected_height=480,
    )
    assert loaded == calibration

    with pytest.raises(EyeToHandCalibrationError, match="设备不匹配"):
        load_eye_to_hand_calibration(
            calibration_path,
            expected_camera_serial="other",
            expected_width=640,
            expected_height=480,
        )
    with pytest.raises(EyeToHandCalibrationError, match="分辨率"):
        load_eye_to_hand_calibration(
            calibration_path,
            expected_camera_serial="serial-1",
            expected_width=1280,
            expected_height=720,
        )

    payload = json.loads(calibration_path.read_text(encoding="utf-8"))
    payload["T_base_from_camera"][0][3] += 0.01
    calibration_path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(EyeToHandCalibrationError, match="SHA-256 不匹配"):
        load_eye_to_hand_calibration(
            calibration_path,
            expected_camera_serial="serial-1",
            expected_width=640,
            expected_height=480,
        )


def test_camera_point_transform_uses_active_calibration():
    calibration = solve_eye_to_hand_calibration(dataset())
    actual = transform_camera_point_to_base(CAMERA_POINTS[0], calibration)
    expected = (
        transform()[:3, :3] @ np.asarray(CAMERA_POINTS[0])
        + transform()[:3, 3]
    )
    assert actual == pytest.approx(expected)


def test_invalid_rotation_matrix_is_rejected():
    calibration = solve_eye_to_hand_calibration(dataset())
    invalid = [list(row) for row in calibration.base_from_camera]
    invalid[0][0] = 2.0
    with pytest.raises(EyeToHandCalibrationError, match="不正交"):
        EyeToHandCalibration(
            **{**calibration.__dict__, "base_from_camera": tuple(tuple(row) for row in invalid)}
        )
