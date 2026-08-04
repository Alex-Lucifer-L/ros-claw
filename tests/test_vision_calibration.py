from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

import rosclaw_mini.vision.calibration as calibration_module
from rosclaw_mini.vision.calibration import (
    CameraCalibrationIdentity,
    CameraIntrinsicCalibration,
    CheckerboardSpec,
    SO100_PLUS_WRIST_CHECKERBOARD,
    calibrate_camera_intrinsics,
    detect_checkerboard_corners,
    load_camera_intrinsic_calibration,
    undistort_camera_frame,
    validate_camera_intrinsic_binding,
    write_camera_intrinsic_calibration,
)
from rosclaw_mini.vision.exceptions import (
    CameraCalibrationError,
    CheckerboardDetectionError,
    InsufficientCalibrationDataError,
)


def make_checkerboard_image(square_pixels=48, margin=40):
    spec = SO100_PLUS_WRIST_CHECKERBOARD
    height = spec.square_rows * square_pixels + 2 * margin
    width = spec.square_columns * square_pixels + 2 * margin
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    for row in range(spec.square_rows):
        for column in range(spec.square_columns):
            if (row + column) % 2 == 0:
                start = (margin + column * square_pixels, margin + row * square_pixels)
                end = (
                    margin + (column + 1) * square_pixels,
                    margin + (row + 1) * square_pixels,
                )
                cv2.rectangle(image, start, end, (0, 0, 0), thickness=-1)
    return image


def camera_identity():
    return CameraCalibrationIdentity(
        device="/dev/v4l/by-id/wrist-camera-video-index0",
        vendor_id="0c58",
        product_id="637a",
        serial="fake-serial",
        width=640,
        height=480,
        pixel_format="YUYV",
    )


def intrinsic_calibration():
    return CameraIntrinsicCalibration(
        camera_identity=camera_identity(),
        checkerboard=SO100_PLUS_WRIST_CHECKERBOARD,
        camera_matrix=(
            (500.0, 0.0, 320.0),
            (0.0, 510.0, 240.0),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.01, -0.02, 0.0, 0.0, 0.0),
        rms_reprojection_error_px=0.25,
        per_view_reprojection_errors_px=(0.2,),
        accepted_images=("view.jpg",),
        rejected_images=(),
        created_at="2026-08-05T00:00:00+00:00",
    )


def test_wrist_checkerboard_uses_confirmed_24_mm_scale():
    spec = SO100_PLUS_WRIST_CHECKERBOARD
    assert spec.pattern_size == (7, 6)
    assert spec.square_size_m == pytest.approx(0.024)
    assert spec.board_width_m == pytest.approx(0.192)
    assert spec.board_height_m == pytest.approx(0.168)


def test_checkerboard_spec_rejects_invalid_scale():
    with pytest.raises(CameraCalibrationError, match="finite positive|有限正数"):
        CheckerboardSpec(square_size_m=float("nan"))


def test_detect_checkerboard_corners_finds_complete_7_by_6_pattern():
    corners = detect_checkerboard_corners(make_checkerboard_image())
    assert corners.shape == (42, 1, 2)


def test_detect_checkerboard_corners_rejects_incomplete_image():
    image = make_checkerboard_image()[:, :150]
    with pytest.raises(CheckerboardDetectionError, match="42 个内角点"):
        detect_checkerboard_corners(image)


class FakeCalibrationCV2:
    IMREAD_COLOR = 1

    def __init__(self, *, width=640, height=480):
        self.width = width
        self.height = height
        self.object_points = None

    def imread(self, _path, _mode):
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def calibrateCamera(self, object_points, image_points, image_size, _a, _b):
        self.object_points = object_points
        count = len(object_points)
        return (
            0.25,
            np.array([[500.0, 0.0, 320.0], [0.0, 510.0, 240.0], [0.0, 0.0, 1.0]]),
            np.array([[0.01, -0.02, 0.0, 0.0, 0.0]]),
            [np.zeros((3, 1)) for _ in range(count)],
            [np.zeros((3, 1)) for _ in range(count)],
        )

    def projectPoints(self, points, _rotation, _translation, _matrix, _distortion):
        return points[:, :2].reshape(-1, 1, 2), None


def test_calibration_uses_confirmed_metric_object_points(monkeypatch, tmp_path: Path):
    fake_cv2 = FakeCalibrationCV2()

    def fake_detect(_image, spec, **_kwargs):
        points = calibration_module._object_points(spec, np)
        return points[:, :2].reshape(-1, 1, 2)

    monkeypatch.setattr(calibration_module, "detect_checkerboard_corners", fake_detect)
    paths = tuple(tmp_path / f"view_{index}.jpg" for index in range(10))
    result = calibrate_camera_intrinsics(
        paths,
        camera_identity=camera_identity(),
        cv2_module=fake_cv2,
    )

    assert len(result.accepted_images) == 10
    assert result.rms_reprojection_error_px == pytest.approx(0.25)
    assert result.per_view_reprojection_errors_px == pytest.approx((0.0,) * 10)
    grid = fake_cv2.object_points[0]
    assert grid[1, 0] - grid[0, 0] == pytest.approx(0.024)
    assert grid[7, 1] - grid[0, 1] == pytest.approx(0.024)


def test_calibration_rejects_too_few_valid_views(monkeypatch, tmp_path: Path):
    fake_cv2 = FakeCalibrationCV2()
    monkeypatch.setattr(
        calibration_module,
        "detect_checkerboard_corners",
        lambda image, spec, **kwargs: np.zeros((42, 1, 2), dtype=np.float32),
    )
    with pytest.raises(InsufficientCalibrationDataError, match="至少需要 10"):
        calibrate_camera_intrinsics(
            (tmp_path / "only.jpg",),
            camera_identity=camera_identity(),
            cv2_module=fake_cv2,
        )


def test_calibration_rejects_mixed_resolution_before_solving(monkeypatch, tmp_path: Path):
    fake_cv2 = FakeCalibrationCV2(width=800, height=600)
    with pytest.raises(InsufficientCalibrationDataError, match="有效标定图片只有 0"):
        calibrate_camera_intrinsics(
            tuple(tmp_path / f"view_{index}.jpg" for index in range(10)),
            camera_identity=camera_identity(),
            cv2_module=fake_cv2,
        )
    assert fake_cv2.object_points is None


def test_calibration_file_contains_hash_and_refuses_implicit_overwrite(tmp_path: Path):
    result = intrinsic_calibration()
    output = tmp_path / "intrinsics.json"
    write_camera_intrinsic_calibration(result, output)
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["checkerboard"]["square_size_m"] == pytest.approx(0.024)
    assert payload["calibration_sha256"] == result.calibration_sha256
    with pytest.raises(CameraCalibrationError, match="显式允许覆盖"):
        write_camera_intrinsic_calibration(result, output)


def test_intrinsic_calibration_round_trip_verifies_sha256(tmp_path: Path):
    expected = intrinsic_calibration()
    output = tmp_path / "intrinsics.json"
    write_camera_intrinsic_calibration(expected, output)

    actual = load_camera_intrinsic_calibration(output)

    assert actual == expected
    assert actual.calibration_sha256 == expected.calibration_sha256


def test_intrinsic_calibration_loader_rejects_tampered_payload(tmp_path: Path):
    output = tmp_path / "intrinsics.json"
    write_camera_intrinsic_calibration(intrinsic_calibration(), output)
    payload = json.loads(output.read_text(encoding="utf-8"))
    payload["camera_matrix"][0][0] = 999.0
    output.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(CameraCalibrationError, match="SHA-256 不匹配"):
        load_camera_intrinsic_calibration(output)


def test_intrinsic_binding_rejects_other_device_resolution_and_pixel_format():
    calibration = intrinsic_calibration()
    with pytest.raises(CameraCalibrationError, match="摄像头与当前设备不一致"):
        validate_camera_intrinsic_binding(
            calibration,
            device="/dev/video9",
            width=640,
            height=480,
            pixel_format="YUYV",
        )
    with pytest.raises(CameraCalibrationError, match="分辨率"):
        validate_camera_intrinsic_binding(
            calibration,
            device=calibration.camera_identity.device,
            width=1280,
            height=720,
            pixel_format="YUYV",
        )
    with pytest.raises(CameraCalibrationError, match="像素格式"):
        validate_camera_intrinsic_binding(
            calibration,
            device=calibration.camera_identity.device,
            width=640,
            height=480,
            pixel_format="MJPG",
        )


class FakeUndistortCV2:
    def __init__(self):
        self.optimal_call = None
        self.undistort_call = None

    def getOptimalNewCameraMatrix(
        self, matrix, distortion, image_size, alpha, output_size
    ):
        self.optimal_call = (
            matrix.copy(),
            distortion.copy(),
            image_size,
            alpha,
            output_size,
        )
        return matrix.copy(), (0, 0, image_size[0], image_size[1])

    def undistort(self, frame, matrix, distortion, _unused, corrected_matrix):
        self.undistort_call = (
            frame,
            matrix.copy(),
            distortion.copy(),
            corrected_matrix.copy(),
        )
        return frame.copy()


def test_undistort_uses_loaded_matrix_and_preserves_resolution():
    calibration = intrinsic_calibration()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2 = FakeUndistortCV2()

    corrected = undistort_camera_frame(
        frame,
        calibration,
        alpha=1.0,
        cv2_module=cv2,
    )

    assert corrected.shape == frame.shape
    assert cv2.optimal_call[2:] == ((640, 480), 1.0, (640, 480))
    assert cv2.undistort_call is not None
    assert cv2.undistort_call[1] == pytest.approx(
        np.asarray(calibration.camera_matrix)
    )


def test_undistort_rejects_frame_from_another_resolution_before_opencv():
    cv2 = FakeUndistortCV2()
    with pytest.raises(CameraCalibrationError, match="分辨率"):
        undistort_camera_frame(
            np.zeros((720, 1280, 3), dtype=np.uint8),
            intrinsic_calibration(),
            cv2_module=cv2,
        )
    assert cv2.optimal_call is None
