"""Offline checkerboard intrinsic calibration for the wrist RGB camera."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import hmac
import json
import math
from pathlib import Path
from typing import Any, Sequence

from rosclaw_mini.vision.exceptions import (
    CameraCalibrationError,
    CheckerboardDetectionError,
    InsufficientCalibrationDataError,
)


SO100_PLUS_WRIST_CHECKERBOARD_INNER_COLUMNS = 7
SO100_PLUS_WRIST_CHECKERBOARD_INNER_ROWS = 6
SO100_PLUS_WRIST_CHECKERBOARD_SQUARE_SIZE_M = 0.024
DEFAULT_MINIMUM_CALIBRATION_VIEWS = 10


def _import_cv2_and_numpy():
    try:
        import cv2
        import numpy as np
    except (ImportError, OSError) as error:
        raise CameraCalibrationError(
            "相机标定需要 OpenCV 和 NumPy。"
        ) from error
    return cv2, np


@dataclass(frozen=True)
class CheckerboardSpec:
    inner_columns: int = SO100_PLUS_WRIST_CHECKERBOARD_INNER_COLUMNS
    inner_rows: int = SO100_PLUS_WRIST_CHECKERBOARD_INNER_ROWS
    square_size_m: float = SO100_PLUS_WRIST_CHECKERBOARD_SQUARE_SIZE_M

    def __post_init__(self) -> None:
        for name, value in (
            ("inner_columns", self.inner_columns),
            ("inner_rows", self.inner_rows),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 2:
                raise CameraCalibrationError(f"{name} 必须是至少为 2 的整数。")
        if (
            isinstance(self.square_size_m, bool)
            or not isinstance(self.square_size_m, (int, float))
            or not math.isfinite(float(self.square_size_m))
            or float(self.square_size_m) <= 0
        ):
            raise CameraCalibrationError("square_size_m 必须是有限正数。")
        object.__setattr__(self, "square_size_m", float(self.square_size_m))

    @property
    def pattern_size(self) -> tuple[int, int]:
        return self.inner_columns, self.inner_rows

    @property
    def square_columns(self) -> int:
        return self.inner_columns + 1

    @property
    def square_rows(self) -> int:
        return self.inner_rows + 1

    @property
    def board_width_m(self) -> float:
        return self.square_columns * self.square_size_m

    @property
    def board_height_m(self) -> float:
        return self.square_rows * self.square_size_m

    def to_dict(self) -> dict[str, Any]:
        return {
            "inner_columns": self.inner_columns,
            "inner_rows": self.inner_rows,
            "square_size_m": self.square_size_m,
            "square_columns": self.square_columns,
            "square_rows": self.square_rows,
            "board_width_m": self.board_width_m,
            "board_height_m": self.board_height_m,
        }


SO100_PLUS_WRIST_CHECKERBOARD = CheckerboardSpec()


@dataclass(frozen=True)
class CameraCalibrationIdentity:
    device: str
    vendor_id: str
    product_id: str
    serial: str
    width: int
    height: int
    pixel_format: str

    def __post_init__(self) -> None:
        for field_name in ("device", "vendor_id", "product_id", "serial", "pixel_format"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value.strip():
                raise CameraCalibrationError(f"camera_identity.{field_name} 不能为空。")
            object.__setattr__(self, field_name, value.strip())
        for field_name in ("width", "height"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise CameraCalibrationError(
                    f"camera_identity.{field_name} 必须是正整数。"
                )

    def to_dict(self) -> dict[str, Any]:
        return {
            "device": self.device,
            "vendor_id": self.vendor_id,
            "product_id": self.product_id,
            "serial": self.serial,
            "width": self.width,
            "height": self.height,
            "pixel_format": self.pixel_format,
        }


@dataclass(frozen=True)
class CameraIntrinsicCalibration:
    camera_identity: CameraCalibrationIdentity
    checkerboard: CheckerboardSpec
    camera_matrix: tuple[tuple[float, float, float], ...]
    distortion_coefficients: tuple[float, ...]
    rms_reprojection_error_px: float
    per_view_reprojection_errors_px: tuple[float, ...]
    accepted_images: tuple[str, ...]
    rejected_images: tuple[str, ...]
    created_at: str

    def __post_init__(self) -> None:
        if len(self.camera_matrix) != 3 or any(
            len(row) != 3 for row in self.camera_matrix
        ):
            raise CameraCalibrationError("camera_matrix 必须是 3×3。")
        numeric_values = [
            value for row in self.camera_matrix for value in row
        ] + list(self.distortion_coefficients) + [
            self.rms_reprojection_error_px,
            *self.per_view_reprojection_errors_px,
        ]
        if not numeric_values or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in numeric_values
        ):
            raise CameraCalibrationError("标定结果必须全部是有限数值。")
        if self.camera_matrix[0][0] <= 0 or self.camera_matrix[1][1] <= 0:
            raise CameraCalibrationError("相机焦距 fx/fy 必须为正数。")
        if self.rms_reprojection_error_px < 0 or any(
            value < 0 for value in self.per_view_reprojection_errors_px
        ):
            raise CameraCalibrationError("重投影误差不能为负数。")
        if len(self.accepted_images) != len(self.per_view_reprojection_errors_px):
            raise CameraCalibrationError("逐图误差数量必须与有效图片数量一致。")

    def _payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "camera_identity": self.camera_identity.to_dict(),
            "checkerboard": self.checkerboard.to_dict(),
            "camera_matrix": [list(row) for row in self.camera_matrix],
            "distortion_coefficients": list(self.distortion_coefficients),
            "rms_reprojection_error_px": self.rms_reprojection_error_px,
            "per_view_reprojection_errors_px": list(
                self.per_view_reprojection_errors_px
            ),
            "accepted_images": list(self.accepted_images),
            "rejected_images": list(self.rejected_images),
            "created_at": self.created_at,
        }

    @property
    def calibration_sha256(self) -> str:
        canonical = json.dumps(
            self._payload_without_hash(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def to_dict(self) -> dict[str, Any]:
        payload = self._payload_without_hash()
        payload["calibration_sha256"] = self.calibration_sha256
        return payload


def detect_checkerboard_corners(
    image,
    spec: CheckerboardSpec = SO100_PLUS_WRIST_CHECKERBOARD,
    *,
    cv2_module=None,
):
    cv2, _np = _import_cv2_and_numpy()
    if cv2_module is not None:
        cv2 = cv2_module
    shape = getattr(image, "shape", None)
    if not isinstance(shape, tuple) or len(shape) < 2:
        raise CheckerboardDetectionError("标定图像没有有效 shape。")
    gray = (
        image
        if len(shape) == 2
        else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    )
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    success, corners = cv2.findChessboardCornersSB(
        gray,
        spec.pattern_size,
        flags=flags,
    )
    expected_count = spec.inner_columns * spec.inner_rows
    if not success or corners is None or len(corners) != expected_count:
        raise CheckerboardDetectionError(
            "未检测到完整棋盘格：期望 "
            f"{spec.inner_columns}×{spec.inner_rows}={expected_count} 个内角点。"
        )
    return corners


def _object_points(spec: CheckerboardSpec, np_module):
    points = np_module.zeros(
        (spec.inner_columns * spec.inner_rows, 3),
        dtype=np_module.float32,
    )
    points[:, :2] = (
        np_module.mgrid[
            0 : spec.inner_columns,
            0 : spec.inner_rows,
        ].T.reshape(-1, 2)
        * spec.square_size_m
    )
    return points


def calibrate_camera_intrinsics(
    image_paths: Sequence[Path],
    *,
    camera_identity: CameraCalibrationIdentity,
    checkerboard: CheckerboardSpec = SO100_PLUS_WRIST_CHECKERBOARD,
    minimum_views: int = DEFAULT_MINIMUM_CALIBRATION_VIEWS,
    cv2_module=None,
) -> CameraIntrinsicCalibration:
    if isinstance(minimum_views, bool) or not isinstance(minimum_views, int):
        raise CameraCalibrationError("minimum_views 必须是整数。")
    if minimum_views < 3:
        raise CameraCalibrationError("minimum_views 不能小于 3。")
    cv2, np = _import_cv2_and_numpy()
    if cv2_module is not None:
        cv2 = cv2_module

    object_template = _object_points(checkerboard, np)
    object_points = []
    image_points = []
    accepted: list[str] = []
    rejected: list[str] = []
    image_size = (camera_identity.width, camera_identity.height)

    for input_path in image_paths:
        path = Path(input_path)
        try:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        except Exception as error:
            rejected.append(f"{path}: 读取失败 ({error})")
            continue
        if image is None:
            rejected.append(f"{path}: 无法解码")
            continue
        actual_height, actual_width = image.shape[:2]
        if (actual_width, actual_height) != image_size:
            rejected.append(
                f"{path}: 分辨率 {actual_width}×{actual_height}，"
                f"期望 {image_size[0]}×{image_size[1]}"
            )
            continue
        try:
            corners = detect_checkerboard_corners(
                image,
                checkerboard,
                cv2_module=cv2,
            )
        except CheckerboardDetectionError as error:
            rejected.append(f"{path}: {error}")
            continue
        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted.append(str(path))

    if len(accepted) < minimum_views:
        raise InsufficientCalibrationDataError(
            f"有效标定图片只有 {len(accepted)} 张，至少需要 {minimum_views} 张；"
            f"拒绝 {len(rejected)} 张。"
        )

    try:
        rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
            object_points,
            image_points,
            image_size,
            None,
            None,
        )
    except Exception as error:
        raise CameraCalibrationError(f"OpenCV 相机内参求解失败：{error}") from error

    per_view_errors: list[float] = []
    for points_3d, points_2d, rotation, translation in zip(
        object_points,
        image_points,
        rotations,
        translations,
    ):
        projected, _jacobian = cv2.projectPoints(
            points_3d,
            rotation,
            translation,
            matrix,
            distortion,
        )
        difference = points_2d.reshape(-1, 2) - projected.reshape(-1, 2)
        error = float(np.sqrt(np.mean(np.sum(difference * difference, axis=1))))
        per_view_errors.append(error)

    matrix_tuple = tuple(
        tuple(float(value) for value in row)
        for row in np.asarray(matrix).reshape(3, 3)
    )
    distortion_tuple = tuple(
        float(value) for value in np.asarray(distortion).reshape(-1)
    )
    return CameraIntrinsicCalibration(
        camera_identity=camera_identity,
        checkerboard=checkerboard,
        camera_matrix=matrix_tuple,
        distortion_coefficients=distortion_tuple,
        rms_reprojection_error_px=float(rms),
        per_view_reprojection_errors_px=tuple(per_view_errors),
        accepted_images=tuple(accepted),
        rejected_images=tuple(rejected),
        created_at=datetime.now(timezone.utc).isoformat(),
    )


def write_camera_intrinsic_calibration(
    calibration: CameraIntrinsicCalibration,
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    path = Path(output_path)
    if path.exists() and not overwrite:
        raise CameraCalibrationError(
            f"标定输出已存在：{path}；请显式允许覆盖。"
        )
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = path.with_suffix(path.suffix + ".tmp")
        temporary_path.write_text(
            json.dumps(calibration.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(path)
    except OSError as error:
        raise CameraCalibrationError(f"写入标定文件失败：{path}") from error


def _require_mapping(payload: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise CameraCalibrationError(f"{field_name} 必须是 JSON 对象。")
    return payload


def _require_sequence(payload: Any, field_name: str) -> list[Any]:
    if not isinstance(payload, list):
        raise CameraCalibrationError(f"{field_name} 必须是 JSON 数组。")
    return payload


def _require_string(payload: Any, field_name: str) -> str:
    if not isinstance(payload, str) or not payload.strip():
        raise CameraCalibrationError(f"{field_name} 必须是非空字符串。")
    return payload.strip()


def _require_number(payload: Any, field_name: str) -> float:
    if (
        isinstance(payload, bool)
        or not isinstance(payload, (int, float))
        or not math.isfinite(float(payload))
    ):
        raise CameraCalibrationError(f"{field_name} 必须是有限数值。")
    return float(payload)


def _require_integer(payload: Any, field_name: str) -> int:
    if isinstance(payload, bool) or not isinstance(payload, int):
        raise CameraCalibrationError(f"{field_name} 必须是整数。")
    return payload


def load_camera_intrinsic_calibration(
    input_path: Path,
) -> CameraIntrinsicCalibration:
    """Load one calibration file and reject malformed or tampered content."""

    path = Path(input_path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise CameraCalibrationError(f"标定文件不存在：{path}") from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CameraCalibrationError(f"无法读取标定文件：{path}") from error

    root = _require_mapping(payload, "calibration")
    schema_version = _require_integer(
        root.get("schema_version"), "schema_version"
    )
    if schema_version != 1:
        raise CameraCalibrationError(
            f"不支持的标定 schema_version：{schema_version}。"
        )

    identity_payload = _require_mapping(
        root.get("camera_identity"), "camera_identity"
    )
    identity = CameraCalibrationIdentity(
        device=_require_string(
            identity_payload.get("device"), "camera_identity.device"
        ),
        vendor_id=_require_string(
            identity_payload.get("vendor_id"), "camera_identity.vendor_id"
        ),
        product_id=_require_string(
            identity_payload.get("product_id"), "camera_identity.product_id"
        ),
        serial=_require_string(
            identity_payload.get("serial"), "camera_identity.serial"
        ),
        width=_require_integer(
            identity_payload.get("width"), "camera_identity.width"
        ),
        height=_require_integer(
            identity_payload.get("height"), "camera_identity.height"
        ),
        pixel_format=_require_string(
            identity_payload.get("pixel_format"),
            "camera_identity.pixel_format",
        ),
    )

    checkerboard_payload = _require_mapping(
        root.get("checkerboard"), "checkerboard"
    )
    checkerboard = CheckerboardSpec(
        inner_columns=_require_integer(
            checkerboard_payload.get("inner_columns"),
            "checkerboard.inner_columns",
        ),
        inner_rows=_require_integer(
            checkerboard_payload.get("inner_rows"),
            "checkerboard.inner_rows",
        ),
        square_size_m=_require_number(
            checkerboard_payload.get("square_size_m"),
            "checkerboard.square_size_m",
        ),
    )
    derived_checkerboard_values = (
        (
            "square_columns",
            checkerboard_payload.get("square_columns"),
            checkerboard.square_columns,
        ),
        (
            "square_rows",
            checkerboard_payload.get("square_rows"),
            checkerboard.square_rows,
        ),
        (
            "board_width_m",
            checkerboard_payload.get("board_width_m"),
            checkerboard.board_width_m,
        ),
        (
            "board_height_m",
            checkerboard_payload.get("board_height_m"),
            checkerboard.board_height_m,
        ),
    )
    for field_name, actual, expected in derived_checkerboard_values:
        actual_number = _require_number(
            actual, f"checkerboard.{field_name}"
        )
        if not math.isclose(
            actual_number,
            float(expected),
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise CameraCalibrationError(
                f"checkerboard.{field_name} 与棋盘规格不一致。"
            )

    matrix_rows = _require_sequence(root.get("camera_matrix"), "camera_matrix")
    if len(matrix_rows) != 3:
        raise CameraCalibrationError("camera_matrix 必须是 3×3。")
    camera_matrix = []
    for row_index, raw_row in enumerate(matrix_rows):
        row = _require_sequence(raw_row, f"camera_matrix[{row_index}]")
        if len(row) != 3:
            raise CameraCalibrationError("camera_matrix 必须是 3×3。")
        camera_matrix.append(
            tuple(
                _require_number(value, f"camera_matrix[{row_index}][{column}]")
                for column, value in enumerate(row)
            )
        )

    distortion_payload = _require_sequence(
        root.get("distortion_coefficients"), "distortion_coefficients"
    )
    if not distortion_payload:
        raise CameraCalibrationError("distortion_coefficients 不能为空。")
    distortion = tuple(
        _require_number(value, f"distortion_coefficients[{index}]")
        for index, value in enumerate(distortion_payload)
    )

    per_view_payload = _require_sequence(
        root.get("per_view_reprojection_errors_px"),
        "per_view_reprojection_errors_px",
    )
    accepted_payload = _require_sequence(
        root.get("accepted_images"), "accepted_images"
    )
    rejected_payload = _require_sequence(
        root.get("rejected_images"), "rejected_images"
    )
    calibration = CameraIntrinsicCalibration(
        camera_identity=identity,
        checkerboard=checkerboard,
        camera_matrix=tuple(camera_matrix),
        distortion_coefficients=distortion,
        rms_reprojection_error_px=_require_number(
            root.get("rms_reprojection_error_px"),
            "rms_reprojection_error_px",
        ),
        per_view_reprojection_errors_px=tuple(
            _require_number(
                value, f"per_view_reprojection_errors_px[{index}]"
            )
            for index, value in enumerate(per_view_payload)
        ),
        accepted_images=tuple(
            _require_string(value, f"accepted_images[{index}]")
            for index, value in enumerate(accepted_payload)
        ),
        rejected_images=tuple(
            _require_string(value, f"rejected_images[{index}]")
            for index, value in enumerate(rejected_payload)
        ),
        created_at=_require_string(root.get("created_at"), "created_at"),
    )
    claimed_hash = _require_string(
        root.get("calibration_sha256"), "calibration_sha256"
    ).lower()
    if not hmac.compare_digest(claimed_hash, calibration.calibration_sha256):
        raise CameraCalibrationError(
            "标定文件 SHA-256 不匹配；文件可能已损坏或被修改。"
        )
    return calibration


def validate_camera_intrinsic_binding(
    calibration: CameraIntrinsicCalibration,
    *,
    device: str | Path,
    width: int,
    height: int,
    pixel_format: str | None = None,
) -> None:
    """Fail closed when a calibration is used with another camera or size."""

    actual_device = str(device)
    expected = calibration.camera_identity
    if actual_device != expected.device:
        raise CameraCalibrationError(
            "标定文件绑定的摄像头与当前设备不一致："
            f"期望 {expected.device}，实际 {actual_device}。"
        )
    if (width, height) != (expected.width, expected.height):
        raise CameraCalibrationError(
            "标定分辨率与当前画面不一致："
            f"期望 {expected.width}×{expected.height}，"
            f"实际 {width}×{height}。"
        )
    if pixel_format is not None:
        actual_pixel_format = _require_string(pixel_format, "pixel_format")
        if actual_pixel_format.upper() != expected.pixel_format.upper():
            raise CameraCalibrationError(
                "标定像素格式与当前采集配置不一致："
                f"期望 {expected.pixel_format}，"
                f"实际 {actual_pixel_format}。"
            )


def undistort_camera_frame(
    frame,
    calibration: CameraIntrinsicCalibration,
    *,
    alpha: float = 1.0,
    cv2_module=None,
):
    """Return a full-size corrected frame using the authenticated intrinsics."""

    alpha_value = _require_number(alpha, "alpha")
    if not 0.0 <= alpha_value <= 1.0:
        raise CameraCalibrationError("alpha 必须介于 0.0 和 1.0 之间。")
    shape = getattr(frame, "shape", None)
    if not isinstance(shape, tuple) or len(shape) < 2:
        raise CameraCalibrationError("待校正图像没有有效 shape。")
    height, width = shape[:2]
    validate_camera_intrinsic_binding(
        calibration,
        device=calibration.camera_identity.device,
        width=width,
        height=height,
    )

    cv2, np = _import_cv2_and_numpy()
    if cv2_module is not None:
        cv2 = cv2_module
    matrix = np.asarray(calibration.camera_matrix, dtype=np.float64)
    distortion = np.asarray(
        calibration.distortion_coefficients, dtype=np.float64
    )
    image_size = (width, height)
    try:
        corrected_matrix, _roi = cv2.getOptimalNewCameraMatrix(
            matrix,
            distortion,
            image_size,
            alpha_value,
            image_size,
        )
        corrected = cv2.undistort(
            frame,
            matrix,
            distortion,
            None,
            corrected_matrix,
        )
    except Exception as error:
        raise CameraCalibrationError(f"图像去畸变失败：{error}") from error
    corrected_shape = getattr(corrected, "shape", None)
    if corrected_shape is None or corrected_shape[:2] != (height, width):
        raise CameraCalibrationError("去畸变输出分辨率异常。")
    return corrected


def measure_checkerboard_reprojection_error(
    frame,
    calibration: CameraIntrinsicCalibration,
    *,
    cv2_module=None,
) -> float:
    """Measure one new checkerboard view without changing the calibration."""

    shape = getattr(frame, "shape", None)
    if not isinstance(shape, tuple) or len(shape) < 2:
        raise CameraCalibrationError("重投影检查图像没有有效 shape。")
    height, width = shape[:2]
    validate_camera_intrinsic_binding(
        calibration,
        device=calibration.camera_identity.device,
        width=width,
        height=height,
    )
    cv2, np = _import_cv2_and_numpy()
    if cv2_module is not None:
        cv2 = cv2_module
    corners = detect_checkerboard_corners(
        frame,
        calibration.checkerboard,
        cv2_module=cv2,
    )
    object_points = _object_points(calibration.checkerboard, np)
    matrix = np.asarray(calibration.camera_matrix, dtype=np.float64)
    distortion = np.asarray(
        calibration.distortion_coefficients, dtype=np.float64
    )
    try:
        success, rotation, translation = cv2.solvePnP(
            object_points,
            corners,
            matrix,
            distortion,
        )
        if not success:
            raise CameraCalibrationError("新棋盘视角的位姿求解失败。")
        projected, _jacobian = cv2.projectPoints(
            object_points,
            rotation,
            translation,
            matrix,
            distortion,
        )
    except CameraCalibrationError:
        raise
    except Exception as error:
        raise CameraCalibrationError(
            f"新棋盘视角的重投影检查失败：{error}"
        ) from error
    difference = corners.reshape(-1, 2) - projected.reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum(difference * difference, axis=1))))
    if not math.isfinite(error):
        raise CameraCalibrationError("新视角重投影误差不是有限数值。")
    return error
