"""Eye-to-hand point-pair calibration for a fixed external RGBD camera."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import hmac
import json
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from rosclaw_mini.vision.exceptions import EyeToHandCalibrationError


EYE_TO_HAND_SCHEMA_VERSION = 1
EYE_TO_HAND_CAMERA_FRAME = "realsense_color_optical_frame"
EYE_TO_HAND_BASE_FRAME = "so100_plus_base"
EYE_TO_HAND_UNITS = "m"
EYE_TO_HAND_SPLITS = frozenset({"fit", "validation"})


def _canonical_sha256(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _finite_vector(
    values: Sequence[float], *, length: int, label: str
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise EyeToHandCalibrationError(f"{label}需要 {length} 个有限数值。")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise EyeToHandCalibrationError(
            f"{label}需要 {length} 个有限数值。"
        ) from error
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise EyeToHandCalibrationError(f"{label}需要 {length} 个有限数值。")
    return result


def _positive_int(value: Any, *, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise EyeToHandCalibrationError(f"{label}必须是正整数。")
    return value


def _nonnegative_finite(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EyeToHandCalibrationError(f"{label}必须是有限非负数。")
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise EyeToHandCalibrationError(f"{label}必须是有限非负数。")
    return result


def _homogeneous_matrix(values: Any, *, label: str) -> np.ndarray:
    try:
        matrix = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as error:
        raise EyeToHandCalibrationError(f"{label}必须是有限 4×4 矩阵。") from error
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise EyeToHandCalibrationError(f"{label}必须是有限 4×4 矩阵。")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise EyeToHandCalibrationError(f"{label}的齐次末行无效。")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise EyeToHandCalibrationError(f"{label}的旋转矩阵不正交。")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise EyeToHandCalibrationError(f"{label}的旋转矩阵行列式不为 1。")
    return matrix.copy()


def _matrix_tuple(matrix: Any, *, label: str) -> tuple[tuple[float, ...], ...]:
    checked = _homogeneous_matrix(matrix, label=label)
    return tuple(tuple(float(value) for value in row) for row in checked)


def _nonempty_string(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise EyeToHandCalibrationError(f"{label}不能为空。")
    return value.strip()


@dataclass(frozen=True)
class EyeToHandPointPair:
    point_id: str
    captured_at: str
    camera_point_m: tuple[float, float, float]
    base_point_m: tuple[float, float, float]
    split: str = "fit"

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "point_id", _nonempty_string(self.point_id, label="point_id")
        )
        object.__setattr__(
            self,
            "captured_at",
            _nonempty_string(self.captured_at, label="captured_at"),
        )
        object.__setattr__(
            self,
            "camera_point_m",
            _finite_vector(self.camera_point_m, length=3, label="camera_point_m"),
        )
        object.__setattr__(
            self,
            "base_point_m",
            _finite_vector(self.base_point_m, length=3, label="base_point_m"),
        )
        if self.split not in EYE_TO_HAND_SPLITS:
            raise EyeToHandCalibrationError(
                f"split 必须是 {sorted(EYE_TO_HAND_SPLITS)} 之一。"
            )

    def to_dict(self) -> dict[str, Any]:
        return {
            "point_id": self.point_id,
            "captured_at": self.captured_at,
            "camera_point_m": list(self.camera_point_m),
            "base_point_m": list(self.base_point_m),
            "split": self.split,
        }


@dataclass(frozen=True)
class EyeToHandDataset:
    camera_serial: str
    width: int
    height: int
    camera_frame: str
    base_frame: str
    units: str
    points: tuple[EyeToHandPointPair, ...]
    created_at: str

    def __post_init__(self) -> None:
        for field_name in (
            "camera_serial",
            "camera_frame",
            "base_frame",
            "units",
            "created_at",
        ):
            object.__setattr__(
                self,
                field_name,
                _nonempty_string(getattr(self, field_name), label=field_name),
            )
        object.__setattr__(self, "width", _positive_int(self.width, label="width"))
        object.__setattr__(
            self, "height", _positive_int(self.height, label="height")
        )
        if self.units != EYE_TO_HAND_UNITS:
            raise EyeToHandCalibrationError("eye-to-hand 点对单位必须是 m。")
        point_ids = tuple(point.point_id for point in self.points)
        if len(set(point_ids)) != len(point_ids):
            raise EyeToHandCalibrationError("eye-to-hand point_id 重复。")

    def _payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": EYE_TO_HAND_SCHEMA_VERSION,
            "camera_serial": self.camera_serial,
            "width": self.width,
            "height": self.height,
            "camera_frame": self.camera_frame,
            "base_frame": self.base_frame,
            "units": self.units,
            "points": [point.to_dict() for point in self.points],
            "created_at": self.created_at,
        }

    @property
    def dataset_sha256(self) -> str:
        return _canonical_sha256(self._payload_without_hash())

    def to_dict(self) -> dict[str, Any]:
        payload = self._payload_without_hash()
        payload["dataset_sha256"] = self.dataset_sha256
        return payload


@dataclass(frozen=True)
class EyeToHandCalibration:
    camera_serial: str
    width: int
    height: int
    camera_frame: str
    base_frame: str
    units: str
    dataset_sha256: str
    method: str
    base_from_camera: tuple[tuple[float, ...], ...]
    fit_point_count: int
    validation_point_count: int
    fit_rmse_m: float
    fit_max_error_m: float
    validation_rmse_m: float | None
    validation_max_error_m: float | None
    per_point_error_m: tuple[tuple[str, float], ...]
    activation_max_rmse_m: float
    activation_max_error_m: float
    active: bool
    activation_message: str
    created_at: str

    def __post_init__(self) -> None:
        for field_name in (
            "camera_serial",
            "camera_frame",
            "base_frame",
            "units",
            "dataset_sha256",
            "method",
            "activation_message",
            "created_at",
        ):
            object.__setattr__(
                self,
                field_name,
                _nonempty_string(getattr(self, field_name), label=field_name),
            )
        object.__setattr__(self, "width", _positive_int(self.width, label="width"))
        object.__setattr__(
            self, "height", _positive_int(self.height, label="height")
        )
        object.__setattr__(
            self,
            "base_from_camera",
            _matrix_tuple(self.base_from_camera, label="T_base_from_camera"),
        )
        for field_name in ("fit_point_count", "validation_point_count"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise EyeToHandCalibrationError(f"{field_name}必须是非负整数。")
        for field_name in (
            "fit_rmse_m",
            "fit_max_error_m",
            "activation_max_rmse_m",
            "activation_max_error_m",
        ):
            object.__setattr__(
                self,
                field_name,
                _nonnegative_finite(getattr(self, field_name), label=field_name),
            )
        for field_name in ("validation_rmse_m", "validation_max_error_m"):
            value = getattr(self, field_name)
            if value is not None:
                object.__setattr__(
                    self,
                    field_name,
                    _nonnegative_finite(value, label=field_name),
                )
        checked_errors = []
        for point_id, error_m in self.per_point_error_m:
            checked_errors.append(
                (
                    _nonempty_string(point_id, label="per_point point_id"),
                    _nonnegative_finite(error_m, label="per_point error_m"),
                )
            )
        object.__setattr__(self, "per_point_error_m", tuple(checked_errors))
        if not isinstance(self.active, bool):
            raise EyeToHandCalibrationError("active 必须是布尔值。")

    def _payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": EYE_TO_HAND_SCHEMA_VERSION,
            "camera_serial": self.camera_serial,
            "width": self.width,
            "height": self.height,
            "camera_frame": self.camera_frame,
            "base_frame": self.base_frame,
            "units": self.units,
            "dataset_sha256": self.dataset_sha256,
            "method": self.method,
            "T_base_from_camera": [list(row) for row in self.base_from_camera],
            "fit_point_count": self.fit_point_count,
            "validation_point_count": self.validation_point_count,
            "fit_rmse_m": self.fit_rmse_m,
            "fit_max_error_m": self.fit_max_error_m,
            "validation_rmse_m": self.validation_rmse_m,
            "validation_max_error_m": self.validation_max_error_m,
            "per_point_error_m": {
                point_id: error_m for point_id, error_m in self.per_point_error_m
            },
            "activation_max_rmse_m": self.activation_max_rmse_m,
            "activation_max_error_m": self.activation_max_error_m,
            "active": self.active,
            "activation_message": self.activation_message,
            "created_at": self.created_at,
        }

    @property
    def calibration_sha256(self) -> str:
        return _canonical_sha256(self._payload_without_hash())

    def to_dict(self) -> dict[str, Any]:
        payload = self._payload_without_hash()
        payload["calibration_sha256"] = self.calibration_sha256
        return payload


def _validate_point_distribution(
    points: np.ndarray, *, label: str, minimum_spread_m: float
) -> None:
    centered = points - np.mean(points, axis=0)
    if np.linalg.matrix_rank(centered, tol=1e-8) < 2:
        raise EyeToHandCalibrationError(
            f"{label}点分布共线，至少需要分布合理的非共线点。"
        )
    pairwise = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
    if float(np.max(pairwise)) < minimum_spread_m:
        raise EyeToHandCalibrationError(
            f"{label}点覆盖范围不足 {minimum_spread_m:.3f} m。"
        )


def _fit_rigid_transform(camera_points: np.ndarray, base_points: np.ndarray) -> np.ndarray:
    camera_centroid = np.mean(camera_points, axis=0)
    base_centroid = np.mean(base_points, axis=0)
    centered_camera = camera_points - camera_centroid
    centered_base = base_points - base_centroid
    covariance = centered_camera.T @ centered_base
    left, _singular, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0:
        right[-1, :] *= -1
        rotation = right.T @ left.T
    translation = base_centroid - rotation @ camera_centroid
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return _homogeneous_matrix(transform, label="T_base_from_camera")


def _errors_for_points(
    transform: np.ndarray,
    points: Sequence[EyeToHandPointPair],
) -> tuple[float, ...]:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return tuple(
        float(
            np.linalg.norm(
                rotation @ np.asarray(point.camera_point_m)
                + translation
                - np.asarray(point.base_point_m)
            )
        )
        for point in points
    )


def _rms(errors: Sequence[float]) -> float:
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def solve_eye_to_hand_calibration(
    dataset: EyeToHandDataset,
    *,
    minimum_fit_points: int = 6,
    minimum_spread_m: float = 0.08,
    activation_max_rmse_m: float = 0.02,
    activation_max_error_m: float = 0.04,
) -> EyeToHandCalibration:
    if (
        isinstance(minimum_fit_points, bool)
        or not isinstance(minimum_fit_points, int)
        or minimum_fit_points < 3
    ):
        raise EyeToHandCalibrationError("minimum_fit_points 必须是至少 3 的整数。")
    for label, value in (
        ("minimum_spread_m", minimum_spread_m),
        ("activation_max_rmse_m", activation_max_rmse_m),
        ("activation_max_error_m", activation_max_error_m),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise EyeToHandCalibrationError(f"{label}必须是有限正数。")
    fit_points = tuple(point for point in dataset.points if point.split == "fit")
    validation_points = tuple(
        point for point in dataset.points if point.split == "validation"
    )
    if len(fit_points) < minimum_fit_points:
        raise EyeToHandCalibrationError(
            f"用于拟合的 eye-to-hand 点只有 {len(fit_points)} 组，"
            f"至少需要 {minimum_fit_points} 组。"
        )
    camera_points = np.asarray(
        [point.camera_point_m for point in fit_points], dtype=float
    )
    base_points = np.asarray([point.base_point_m for point in fit_points], dtype=float)
    _validate_point_distribution(
        camera_points,
        label="相机坐标",
        minimum_spread_m=float(minimum_spread_m),
    )
    _validate_point_distribution(
        base_points,
        label="基座坐标",
        minimum_spread_m=float(minimum_spread_m),
    )
    transform = _fit_rigid_transform(camera_points, base_points)
    fit_errors = _errors_for_points(transform, fit_points)
    validation_errors = _errors_for_points(transform, validation_points)
    fit_rmse = _rms(fit_errors)
    fit_max = max(fit_errors)
    validation_rmse = _rms(validation_errors) if validation_errors else None
    validation_max = max(validation_errors) if validation_errors else None
    all_rmse = validation_rmse if validation_rmse is not None else fit_rmse
    all_max = validation_max if validation_max is not None else fit_max
    active = (
        fit_rmse <= activation_max_rmse_m
        and fit_max <= activation_max_error_m
        and all_rmse <= activation_max_rmse_m
        and all_max <= activation_max_error_m
    )
    if active:
        activation_message = "标定误差通过配置阈值，可用于只读坐标验收。"
    else:
        activation_message = (
            "标定误差超过激活阈值，结果保持未激活："
            f"fit_rmse={fit_rmse:.6f} m, fit_max={fit_max:.6f} m, "
            f"validation_rmse={validation_rmse}, validation_max={validation_max}。"
        )
    per_point = tuple(
        (point.point_id, error)
        for point, error in zip(fit_points, fit_errors, strict=True)
    ) + tuple(
        (point.point_id, error)
        for point, error in zip(validation_points, validation_errors, strict=True)
    )
    return EyeToHandCalibration(
        camera_serial=dataset.camera_serial,
        width=dataset.width,
        height=dataset.height,
        camera_frame=dataset.camera_frame,
        base_frame=dataset.base_frame,
        units=dataset.units,
        dataset_sha256=dataset.dataset_sha256,
        method="kabsch_svd_point_pairs",
        base_from_camera=_matrix_tuple(transform, label="T_base_from_camera"),
        fit_point_count=len(fit_points),
        validation_point_count=len(validation_points),
        fit_rmse_m=fit_rmse,
        fit_max_error_m=fit_max,
        validation_rmse_m=validation_rmse,
        validation_max_error_m=validation_max,
        per_point_error_m=per_point,
        activation_max_rmse_m=float(activation_max_rmse_m),
        activation_max_error_m=float(activation_max_error_m),
        active=active,
        activation_message=activation_message,
        created_at=datetime.now(timezone.utc).isoformat(),
    )


def transform_camera_point_to_base(
    camera_point_m: Sequence[float],
    calibration: EyeToHandCalibration,
) -> tuple[float, float, float]:
    if not calibration.active:
        raise EyeToHandCalibrationError(
            "eye-to-hand 标定未激活，禁止转换到机械臂基座坐标。"
        )
    point = np.asarray(
        _finite_vector(camera_point_m, length=3, label="camera_point_m"),
        dtype=float,
    )
    transform = _homogeneous_matrix(
        calibration.base_from_camera,
        label="T_base_from_camera",
    )
    transformed = transform[:3, :3] @ point + transform[:3, 3]
    return tuple(float(value) for value in transformed)


def _write_json(
    payload: dict[str, Any],
    output_path: Path,
    *,
    overwrite: bool,
    label: str,
) -> None:
    path = Path(output_path)
    if path.exists() and not overwrite:
        raise EyeToHandCalibrationError(f"{label}输出已存在：{path}。")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    except OSError as error:
        raise EyeToHandCalibrationError(f"写入{label}失败：{path}。") from error


def write_eye_to_hand_dataset(
    dataset: EyeToHandDataset,
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    _write_json(
        dataset.to_dict(),
        output_path,
        overwrite=overwrite,
        label="eye-to-hand 数据集",
    )


def append_eye_to_hand_point(
    dataset_path: Path,
    *,
    camera_serial: str,
    width: int,
    height: int,
    camera_point_m: Sequence[float],
    base_point_m: Sequence[float],
    split: str = "fit",
    point_id: str | None = None,
    captured_at: str | None = None,
) -> EyeToHandDataset:
    """Append one identity-bound point pair and atomically update its hash."""

    path = Path(dataset_path)
    if path.exists():
        current = load_eye_to_hand_dataset(path)
        if current.camera_serial != camera_serial:
            raise EyeToHandCalibrationError(
                "现有数据集与指定相机序列号不匹配。"
            )
        if current.width != width or current.height != height:
            raise EyeToHandCalibrationError(
                "现有数据集与指定分辨率不匹配。"
            )
    else:
        current = EyeToHandDataset(
            camera_serial=camera_serial,
            width=width,
            height=height,
            camera_frame=EYE_TO_HAND_CAMERA_FRAME,
            base_frame=EYE_TO_HAND_BASE_FRAME,
            units=EYE_TO_HAND_UNITS,
            points=(),
            created_at=datetime.now(timezone.utc).isoformat(),
        )
    next_id = point_id or f"point_{len(current.points) + 1:03d}"
    point = EyeToHandPointPair(
        point_id=next_id,
        captured_at=captured_at or datetime.now(timezone.utc).isoformat(),
        camera_point_m=tuple(camera_point_m),
        base_point_m=tuple(base_point_m),
        split=split,
    )
    updated = EyeToHandDataset(
        **{**current.__dict__, "points": (*current.points, point)}
    )
    write_eye_to_hand_dataset(updated, path, overwrite=path.exists())
    return updated


def load_eye_to_hand_dataset(input_path: Path) -> EyeToHandDataset:
    path = Path(input_path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise EyeToHandCalibrationError(f"eye-to-hand 数据集不存在：{path}。") from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise EyeToHandCalibrationError(f"无法读取 eye-to-hand 数据集：{path}。") from error
    if not isinstance(payload, dict) or payload.get("schema_version") != EYE_TO_HAND_SCHEMA_VERSION:
        raise EyeToHandCalibrationError("eye-to-hand 数据集 schema_version 无效。")
    try:
        raw_points = payload["points"]
        if not isinstance(raw_points, list):
            raise EyeToHandCalibrationError("eye-to-hand points 必须是数组。")
        dataset = EyeToHandDataset(
            camera_serial=payload["camera_serial"],
            width=payload["width"],
            height=payload["height"],
            camera_frame=payload["camera_frame"],
            base_frame=payload["base_frame"],
            units=payload["units"],
            points=tuple(
                EyeToHandPointPair(
                    point_id=item["point_id"],
                    captured_at=item["captured_at"],
                    camera_point_m=tuple(item["camera_point_m"]),
                    base_point_m=tuple(item["base_point_m"]),
                    split=item["split"],
                )
                for item in raw_points
            ),
            created_at=payload["created_at"],
        )
    except KeyError as error:
        raise EyeToHandCalibrationError(
            f"eye-to-hand 数据集缺少字段：{error.args[0]}。"
        ) from error
    except (TypeError, ValueError) as error:
        raise EyeToHandCalibrationError("eye-to-hand 数据集字段类型无效。") from error
    claimed = payload.get("dataset_sha256")
    if not isinstance(claimed, str) or not hmac.compare_digest(
        claimed.lower(), dataset.dataset_sha256
    ):
        raise EyeToHandCalibrationError(
            "eye-to-hand 数据集 SHA-256 不匹配；文件可能已被修改。"
        )
    return dataset


def write_eye_to_hand_calibration(
    calibration: EyeToHandCalibration,
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    _write_json(
        calibration.to_dict(),
        output_path,
        overwrite=overwrite,
        label="eye-to-hand 标定",
    )


def load_eye_to_hand_calibration(
    input_path: Path,
    *,
    expected_camera_serial: str,
    expected_width: int,
    expected_height: int,
    require_active: bool = True,
) -> EyeToHandCalibration:
    path = Path(input_path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise EyeToHandCalibrationError(f"eye-to-hand 标定不存在：{path}。") from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise EyeToHandCalibrationError(f"无法读取 eye-to-hand 标定：{path}。") from error
    if not isinstance(payload, dict) or payload.get("schema_version") != EYE_TO_HAND_SCHEMA_VERSION:
        raise EyeToHandCalibrationError("eye-to-hand 标定 schema_version 无效。")
    try:
        raw_errors = payload["per_point_error_m"]
        if not isinstance(raw_errors, dict):
            raise EyeToHandCalibrationError("per_point_error_m 必须是对象。")
        calibration = EyeToHandCalibration(
            camera_serial=payload["camera_serial"],
            width=payload["width"],
            height=payload["height"],
            camera_frame=payload["camera_frame"],
            base_frame=payload["base_frame"],
            units=payload["units"],
            dataset_sha256=payload["dataset_sha256"],
            method=payload["method"],
            base_from_camera=tuple(
                tuple(row) for row in payload["T_base_from_camera"]
            ),
            fit_point_count=payload["fit_point_count"],
            validation_point_count=payload["validation_point_count"],
            fit_rmse_m=payload["fit_rmse_m"],
            fit_max_error_m=payload["fit_max_error_m"],
            validation_rmse_m=payload["validation_rmse_m"],
            validation_max_error_m=payload["validation_max_error_m"],
            per_point_error_m=tuple(raw_errors.items()),
            activation_max_rmse_m=payload["activation_max_rmse_m"],
            activation_max_error_m=payload["activation_max_error_m"],
            active=payload["active"],
            activation_message=payload["activation_message"],
            created_at=payload["created_at"],
        )
    except KeyError as error:
        raise EyeToHandCalibrationError(
            f"eye-to-hand 标定缺少字段：{error.args[0]}。"
        ) from error
    except (TypeError, ValueError) as error:
        raise EyeToHandCalibrationError("eye-to-hand 标定字段类型无效。") from error
    claimed = payload.get("calibration_sha256")
    if not isinstance(claimed, str) or not hmac.compare_digest(
        claimed.lower(), calibration.calibration_sha256
    ):
        raise EyeToHandCalibrationError(
            "eye-to-hand 标定 SHA-256 不匹配；文件可能已被修改。"
        )
    expected_serial = _nonempty_string(
        expected_camera_serial, label="expected_camera_serial"
    )
    if calibration.camera_serial != expected_serial:
        raise EyeToHandCalibrationError(
            "eye-to-hand 标定设备不匹配："
            f"期望 {expected_serial}，文件为 {calibration.camera_serial}。"
        )
    if (
        calibration.width != _positive_int(expected_width, label="expected_width")
        or calibration.height
        != _positive_int(expected_height, label="expected_height")
    ):
        raise EyeToHandCalibrationError(
            "eye-to-hand 标定分辨率与当前 Color 流不匹配。"
        )
    if require_active and not calibration.active:
        raise EyeToHandCalibrationError(
            "eye-to-hand 标定未激活：" + calibration.activation_message
        )
    return calibration
