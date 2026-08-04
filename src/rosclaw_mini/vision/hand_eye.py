"""Eye-in-hand calibration using the project's existing TCP kinematics."""

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

from rosclaw_mini.vision.calibration import (
    CameraIntrinsicCalibration,
    _object_points,
    detect_checkerboard_corners,
    validate_camera_intrinsic_binding,
)
from rosclaw_mini.vision.exceptions import HandEyeCalibrationError


DEFAULT_MINIMUM_HAND_EYE_SAMPLES = 10
SO100_PLUS_HAND_EYE_REFERENCE_FRAME = "so100_plus_tcp"
SO100_PLUS_HAND_EYE_KINEMATICS_MODEL = "lerobot_kinematics:so100_plus"


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
        raise HandEyeCalibrationError(f"{label}需要 {length} 个有限数值。")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise HandEyeCalibrationError(
            f"{label}需要 {length} 个有限数值。"
        ) from error
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise HandEyeCalibrationError(f"{label}需要 {length} 个有限数值。")
    return result


def _homogeneous_matrix(values: Any, *, label: str) -> np.ndarray:
    try:
        matrix = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as error:
        raise HandEyeCalibrationError(f"{label}必须是有限 4×4 矩阵。") from error
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise HandEyeCalibrationError(f"{label}必须是有限 4×4 矩阵。")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise HandEyeCalibrationError(f"{label}的齐次末行无效。")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise HandEyeCalibrationError(f"{label}的旋转矩阵不正交。")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise HandEyeCalibrationError(f"{label}的旋转矩阵行列式不为 1。")
    return matrix.copy()


def _matrix_tuple(matrix: Any, *, label: str) -> tuple[tuple[float, ...], ...]:
    checked = _homogeneous_matrix(matrix, label=label)
    return tuple(tuple(float(value) for value in row) for row in checked)


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = min(1.0, max(-1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    return math.acos(cosine)


@dataclass(frozen=True)
class CheckerboardPoseEstimate:
    """``camera_T_target`` and its pixel reprojection error."""

    camera_from_target: tuple[tuple[float, ...], ...]
    reprojection_error_px: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "camera_from_target",
            _matrix_tuple(
                self.camera_from_target,
                label="camera_T_target",
            ),
        )
        if (
            isinstance(self.reprojection_error_px, bool)
            or not isinstance(self.reprojection_error_px, (int, float))
            or not math.isfinite(float(self.reprojection_error_px))
            or float(self.reprojection_error_px) < 0.0
        ):
            raise HandEyeCalibrationError("重投影误差必须是有限非负数。")
        object.__setattr__(
            self, "reprojection_error_px", float(self.reprojection_error_px)
        )


def estimate_checkerboard_pose(
    frame,
    calibration: CameraIntrinsicCalibration,
    *,
    cv2_module=None,
) -> CheckerboardPoseEstimate:
    """Estimate a board pose from one raw, full-resolution camera frame."""

    shape = getattr(frame, "shape", None)
    if not isinstance(shape, tuple) or len(shape) < 2:
        raise HandEyeCalibrationError("棋盘图像没有有效 shape。")
    height, width = shape[:2]
    validate_camera_intrinsic_binding(
        calibration,
        device=calibration.camera_identity.device,
        width=width,
        height=height,
    )
    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except (ImportError, OSError) as error:
            raise HandEyeCalibrationError("手眼标定需要 OpenCV。") from error

    corners = detect_checkerboard_corners(
        frame,
        calibration.checkerboard,
        cv2_module=cv2_module,
    )
    object_points = _object_points(calibration.checkerboard, np)
    camera_matrix = np.asarray(calibration.camera_matrix, dtype=np.float64)
    distortion = np.asarray(
        calibration.distortion_coefficients,
        dtype=np.float64,
    )
    try:
        success, rotation_vector, translation = cv2_module.solvePnP(
            object_points,
            corners,
            camera_matrix,
            distortion,
        )
        if not success:
            raise HandEyeCalibrationError("棋盘 camera_T_target 位姿求解失败。")
        rotation, _jacobian = cv2_module.Rodrigues(rotation_vector)
        projected, _jacobian = cv2_module.projectPoints(
            object_points,
            rotation_vector,
            translation,
            camera_matrix,
            distortion,
        )
    except HandEyeCalibrationError:
        raise
    except Exception as error:
        raise HandEyeCalibrationError(f"棋盘位姿求解失败：{error}") from error

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    difference = corners.reshape(-1, 2) - projected.reshape(-1, 2)
    reprojection_error = float(
        np.sqrt(np.mean(np.sum(difference * difference, axis=1)))
    )
    return CheckerboardPoseEstimate(
        camera_from_target=_matrix_tuple(transform, label="camera_T_target"),
        reprojection_error_px=reprojection_error,
    )


@dataclass(frozen=True)
class HandEyeSample:
    sample_id: str
    captured_at: str
    image_path: str
    joint_radians: tuple[float, ...]
    base_from_tcp: tuple[tuple[float, ...], ...]
    camera_from_target: tuple[tuple[float, ...], ...]
    reprojection_error_px: float

    def __post_init__(self) -> None:
        for field_name in ("sample_id", "captured_at", "image_path"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value.strip():
                raise HandEyeCalibrationError(f"{field_name} 不能为空。")
            object.__setattr__(self, field_name, value.strip())
        object.__setattr__(
            self,
            "joint_radians",
            _finite_vector(self.joint_radians, length=6, label="joint_radians"),
        )
        object.__setattr__(
            self,
            "base_from_tcp",
            _matrix_tuple(self.base_from_tcp, label="base_T_tcp"),
        )
        object.__setattr__(
            self,
            "camera_from_target",
            _matrix_tuple(self.camera_from_target, label="camera_T_target"),
        )
        if (
            isinstance(self.reprojection_error_px, bool)
            or not isinstance(self.reprojection_error_px, (int, float))
            or not math.isfinite(float(self.reprojection_error_px))
            or float(self.reprojection_error_px) < 0.0
        ):
            raise HandEyeCalibrationError("样本重投影误差必须是有限非负数。")
        object.__setattr__(
            self, "reprojection_error_px", float(self.reprojection_error_px)
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "sample_id": self.sample_id,
            "captured_at": self.captured_at,
            "image_path": self.image_path,
            "joint_radians": list(self.joint_radians),
            "base_T_tcp": [list(row) for row in self.base_from_tcp],
            "camera_T_target": [list(row) for row in self.camera_from_target],
            "reprojection_error_px": self.reprojection_error_px,
        }


@dataclass(frozen=True)
class HandEyeDataset:
    camera_device: str
    intrinsics_sha256: str
    robot_port: str
    robot_calibration_filename: str
    robot_calibration_sha256: str
    reference_frame: str
    kinematics_model: str
    tcp_offset_m: tuple[float, float, float]
    samples: tuple[HandEyeSample, ...]
    created_at: str

    def __post_init__(self) -> None:
        string_fields = (
            "camera_device",
            "intrinsics_sha256",
            "robot_port",
            "robot_calibration_filename",
            "robot_calibration_sha256",
            "reference_frame",
            "kinematics_model",
            "created_at",
        )
        for field_name in string_fields:
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value.strip():
                raise HandEyeCalibrationError(f"{field_name} 不能为空。")
            object.__setattr__(self, field_name, value.strip())
        for field_name in ("intrinsics_sha256", "robot_calibration_sha256"):
            value = getattr(self, field_name).lower()
            if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
                raise HandEyeCalibrationError(f"{field_name} 必须是 SHA-256。")
            object.__setattr__(self, field_name, value)
        object.__setattr__(
            self,
            "tcp_offset_m",
            _finite_vector(self.tcp_offset_m, length=3, label="tcp_offset_m"),
        )
        sample_ids = tuple(sample.sample_id for sample in self.samples)
        if len(set(sample_ids)) != len(sample_ids):
            raise HandEyeCalibrationError("手眼样本 sample_id 重复。")

    def _payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "camera_device": self.camera_device,
            "intrinsics_sha256": self.intrinsics_sha256,
            "robot_port": self.robot_port,
            "robot_calibration_filename": self.robot_calibration_filename,
            "robot_calibration_sha256": self.robot_calibration_sha256,
            "reference_frame": self.reference_frame,
            "kinematics_model": self.kinematics_model,
            "tcp_offset_m": list(self.tcp_offset_m),
            "samples": [sample.to_dict() for sample in self.samples],
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
class HandEyeCalibration:
    intrinsics_sha256: str
    dataset_sha256: str
    reference_frame: str
    method: str
    tcp_from_camera: tuple[tuple[float, ...], ...]
    base_from_target: tuple[tuple[float, ...], ...]
    sample_count: int
    translation_rms_m: float
    translation_max_m: float
    rotation_rms_degrees: float
    rotation_max_degrees: float
    per_sample_translation_error_m: tuple[float, ...]
    per_sample_rotation_error_degrees: tuple[float, ...]
    checkerboard_flipped_sample_ids: tuple[str, ...]
    created_at: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "tcp_from_camera",
            _matrix_tuple(self.tcp_from_camera, label="tcp_T_camera"),
        )
        object.__setattr__(
            self,
            "base_from_target",
            _matrix_tuple(self.base_from_target, label="base_T_target"),
        )

    def _payload_without_hash(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "intrinsics_sha256": self.intrinsics_sha256,
            "dataset_sha256": self.dataset_sha256,
            "reference_frame": self.reference_frame,
            "method": self.method,
            "tcp_T_camera": [list(row) for row in self.tcp_from_camera],
            "base_T_target": [list(row) for row in self.base_from_target],
            "sample_count": self.sample_count,
            "translation_rms_m": self.translation_rms_m,
            "translation_max_m": self.translation_max_m,
            "rotation_rms_degrees": self.rotation_rms_degrees,
            "rotation_max_degrees": self.rotation_max_degrees,
            "per_sample_translation_error_m": list(
                self.per_sample_translation_error_m
            ),
            "per_sample_rotation_error_degrees": list(
                self.per_sample_rotation_error_degrees
            ),
            "checkerboard_flipped_sample_ids": list(
                self.checkerboard_flipped_sample_ids
            ),
            "created_at": self.created_at,
        }

    @property
    def calibration_sha256(self) -> str:
        return _canonical_sha256(self._payload_without_hash())

    def to_dict(self) -> dict[str, Any]:
        payload = self._payload_without_hash()
        payload["calibration_sha256"] = self.calibration_sha256
        return payload


def _mean_rotation(rotations: Sequence[np.ndarray]) -> np.ndarray:
    accumulated = sum((rotation for rotation in rotations), np.zeros((3, 3)))
    left, _singular, right = np.linalg.svd(accumulated)
    mean = left @ right
    if np.linalg.det(mean) < 0:
        left[:, -1] *= -1
        mean = left @ right
    return mean


def _validate_rotational_observability(samples: Sequence[HandEyeSample]) -> None:
    reference = _homogeneous_matrix(
        samples[0].base_from_tcp,
        label="base_T_tcp[0]",
    )[:3, :3]
    rotation_vectors = []
    for sample in samples[1:]:
        current = _homogeneous_matrix(
            sample.base_from_tcp,
            label=f"base_T_tcp[{sample.sample_id}]",
        )[:3, :3]
        relative = reference.T @ current
        angle = _rotation_angle(relative)
        if angle <= 1e-8:
            continue
        skew = np.asarray(
            (
                relative[2, 1] - relative[1, 2],
                relative[0, 2] - relative[2, 0],
                relative[1, 0] - relative[0, 1],
            )
        )
        norm = float(np.linalg.norm(skew))
        if norm > 1e-12:
            rotation_vectors.append(skew / norm * angle)
    if len(rotation_vectors) < 2:
        raise HandEyeCalibrationError(
            "手眼样本缺少足够的末端旋转变化，无法观测相机外参。"
        )
    if np.linalg.matrix_rank(np.asarray(rotation_vectors), tol=1e-6) < 2:
        raise HandEyeCalibrationError(
            "手眼样本的旋转轴过于单一，至少需要两个非平行旋转方向。"
        )


def _checkerboard_half_turn_transform(
    intrinsics: CameraIntrinsicCalibration,
) -> np.ndarray:
    """Return the same symmetric board frame expressed from the far corner."""

    checkerboard = intrinsics.checkerboard
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.diag((-1.0, -1.0, 1.0))
    transform[:3, 3] = (
        (checkerboard.inner_columns - 1) * checkerboard.square_size_m,
        (checkerboard.inner_rows - 1) * checkerboard.square_size_m,
        0.0,
    )
    return transform


def _resolve_checkerboard_half_turns(
    samples: Sequence[HandEyeSample],
    base_from_tcp: Sequence[np.ndarray],
    camera_from_target: Sequence[np.ndarray],
    intrinsics: CameraIntrinsicCalibration,
) -> tuple[list[np.ndarray], tuple[str, ...]]:
    """Resolve the unavoidable 180-degree origin ambiguity of a plain board.

    For a fixed target and an eye-in-hand camera, corresponding gripper and
    camera relative rotations are conjugate and therefore have the same
    rotation angle.  A symmetric checkerboard may make OpenCV switch to the
    diagonally opposite target origin in an individual view.  Compare both
    physically equivalent target-frame choices against the measured gripper
    motion and retain the one with the matching relative-rotation angle.

    The first sample fixes only the arbitrary global target-frame convention;
    flipping every sample would describe the same physical board and does not
    change ``tcp_T_camera``.
    """

    reference_gripper = base_from_tcp[0]
    reference_target = camera_from_target[0]
    half_turn = _checkerboard_half_turn_transform(intrinsics)
    resolved = [reference_target]
    flipped_sample_ids: list[str] = []
    for sample, gripper, target in zip(
        samples[1:],
        base_from_tcp[1:],
        camera_from_target[1:],
        strict=True,
    ):
        gripper_motion = np.linalg.inv(reference_gripper) @ gripper
        gripper_angle = _rotation_angle(gripper_motion[:3, :3])

        raw_camera_motion = reference_target @ np.linalg.inv(target)
        flipped_target = target @ half_turn
        flipped_camera_motion = reference_target @ np.linalg.inv(
            flipped_target
        )
        raw_error = abs(
            gripper_angle - _rotation_angle(raw_camera_motion[:3, :3])
        )
        flipped_error = abs(
            gripper_angle - _rotation_angle(flipped_camera_motion[:3, :3])
        )
        if flipped_error < raw_error:
            resolved.append(flipped_target)
            flipped_sample_ids.append(sample.sample_id)
        else:
            resolved.append(target)
    return resolved, tuple(flipped_sample_ids)


def solve_hand_eye_calibration(
    dataset: HandEyeDataset,
    intrinsics: CameraIntrinsicCalibration,
    *,
    minimum_samples: int = DEFAULT_MINIMUM_HAND_EYE_SAMPLES,
    cv2_module=None,
) -> HandEyeCalibration:
    """Solve ``tcp_T_camera`` and report fixed-board consistency."""

    if isinstance(minimum_samples, bool) or not isinstance(minimum_samples, int):
        raise HandEyeCalibrationError("minimum_samples 必须是整数。")
    if minimum_samples < 3:
        raise HandEyeCalibrationError("minimum_samples 不能小于 3。")
    if len(dataset.samples) < minimum_samples:
        raise HandEyeCalibrationError(
            f"手眼样本只有 {len(dataset.samples)} 组，"
            f"至少需要 {minimum_samples} 组。"
        )
    if not hmac.compare_digest(
        dataset.intrinsics_sha256,
        intrinsics.calibration_sha256,
    ):
        raise HandEyeCalibrationError("手眼数据集与当前相机内参哈希不匹配。")
    if dataset.camera_device != intrinsics.camera_identity.device:
        raise HandEyeCalibrationError("手眼数据集与当前摄像头不匹配。")
    if dataset.reference_frame != SO100_PLUS_HAND_EYE_REFERENCE_FRAME:
        raise HandEyeCalibrationError("手眼数据集的机械臂参考系不受支持。")
    _validate_rotational_observability(dataset.samples)

    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except (ImportError, OSError) as error:
            raise HandEyeCalibrationError("手眼标定需要 OpenCV。") from error

    base_from_tcp = [
        _homogeneous_matrix(sample.base_from_tcp, label="base_T_tcp")
        for sample in dataset.samples
    ]
    raw_camera_from_target = [
        _homogeneous_matrix(sample.camera_from_target, label="camera_T_target")
        for sample in dataset.samples
    ]
    camera_from_target, flipped_sample_ids = (
        _resolve_checkerboard_half_turns(
            dataset.samples,
            base_from_tcp,
            raw_camera_from_target,
            intrinsics,
        )
    )
    try:
        rotation, translation = cv2_module.calibrateHandEye(
            [transform[:3, :3] for transform in base_from_tcp],
            [transform[:3, 3].reshape(3, 1) for transform in base_from_tcp],
            [transform[:3, :3] for transform in camera_from_target],
            [
                transform[:3, 3].reshape(3, 1)
                for transform in camera_from_target
            ],
            method=cv2_module.CALIB_HAND_EYE_PARK,
        )
    except Exception as error:
        raise HandEyeCalibrationError(f"OpenCV 手眼求解失败：{error}") from error

    tcp_from_camera = np.eye(4, dtype=float)
    tcp_from_camera[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    tcp_from_camera[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    tcp_from_camera = _homogeneous_matrix(
        tcp_from_camera,
        label="tcp_T_camera",
    )

    base_from_targets = [
        base_tcp @ tcp_from_camera @ camera_target
        for base_tcp, camera_target in zip(
            base_from_tcp,
            camera_from_target,
            strict=True,
        )
    ]
    mean_translation = np.mean(
        [transform[:3, 3] for transform in base_from_targets],
        axis=0,
    )
    mean_rotation = _mean_rotation(
        [transform[:3, :3] for transform in base_from_targets]
    )
    base_from_target = np.eye(4, dtype=float)
    base_from_target[:3, :3] = mean_rotation
    base_from_target[:3, 3] = mean_translation

    translation_errors = tuple(
        float(np.linalg.norm(transform[:3, 3] - mean_translation))
        for transform in base_from_targets
    )
    rotation_errors_degrees = tuple(
        math.degrees(_rotation_angle(mean_rotation.T @ transform[:3, :3]))
        for transform in base_from_targets
    )
    translation_rms = math.sqrt(
        sum(error * error for error in translation_errors)
        / len(translation_errors)
    )
    rotation_rms = math.sqrt(
        sum(error * error for error in rotation_errors_degrees)
        / len(rotation_errors_degrees)
    )
    return HandEyeCalibration(
        intrinsics_sha256=intrinsics.calibration_sha256,
        dataset_sha256=dataset.dataset_sha256,
        reference_frame=dataset.reference_frame,
        method="opencv_park_checkerboard_180_resolved",
        tcp_from_camera=_matrix_tuple(tcp_from_camera, label="tcp_T_camera"),
        base_from_target=_matrix_tuple(base_from_target, label="base_T_target"),
        sample_count=len(dataset.samples),
        translation_rms_m=translation_rms,
        translation_max_m=max(translation_errors),
        rotation_rms_degrees=rotation_rms,
        rotation_max_degrees=max(rotation_errors_degrees),
        per_sample_translation_error_m=translation_errors,
        per_sample_rotation_error_degrees=rotation_errors_degrees,
        checkerboard_flipped_sample_ids=flipped_sample_ids,
        created_at=datetime.now(timezone.utc).isoformat(),
    )


def write_hand_eye_dataset(
    dataset: HandEyeDataset,
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    _write_hashed_json(
        dataset.to_dict(),
        Path(output_path),
        overwrite=overwrite,
        label="手眼数据集",
    )


def load_hand_eye_dataset(input_path: Path) -> HandEyeDataset:
    path = Path(input_path)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise HandEyeCalibrationError(f"手眼数据集不存在：{path}。") from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise HandEyeCalibrationError(f"无法读取手眼数据集：{path}。") from error
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise HandEyeCalibrationError("手眼数据集 schema_version 无效。")
    raw_samples = payload.get("samples")
    if not isinstance(raw_samples, list):
        raise HandEyeCalibrationError("手眼数据集 samples 必须是数组。")
    samples = []
    try:
        for raw_sample in raw_samples:
            if not isinstance(raw_sample, dict):
                raise HandEyeCalibrationError("手眼样本必须是 JSON 对象。")
            samples.append(
                HandEyeSample(
                    sample_id=raw_sample["sample_id"],
                    captured_at=raw_sample["captured_at"],
                    image_path=raw_sample["image_path"],
                    joint_radians=tuple(raw_sample["joint_radians"]),
                    base_from_tcp=tuple(
                        tuple(row) for row in raw_sample["base_T_tcp"]
                    ),
                    camera_from_target=tuple(
                        tuple(row) for row in raw_sample["camera_T_target"]
                    ),
                    reprojection_error_px=raw_sample[
                        "reprojection_error_px"
                    ],
                )
            )
        dataset = HandEyeDataset(
            camera_device=payload["camera_device"],
            intrinsics_sha256=payload["intrinsics_sha256"],
            robot_port=payload["robot_port"],
            robot_calibration_filename=payload[
                "robot_calibration_filename"
            ],
            robot_calibration_sha256=payload[
                "robot_calibration_sha256"
            ],
            reference_frame=payload["reference_frame"],
            kinematics_model=payload["kinematics_model"],
            tcp_offset_m=tuple(payload["tcp_offset_m"]),
            samples=tuple(samples),
            created_at=payload["created_at"],
        )
    except KeyError as error:
        raise HandEyeCalibrationError(
            f"手眼数据集缺少字段：{error.args[0]}。"
        ) from error
    except (TypeError, ValueError) as error:
        raise HandEyeCalibrationError("手眼数据集字段类型无效。") from error
    claimed_hash = payload.get("dataset_sha256")
    if (
        not isinstance(claimed_hash, str)
        or not hmac.compare_digest(
            claimed_hash.lower(),
            dataset.dataset_sha256,
        )
    ):
        raise HandEyeCalibrationError(
            "手眼数据集 SHA-256 不匹配；文件可能已被修改。"
        )
    return dataset


def write_hand_eye_calibration(
    calibration: HandEyeCalibration,
    output_path: Path,
    *,
    overwrite: bool = False,
) -> None:
    _write_hashed_json(
        calibration.to_dict(),
        Path(output_path),
        overwrite=overwrite,
        label="手眼标定",
    )


def _write_hashed_json(
    payload: dict[str, Any],
    path: Path,
    *,
    overwrite: bool,
    label: str,
) -> None:
    if path.exists() and not overwrite:
        raise HandEyeCalibrationError(f"{label}输出已存在：{path}。")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    except OSError as error:
        raise HandEyeCalibrationError(f"写入{label}失败：{path}。") from error
