"""Simulation-only eye-to-hand transform with explicit provenance."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from typing import Sequence

import numpy as np

from rosclaw_mini.simulation.config import SimulationCameraConfig
from rosclaw_mini.vision.localization import BasePositionEstimate, PositionEstimate


class SimulationCalibrationError(ValueError):
    """A calibration file/object is not explicitly simulation-only."""


def _canonical_sha256(payload: dict) -> str:
    data = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(data.encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class SimulationEyeToHandCalibration:
    """A transform used only by virtual camera data and simulation metrics."""

    camera: SimulationCameraConfig
    base_frame: str = "sim_base"
    camera_frame: str = "sim_d435i_color"
    simulation_only: bool = True

    def __post_init__(self) -> None:
        if self.simulation_only is not True or self.camera.simulation_only is not True:
            raise SimulationCalibrationError("仿真外参必须明确 simulation_only=true。")

    @property
    def base_from_camera(self) -> np.ndarray:
        return np.asarray(self.camera.base_from_camera, dtype=float)

    @property
    def calibration_sha256(self) -> str:
        return "simulation:" + _canonical_sha256(
            {
                "simulation_only": True,
                "camera_frame": self.camera_frame,
                "base_frame": self.base_frame,
                "T_base_from_camera": self.base_from_camera.tolist(),
                "version": self.camera.extrinsics_version,
            }
        )

    def transform_camera_point_to_base(
        self,
        point_m: Sequence[float],
    ) -> tuple[float, float, float]:
        point = np.asarray(tuple(point_m), dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise SimulationCalibrationError("仿真相机点必须是三个有限数值。")
        homogeneous = np.concatenate((point, (1.0,)))
        result = self.base_from_camera @ homogeneous
        if not np.all(np.isfinite(result[:3])):
            raise SimulationCalibrationError("仿真外参转换没有得到有限基座坐标。")
        return tuple(float(value) for value in result[:3])

    def transform_position_estimate(
        self,
        estimate: PositionEstimate,
    ) -> BasePositionEstimate:
        base_point = self.transform_camera_point_to_base(estimate.camera_point_m)
        return BasePositionEstimate(
            observation_id=estimate.observation_id,
            object_name=estimate.object_name,
            camera_point_m=estimate.camera_point_m,
            base_point_m=base_point,
            camera_frame=self.camera_frame,
            base_frame=self.base_frame,
            source_frame=estimate.source_frame,
            source_timestamp_ms=estimate.source_timestamp_ms,
            localization_quality=estimate.quality,
            localization_uncertainty_m=estimate.uncertainty_m,
            calibration_sha256=self.calibration_sha256,
            calibration_created_at="simulation-only",
            fit_rmse_m=0.0,
            fit_max_error_m=0.0,
            validation_rmse_m=0.0,
            validation_max_error_m=0.0,
            calibration_active=True,
            activation_max_rmse_m=0.02,
            activation_max_error_m=0.04,
        )

    def to_dict(self) -> dict:
        return {
            "simulation_only": True,
            "camera_frame": self.camera_frame,
            "base_frame": self.base_frame,
            "extrinsics_version": self.camera.extrinsics_version,
            "T_base_from_camera": self.base_from_camera.tolist(),
            "calibration_sha256": self.calibration_sha256,
            "warning": "仿真外参；不得用于真实 RealSense 或 SO-100 Plus。",
        }
