"""Robust target depth and camera-frame 3D localization from aligned RGBD."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import json
import math
from typing import Any

import numpy as np

from rosclaw_mini.vision.exceptions import TargetLocalizationError
from rosclaw_mini.vision.eye_to_hand import (
    EyeToHandCalibration,
    transform_camera_point_to_base,
)
from rosclaw_mini.vision.image import OpenCVImageProcessor
from rosclaw_mini.vision.parser import (
    SceneObservationParser,
    decode_scene_observation_payload,
)
from rosclaw_mini.vision.prompt import build_localization_prompt
from rosclaw_mini.vision.realsense import (
    ColorIntrinsics,
    RealSenseCameraAdapter,
    RealSenseFrame,
)
from rosclaw_mini.vision.schemas import SceneObject, SceneObservation
from rosclaw_mini.vision.vlm_client import VLMClient


def _finite_float(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TargetLocalizationError(f"{label} 必须是有限数值。")
    result = float(value)
    if not math.isfinite(result):
        raise TargetLocalizationError(f"{label} 必须是有限数值。")
    return result


@dataclass(frozen=True)
class PixelBoundingBox:
    """Half-open RGB pixel bounds: x/y minimum inclusive, maximum exclusive."""

    x_min: int
    y_min: int
    x_max: int
    y_max: int

    def __post_init__(self) -> None:
        values = (self.x_min, self.y_min, self.x_max, self.y_max)
        if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
            raise TargetLocalizationError("像素 bounding_box 必须是整数。")
        if self.x_min < 0 or self.y_min < 0:
            raise TargetLocalizationError("像素 bounding_box 不能为负。")
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise TargetLocalizationError("像素 bounding_box 必须是非空区域。")

    @property
    def width(self) -> int:
        return self.x_max - self.x_min

    @property
    def height(self) -> int:
        return self.y_max - self.y_min

    @property
    def center(self) -> tuple[int, int]:
        return (
            (self.x_min + self.x_max - 1) // 2,
            (self.y_min + self.y_max - 1) // 2,
        )


@dataclass(frozen=True)
class DepthEstimationConfig:
    roi_fraction: float = 0.5
    minimum_box_size_pixels: int = 6
    minimum_valid_samples: int = 20
    minimum_valid_ratio: float = 0.20
    outlier_sigma: float = 3.5
    minimum_outlier_tolerance_m: float = 0.01
    maximum_depth_spread_m: float = 0.08

    def __post_init__(self) -> None:
        for field_name in ("minimum_box_size_pixels", "minimum_valid_samples"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{field_name} 必须是正整数。")
        for field_name in (
            "roi_fraction",
            "minimum_valid_ratio",
            "outlier_sigma",
            "minimum_outlier_tolerance_m",
            "maximum_depth_spread_m",
        ):
            value = getattr(self, field_name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ValueError(f"{field_name} 必须是有限正数。")
            object.__setattr__(self, field_name, float(value))
        if self.roi_fraction > 1.0:
            raise ValueError("roi_fraction 不能大于 1。")
        if self.minimum_valid_ratio > 1.0:
            raise ValueError("minimum_valid_ratio 不能大于 1。")


@dataclass(frozen=True)
class DepthEstimate:
    depth_m: float
    valid_depth_ratio: float
    uncertainty_m: float
    valid_samples: int
    total_samples: int
    quality: str


@dataclass(frozen=True)
class PositionEstimate:
    observation_id: str
    object_name: str
    bounding_box: tuple[float, float, float, float]
    center_pixel: tuple[int, int]
    depth_m: float
    camera_point_m: tuple[float, float, float]
    valid_depth_ratio: float
    uncertainty_m: float
    quality: str
    source: str
    source_frame: int
    source_timestamp_ms: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "observation_id": self.observation_id,
            "object_name": self.object_name,
            "bounding_box": list(self.bounding_box),
            "center_pixel": list(self.center_pixel),
            "depth_m": self.depth_m,
            "camera_point_m": list(self.camera_point_m),
            "camera_axes": "+X right, +Y down, +Z forward",
            "units": "m",
            "valid_depth_ratio": self.valid_depth_ratio,
            "uncertainty_m": self.uncertainty_m,
            "quality": self.quality,
            "source": self.source,
            "source_frame": self.source_frame,
            "source_timestamp_ms": self.source_timestamp_ms,
        }


@dataclass(frozen=True)
class LocalizationResult:
    observation: SceneObservation
    target: SceneObject
    position: PositionEstimate


@dataclass(frozen=True)
class BasePositionEstimate:
    """One camera estimate transformed by one identity-bound calibration."""

    observation_id: str
    object_name: str
    camera_point_m: tuple[float, float, float]
    base_point_m: tuple[float, float, float]
    camera_frame: str
    base_frame: str
    source_frame: int
    source_timestamp_ms: float
    localization_quality: str
    localization_uncertainty_m: float
    calibration_sha256: str
    calibration_created_at: str
    fit_rmse_m: float
    fit_max_error_m: float
    validation_rmse_m: float | None
    validation_max_error_m: float | None
    calibration_active: bool
    activation_max_rmse_m: float
    activation_max_error_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "observation_id": self.observation_id,
            "object_name": self.object_name,
            "camera_point_m": list(self.camera_point_m),
            "base_point_m": list(self.base_point_m),
            "camera_frame": self.camera_frame,
            "base_frame": self.base_frame,
            "units": "m",
            "source_frame": self.source_frame,
            "source_timestamp_ms": self.source_timestamp_ms,
            "localization_quality": self.localization_quality,
            "localization_uncertainty_m": self.localization_uncertainty_m,
            "calibration_sha256": self.calibration_sha256,
            "calibration_created_at": self.calibration_created_at,
            "fit_rmse_m": self.fit_rmse_m,
            "fit_max_error_m": self.fit_max_error_m,
            "validation_rmse_m": self.validation_rmse_m,
            "validation_max_error_m": self.validation_max_error_m,
            "calibration_active": self.calibration_active,
            "activation_max_rmse_m": self.activation_max_rmse_m,
            "activation_max_error_m": self.activation_max_error_m,
        }


def transform_position_estimate_to_base(
    position: PositionEstimate,
    calibration: EyeToHandCalibration,
) -> BasePositionEstimate:
    """Transform a frame-bound camera estimate without losing its provenance."""

    base_point = transform_camera_point_to_base(
        position.camera_point_m,
        calibration,
    )
    return BasePositionEstimate(
        observation_id=position.observation_id,
        object_name=position.object_name,
        camera_point_m=position.camera_point_m,
        base_point_m=base_point,
        camera_frame=calibration.camera_frame,
        base_frame=calibration.base_frame,
        source_frame=position.source_frame,
        source_timestamp_ms=position.source_timestamp_ms,
        localization_quality=position.quality,
        localization_uncertainty_m=position.uncertainty_m,
        calibration_sha256=calibration.calibration_sha256,
        calibration_created_at=calibration.created_at,
        fit_rmse_m=calibration.fit_rmse_m,
        fit_max_error_m=calibration.fit_max_error_m,
        validation_rmse_m=calibration.validation_rmse_m,
        validation_max_error_m=calibration.validation_max_error_m,
        calibration_active=calibration.active,
        activation_max_rmse_m=calibration.activation_max_rmse_m,
        activation_max_error_m=calibration.activation_max_error_m,
    )


def normalized_box_to_pixels(
    bounding_box: Sequence[float],
    *,
    width: int,
    height: int,
) -> PixelBoundingBox:
    if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
        raise TargetLocalizationError("图像 width 必须是正整数。")
    if isinstance(height, bool) or not isinstance(height, int) or height <= 0:
        raise TargetLocalizationError("图像 height 必须是正整数。")
    if isinstance(bounding_box, (str, bytes)):
        raise TargetLocalizationError("bounding_box 必须包含四个归一化数值。")
    try:
        raw_values = tuple(bounding_box)
    except TypeError as error:
        raise TargetLocalizationError(
            "bounding_box 必须包含四个归一化数值。"
        ) from error
    if len(raw_values) != 4:
        raise TargetLocalizationError("bounding_box 必须包含四个归一化数值。")
    values = tuple(
        _finite_float(value, label="bounding_box") for value in raw_values
    )
    if any(value < 0.0 or value > 1.0 for value in values):
        raise TargetLocalizationError("bounding_box 必须位于 [0, 1]。")
    x_min, y_min, x_max, y_max = values
    if x_min >= x_max or y_min >= y_max:
        raise TargetLocalizationError("bounding_box 坐标顺序无效或区域为空。")
    pixel_x_min = max(0, min(width, math.floor(x_min * width)))
    pixel_y_min = max(0, min(height, math.floor(y_min * height)))
    pixel_x_max = max(0, min(width, math.ceil(x_max * width)))
    pixel_y_max = max(0, min(height, math.ceil(y_max * height)))
    return PixelBoundingBox(
        x_min=pixel_x_min,
        y_min=pixel_y_min,
        x_max=pixel_x_max,
        y_max=pixel_y_max,
    )


def _central_roi(bounds: PixelBoundingBox, fraction: float) -> PixelBoundingBox:
    roi_width = max(1, round(bounds.width * fraction))
    roi_height = max(1, round(bounds.height * fraction))
    center_x, center_y = bounds.center
    x_min = max(bounds.x_min, center_x - roi_width // 2)
    y_min = max(bounds.y_min, center_y - roi_height // 2)
    x_max = min(bounds.x_max, x_min + roi_width)
    y_max = min(bounds.y_max, y_min + roi_height)
    return PixelBoundingBox(x_min, y_min, x_max, y_max)


def estimate_robust_depth(
    aligned_depth,
    *,
    pixel_bounds: PixelBoundingBox,
    depth_scale_m_per_unit: float,
    config: DepthEstimationConfig = DepthEstimationConfig(),
) -> DepthEstimate:
    shape = getattr(aligned_depth, "shape", None)
    if not isinstance(shape, tuple) or len(shape) != 2:
        raise TargetLocalizationError("aligned_depth 必须是二维数组。")
    image_height, image_width = shape
    if pixel_bounds.x_max > image_width or pixel_bounds.y_max > image_height:
        raise TargetLocalizationError("bounding_box 超出 Depth 图像范围。")
    if (
        pixel_bounds.width < config.minimum_box_size_pixels
        or pixel_bounds.height < config.minimum_box_size_pixels
    ):
        raise TargetLocalizationError(
            "目标检测框太小，无法获得可靠深度："
            f"{pixel_bounds.width}×{pixel_bounds.height} px。"
        )
    scale = _finite_float(depth_scale_m_per_unit, label="depth scale")
    if scale <= 0.0:
        raise TargetLocalizationError("depth scale 必须为正数。")
    roi_bounds = _central_roi(pixel_bounds, config.roi_fraction)
    roi = np.asarray(
        aligned_depth[
            roi_bounds.y_min : roi_bounds.y_max,
            roi_bounds.x_min : roi_bounds.x_max,
        ],
        dtype=float,
    )
    total_samples = int(roi.size)
    metric = roi.reshape(-1) * scale
    finite_valid = metric[np.isfinite(metric) & (metric > 0.0)]
    initial_ratio = len(finite_valid) / total_samples if total_samples else 0.0
    if (
        len(finite_valid) < config.minimum_valid_samples
        or initial_ratio < config.minimum_valid_ratio
    ):
        raise TargetLocalizationError(
            "目标深度有效样本不足："
            f"valid={len(finite_valid)}/{total_samples} "
            f"({initial_ratio:.1%})。"
        )
    median = float(np.median(finite_valid))
    absolute_deviation = np.abs(finite_valid - median)
    mad = float(np.median(absolute_deviation))
    tolerance = max(
        config.minimum_outlier_tolerance_m,
        config.outlier_sigma * 1.4826 * mad,
    )
    robust = finite_valid[absolute_deviation <= tolerance]
    valid_ratio = len(robust) / total_samples if total_samples else 0.0
    if (
        len(robust) < config.minimum_valid_samples
        or valid_ratio < config.minimum_valid_ratio
    ):
        raise TargetLocalizationError(
            "过滤离群点后目标深度有效样本不足："
            f"valid={len(robust)}/{total_samples} ({valid_ratio:.1%})。"
        )
    low, high = np.percentile(robust, (10.0, 90.0))
    spread = float(high - low)
    if spread > config.maximum_depth_spread_m:
        raise TargetLocalizationError(
            "目标深度离散程度过大，可能被遮挡或检测框包含背景："
            f"P90-P10={spread:.4f} m，允许上限 "
            f"{config.maximum_depth_spread_m:.4f} m。"
        )
    depth_m = float(np.median(robust))
    robust_mad = float(np.median(np.abs(robust - depth_m)))
    uncertainty_m = max(1.4826 * robust_mad, spread / 2.0)
    quality = "good" if valid_ratio >= 0.60 and spread <= 0.03 else "usable"
    return DepthEstimate(
        depth_m=depth_m,
        valid_depth_ratio=valid_ratio,
        uncertainty_m=uncertainty_m,
        valid_samples=len(robust),
        total_samples=total_samples,
        quality=quality,
    )


def deproject_color_pixel(
    pixel: tuple[int, int],
    depth_m: float,
    intrinsics: ColorIntrinsics,
) -> tuple[float, float, float]:
    if (
        not isinstance(pixel, tuple)
        or len(pixel) != 2
        or any(isinstance(value, bool) or not isinstance(value, int) for value in pixel)
    ):
        raise TargetLocalizationError("pixel 必须是两个整数。")
    u, v = pixel
    if not 0 <= u < intrinsics.width or not 0 <= v < intrinsics.height:
        raise TargetLocalizationError("反投影像素超出 Color 图像范围。")
    z = _finite_float(depth_m, label="depth_m")
    if z <= 0.0:
        raise TargetLocalizationError("depth_m 必须为正数。")
    if any(abs(value) > 1e-12 for value in intrinsics.coefficients):
        raise TargetLocalizationError(
            "当前纯数学反投影只接受已校正或零畸变 Color 内参；"
            "非零畸变必须由 RealSense SDK 去畸变后再定位。"
        )
    x = (u - intrinsics.ppx) / intrinsics.fx * z
    y = (v - intrinsics.ppy) / intrinsics.fy * z
    point = (float(x), float(y), float(z))
    if not all(math.isfinite(value) for value in point):
        raise TargetLocalizationError("相机三维反投影结果不是有限数值。")
    return point


def localize_scene_object(
    observation: SceneObservation,
    target: SceneObject,
    frame: RealSenseFrame,
    *,
    depth_config: DepthEstimationConfig = DepthEstimationConfig(),
) -> PositionEstimate:
    if target.bounding_box is None:
        raise TargetLocalizationError(
            f"目标 {target.name!r} 没有可靠 bounding_box，无法定位。"
        )
    bounds = normalized_box_to_pixels(
        target.bounding_box,
        width=frame.color_intrinsics.width,
        height=frame.color_intrinsics.height,
    )
    depth = estimate_robust_depth(
        frame.aligned_depth,
        pixel_bounds=bounds,
        depth_scale_m_per_unit=frame.depth_scale_m_per_unit,
        config=depth_config,
    )
    center = bounds.center
    camera_point = deproject_color_pixel(
        center,
        depth.depth_m,
        frame.color_intrinsics,
    )
    return PositionEstimate(
        observation_id=observation.observation_id,
        object_name=target.name,
        bounding_box=target.bounding_box,
        center_pixel=center,
        depth_m=depth.depth_m,
        camera_point_m=camera_point,
        valid_depth_ratio=depth.valid_depth_ratio,
        uncertainty_m=depth.uncertainty_m,
        quality=depth.quality,
        source=frame.source,
        source_frame=frame.frame_number,
        source_timestamp_ms=frame.timestamp_ms,
    )


CameraFactory = Callable[[], RealSenseCameraAdapter]


def parse_pixel_grounding_observation(
    raw_response: str,
    *,
    source: str,
    model: str,
    image_width: int,
    image_height: int,
    parser: SceneObservationParser | None = None,
) -> SceneObservation:
    """Validate integer source-image boxes and normalize them locally."""

    for name, value in (("image_width", image_width), ("image_height", image_height)):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise TargetLocalizationError(f"{name} 必须是正整数。")
    payload = decode_scene_observation_payload(raw_response)
    objects = payload.get("objects")
    if not isinstance(objects, list):
        # Let the ordinary parser keep ownership of the canonical error text.
        return (parser or SceneObservationParser()).parse(
            raw_response,
            source=source,
            model=model,
        )
    normalized_payload = dict(payload)
    normalized_objects: list[Any] = []
    for index, item in enumerate(objects):
        if not isinstance(item, dict):
            normalized_objects.append(item)
            continue
        pixel_box = item.get("bounding_box_pixels")
        if pixel_box is None:
            raise TargetLocalizationError(
                f"objects[{index}].bounding_box_pixels 缺失；"
                "定位模式不接受模型直接生成的归一化估计框。"
            )
        if not isinstance(pixel_box, list) or len(pixel_box) != 4:
            raise TargetLocalizationError(
                f"objects[{index}].bounding_box_pixels 必须是四个整数。"
            )
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in pixel_box
        ):
            raise TargetLocalizationError(
                f"objects[{index}].bounding_box_pixels 必须是四个整数。"
            )
        x_min, y_min, x_max, y_max = pixel_box
        if not (
            0 <= x_min < x_max <= image_width
            and 0 <= y_min < y_max <= image_height
        ):
            raise TargetLocalizationError(
                f"objects[{index}].bounding_box_pixels={pixel_box!r} "
                f"超出原始图像 {image_width}×{image_height} 或区域为空。"
            )
        normalized = dict(item)
        normalized["bounding_box"] = [
            x_min / image_width,
            y_min / image_height,
            x_max / image_width,
            y_max / image_height,
        ]
        normalized.pop("bounding_box_pixels", None)
        normalized_objects.append(normalized)
    normalized_payload["objects"] = normalized_objects
    return (parser or SceneObservationParser()).parse(
        json.dumps(normalized_payload, ensure_ascii=False),
        source=source,
        model=model,
    )


class RealSenseLocalizationService:
    """Capture one RGBD frame, let VLM select one target, then use depth."""

    def __init__(
        self,
        *,
        client: VLMClient,
        camera_factory: CameraFactory,
        max_width: int = 1280,
        image_processor: OpenCVImageProcessor | None = None,
        parser: SceneObservationParser | None = None,
        depth_config: DepthEstimationConfig = DepthEstimationConfig(),
    ) -> None:
        if isinstance(max_width, bool) or not isinstance(max_width, int) or max_width <= 0:
            raise ValueError("max_width 必须是正整数。")
        self._client = client
        self._camera_factory = camera_factory
        self._max_width = max_width
        self._image_processor = image_processor or OpenCVImageProcessor()
        self._parser = parser or SceneObservationParser()
        self._depth_config = depth_config

    def locate(self, question: str) -> LocalizationResult:
        if not isinstance(question, str) or not question.strip():
            raise TargetLocalizationError("目标定位问题不能为空。")
        with self._camera_factory() as camera:
            frame = camera.capture_frame()
        # OpenCV JPEG encoding expects BGR; retain the original RGBD frame.
        bgr = np.ascontiguousarray(frame.rgb[..., ::-1])
        encoded = self._image_processor.prepare(bgr, max_width=self._max_width)
        response = self._client.generate(
            image_bytes=encoded.data,
            mime_type=encoded.mime_type,
            prompt=build_localization_prompt(
                question,
                image_width=encoded.width,
                image_height=encoded.height,
            ),
        )
        observation = parse_pixel_grounding_observation(
            response,
            source=frame.source,
            model=self._client.model,
            image_width=encoded.width,
            image_height=encoded.height,
            parser=self._parser,
        )
        candidates = tuple(
            item for item in observation.objects if item.bounding_box is not None
        )
        if len(candidates) != 1:
            raise TargetLocalizationError(
                "VLM 必须返回且只返回一个带 bounding_box 的明确目标；"
                f"当前候选数量为 {len(candidates)}。"
            )
        target = candidates[0]
        position = localize_scene_object(
            observation,
            target,
            frame,
            depth_config=self._depth_config,
        )
        return LocalizationResult(
            observation=observation,
            target=target,
            position=position,
        )
