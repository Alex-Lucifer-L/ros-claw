"""Optional synchronized Intel RealSense RGBD capture.

This module deliberately has no import-time dependency on ``pyrealsense2``.
The existing OpenCV webcam adapter remains the default camera path.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
from typing import Any

from rosclaw_mini.vision.exceptions import (
    RealSenseDependencyError,
    RealSenseDeviceError,
    RealSenseFrameError,
)


def _positive_int(value: Any, *, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{label} 必须是正整数。")
    return value


def _finite_float(value: Any, *, label: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须是有限数值。")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "有限正数" if positive else "有限数值"
        raise ValueError(f"{label} 必须是{qualifier}。")
    return result


def _import_realsense():
    try:
        import pyrealsense2 as rs
    except (ImportError, OSError) as error:
        raise RealSenseDependencyError(
            "缺少 pyrealsense2；请在 RealSense 可选环境中安装 Intel "
            "librealsense Python 绑定。"
        ) from error
    return rs


def _import_numpy():
    try:
        import numpy as np
    except (ImportError, OSError) as error:
        raise RealSenseDependencyError("RealSense RGBD 读取需要 NumPy。") from error
    return np


@dataclass(frozen=True)
class ColorIntrinsics:
    """Runtime intrinsics for the color stream used by aligned depth."""

    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float
    distortion_model: str
    coefficients: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "width", _positive_int(self.width, label="width"))
        object.__setattr__(
            self, "height", _positive_int(self.height, label="height")
        )
        for field_name in ("fx", "fy"):
            object.__setattr__(
                self,
                field_name,
                _finite_float(
                    getattr(self, field_name),
                    label=field_name,
                    positive=True,
                ),
            )
        for field_name in ("ppx", "ppy"):
            object.__setattr__(
                self,
                field_name,
                _finite_float(getattr(self, field_name), label=field_name),
            )
        if not isinstance(self.distortion_model, str) or not self.distortion_model:
            raise ValueError("distortion_model 不能为空。")
        coefficients = tuple(
            _finite_float(value, label="coefficients")
            for value in self.coefficients
        )
        object.__setattr__(self, "coefficients", coefficients)

    def to_dict(self) -> dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "fx": self.fx,
            "fy": self.fy,
            "ppx": self.ppx,
            "ppy": self.ppy,
            "distortion_model": self.distortion_model,
            "coefficients": list(self.coefficients),
        }


@dataclass(frozen=True)
class RealSenseFrame:
    """One synchronized color frame and depth frame aligned to color pixels."""

    rgb: Any
    aligned_depth: Any
    color_intrinsics: ColorIntrinsics
    depth_scale_m_per_unit: float
    source: str
    frame_number: int
    timestamp_ms: float

    def __post_init__(self) -> None:
        rgb_shape = getattr(self.rgb, "shape", None)
        depth_shape = getattr(self.aligned_depth, "shape", None)
        expected_rgb = (
            self.color_intrinsics.height,
            self.color_intrinsics.width,
            3,
        )
        expected_depth = (
            self.color_intrinsics.height,
            self.color_intrinsics.width,
        )
        if rgb_shape != expected_rgb:
            raise RealSenseFrameError(
                f"Color shape {rgb_shape!r} 与运行时内参 {expected_rgb!r} 不匹配。"
            )
        if depth_shape != expected_depth:
            raise RealSenseFrameError(
                "对齐 Depth shape "
                f"{depth_shape!r} 与 Color 分辨率 {expected_depth!r} 不匹配。"
            )
        object.__setattr__(
            self,
            "depth_scale_m_per_unit",
            _finite_float(
                self.depth_scale_m_per_unit,
                label="depth_scale_m_per_unit",
                positive=True,
            ),
        )
        if not isinstance(self.source, str) or not self.source.strip():
            raise RealSenseFrameError("RealSense frame source 不能为空。")
        object.__setattr__(self, "source", self.source.strip())
        if (
            isinstance(self.frame_number, bool)
            or not isinstance(self.frame_number, int)
            or self.frame_number < 0
        ):
            raise RealSenseFrameError("frame_number 必须是非负整数。")
        object.__setattr__(
            self,
            "timestamp_ms",
            _finite_float(self.timestamp_ms, label="timestamp_ms"),
        )


PipelineFactory = Callable[[Any], Any]


class RealSenseCameraAdapter:
    """Own a RealSense pipeline and return synchronized, aligned RGBD frames."""

    def __init__(
        self,
        serial_number: str,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        timeout_ms: int = 5000,
        rs_module: Any | None = None,
        numpy_module: Any | None = None,
        pipeline_factory: PipelineFactory | None = None,
    ) -> None:
        if not isinstance(serial_number, str) or not serial_number.strip():
            raise ValueError("RealSense serial_number 不能为空。")
        self.serial_number = serial_number.strip()
        self.width = _positive_int(width, label="width")
        self.height = _positive_int(height, label="height")
        self.fps = _positive_int(fps, label="fps")
        self.timeout_ms = _positive_int(timeout_ms, label="timeout_ms")
        self._rs = rs_module
        self._np = numpy_module
        self._pipeline_factory = pipeline_factory
        self._pipeline = None
        self._align = None
        self._intrinsics: ColorIntrinsics | None = None
        self._depth_scale: float | None = None

    @property
    def is_open(self) -> bool:
        return self._pipeline is not None

    @property
    def color_intrinsics(self) -> ColorIntrinsics:
        if self._intrinsics is None:
            raise RealSenseDeviceError("RealSense 尚未打开，无法读取 Color 内参。")
        return self._intrinsics

    @property
    def depth_scale_m_per_unit(self) -> float:
        if self._depth_scale is None:
            raise RealSenseDeviceError("RealSense 尚未打开，无法读取 depth scale。")
        return self._depth_scale

    def open(self) -> None:
        if self._pipeline is not None:
            return
        rs = self._rs or _import_realsense()
        pipeline = (
            self._pipeline_factory(rs)
            if self._pipeline_factory is not None
            else rs.pipeline()
        )
        started = False
        try:
            config = rs.config()
            config.enable_device(self.serial_number)
            config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.rgb8,
                self.fps,
            )
            config.enable_stream(
                rs.stream.depth,
                self.width,
                self.height,
                rs.format.z16,
                self.fps,
            )
            profile = pipeline.start(config)
            started = True
            device = profile.get_device()
            actual_serial = device.get_info(rs.camera_info.serial_number)
            if actual_serial != self.serial_number:
                raise RealSenseDeviceError(
                    "RealSense 设备身份不匹配："
                    f"期望 {self.serial_number}，实际 {actual_serial}。"
                )
            color_profile = profile.get_stream(
                rs.stream.color
            ).as_video_stream_profile()
            native_intrinsics = color_profile.get_intrinsics()
            intrinsics = ColorIntrinsics(
                width=int(native_intrinsics.width),
                height=int(native_intrinsics.height),
                fx=float(native_intrinsics.fx),
                fy=float(native_intrinsics.fy),
                ppx=float(native_intrinsics.ppx),
                ppy=float(native_intrinsics.ppy),
                distortion_model=str(native_intrinsics.model),
                coefficients=tuple(float(v) for v in native_intrinsics.coeffs),
            )
            depth_scale = _finite_float(
                device.first_depth_sensor().get_depth_scale(),
                label="depth scale",
                positive=True,
            )
            align = rs.align(rs.stream.color)
        except RealSenseDeviceError:
            if started:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            raise
        except Exception as error:
            if started:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            raise RealSenseDeviceError(
                f"无法打开 RealSense {self.serial_number}：{error}"
            ) from error

        self._rs = rs
        self._pipeline = pipeline
        self._align = align
        self._intrinsics = intrinsics
        self._depth_scale = depth_scale

    def capture_frame(self) -> RealSenseFrame:
        pipeline = self._pipeline
        align = self._align
        if pipeline is None or align is None:
            raise RealSenseFrameError("RealSense 尚未打开。")
        np = self._np or _import_numpy()
        try:
            frames = pipeline.wait_for_frames(self.timeout_ms)
            aligned_frames = align.process(frames)
            color = aligned_frames.get_color_frame()
            depth = aligned_frames.get_depth_frame()
        except Exception as error:
            raise RealSenseFrameError(
                f"RealSense 等待同步帧超时或失败：{error}"
            ) from error
        if not color:
            raise RealSenseFrameError("RealSense 同步帧缺少 Color 数据流。")
        if not depth:
            raise RealSenseFrameError("RealSense 同步帧缺少 Depth 数据流。")
        try:
            rgb = np.asanyarray(color.get_data()).copy()
            aligned_depth = np.asanyarray(depth.get_data()).copy()
            frame_number = int(color.get_frame_number())
            timestamp_ms = float(color.get_timestamp())
        except Exception as error:
            raise RealSenseFrameError(
                f"RealSense 帧数据转换失败：{error}"
            ) from error
        return RealSenseFrame(
            rgb=rgb,
            aligned_depth=aligned_depth,
            color_intrinsics=self.color_intrinsics,
            depth_scale_m_per_unit=self.depth_scale_m_per_unit,
            source=f"realsense:{self.serial_number}",
            frame_number=frame_number,
            timestamp_ms=timestamp_ms,
        )

    def close(self) -> None:
        pipeline = self._pipeline
        self._pipeline = None
        self._align = None
        self._intrinsics = None
        self._depth_scale = None
        if pipeline is None:
            return
        try:
            pipeline.stop()
        except Exception as error:
            raise RealSenseDeviceError(
                f"关闭 RealSense {self.serial_number} pipeline 失败：{error}"
            ) from error

    def __enter__(self) -> "RealSenseCameraAdapter":
        self.open()
        return self

    def __exit__(self, _error_type, _error, _traceback) -> None:
        self.close()


def frame_number_gaps(frame_numbers: Sequence[int]) -> int:
    """Count missing frame numbers in an already ordered capture sequence."""

    numbers = tuple(frame_numbers)
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in numbers
    ):
        raise ValueError("frame_numbers 必须是非负整数序列。")
    return sum(max(0, current - previous - 1) for previous, current in zip(numbers, numbers[1:]))
