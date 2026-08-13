"""Configuration for reproducible, simulation-only tabletop scenes.

The values in this module are research assumptions.  They are intentionally
not imported by the real arm or RealSense runtime, and must never be copied to
real calibration files.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from numbers import Real
from typing import Sequence

from rosclaw_mini.arm.so100_plus_session import (
    SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
)


class SimulationConfigurationError(ValueError):
    """A simulation-only scene/configuration is not internally valid."""


def _finite(value: object, *, label: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise SimulationConfigurationError(f"{label} 必须是有限数值。")
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "有限正数" if positive else "有限数值"
        raise SimulationConfigurationError(f"{label} 必须是{qualifier}。")
    return result


def _triplet(values: Sequence[float], *, label: str) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)):
        raise SimulationConfigurationError(f"{label} 必须是三个有限数值。")
    try:
        result = tuple(values)
    except TypeError as error:
        raise SimulationConfigurationError(f"{label} 必须是三个有限数值。") from error
    if len(result) != 3:
        raise SimulationConfigurationError(f"{label} 必须是三个有限数值。")
    return tuple(_finite(value, label=f"{label}[{index}]") for index, value in enumerate(result))  # type: ignore[return-value]


@dataclass(frozen=True)
class SimulationCameraConfig:
    """Virtual D435i-like Color/Depth camera, explicitly not a real profile."""

    width: int = 640
    height: int = 480
    fps: int = 30
    horizontal_fov_degrees: float = 69.0
    depth_scale_m_per_unit: float = 0.001
    # Estimated research-only eye-to-hand pose, base_T_camera.  Camera axes
    # are +X right, +Y down, +Z forward, matching the existing localization.
    base_from_camera: tuple[tuple[float, float, float, float], ...] = (
        (0.992404, 0.057080, 0.109128, 0.30),
        (-0.123021, -0.460250, 0.879064, -0.60),
        (0.000000, -0.885964, -0.463761, 0.55),
        (0.0, 0.0, 0.0, 1.0),
    )
    simulation_only: bool = True
    extrinsics_version: str = "simulation-eye-to-hand-v1"

    def __post_init__(self) -> None:
        for field_name in ("width", "height", "fps"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise SimulationConfigurationError(f"{field_name} 必须是正整数。")
        object.__setattr__(
            self,
            "horizontal_fov_degrees",
            _finite(self.horizontal_fov_degrees, label="horizontal_fov_degrees", positive=True),
        )
        if not 1.0 < self.horizontal_fov_degrees < 179.0:
            raise SimulationConfigurationError("horizontal_fov_degrees 必须在 1 到 179 之间。")
        object.__setattr__(
            self,
            "depth_scale_m_per_unit",
            _finite(self.depth_scale_m_per_unit, label="depth_scale_m_per_unit", positive=True),
        )
        matrix = tuple(tuple(row) for row in self.base_from_camera)
        if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
            raise SimulationConfigurationError("base_from_camera 必须是 4×4 矩阵。")
        checked = tuple(tuple(_finite(value, label="base_from_camera") for value in row) for row in matrix)
        if checked[3] != (0.0, 0.0, 0.0, 1.0):
            raise SimulationConfigurationError("base_from_camera 最后一行必须是 [0,0,0,1]。")
        object.__setattr__(self, "base_from_camera", checked)
        if self.simulation_only is not True:
            raise SimulationConfigurationError("仿真相机配置必须明确 simulation_only=true。")
        if not isinstance(self.extrinsics_version, str) or not self.extrinsics_version.strip():
            raise SimulationConfigurationError("extrinsics_version 不能为空。")

    @property
    def focal_length_pixels(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.horizontal_fov_degrees) / 2.0)


@dataclass(frozen=True)
class SimulationObjectSpec:
    """One simple tabletop object; ``grasp_point_m`` is its visible top centre."""

    name: str
    shape: str
    color_rgb: tuple[int, int, int]
    dimensions_m: tuple[float, float, float]
    grasp_point_m: tuple[float, float, float]
    yaw_radians: float = 0.0
    mass_kg: float = 0.05
    friction: float = 0.7

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise SimulationConfigurationError("仿真物体 name 不能为空。")
        if self.shape not in {"cube", "box", "cylinder"}:
            raise SimulationConfigurationError("仿真物体 shape 只支持 cube、box 或 cylinder。")
        rgb = tuple(self.color_rgb)
        if len(rgb) != 3 or any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255 for value in rgb):
            raise SimulationConfigurationError("color_rgb 必须是三个 0 到 255 的整数。")
        object.__setattr__(self, "color_rgb", rgb)
        dimensions = _triplet(self.dimensions_m, label="dimensions_m")
        if any(value <= 0.0 for value in dimensions):
            raise SimulationConfigurationError("dimensions_m 必须全部为正数。")
        object.__setattr__(self, "dimensions_m", dimensions)
        object.__setattr__(self, "grasp_point_m", _triplet(self.grasp_point_m, label="grasp_point_m"))
        object.__setattr__(self, "yaw_radians", _finite(self.yaw_radians, label="yaw_radians"))
        object.__setattr__(self, "mass_kg", _finite(self.mass_kg, label="mass_kg", positive=True))
        object.__setattr__(self, "friction", _finite(self.friction, label="friction", positive=True))


@dataclass(frozen=True)
class SimulationSceneConfig:
    """A deterministic scene configuration for headless research experiments."""

    name: str
    objects: tuple[SimulationObjectSpec, ...]
    seed: int = 0
    table_z_m: float = 0.19932848277083313
    camera: SimulationCameraConfig = SimulationCameraConfig()
    depth_noise_std_m: float = 0.0
    depth_hole_probability: float = 0.0
    rgb_noise_std: float = 0.0
    camera_position_jitter_m: float = 0.0
    occlusion_fraction: float = 0.0
    simulation_only: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise SimulationConfigurationError("仿真场景 name 不能为空。")
        objects = tuple(self.objects)
        if not objects:
            raise SimulationConfigurationError("仿真场景至少需要一个物体。")
        if len({item.name for item in objects}) != len(objects):
            raise SimulationConfigurationError("仿真物体 name 不能重复。")
        object.__setattr__(self, "objects", objects)
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise SimulationConfigurationError("seed 必须是整数。")
        object.__setattr__(self, "table_z_m", _finite(self.table_z_m, label="table_z_m"))
        for field_name in ("depth_noise_std_m", "depth_hole_probability", "rgb_noise_std", "camera_position_jitter_m", "occlusion_fraction"):
            object.__setattr__(self, field_name, _finite(getattr(self, field_name), label=field_name))
        if not 0.0 <= self.depth_hole_probability < 1.0 or not 0.0 <= self.occlusion_fraction < 1.0:
            raise SimulationConfigurationError("depth_hole_probability/occlusion_fraction 必须在 [0, 1) 内。")
        if self.simulation_only is not True:
            raise SimulationConfigurationError("仿真场景必须明确 simulation_only=true。")


def _object(
    name: str,
    shape: str,
    color: tuple[int, int, int],
    position: tuple[float, float, float],
    *,
    dimensions: tuple[float, float, float] = (0.04, 0.04, 0.04),
    yaw: float = 0.0,
) -> SimulationObjectSpec:
    return SimulationObjectSpec(
        name=name,
        shape=shape,
        color_rgb=color,
        dimensions_m=dimensions,
        grasp_point_m=position,
        yaw_radians=yaw,
    )


def build_simulation_scene(name: str = "standard", *, seed: int = 0) -> SimulationSceneConfig:
    """Return one of the documented, deterministic simulation-only scenes."""

    if not isinstance(name, str) or not name.strip():
        raise SimulationConfigurationError("仿真场景名称不能为空。")
    reference = SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M
    cube = _object("red_cube", "cube", (220, 45, 40), reference)
    if name == "standard":
        return SimulationSceneConfig(name=name, objects=(cube,), seed=seed)
    if name == "multi_object":
        return SimulationSceneConfig(
            name=name,
            objects=(
                cube,
                _object("blue_box", "box", (45, 90, 210), (reference[0] + 0.03, reference[1] - 0.02, reference[2]), dimensions=(0.06, 0.035, 0.03), yaw=0.25),
                _object("green_cylinder", "cylinder", (35, 160, 70), (reference[0] - 0.03, reference[1] + 0.02, reference[2]), dimensions=(0.035, 0.035, 0.05)),
            ),
            seed=seed,
        )
    if name == "randomized":
        # Randomisation is applied by SimulationWorld with this deterministic seed.
        return SimulationSceneConfig(name=name, objects=(cube,), seed=seed)
    if name == "noisy":
        return SimulationSceneConfig(
            name=name,
            objects=(cube,),
            seed=seed,
            depth_noise_std_m=0.003,
            depth_hole_probability=0.08,
            rgb_noise_std=4.0,
            camera_position_jitter_m=0.002,
            occlusion_fraction=0.15,
        )
    if name == "depth_holes":
        return SimulationSceneConfig(
            name=name,
            objects=(cube,),
            seed=seed,
            depth_noise_std_m=0.004,
            depth_hole_probability=0.40,
        )
    if name == "boundary_reject":
        return SimulationSceneConfig(
            name=name,
            objects=(
                _object("edge_cube", "cube", (220, 45, 40), (0.5435714232704467, reference[1], reference[2])),
            ),
            seed=seed,
        )
    raise SimulationConfigurationError(
        "未知仿真场景："
        f"{name!r}；可用 standard、multi_object、randomized、noisy、depth_holes、boundary_reject。"
    )


def load_simulation_camera_config(path: str | Path) -> SimulationCameraConfig:
    """Load a dedicated virtual-camera file and reject real calibration schema."""

    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise SimulationConfigurationError(f"无法读取仿真相机配置：{source}。") from error
    except json.JSONDecodeError as error:
        raise SimulationConfigurationError(f"仿真相机配置不是合法 JSON：{source}。") from error
    if not isinstance(payload, dict) or payload.get("simulation_only") is not True:
        raise SimulationConfigurationError(
            "仿真相机配置必须包含 simulation_only=true；拒绝使用真实标定文件。"
        )
    required = {
        "width",
        "height",
        "fps",
        "horizontal_fov_degrees",
        "depth_scale_m_per_unit",
        "T_base_from_camera",
        "extrinsics_version",
    }
    missing = sorted(required.difference(payload))
    if missing:
        raise SimulationConfigurationError("仿真相机配置缺少字段：" + ", ".join(missing))
    return SimulationCameraConfig(
        width=payload["width"],
        height=payload["height"],
        fps=payload["fps"],
        horizontal_fov_degrees=payload["horizontal_fov_degrees"],
        depth_scale_m_per_unit=payload["depth_scale_m_per_unit"],
        base_from_camera=tuple(tuple(row) for row in payload["T_base_from_camera"]),
        simulation_only=True,
        extrinsics_version=payload["extrinsics_version"],
    )
