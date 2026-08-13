"""Synthetic synchronized RGB-D frames compatible with ``RealSenseFrame``."""

from __future__ import annotations

import math
import time

import numpy as np

from rosclaw_mini.simulation.config import SimulationCameraConfig
from rosclaw_mini.simulation.world import SimulationWorld
from rosclaw_mini.vision.realsense import ColorIntrinsics, RealSenseFrame


class VirtualRGBDCamera:
    """No-device RGB-D camera that projects virtual objects through a pinhole."""

    def __init__(self, world: SimulationWorld) -> None:
        self._world = world
        self._config: SimulationCameraConfig = world.scene.camera
        self._frame_number = 0
        self._is_open = False
        self._rng = np.random.default_rng(world.scene.seed + 101)
        self._base_from_camera = np.asarray(self._config.base_from_camera, dtype=float).copy()
        if world.scene.camera_position_jitter_m:
            self._base_from_camera[:3, 3] += self._rng.normal(
                0.0,
                world.scene.camera_position_jitter_m,
                size=3,
            )
        focal = self._config.focal_length_pixels
        self._intrinsics = ColorIntrinsics(
            width=self._config.width,
            height=self._config.height,
            fx=focal,
            fy=focal,
            ppx=(self._config.width - 1) / 2.0,
            ppy=(self._config.height - 1) / 2.0,
            distortion_model="simulated_pinhole",
            coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def __enter__(self) -> "VirtualRGBDCamera":
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        del exc_type, exc_value, traceback
        self.close()

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def color_intrinsics(self) -> ColorIntrinsics:
        return self._intrinsics

    def open(self) -> None:
        self._is_open = True

    def close(self) -> None:
        self._is_open = False

    def _camera_from_base(self) -> np.ndarray:
        return np.linalg.inv(self._base_from_camera)

    def _project(self, base_point: tuple[float, float, float]) -> tuple[float, float, float] | None:
        homogeneous = np.asarray((*base_point, 1.0), dtype=float)
        camera_point = self._camera_from_base() @ homogeneous
        x, y, z = (float(value) for value in camera_point[:3])
        if z <= 0.03:
            return None
        u = self._intrinsics.fx * x / z + self._intrinsics.ppx
        v = self._intrinsics.fy * y / z + self._intrinsics.ppy
        if not (-100.0 <= u <= self._config.width + 100.0 and -100.0 <= v <= self._config.height + 100.0):
            return None
        return u, v, z

    def capture_frame(self) -> RealSenseFrame:
        if not self._is_open:
            raise RuntimeError("虚拟 RGB-D 相机尚未打开。")
        height, width = self._config.height, self._config.width
        rgb = np.full((height, width, 3), (38, 42, 48), dtype=np.uint8)
        depth = np.zeros((height, width), dtype=np.uint16)
        renderables = []
        for item in self._world.camera_object_truth():
            projection = self._project(item.grasp_point_m)
            if projection is not None:
                renderables.append((projection[2], projection, item))
        # painter's algorithm: farther first, nearer objects then occlude them.
        for camera_depth_m, projection, item in sorted(
            renderables,
            key=lambda item: item[0],
            reverse=True,
        ):
            u, v, z = projection
            apparent_half_width = max(
                4,
                int(
                    self._intrinsics.fx
                    * max(item.spec.dimensions_m[0], item.spec.dimensions_m[1])
                    / max(z, 1e-6)
                    / 2.0
                ),
            )
            center_x, center_y = int(round(u)), int(round(v))
            x_min = max(0, center_x - apparent_half_width)
            x_max = min(width, center_x + apparent_half_width + 1)
            y_min = max(0, center_y - apparent_half_width)
            y_max = min(height, center_y + apparent_half_width + 1)
            if x_min >= x_max or y_min >= y_max:
                continue
            region_shape = (y_max - y_min, x_max - x_min)
            rendered_depth = np.full(region_shape, z, dtype=float)
            if self._world.scene.depth_noise_std_m:
                rendered_depth += self._rng.normal(
                    0.0,
                    self._world.scene.depth_noise_std_m,
                    size=region_shape,
                )
            raw_depth = np.clip(
                np.rint(rendered_depth / self._config.depth_scale_m_per_unit),
                1,
                np.iinfo(np.uint16).max,
            ).astype(np.uint16)
            holes = self._rng.random(region_shape) < self._world.scene.depth_hole_probability
            raw_depth[holes] = 0
            rgb[y_min:y_max, x_min:x_max] = np.asarray(item.spec.color_rgb, dtype=np.uint8)
            depth[y_min:y_max, x_min:x_max] = raw_depth
            if self._world.scene.occlusion_fraction:
                occluded_rows = int((y_max - y_min) * self._world.scene.occlusion_fraction)
                if occluded_rows:
                    rgb[y_min:y_min + occluded_rows, x_min:x_max] = (38, 42, 48)
                    depth[y_min:y_min + occluded_rows, x_min:x_max] = 0
        if self._world.scene.rgb_noise_std:
            rgb = np.clip(
                rgb.astype(float) + self._rng.normal(0.0, self._world.scene.rgb_noise_std, size=rgb.shape),
                0,
                255,
            ).astype(np.uint8)
        self._frame_number += 1
        return RealSenseFrame(
            rgb=rgb,
            aligned_depth=depth,
            color_intrinsics=self._intrinsics,
            depth_scale_m_per_unit=self._config.depth_scale_m_per_unit,
            source=f"simulation:{self._world.scene.name}:{self._config.extrinsics_version}",
            frame_number=self._frame_number,
            timestamp_ms=time.time() * 1000.0,
        )
