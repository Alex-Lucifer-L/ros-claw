from __future__ import annotations

import json

import numpy as np
import pytest

from rosclaw_mini.vision.exceptions import TargetLocalizationError
from rosclaw_mini.vision.image import EncodedImage
from rosclaw_mini.vision.localization import (
    BasePositionEstimate,
    DepthEstimationConfig,
    PositionEstimate,
    RealSenseLocalizationService,
    deproject_color_pixel,
    estimate_robust_depth,
    localize_scene_object,
    normalized_box_to_pixels,
    parse_pixel_grounding_observation,
    transform_position_estimate_to_base,
)
from rosclaw_mini.vision.eye_to_hand import EyeToHandCalibration
from rosclaw_mini.vision.realsense import ColorIntrinsics, RealSenseFrame
from rosclaw_mini.vision.schemas import SceneObject, SceneObservation


def intrinsics(*, coefficients=(0.0,) * 5):
    return ColorIntrinsics(
        width=100,
        height=80,
        fx=100.0,
        fy=100.0,
        ppx=50.0,
        ppy=40.0,
        distortion_model="distortion.none",
        coefficients=coefficients,
    )


def frame(depth=None):
    return RealSenseFrame(
        rgb=np.zeros((80, 100, 3), dtype=np.uint8),
        aligned_depth=(
            np.full((80, 100), 1000, dtype=np.uint16)
            if depth is None
            else depth
        ),
        color_intrinsics=intrinsics(),
        depth_scale_m_per_unit=0.001,
        source="realsense:serial-1",
        frame_number=42,
        timestamp_ms=1234.5,
    )


def observation_and_target(box=(0.25, 0.25, 0.75, 0.75)):
    target = SceneObject(
        name="red box",
        location_in_image="center",
        bounding_box=box,
    )
    observation = SceneObservation(
        observation_id="obs-1",
        timestamp="2026-08-05T00:00:00+00:00",
        scene_description="桌面上有红盒子",
        objects=(target,),
        warnings=(),
        source="realsense:serial-1",
        model="fake-vl",
    )
    return observation, target


def test_normalized_box_to_pixels_is_strict_and_handles_image_edges():
    bounds = normalized_box_to_pixels((0.0, 0.1, 1.0, 0.9), width=100, height=80)
    assert (bounds.x_min, bounds.y_min, bounds.x_max, bounds.y_max) == (
        0,
        8,
        100,
        72,
    )

    invalid_boxes = (
        (0.2, 0.2, 0.2, 0.5),
        (-0.1, 0.2, 0.5, 0.5),
        (0.1, 0.2, 1.1, 0.5),
        (0.1, 0.2, float("nan"), 0.5),
        (True, 0.2, 0.5, 0.5),
        (0.1, 0.2, 0.5),
    )
    for invalid in invalid_boxes:
        with pytest.raises(TargetLocalizationError):
            normalized_box_to_pixels(invalid, width=100, height=80)


def test_robust_depth_filters_zero_nan_inf_and_outlier_values():
    depth = np.full((80, 100), 1000.0)
    bounds = normalized_box_to_pixels((0.2, 0.2, 0.8, 0.8), width=100, height=80)
    roi = depth[28:52, 35:65]
    roi.flat[:20] = 0.0
    roi.flat[20] = np.nan
    roi.flat[21] = np.inf
    roi.flat[22:30] = 5000.0

    result = estimate_robust_depth(
        depth,
        pixel_bounds=bounds,
        depth_scale_m_per_unit=0.001,
    )

    assert result.depth_m == pytest.approx(1.0)
    assert result.valid_samples < result.total_samples
    assert result.valid_depth_ratio > 0.8
    assert result.quality == "good"


def test_depth_rejects_no_data_low_ratio_and_mixed_surfaces():
    bounds = normalized_box_to_pixels((0.2, 0.2, 0.8, 0.8), width=100, height=80)
    with pytest.raises(TargetLocalizationError, match="有效样本不足"):
        estimate_robust_depth(
            np.zeros((80, 100), dtype=np.uint16),
            pixel_bounds=bounds,
            depth_scale_m_per_unit=0.001,
        )

    sparse = np.zeros((80, 100), dtype=np.uint16)
    sparse[39:41, 49:51] = 1000
    with pytest.raises(TargetLocalizationError, match="有效样本不足"):
        estimate_robust_depth(
            sparse,
            pixel_bounds=bounds,
            depth_scale_m_per_unit=0.001,
        )

    mixed = np.where(
        np.indices((80, 100))[1] % 2 == 0,
        500,
        900,
    ).astype(np.uint16)
    with pytest.raises(TargetLocalizationError, match="离散程度过大"):
        estimate_robust_depth(
            mixed,
            pixel_bounds=bounds,
            depth_scale_m_per_unit=0.001,
            config=DepthEstimationConfig(maximum_depth_spread_m=0.05),
        )


def test_depth_rejects_tiny_box_before_reading_single_pixel():
    bounds = normalized_box_to_pixels((0.49, 0.49, 0.51, 0.51), width=100, height=80)
    with pytest.raises(TargetLocalizationError, match="检测框太小"):
        estimate_robust_depth(
            np.full((80, 100), 1000, dtype=np.uint16),
            pixel_bounds=bounds,
            depth_scale_m_per_unit=0.001,
        )


def test_deprojection_uses_runtime_intrinsics_and_camera_optical_axes():
    point = deproject_color_pixel((60, 30), 2.0, intrinsics())
    assert point == pytest.approx((0.2, -0.2, 2.0))

    with pytest.raises(TargetLocalizationError, match="非零畸变"):
        deproject_color_pixel(
            (60, 30),
            2.0,
            intrinsics(coefficients=(0.1, 0.0, 0.0, 0.0, 0.0)),
        )


def test_position_estimate_is_bound_to_observation_and_source_frame():
    observation, target = observation_and_target()
    estimate = localize_scene_object(observation, target, frame())

    assert estimate.observation_id == "obs-1"
    assert estimate.object_name == "red box"
    assert estimate.center_pixel == (49, 39)
    assert estimate.depth_m == pytest.approx(1.0)
    assert estimate.camera_point_m == pytest.approx((-0.01, -0.01, 1.0))
    assert estimate.source_frame == 42
    assert estimate.source_timestamp_ms == pytest.approx(1234.5)
    assert estimate.to_dict()["camera_axes"] == "+X right, +Y down, +Z forward"


def test_position_estimate_transforms_to_base_with_calibration_provenance():
    position = PositionEstimate(
        observation_id="obs-1",
        object_name="red cap",
        bounding_box=(0.4, 0.4, 0.6, 0.6),
        center_pixel=(50, 40),
        depth_m=0.3,
        camera_point_m=(0.1, 0.2, 0.3),
        valid_depth_ratio=0.9,
        uncertainty_m=0.004,
        quality="good",
        source="realsense:serial-1",
        source_frame=42,
        source_timestamp_ms=1234.5,
    )
    calibration = EyeToHandCalibration(
        camera_serial="serial-1",
        width=100,
        height=80,
        camera_frame="realsense_color_optical_frame",
        base_frame="so100_plus_base",
        units="m",
        dataset_sha256="a" * 64,
        method="test",
        base_from_camera=(
            (1.0, 0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0, 2.0),
            (0.0, 0.0, 1.0, 3.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
        fit_point_count=6,
        validation_point_count=3,
        fit_rmse_m=0.01,
        fit_max_error_m=0.02,
        validation_rmse_m=0.015,
        validation_max_error_m=0.025,
        per_point_error_m=(("fit_001", 0.01),),
        activation_max_rmse_m=0.025,
        activation_max_error_m=0.04,
        active=True,
        activation_message="ok",
        created_at="2026-08-05T00:00:00+00:00",
    )

    result = transform_position_estimate_to_base(position, calibration)

    assert isinstance(result, BasePositionEstimate)
    assert result.base_point_m == pytest.approx((1.1, 2.2, 3.3))
    assert result.source_frame == 42
    assert result.localization_uncertainty_m == pytest.approx(0.004)
    assert result.calibration_sha256 == calibration.calibration_sha256
    assert result.to_dict()["base_frame"] == "so100_plus_base"


def test_pixel_grounding_is_strictly_validated_and_normalized_locally():
    raw = json.dumps(
        {
            "scene_description": "红盒子",
            "objects": [
                {
                    "name": "red box",
                    "location_in_image": "center",
                    "bounding_box": None,
                    "bounding_box_pixels": [25, 20, 75, 60],
                }
            ],
            "warnings": [],
        }
    )

    observation = parse_pixel_grounding_observation(
        raw,
        source="realsense:serial-1",
        model="fake-vl",
        image_width=100,
        image_height=80,
    )

    assert observation.objects[0].bounding_box == (0.25, 0.25, 0.75, 0.75)


@pytest.mark.parametrize(
    "pixel_box",
    (
        None,
        [25, 20, 75],
        [25.0, 20, 75, 60],
        [True, 20, 75, 60],
        [-1, 20, 75, 60],
        [25, 20, 101, 60],
        [25, 20, 25, 60],
        [25, 60, 75, 20],
    ),
)
def test_pixel_grounding_rejects_missing_non_integer_empty_or_out_of_range_box(
    pixel_box,
):
    raw = json.dumps(
        {
            "scene_description": "红盒子",
            "objects": [
                {
                    "name": "red box",
                    "location_in_image": "center",
                    "bounding_box": [0.25, 0.25, 0.75, 0.75],
                    "bounding_box_pixels": pixel_box,
                }
            ],
            "warnings": [],
        }
    )
    with pytest.raises(TargetLocalizationError, match="bounding_box_pixels"):
        parse_pixel_grounding_observation(
            raw,
            source="realsense:serial-1",
            model="fake-vl",
            image_width=100,
            image_height=80,
        )


def test_native_640_by_480_pixel_grounding_maps_to_public_normalized_box():
    raw = json.dumps(
        {
            "scene_description": "卷尺",
            "objects": [
                {
                    "name": "红色卷尺",
                    "location_in_image": "lower_center",
                    "bounding_box": None,
                    "bounding_box_pixels": [397, 360, 514, 452],
                }
            ],
            "warnings": [],
        }
    )
    observation = parse_pixel_grounding_observation(
        raw,
        source="realsense:046222070616",
        model="fake-vl",
        image_width=640,
        image_height=480,
    )
    assert observation.objects[0].bounding_box == pytest.approx(
        (397 / 640, 360 / 480, 514 / 640, 452 / 480)
    )


class FakeCamera:
    def __init__(self, events, rgbd_frame):
        self.events = events
        self.frame = rgbd_frame

    def __enter__(self):
        self.events.append("open")
        return self

    def capture_frame(self):
        self.events.append("capture")
        return self.frame

    def __exit__(self, *_args):
        self.events.append("close")


class FakeImageProcessor:
    def __init__(self, events):
        self.events = events

    def prepare(self, image, *, max_width):
        self.events.append(("prepare", image[0, 0].tolist(), max_width))
        return EncodedImage(b"jpeg", "image/jpeg", 100, 80)


class FakeVLM:
    model = "fake-vl"

    def __init__(self, events, objects=None):
        self.events = events
        self.objects = objects or [
            {
                "name": "red box",
                "location_in_image": "center",
                "bounding_box": None,
                "bounding_box_pixels": [25, 20, 75, 60],
            }
        ]

    def generate(self, *, image_bytes, mime_type, prompt):
        self.events.append(("vlm", image_bytes, mime_type, prompt))
        return json.dumps(
            {
                "scene_description": "目标",
                "objects": self.objects,
                "warnings": [],
            }
        )


def test_localization_service_uses_same_rgbd_frame_and_closes_before_vlm():
    events = []
    rgbd = frame()
    rgbd.rgb[0, 0] = (1, 2, 3)
    service = RealSenseLocalizationService(
        client=FakeVLM(events),
        camera_factory=lambda: FakeCamera(events, rgbd),
        image_processor=FakeImageProcessor(events),
    )

    result = service.locate("定位红盒子")

    assert events[:3] == ["open", "capture", "close"]
    assert events[3] == ("prepare", [3, 2, 1], 1280)
    assert events[4][0] == "vlm"
    assert "bounding_box_pixels" in events[4][3]
    assert "100×80" in events[4][3]
    assert result.position.source_frame == rgbd.frame_number
    assert result.position.observation_id == result.observation.observation_id


def test_localization_service_rejects_ambiguous_or_missing_target_box():
    events = []
    client = FakeVLM(
        events,
        objects=[
            {
                "name": "a",
                "location_in_image": "left",
                "bounding_box": None,
                "bounding_box_pixels": [10, 8, 30, 24],
            },
            {
                "name": "b",
                "location_in_image": "right",
                "bounding_box": None,
                "bounding_box_pixels": [60, 8, 80, 24],
            },
        ],
    )
    service = RealSenseLocalizationService(
        client=client,
        camera_factory=lambda: FakeCamera(events, frame()),
        image_processor=FakeImageProcessor(events),
    )
    with pytest.raises(TargetLocalizationError, match="候选数量为 2"):
        service.locate("定位一个物体")
