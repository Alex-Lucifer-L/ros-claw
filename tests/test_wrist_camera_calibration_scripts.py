from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from rosclaw_mini.vision.calibration import (
    CameraCalibrationIdentity,
    CameraIntrinsicCalibration,
    SO100_PLUS_WRIST_CHECKERBOARD,
)
from rosclaw_mini.vision.exceptions import CameraCalibrationError
from rosclaw_mini.vision.exceptions import CheckerboardDetectionError

from scripts.calibrate_wrist_camera_intrinsics import build_parser as build_solver_parser
from scripts.check_wrist_camera_intrinsics import (
    build_parser as build_intrinsic_check_parser,
    preview_intrinsic_calibration,
)
from scripts.collect_wrist_camera_calibration_images import (
    build_parser as build_collector_parser,
    collect_images,
    collect_images_with_preview,
)


class FakeFrame:
    shape = (480, 640, 3)


class FakeCamera:
    def __init__(self, source, events):
        self.source = source
        self.events = events

    def __enter__(self):
        self.events.append(("open", self.source))
        return self

    def capture_frame(self):
        self.events.append(("capture", self.source))
        return FakeFrame()

    def __exit__(self, *args):
        self.events.append(("close", self.source))


class FakeProcessor:
    def __init__(self, events):
        self.events = events

    def dimensions(self, _frame):
        return 640, 480

    def save(self, _frame, path):
        self.events.append(("save", path))


def test_collection_is_manual_and_never_constructs_an_arm(tmp_path: Path):
    events = []
    answers = iter(["", "", "q"])
    device = Path("/dev/v4l/by-id/fake-wrist-camera-video-index0")
    paths = collect_images(
        device=device,
        output_directory=tmp_path,
        count=5,
        expected_width=640,
        expected_height=480,
        overwrite=False,
        input_func=lambda _prompt: next(answers),
        output_func=lambda _message: None,
        camera_factory=lambda source: FakeCamera(source, events),
        image_processor=FakeProcessor(events),
    )
    assert paths == (
        tmp_path / "wrist_view_001.jpg",
        tmp_path / "wrist_view_002.jpg",
    )
    assert [event[0] for event in events] == [
        "open", "capture", "close", "save",
        "open", "capture", "close", "save",
    ]


def test_intrinsic_solver_defaults_match_confirmed_board():
    args = build_solver_parser().parse_args(
        [
            "--images-dir", "/tmp/images",
            "--output", "/tmp/intrinsics.json",
            "--device", "/dev/video2",
            "--vendor-id", "0c58",
            "--product-id", "637a",
            "--serial", "placeholder",
        ]
    )
    assert args.inner_columns == 7
    assert args.inner_rows == 6
    assert args.square_size_mm == 24.0
    assert args.minimum_views == 10
    assert (args.width, args.height, args.pixel_format) == (640, 480, "YUYV")


class FakePreviewCV2:
    WINDOW_NORMAL = 0
    FONT_HERSHEY_SIMPLEX = 0
    LINE_AA = 0

    def __init__(self, keys=None):
        self.shown = 0
        self.destroyed = False
        self.keys = list(keys or [ord("c")])

    def namedWindow(self, _name, _mode):
        pass

    def drawChessboardCorners(self, image, pattern, corners, found):
        assert pattern == (7, 6)
        assert corners.shape == (42, 1, 2)
        assert found is True

    def putText(self, *args, **kwargs):
        pass

    def imshow(self, _name, _image):
        self.shown += 1

    def waitKey(self, _milliseconds):
        return self.keys.pop(0)

    def destroyWindow(self, _name):
        self.destroyed = True


class FakePreviewCamera(FakeCamera):
    def capture_frame(self):
        self.events.append(("capture", self.source))
        return np.zeros((480, 640, 3), dtype=np.uint8)


def test_live_preview_saves_only_after_complete_corner_detection(tmp_path: Path):
    events = []
    cv2 = FakePreviewCV2()
    paths = collect_images_with_preview(
        device=Path("/dev/v4l/by-id/fake-wrist-camera-video-index0"),
        output_directory=tmp_path,
        count=1,
        expected_width=640,
        expected_height=480,
        overwrite=False,
        output_func=lambda _message: None,
        camera_factory=lambda source: FakePreviewCamera(source, events),
        image_processor=FakeProcessor(events),
        cv2_module=cv2,
        corner_detector=lambda frame, spec, **kwargs: np.zeros(
            (42, 1, 2), dtype=np.float32
        ),
    )
    assert paths == (tmp_path / "wrist_view_001.jpg",)
    assert [event[0] for event in events] == [
        "open", "capture", "save", "close"
    ]
    assert cv2.shown == 1
    assert cv2.destroyed is True


def test_collector_enables_live_preview_by_default():
    args = build_collector_parser().parse_args(
        [
            "--device", "/dev/video2",
            "--output-dir", "/tmp/images",
        ]
    )
    assert args.preview is True
    assert args.count == 15


def test_live_preview_refuses_capture_without_complete_corners(tmp_path: Path):
    events = []
    outputs = []
    cv2 = FakePreviewCV2(keys=[ord("c"), ord("q")])

    def reject_corners(*args, **kwargs):
        raise CheckerboardDetectionError("incomplete")

    paths = collect_images_with_preview(
        device=Path("/dev/v4l/by-id/fake-wrist-camera-video-index0"),
        output_directory=tmp_path,
        count=1,
        expected_width=640,
        expected_height=480,
        overwrite=False,
        output_func=outputs.append,
        camera_factory=lambda source: FakePreviewCamera(source, events),
        image_processor=FakeProcessor(events),
        cv2_module=cv2,
        corner_detector=reject_corners,
    )
    assert paths == ()
    assert not any(event[0] == "save" for event in events)
    assert any("未保存" in message for message in outputs)
    assert cv2.destroyed is True


def validation_calibration(device: str):
    return CameraIntrinsicCalibration(
        camera_identity=CameraCalibrationIdentity(
            device=device,
            vendor_id="0c58",
            product_id="637a",
            serial="fake-serial",
            width=640,
            height=480,
            pixel_format="YUYV",
        ),
        checkerboard=SO100_PLUS_WRIST_CHECKERBOARD,
        camera_matrix=(
            (500.0, 0.0, 320.0),
            (0.0, 510.0, 240.0),
            (0.0, 0.0, 1.0),
        ),
        distortion_coefficients=(0.01, -0.02, 0.0, 0.0, 0.0),
        rms_reprojection_error_px=0.25,
        per_view_reprojection_errors_px=(0.2,),
        accepted_images=("training.jpg",),
        rejected_images=(),
        created_at="2026-08-05T00:00:00+00:00",
    )


class FakeIntrinsicCheckCV2(FakePreviewCV2):
    def hconcat(self, images):
        return np.concatenate(images, axis=1)


class ValidationProcessor(FakeProcessor):
    def save(self, frame, path):
        self.events.append(("save", path, frame.copy()))


def test_intrinsic_check_parser_requires_explicit_paths_and_defaults():
    args = build_intrinsic_check_parser().parse_args(
        [
            "--device", "/dev/v4l/by-id/fake-camera",
            "--calibration", "/tmp/intrinsics.json",
            "--output-dir", "/tmp/validation",
        ]
    )
    assert args.count == 3
    assert args.alpha == 1.0
    assert args.pixel_format == "YUYV"
    assert args.acknowledge_camera_capture is False


def test_intrinsic_check_saves_the_same_new_raw_and_corrected_view(tmp_path: Path):
    events = []
    outputs = []
    device = Path("/dev/v4l/by-id/fake-wrist-camera-video-index0")
    cv2 = FakeIntrinsicCheckCV2(keys=[ord("c")])

    samples = preview_intrinsic_calibration(
        device=device,
        calibration=validation_calibration(str(device)),
        output_directory=tmp_path,
        count=1,
        alpha=1.0,
        pixel_format="YUYV",
        overwrite=False,
        output_func=outputs.append,
        camera_factory=lambda source: FakePreviewCamera(source, events),
        image_processor=ValidationProcessor(events),
        cv2_module=cv2,
        undistorter=lambda frame, calibration, **kwargs: frame + 7,
        reprojection_error_measure=lambda frame, calibration, **kwargs: 0.42,
    )

    assert len(samples) == 1
    raw_path, corrected_path, error = samples[0]
    assert raw_path.name == "validation_view_001_raw.jpg"
    assert corrected_path.name == "validation_view_001_undistorted.jpg"
    assert error == pytest.approx(0.42)
    assert [event[0] for event in events] == [
        "open", "capture", "save", "save", "close"
    ]
    assert np.all(events[2][2] == 0)
    assert np.all(events[3][2] == 7)
    assert cv2.shown == 1
    assert cv2.destroyed is True
    assert any("0.420000 px" in message for message in outputs)


def test_intrinsic_check_rejects_other_device_before_opening_camera(tmp_path: Path):
    events = []
    calibration = validation_calibration(
        "/dev/v4l/by-id/certified-wrist-camera-video-index0"
    )
    with pytest.raises(CameraCalibrationError, match="摄像头与当前设备不一致"):
        preview_intrinsic_calibration(
            device=Path("/dev/v4l/by-id/other-camera-video-index0"),
            calibration=calibration,
            output_directory=tmp_path,
            count=1,
            alpha=1.0,
            pixel_format="YUYV",
            overwrite=False,
            camera_factory=lambda source: FakePreviewCamera(source, events),
            cv2_module=FakeIntrinsicCheckCV2(),
        )
    assert events == []
