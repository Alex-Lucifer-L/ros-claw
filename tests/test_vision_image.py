from __future__ import annotations

from pathlib import Path

import pytest

from rosclaw_mini.vision.exceptions import ImageLoadError
from rosclaw_mini.vision.image import OpenCVImageProcessor


class FakeFrame:
    def __init__(self, height, width):
        self.shape = (height, width, 3)


class FakeBuffer:
    def tobytes(self):
        return b"jpeg-data"


class FakeCV2:
    IMREAD_COLOR = 1
    INTER_AREA = 3
    IMWRITE_JPEG_QUALITY = 4

    def __init__(self):
        self.resize_call = None
        self.imread_result = FakeFrame(10, 20)

    def resize(self, frame, dimensions, interpolation):
        self.resize_call = (frame, dimensions, interpolation)
        return FakeFrame(dimensions[1], dimensions[0])

    def imencode(self, extension, frame, options):
        return True, FakeBuffer()

    def imread(self, path, mode):
        return self.imread_result

    def imwrite(self, path, frame):
        return True


def test_image_resize_preserves_aspect_ratio_and_encodes_jpeg():
    cv2 = FakeCV2()
    processor = OpenCVImageProcessor(cv2_module=cv2)
    encoded = processor.prepare(FakeFrame(600, 1200), max_width=300)

    assert cv2.resize_call[1] == (300, 150)
    assert encoded.width == 300
    assert encoded.height == 150
    assert encoded.mime_type == "image/jpeg"
    assert encoded.data == b"jpeg-data"


def test_small_image_is_not_resized():
    cv2 = FakeCV2()
    processor = OpenCVImageProcessor(cv2_module=cv2)
    encoded = processor.prepare(FakeFrame(100, 200), max_width=300)
    assert cv2.resize_call is None
    assert (encoded.width, encoded.height) == (200, 100)


def test_local_image_missing_is_clear_error(tmp_path: Path):
    processor = OpenCVImageProcessor(cv2_module=FakeCV2())
    with pytest.raises(ImageLoadError, match="本地图像不存在"):
        processor.load(tmp_path / "missing.jpg")

