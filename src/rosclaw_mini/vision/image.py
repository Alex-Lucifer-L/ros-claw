"""本地图像加载、等比例缩放、JPEG 编码和显式保存。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rosclaw_mini.vision.exceptions import (
    ImageEncodeError,
    ImageLoadError,
)


@dataclass(frozen=True)
class EncodedImage:
    data: bytes
    mime_type: str
    width: int
    height: int


def _import_cv2_for_image():
    try:
        import cv2
    except (ImportError, OSError) as error:
        raise ImageEncodeError(
            "OpenCV 不可用；请安装 requirements.txt 中的 opencv-python-headless。"
        ) from error
    return cv2


class OpenCVImageProcessor:
    def __init__(self, *, cv2_module: Any | None = None) -> None:
        self._cv2 = cv2_module

    @property
    def cv2(self):
        if self._cv2 is None:
            self._cv2 = _import_cv2_for_image()
        return self._cv2

    def load(self, image_path: Path):
        path = Path(image_path)
        if not path.is_file():
            raise ImageLoadError(f"本地图像不存在：{path}")
        try:
            frame = self.cv2.imread(str(path), self.cv2.IMREAD_COLOR)
        except Exception as error:
            raise ImageLoadError(f"无法读取本地图像 {path}：{error}") from error
        if frame is None:
            raise ImageLoadError(f"本地图像不存在或无法读取：{path}")
        return frame

    @staticmethod
    def dimensions(frame) -> tuple[int, int]:
        shape = getattr(frame, "shape", None)
        if not isinstance(shape, tuple) or len(shape) < 2:
            raise ImageEncodeError("图像没有有效的 height/width shape。")
        height, width = shape[:2]
        if (
            isinstance(width, bool)
            or isinstance(height, bool)
            or not isinstance(width, int)
            or not isinstance(height, int)
            or width <= 0
            or height <= 0
        ):
            raise ImageEncodeError("图像宽高必须是正整数。")
        return width, height

    def resize_to_max_width(self, frame, max_width: int):
        if isinstance(max_width, bool) or not isinstance(max_width, int) or max_width <= 0:
            raise ImageEncodeError("vision_max_width 必须是正整数。")
        width, height = self.dimensions(frame)
        if width <= max_width:
            return frame
        target_height = max(1, round(height * max_width / width))
        try:
            return self.cv2.resize(
                frame,
                (max_width, target_height),
                interpolation=self.cv2.INTER_AREA,
            )
        except Exception as error:
            raise ImageEncodeError(f"图像等比例缩放失败：{error}") from error

    def encode_jpeg(self, frame) -> EncodedImage:
        width, height = self.dimensions(frame)
        try:
            success, buffer = self.cv2.imencode(
                ".jpg",
                frame,
                [int(self.cv2.IMWRITE_JPEG_QUALITY), 85],
            )
        except Exception as error:
            raise ImageEncodeError(f"图像 JPEG 编码失败：{error}") from error
        if not success or buffer is None:
            raise ImageEncodeError("图像 JPEG 编码失败。")
        try:
            data = bytes(buffer.tobytes())
        except Exception as error:
            raise ImageEncodeError(f"无法读取 JPEG 编码结果：{error}") from error
        if not data:
            raise ImageEncodeError("JPEG 编码结果为空。")
        return EncodedImage(
            data=data,
            mime_type="image/jpeg",
            width=width,
            height=height,
        )

    def prepare(self, frame, *, max_width: int) -> EncodedImage:
        resized = self.resize_to_max_width(frame, max_width)
        return self.encode_jpeg(resized)

    def save(self, frame, output_path: Path) -> None:
        path = Path(output_path)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            success = self.cv2.imwrite(str(path), frame)
        except Exception as error:
            raise ImageEncodeError(f"保存捕获帧失败 {path}：{error}") from error
        if not success:
            raise ImageEncodeError(f"保存捕获帧失败：{path}")

