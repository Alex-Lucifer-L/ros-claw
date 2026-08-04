"""单帧 OpenCV 摄像头适配器，不长期独占设备。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from rosclaw_mini.vision.exceptions import CameraOpenError, FrameCaptureError


CaptureFactory = Callable[[int], Any]


def _import_cv2():
    try:
        import cv2
    except (ImportError, OSError) as error:
        raise CameraOpenError(
            "OpenCV 不可用；请安装 requirements.txt 中的 opencv-python-headless。"
        ) from error
    return cv2


class CameraAdapter:
    """打开指定摄像头、读取一帧并确保释放。"""

    def __init__(
        self,
        camera_index: int = 0,
        *,
        capture_factory: CaptureFactory | None = None,
    ) -> None:
        if isinstance(camera_index, bool) or not isinstance(camera_index, int):
            raise ValueError("camera_index 必须是整数。")
        if camera_index < 0:
            raise ValueError("camera_index 不能为负数。")
        self.camera_index = camera_index
        self._capture_factory = capture_factory
        self._capture = None

    @property
    def is_open(self) -> bool:
        return self._capture is not None

    def open(self) -> None:
        if self._capture is not None:
            return
        factory = self._capture_factory or _import_cv2().VideoCapture
        try:
            capture = factory(self.camera_index)
        except Exception as error:
            raise CameraOpenError(
                f"无法打开摄像头 {self.camera_index}：{error}"
            ) from error
        try:
            opened = bool(capture is not None and capture.isOpened())
        except Exception as error:
            if capture is not None:
                try:
                    capture.release()
                except Exception:
                    pass
            raise CameraOpenError(
                f"检查摄像头 {self.camera_index} 状态失败：{error}"
            ) from error
        if not opened:
            if capture is not None:
                try:
                    capture.release()
                except Exception:
                    pass
            raise CameraOpenError(
                f"无法打开摄像头 {self.camera_index}，请检查设备编号和权限。"
            )
        self._capture = capture

    def capture_frame(self):
        if self._capture is None:
            raise FrameCaptureError("摄像头尚未打开。")
        try:
            success, frame = self._capture.read()
        except Exception as error:
            raise FrameCaptureError(
                f"摄像头 {self.camera_index} 读取图像失败：{error}"
            ) from error
        if not success or frame is None:
            raise FrameCaptureError("摄像头已打开，但未成功读取图像。")
        return frame

    def close(self) -> None:
        capture = self._capture
        self._capture = None
        if capture is not None:
            try:
                capture.release()
            except Exception as error:
                raise CameraOpenError(
                    f"释放摄像头 {self.camera_index} 失败：{error}"
                ) from error

    def __enter__(self) -> "CameraAdapter":
        self.open()
        return self

    def __exit__(self, _error_type, _error, _traceback) -> None:
        self.close()

