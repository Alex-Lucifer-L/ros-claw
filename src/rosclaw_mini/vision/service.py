"""Independent image -> VLM -> SceneObservation orchestration."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from rosclaw_mini.vision.camera import CameraAdapter
from rosclaw_mini.vision.image import OpenCVImageProcessor
from rosclaw_mini.vision.parser import SceneObservationParser
from rosclaw_mini.vision.prompt import build_vision_prompt
from rosclaw_mini.vision.schemas import SceneObservation
from rosclaw_mini.vision.vlm_client import VLMClient


CameraFactory = Callable[[int | str], CameraAdapter]


class VisionService:
    """Capture/load exactly one frame and return a read-only observation."""

    def __init__(
        self,
        *,
        client: VLMClient,
        camera_index: int = 0,
        camera_device: str | Path | None = None,
        max_width: int = 1280,
        camera_factory: CameraFactory = CameraAdapter,
        image_processor: OpenCVImageProcessor | None = None,
        parser: SceneObservationParser | None = None,
    ) -> None:
        if isinstance(camera_index, bool) or not isinstance(camera_index, int):
            raise ValueError("camera_index 必须是整数。")
        if camera_index < 0:
            raise ValueError("camera_index 不能为负数。")
        if camera_device is not None:
            camera_path = Path(camera_device)
            if not camera_path.is_absolute():
                raise ValueError("camera_device 必须是绝对设备路径。")
            camera_source: int | str = str(camera_path)
        else:
            camera_source = camera_index
        if isinstance(max_width, bool) or not isinstance(max_width, int):
            raise ValueError("vision_max_width 必须是整数。")
        if max_width <= 0:
            raise ValueError("vision_max_width 必须大于 0。")
        self._client = client
        self._camera_source = camera_source
        self._max_width = max_width
        self._camera_factory = camera_factory
        self._image_processor = image_processor or OpenCVImageProcessor()
        self._parser = parser or SceneObservationParser()

    def observe(
        self,
        *,
        question: str | None = None,
        image_path: Path | None = None,
        save_frame_path: Path | None = None,
    ) -> SceneObservation:
        if image_path is not None:
            frame = self._image_processor.load(Path(image_path))
            source = f"image:{Path(image_path)}"
        else:
            # Device ownership is deliberately limited to one capture.
            with self._camera_factory(self._camera_source) as camera:
                frame = camera.capture_frame()
            source = f"camera:{self._camera_source}"
            if save_frame_path is not None:
                self._image_processor.save(frame, Path(save_frame_path))

        encoded = self._image_processor.prepare(
            frame,
            max_width=self._max_width,
        )
        response = self._client.generate(
            image_bytes=encoded.data,
            mime_type=encoded.mime_type,
            prompt=build_vision_prompt(question),
        )
        return self._parser.parse(
            response,
            source=source,
            model=self._client.model,
        )
