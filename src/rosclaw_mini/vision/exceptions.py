"""视觉观察链的分层异常。"""


class VisionError(RuntimeError):
    """所有可向 CLI 友好展示的视觉错误基类。"""


class CameraOpenError(VisionError):
    """摄像头无法打开。"""


class FrameCaptureError(VisionError):
    """摄像头已打开但无法读取一帧。"""


class ImageLoadError(VisionError):
    """本地图像不存在或无法解码。"""


class ImageEncodeError(VisionError):
    """图像缩放、保存或编码失败。"""


class VLMConfigurationError(VisionError):
    """视觉模型配置无效或缺失。"""


class VLMRequestError(VisionError):
    """视觉模型网络请求失败。"""


class VLMResponseParseError(VisionError):
    """视觉模型文本不是可解析的 JSON。"""


class SceneObservationValidationError(VisionError):
    """视觉模型 JSON 不符合 SceneObservation 协议。"""

