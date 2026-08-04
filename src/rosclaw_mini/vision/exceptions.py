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


class CameraCalibrationError(VisionError):
    """相机标定配置、样本或求解结果无效。"""


class CheckerboardDetectionError(CameraCalibrationError):
    """图像中未检测到完整、指定规格的棋盘格。"""


class InsufficientCalibrationDataError(CameraCalibrationError):
    """有效且尺寸一致的标定视角不足。"""


class HandEyeCalibrationError(CameraCalibrationError):
    """手眼标定数据、可观测性或求解结果无效。"""
