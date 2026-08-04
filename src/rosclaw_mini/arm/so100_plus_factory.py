from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import importlib
import json
from dataclasses import dataclass
from pathlib import Path
import sys
from types import ModuleType


SO100_PLUS_MOTOR_NAMES = (
    "shoulder_rotation_joint",
    "shoulder_pitch_joint",
    "ellbow_joint",
    "wrist_pitch_joint",
    "wrist_jaw_joint",
    "wrist_roll_joint",
    "gripper_joint",
)

SO100_PLUS_MOTORS = {
    name: (motor_id, "sts3215")
    for motor_id, name in enumerate(SO100_PLUS_MOTOR_NAMES, start=1)
}

_CALIBRATION_VECTOR_FIELDS = (
    "homing_offset",
    "drive_mode",
    "start_pos",
    "end_pos",
    "calib_mode",
)


class SO100PlusConfigurationError(ValueError):
    """SO-100 Plus 配置未达到连接前的安全要求。"""


@dataclass(frozen=True)
class SO100PlusRobotConfig:
    """创建单臂 Robot 所需的机器相关配置，不负责连接硬件。"""

    port: str | Path
    calibration_dir: str | Path
    follower_name: str

    @property
    def calibration_path(self) -> Path:
        return Path(self.calibration_dir) / f"{self.follower_name}_follower.json"


@dataclass(frozen=True)
class SO100PlusCameraConfig:
    """README 中 SO-100 Plus OpenCV RGB 摄像头的显式配置。"""

    name: str
    device: int | str | Path
    fps: int = 20
    width: int = 640
    height: int = 480
    color_mode: str = "rgb"

    @property
    def connection_target(self) -> int | str:
        if isinstance(self.device, int):
            return self.device
        return str(self.device)


class SO100PlusReadOnlyRobot:
    """只打开 follower 串口并加载已有校准，不写任何电机寄存器。"""

    def __init__(
        self,
        follower_name: str,
        follower_bus,
        calibration: dict,
    ) -> None:
        self.follower_arms = {follower_name: follower_bus}
        self._follower_name = follower_name
        self._calibration = deepcopy(calibration)
        self.is_connected = False

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError("只读 SO-100 Plus Robot 已经连接。")

        follower_bus = self.follower_arms[self._follower_name]
        follower_bus.set_calibration(deepcopy(self._calibration))
        follower_bus.connect()
        self.is_connected = True

    def disconnect(self) -> None:
        follower_bus = self.follower_arms[self._follower_name]
        if getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        self.is_connected = False


def validate_so100_plus_config(config: SO100PlusRobotConfig) -> Path:
    """在导入或连接 LeRobot 前验证端口和校准文件。"""

    if not config.follower_name.isidentifier():
        raise SO100PlusConfigurationError(
            "follower_name 必须是简单名称，例如 'main' 或 'right'。"
        )

    port = Path(config.port)
    if not port.exists():
        raise SO100PlusConfigurationError(f"串口路径不存在：{port}")
    if not port.is_char_device():
        raise SO100PlusConfigurationError(f"串口路径不是字符设备：{port}")

    calibration_path = config.calibration_path
    if not calibration_path.is_file():
        raise SO100PlusConfigurationError(
            f"校准文件不存在：{calibration_path}。已阻止 LeRobot 自动进入校准。"
        )

    try:
        with calibration_path.open(encoding="utf-8") as calibration_file:
            calibration = json.load(calibration_file)
    except (OSError, json.JSONDecodeError) as exc:
        raise SO100PlusConfigurationError(
            f"无法读取校准文件：{calibration_path}"
        ) from exc

    if not isinstance(calibration, dict):
        raise SO100PlusConfigurationError("校准文件顶层必须是 JSON 对象。")

    motor_names = calibration.get("motor_names")
    if motor_names != list(SO100_PLUS_MOTOR_NAMES):
        raise SO100PlusConfigurationError(
            "校准文件的 motor_names 与 SO-100 Plus 的 7 个电机名称或顺序不匹配。"
        )

    for field in _CALIBRATION_VECTOR_FIELDS:
        values = calibration.get(field)
        if not isinstance(values, list) or len(values) != len(SO100_PLUS_MOTOR_NAMES):
            raise SO100PlusConfigurationError(
                f"校准字段 {field!r} 必须包含 7 个值。"
            )

    return calibration_path


def validate_so100_plus_camera_configs(
    configs: tuple[SO100PlusCameraConfig, ...],
) -> None:
    """在打开摄像头前验证名称、视频设备和图像格式。"""

    names: set[str] = set()
    for config in configs:
        if not config.name.isidentifier():
            raise SO100PlusConfigurationError(
                "摄像头 name 必须是简单名称，例如 'right'。"
            )
        if config.name in names:
            raise SO100PlusConfigurationError(
                f"摄像头名称重复：{config.name!r}。"
            )
        names.add(config.name)

        if isinstance(config.device, bool):
            raise SO100PlusConfigurationError(
                "摄像头 device 不能是布尔值。"
            )
        if isinstance(config.device, int):
            if config.device < 0:
                raise SO100PlusConfigurationError(
                    "摄像头索引不能为负数。"
                )
            device_path = Path(f"/dev/video{config.device}")
        elif isinstance(config.device, (str, Path)):
            device_path = Path(config.device)
            if not device_path.is_absolute():
                raise SO100PlusConfigurationError(
                    "摄像头设备路径必须是绝对路径。"
                )
        else:
            raise SO100PlusConfigurationError(
                "摄像头 device 必须是整数索引或设备路径。"
            )
        if not device_path.exists():
            raise SO100PlusConfigurationError(
                f"摄像头设备不存在：{device_path}"
            )
        if not device_path.is_char_device():
            raise SO100PlusConfigurationError(
                f"摄像头路径不是字符设备：{device_path}"
            )

        integer_fields = (
            ("fps", config.fps),
            ("width", config.width),
            ("height", config.height),
        )
        for field_name, value in integer_fields:
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise SO100PlusConfigurationError(
                    f"摄像头 {field_name} 必须是正整数。"
                )
        if config.fps > 120:
            raise SO100PlusConfigurationError(
                "摄像头 fps 不能超过 120。"
            )
        if config.color_mode not in {"rgb", "bgr"}:
            raise SO100PlusConfigurationError(
                "摄像头 color_mode 只能是 'rgb' 或 'bgr'。"
            )


def create_so100_plus_cameras(
    configs: tuple[SO100PlusCameraConfig, ...],
    *,
    opencv_camera_class=None,
) -> dict[str, object]:
    """创建尚未连接的 OpenCV 摄像头，不触碰机械臂。"""

    validate_so100_plus_camera_configs(configs)
    if not configs:
        return {}
    if opencv_camera_class is None:
        try:
            opencv_camera_class = _import_opencv_camera()
        except ImportError as exc:
            raise SO100PlusConfigurationError(
                "无法导入 LeRobot OpenCVCamera，请在已安装 OpenCV "
                "的 rosclaw-mini-py310 环境中运行。"
            ) from exc

    return {
        config.name: opencv_camera_class(
            config.connection_target,
            fps=config.fps,
            width=config.width,
            height=config.height,
            color_mode=config.color_mode,
        )
        for config in configs
    }


def create_so100_plus_robot(
    config: SO100PlusRobotConfig,
    *,
    motors_bus_class=None,
    manipulator_robot_class=None,
):
    """创建尚未连接的单臂 ManipulatorRobot。"""

    calibration_path = validate_so100_plus_config(config)

    if motors_bus_class is None:
        try:
            motors_bus_class = (
                _import_feetech_motors_bus_without_optional_app_dependencies()
            )
        except ImportError as exc:
            raise SO100PlusConfigurationError(
                "无法导入 FeetechMotorsBus，请在已安装 LeRobot Feetech 驱动的环境中运行。"
            ) from exc

    if manipulator_robot_class is None:
        try:
            manipulator_robot_class = (
                _import_manipulator_robot_without_required_torch_runtime()
            )
        except ImportError as exc:
            raise SO100PlusConfigurationError(
                "无法导入 ManipulatorRobot，请在已安装 LeRobot 的环境中运行。"
            ) from exc

    follower_bus = motors_bus_class(
        port=str(config.port),
        motors=dict(SO100_PLUS_MOTORS),
    )

    return manipulator_robot_class(
        robot_type="so100",
        calibration_dir=calibration_path.parent.resolve(),
        follower_arms={config.follower_name: follower_bus},
    )


def create_so100_plus_readonly_robot(
    config: SO100PlusRobotConfig,
    *,
    motors_bus_class=None,
) -> SO100PlusReadOnlyRobot:
    """创建只读单臂包装器；连接阶段不会写扭矩、配置或目标位置。"""

    calibration_path = validate_so100_plus_config(config)
    with calibration_path.open(encoding="utf-8") as calibration_file:
        calibration = json.load(calibration_file)

    if motors_bus_class is None:
        try:
            motors_bus_class = (
                _import_feetech_motors_bus_without_optional_app_dependencies()
            )
        except ImportError as exc:
            raise SO100PlusConfigurationError(
                "无法导入 FeetechMotorsBus，请在已安装 LeRobot Feetech "
                "驱动的环境中运行。"
            ) from exc

    follower_bus = motors_bus_class(
        port=str(config.port),
        motors=dict(SO100_PLUS_MOTORS),
    )
    return SO100PlusReadOnlyRobot(
        follower_name=config.follower_name,
        follower_bus=follower_bus,
        calibration=calibration,
    )


def _import_feetech_motors_bus_without_optional_app_dependencies():
    """绕开旧 LeRobot 日志工具对 Hydra 的无关顶层依赖。"""

    utils_module_name = "lerobot.common.utils.utils"
    existing_utils_module = sys.modules.get(utils_module_name)
    installed_temporary_module = existing_utils_module is None

    if installed_temporary_module:
        lightweight_utils = ModuleType(utils_module_name)
        lightweight_utils.capture_timestamp_utc = (
            lambda: datetime.now(timezone.utc)
        )
        sys.modules[utils_module_name] = lightweight_utils

    try:
        feetech_module = importlib.import_module(
            "lerobot.common.robot_devices.motors.feetech"
        )
        return feetech_module.FeetechMotorsBus
    finally:
        if installed_temporary_module:
            sys.modules.pop(utils_module_name, None)


def _import_manipulator_robot_without_required_torch_runtime():
    """只为 connect/disconnect 兼容旧模块强制存在的 torch 类型注解。"""

    torch_module_name = "torch"
    existing_torch_module = sys.modules.get(torch_module_name)
    installed_temporary_module = existing_torch_module is None

    if installed_temporary_module:
        lightweight_torch = ModuleType(torch_module_name)
        lightweight_torch.Tensor = object
        sys.modules[torch_module_name] = lightweight_torch

    try:
        manipulator_module = importlib.import_module(
            "lerobot.common.robot_devices.robots.manipulator"
        )
        return manipulator_module.ManipulatorRobot
    finally:
        if installed_temporary_module:
            sys.modules.pop(torch_module_name, None)


def _import_opencv_camera():
    utils_module_name = "lerobot.common.utils.utils"
    existing_utils_module = sys.modules.get(utils_module_name)
    installed_temporary_module = existing_utils_module is None

    if installed_temporary_module:
        lightweight_utils = ModuleType(utils_module_name)
        lightweight_utils.capture_timestamp_utc = (
            lambda: datetime.now(timezone.utc)
        )
        sys.modules[utils_module_name] = lightweight_utils

    try:
        camera_module = importlib.import_module(
            "lerobot.common.robot_devices.cameras.opencv"
        )
        return camera_module.OpenCVCamera
    finally:
        if installed_temporary_module:
            sys.modules.pop(utils_module_name, None)
