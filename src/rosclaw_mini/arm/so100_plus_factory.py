from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


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
            from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
        except ImportError as exc:
            raise SO100PlusConfigurationError(
                "无法导入 FeetechMotorsBus，请在已安装 LeRobot Feetech 驱动的环境中运行。"
            ) from exc
        motors_bus_class = FeetechMotorsBus

    if manipulator_robot_class is None:
        try:
            from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
        except ImportError as exc:
            raise SO100PlusConfigurationError(
                "无法导入 ManipulatorRobot，请在已安装 LeRobot 的环境中运行。"
            ) from exc
        manipulator_robot_class = ManipulatorRobot

    follower_bus = motors_bus_class(
        port=str(config.port),
        motors=dict(SO100_PLUS_MOTORS),
    )

    return manipulator_robot_class(
        robot_type="so100",
        calibration_dir=calibration_path.parent.resolve(),
        follower_arms={config.follower_name: follower_bus},
    )
