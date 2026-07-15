import json
from pathlib import Path

import pytest

from rosclaw_mini.arm.so100_plus_factory import (
    SO100_PLUS_MOTOR_NAMES,
    SO100_PLUS_MOTORS,
    SO100PlusConfigurationError,
    SO100PlusRobotConfig,
    create_so100_plus_robot,
    validate_so100_plus_config,
)


def make_calibration() -> dict:
    return {
        "homing_offset": [0] * 7,
        "drive_mode": [0] * 7,
        "start_pos": [0] * 7,
        "end_pos": [1] * 7,
        "calib_mode": ["DEGREE"] * 7,
        "motor_names": list(SO100_PLUS_MOTOR_NAMES),
    }


def write_calibration(
    tmp_path: Path,
    calibration: dict | None = None,
    follower_name: str = "right",
) -> Path:
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    calibration_path = calibration_dir / f"{follower_name}_follower.json"
    calibration_path.write_text(
        json.dumps(calibration or make_calibration()),
        encoding="utf-8",
    )
    return calibration_dir


def test_preflight_accepts_expected_seven_motor_calibration(tmp_path):
    calibration_dir = write_calibration(tmp_path)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    calibration_path = validate_so100_plus_config(config)

    assert calibration_path == calibration_dir / "right_follower.json"
    assert list(SO100_PLUS_MOTORS) == list(SO100_PLUS_MOTOR_NAMES)
    assert list(SO100_PLUS_MOTORS.values()) == [
        (motor_id, "sts3215") for motor_id in range(1, 8)
    ]


def test_preflight_rejects_missing_port(tmp_path):
    calibration_dir = write_calibration(tmp_path)
    config = SO100PlusRobotConfig(
        port=tmp_path / "missing-port",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    with pytest.raises(SO100PlusConfigurationError, match="串口路径不存在"):
        validate_so100_plus_config(config)


def test_preflight_rejects_regular_file_as_port(tmp_path):
    calibration_dir = write_calibration(tmp_path)
    regular_file = tmp_path / "not-a-serial-device"
    regular_file.touch()
    config = SO100PlusRobotConfig(
        port=regular_file,
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    with pytest.raises(SO100PlusConfigurationError, match="不是字符设备"):
        validate_so100_plus_config(config)


def test_preflight_rejects_missing_calibration_file(tmp_path):
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    with pytest.raises(SO100PlusConfigurationError, match="已阻止 LeRobot 自动进入校准"):
        validate_so100_plus_config(config)


def test_preflight_rejects_wrong_motor_name_order(tmp_path):
    calibration = make_calibration()
    calibration["motor_names"][0:2] = reversed(calibration["motor_names"][0:2])
    calibration_dir = write_calibration(tmp_path, calibration)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    with pytest.raises(SO100PlusConfigurationError, match="名称或顺序不匹配"):
        validate_so100_plus_config(config)


def test_preflight_rejects_wrong_calibration_vector_length(tmp_path):
    calibration = make_calibration()
    calibration["homing_offset"] = [0] * 6
    calibration_dir = write_calibration(tmp_path, calibration)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    with pytest.raises(SO100PlusConfigurationError, match="必须包含 7 个值"):
        validate_so100_plus_config(config)


def test_factory_builds_unconnected_single_follower_robot(tmp_path):
    class FakeMotorsBus:
        def __init__(self, port, motors):
            self.port = port
            self.motors = motors
            self.connect_calls = 0

        def connect(self):
            self.connect_calls += 1
            raise AssertionError("factory 不应连接电机总线")

    class FakeManipulatorRobot:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.is_connected = False
            self.connect_calls = 0

        def connect(self):
            self.connect_calls += 1
            raise AssertionError("factory 不应连接 Robot")

    calibration_dir = write_calibration(tmp_path)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    robot = create_so100_plus_robot(
        config,
        motors_bus_class=FakeMotorsBus,
        manipulator_robot_class=FakeManipulatorRobot,
    )

    follower_bus = robot.kwargs["follower_arms"]["right"]
    assert robot.is_connected is False
    assert robot.connect_calls == 0
    assert follower_bus.connect_calls == 0
    assert follower_bus.port == "/dev/null"
    assert follower_bus.motors == SO100_PLUS_MOTORS
    assert robot.kwargs["robot_type"] == "so100"
    assert robot.kwargs["calibration_dir"] == calibration_dir.resolve()


def test_factory_validates_before_constructing_driver_objects(tmp_path):
    class DriverMustNotBeConstructed:
        def __init__(self, *args, **kwargs):
            raise AssertionError("配置无效时不应构造驱动对象")

    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    with pytest.raises(SO100PlusConfigurationError, match="已阻止 LeRobot 自动进入校准"):
        create_so100_plus_robot(
            config,
            motors_bus_class=DriverMustNotBeConstructed,
            manipulator_robot_class=DriverMustNotBeConstructed,
        )
