import json
from pathlib import Path

import pytest

import rosclaw_mini.arm.so100_plus_factory as factory_module
from rosclaw_mini.arm.so100_plus_factory import (
    SO100_PLUS_MOTOR_NAMES,
    SO100_PLUS_MOTORS,
    SO100PlusCameraConfig,
    SO100PlusConfigurationError,
    SO100PlusRobotConfig,
    create_so100_plus_cameras,
    create_so100_plus_readonly_robot,
    create_so100_plus_robot,
    validate_so100_plus_config,
    validate_so100_plus_camera_configs,
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


def test_camera_factory_builds_unconnected_readme_compatible_camera():
    class FakeOpenCVCamera:
        def __init__(self, device, **kwargs):
            self.device = device
            self.kwargs = kwargs
            self.connect_calls = 0

        def connect(self):
            self.connect_calls += 1
            raise AssertionError("摄像头工厂不应连接设备")

    cameras = create_so100_plus_cameras(
        (SO100PlusCameraConfig(name="right", device="/dev/null"),),
        opencv_camera_class=FakeOpenCVCamera,
    )

    camera = cameras["right"]
    assert camera.device == "/dev/null"
    assert camera.kwargs == {
        "fps": 20,
        "width": 640,
        "height": 480,
        "color_mode": "rgb",
    }
    assert camera.connect_calls == 0


def test_camera_preflight_rejects_missing_device():
    configs = (
        SO100PlusCameraConfig(
            name="right",
            device="/dev/definitely-missing-rosclaw-camera",
        ),
    )

    with pytest.raises(SO100PlusConfigurationError, match="摄像头设备不存在"):
        validate_so100_plus_camera_configs(configs)


def test_camera_preflight_rejects_duplicate_names():
    configs = (
        SO100PlusCameraConfig(name="right", device="/dev/null"),
        SO100PlusCameraConfig(name="right", device="/dev/null"),
    )

    with pytest.raises(SO100PlusConfigurationError, match="摄像头名称重复"):
        validate_so100_plus_camera_configs(configs)


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"name": "right-camera"}, "name 必须是简单名称"),
        ({"device": "relative-video0"}, "必须是绝对路径"),
        ({"fps": 0}, "fps 必须是正整数"),
        ({"fps": 121}, "fps 不能超过 120"),
        ({"color_mode": "gray"}, "color_mode 只能是"),
    ],
)
def test_camera_preflight_rejects_invalid_config(kwargs, message):
    values = {"name": "right", "device": "/dev/null", **kwargs}

    with pytest.raises(SO100PlusConfigurationError, match=message):
        validate_so100_plus_camera_configs((SO100PlusCameraConfig(**values),))


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


def test_normal_factory_uses_lightweight_feetech_import(monkeypatch, tmp_path):
    class FakeMotorsBus:
        def __init__(self, port, motors):
            self.port = port
            self.motors = motors

    class FakeManipulatorRobot:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    monkeypatch.setattr(
        factory_module,
        "_import_feetech_motors_bus_without_optional_app_dependencies",
        lambda: FakeMotorsBus,
    )
    calibration_dir = write_calibration(tmp_path)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    robot = create_so100_plus_robot(
        config,
        manipulator_robot_class=FakeManipulatorRobot,
    )

    follower_bus = robot.kwargs["follower_arms"]["right"]
    assert isinstance(follower_bus, FakeMotorsBus)
    assert follower_bus.port == "/dev/null"
    assert follower_bus.motors == SO100_PLUS_MOTORS


def test_normal_factory_uses_lightweight_manipulator_import(monkeypatch, tmp_path):
    class FakeMotorsBus:
        def __init__(self, port, motors):
            self.port = port
            self.motors = motors

    class FakeManipulatorRobot:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    monkeypatch.setattr(
        factory_module,
        "_import_manipulator_robot_without_required_torch_runtime",
        lambda: FakeManipulatorRobot,
    )
    calibration_dir = write_calibration(tmp_path)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    robot = create_so100_plus_robot(
        config,
        motors_bus_class=FakeMotorsBus,
    )

    assert isinstance(robot, FakeManipulatorRobot)
    assert robot.kwargs["robot_type"] == "so100"


def test_readonly_factory_loads_calibration_without_any_motor_write(tmp_path):
    class FakeMotorsBus:
        def __init__(self, port, motors):
            self.port = port
            self.motors = motors
            self.is_connected = False
            self.connect_calls = 0
            self.disconnect_calls = 0
            self.set_calibration_calls = []
            self.write_calls = []

        @property
        def motor_names(self):
            return list(self.motors)

        def connect(self):
            self.connect_calls += 1
            self.is_connected = True

        def set_calibration(self, calibration):
            self.set_calibration_calls.append(calibration)

        def write(self, *args, **kwargs):
            self.write_calls.append((args, kwargs))
            raise AssertionError("只读 Robot 不应写电机")

        def disconnect(self):
            self.disconnect_calls += 1
            self.is_connected = False

    calibration = make_calibration()
    calibration_dir = write_calibration(tmp_path, calibration)
    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=calibration_dir,
        follower_name="right",
    )

    robot = create_so100_plus_readonly_robot(
        config,
        motors_bus_class=FakeMotorsBus,
    )
    bus = robot.follower_arms["right"]

    assert robot.is_connected is False
    assert bus.connect_calls == 0
    assert bus.set_calibration_calls == []

    robot.connect()

    assert robot.is_connected is True
    assert bus.connect_calls == 1
    assert bus.set_calibration_calls == [calibration]
    assert bus.write_calls == []

    robot.disconnect()

    assert robot.is_connected is False
    assert bus.disconnect_calls == 1
    assert bus.write_calls == []


def test_readonly_factory_validates_before_constructing_bus(tmp_path):
    class BusMustNotBeConstructed:
        def __init__(self, *args, **kwargs):
            raise AssertionError("配置无效时不应构造总线")

    config = SO100PlusRobotConfig(
        port="/dev/null",
        calibration_dir=tmp_path / "missing-calibration",
        follower_name="right",
    )

    with pytest.raises(
        SO100PlusConfigurationError,
        match="已阻止 LeRobot 自动进入校准",
    ):
        create_so100_plus_readonly_robot(
            config,
            motors_bus_class=BusMustNotBeConstructed,
        )
