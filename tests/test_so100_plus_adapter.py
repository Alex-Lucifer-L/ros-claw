import pytest

from rosclaw_mini.arm.so100_plus import (
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusGripperSafetyError,
    SO100PlusMotionStoppedError,
)


class FakeFollowerBus:
    def __init__(self):
        self.motor_names = (
            "shoulder_rotation_joint",
            "shoulder_pitch_joint",
            "ellbow_joint",
            "wrist_pitch_joint",
            "wrist_jaw_joint",
            "wrist_roll_joint",
            "gripper_joint",
        )
        self.all_positions = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0]
        self.gripper_degrees = -5.0
        self.gripper_load = 0.0
        self.voltage_raw = [120.0] * 7
        self.current_raw = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
        self.load_raw = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 0.0]
        self.temperature_raw = [25.0] * 7
        self.position_error_degrees = 0.0
        self.read_calls = []
        self.write_calls = []

    def read(self, register, motor_name=None):
        self.read_calls.append((register, motor_name))
        if motor_name is None:
            values = {
                "Present_Position": list(self.all_positions),
                "Present_Voltage": list(self.voltage_raw),
                "Present_Current": list(self.current_raw),
                "Present_Load": list(self.load_raw),
                "Present_Temperature": list(self.temperature_raw),
            }
            values["Present_Position"][-1] = self.gripper_degrees
            values["Present_Load"][-1] = self.gripper_load
            return values[register]
        assert motor_name == "gripper_joint"
        values = {
            "Present_Position": self.gripper_degrees,
            "Present_Load": self.gripper_load,
        }
        return [values[register]]

    def write(self, register, values, motor_name=None):
        self.write_calls.append((register, values, motor_name))
        if register == "Acceleration":
            return
        if motor_name == "gripper_joint":
            self.gripper_degrees = values[0] + self.position_error_degrees
        else:
            self.all_positions = list(values)
            self.gripper_degrees = self.all_positions[-1]


class FakeRobot:
    """模拟 LeRobot 的 ManipulatorRobot，不会控制真实机械臂。"""

    def __init__(self):
        self.is_connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.bus = FakeFollowerBus()
        self.follower_arms = {"right": self.bus}

    def connect(self):
        self.connect_calls += 1
        self.is_connected = True

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


def make_adapter(robot, waits=None, wait_func=None, on_telemetry=None):
    config = SO100PlusGripperConfig(
        follower_name="right",
        open_degrees=60.0,
        close_degrees=-5.0,
    )
    return SO100PlusAdapter(
        robot,
        config,
        wait_func=(
            wait_func
            or (waits.append if waits is not None else lambda _seconds: False)
        ),
        on_telemetry=on_telemetry,
    )


def goal_write_calls(robot):
    return [
        call for call in robot.bus.write_calls if call[0] == "Goal_Position"
    ]


def test_connect():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    assert adapter.is_connected is False

    adapter.connect()

    assert adapter.is_connected is True
    assert robot.connect_calls == 1
    assert robot.bus.write_calls == [("Acceleration", 35, None)]
    assert [register for register, _motor in robot.bus.read_calls] == [
        "Present_Voltage",
        "Present_Current",
        "Present_Load",
        "Present_Temperature",
    ]


def test_connect_twice_only_calls_robot_once():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.connect()
    adapter.connect()

    assert robot.connect_calls == 1
    assert robot.bus.write_calls == [("Acceleration", 35, None)]


def test_disconnect():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.connect()
    adapter.disconnect()

    assert adapter.is_connected is False
    assert robot.disconnect_calls == 1


def test_disconnect_when_not_connected_does_nothing():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    adapter.disconnect()

    assert robot.disconnect_calls == 0


def test_open_gripper_moves_in_checked_steps_to_verified_target():
    robot = FakeRobot()
    waits = []
    adapter = make_adapter(robot, waits)
    adapter.connect()

    adapter.open_gripper()

    gripper_writes = goal_write_calls(robot)
    targets = [call[1][0] for call in gripper_writes]
    assert targets == [5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 60.0]
    assert all(call[2] == "gripper_joint" for call in gripper_writes)
    assert waits == [2.5] * 7


def test_close_gripper_moves_in_checked_steps_to_verified_target():
    robot = FakeRobot()
    robot.bus.gripper_degrees = 60.0
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.close_gripper()

    targets = [call[1][0] for call in goal_write_calls(robot)]
    assert targets == [50.0, 40.0, 30.0, 20.0, 10.0, 0.0, -5.0]


def test_gripper_stops_when_measured_position_is_within_tolerance():
    robot = FakeRobot()
    robot.bus.gripper_degrees = 55.0
    robot.bus.position_error_degrees = -2.0
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.open_gripper()

    assert goal_write_calls(robot) == [
        ("Goal_Position", [60.0], "gripper_joint")
    ]
    assert robot.bus.gripper_degrees == 58.0


def test_gripper_command_requires_explicit_connection():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="必须先显式连接"):
        adapter.open_gripper()

    assert robot.connect_calls == 0
    assert robot.bus.write_calls == []


def test_gripper_holds_observed_position_when_load_is_too_high():
    robot = FakeRobot()
    robot.bus.gripper_load = 301.0
    reported = []
    adapter = make_adapter(robot, on_telemetry=reported.append)
    adapter.connect()

    with pytest.raises(SO100PlusGripperSafetyError, match="负载 301.0"):
        adapter.close_gripper()

    assert goal_write_calls(robot) == [
        ("Goal_Position", [-5.0], "gripper_joint")
    ]
    assert reported[-1].phase == "gripper_start"
    assert reported[-1].load_magnitude[-1] == 301.0


def test_invalid_gripper_direction_is_rejected_during_configuration():
    with pytest.raises(ValueError, match="张开角度必须大于闭合角度"):
        SO100PlusGripperConfig(
            follower_name="right",
            open_degrees=-5.0,
            close_degrees=60.0,
        )


@pytest.mark.parametrize("runtime_acceleration", [-1, 255, 10.5, True])
def test_invalid_runtime_acceleration_is_rejected(runtime_acceleration):
    with pytest.raises(ValueError, match="0 到 254"):
        SO100PlusGripperConfig(
            follower_name="right",
            open_degrees=60.0,
            close_degrees=-5.0,
            runtime_acceleration=runtime_acceleration,
        )


def test_telemetry_records_all_motors_and_notifies_callback():
    robot = FakeRobot()
    reported = []
    adapter = make_adapter(robot, on_telemetry=reported.append)

    adapter.connect()

    assert reported == list(adapter.telemetry_history)
    assert len(reported) == 1
    telemetry = reported[0]
    assert telemetry.phase == "connected"
    assert telemetry.motor_names == robot.bus.motor_names
    assert telemetry.voltage_raw == (120.0,) * 7
    assert telemetry.current_raw == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    assert telemetry.load_magnitude == (10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 0.0)
    assert telemetry.temperature_raw == (25.0,) * 7
    assert goal_write_calls(robot) == []


def test_stop_holds_all_joints_at_their_present_positions():
    robot = FakeRobot()
    adapter = make_adapter(robot)
    adapter.connect()

    adapter.stop()

    expected_positions = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, -5.0]
    assert ("Present_Position", None) in robot.bus.read_calls
    assert goal_write_calls(robot) == [
        ("Goal_Position", expected_positions, None)
    ]
    assert adapter.telemetry_history[-1].phase == "stopped"


def test_stop_requires_explicit_connection():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(RuntimeError, match="停止操作前必须先显式连接"):
        adapter.stop()

    assert robot.bus.read_calls == []
    assert robot.bus.write_calls == []


def test_stop_cancels_remaining_gripper_steps_and_holds_all_joints():
    robot = FakeRobot()
    adapter = None

    def stop_during_first_wait(_seconds):
        adapter.stop()
        return True

    adapter = make_adapter(robot, wait_func=stop_during_first_wait)
    adapter.connect()

    with pytest.raises(SO100PlusMotionStoppedError, match=r"stop\(\) 取消"):
        adapter.open_gripper()

    assert goal_write_calls(robot) == [
        ("Goal_Position", [5.0], "gripper_joint"),
        (
            "Goal_Position",
            [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 5.0],
            None,
        ),
    ]


def test_move_to_remains_disabled():
    robot = FakeRobot()
    adapter = make_adapter(robot)

    with pytest.raises(NotImplementedError):
        adapter.move_to(0.3, 0.0, 0.2)
