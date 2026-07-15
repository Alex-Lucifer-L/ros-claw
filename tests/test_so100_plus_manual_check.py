import pytest

from rosclaw_mini.arm.so100_plus_manual_check import (
    GRIPPER_CYCLE_TARGETS,
    GripperCycleSafetyError,
    move_gripper_to_test_target_once,
    nudge_gripper_open_once,
    read_present_positions_once,
    validate_gripper_cycle_once,
)


class FakeFollowerBus:
    def __init__(self):
        self.is_connected = False
        self.read_calls = []
        self.write_calls = []
        self.disconnect_calls = 0
        self.gripper_degrees = -4.75
        self.gripper_load = 0.0
        self.gripper_current = 10.0
        self.gripper_temperature = 25.0

    def read(self, register, motor_name=None):
        self.read_calls.append((register, motor_name))
        if motor_name == "gripper_joint":
            values = {
                "Present_Position": self.gripper_degrees,
                "Present_Load": self.gripper_load,
                "Present_Current": self.gripper_current,
                "Present_Temperature": self.gripper_temperature,
            }
            return [values[register]]
        return [10, 20, 30, 40, 50, 60, 70]

    def write(self, register, values, motor_name):
        self.write_calls.append((register, values, motor_name))
        self.gripper_degrees = values[0]

    def disconnect(self):
        self.disconnect_calls += 1
        self.is_connected = False


class FakeRobot:
    def __init__(self, fail_during_connect=False):
        self.bus = FakeFollowerBus()
        self.follower_arms = {"right": self.bus}
        self.is_connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0
        self.fail_during_connect = fail_during_connect

    def connect(self):
        self.connect_calls += 1
        self.bus.is_connected = True
        if self.fail_during_connect:
            raise RuntimeError("模拟 connect 中途失败")
        self.is_connected = True

    def disconnect(self):
        self.disconnect_calls += 1
        self.bus.disconnect()
        self.is_connected = False


def test_manual_check_only_connects_reads_and_disconnects_once():
    robot = FakeRobot()

    positions = read_present_positions_once(robot, "right")

    assert positions == [10, 20, 30, 40, 50, 60, 70]
    assert robot.connect_calls == 1
    assert robot.bus.read_calls == [("Present_Position", None)]
    assert robot.disconnect_calls == 1
    assert robot.bus.disconnect_calls == 1
    assert robot.is_connected is False
    assert robot.bus.is_connected is False


def test_manual_check_closes_bus_when_connect_fails_partway():
    robot = FakeRobot(fail_during_connect=True)

    with pytest.raises(RuntimeError, match="模拟 connect 中途失败"):
        read_present_positions_once(robot, "right")

    assert robot.disconnect_calls == 0
    assert robot.bus.disconnect_calls == 1
    assert robot.bus.is_connected is False
    assert robot.bus.read_calls == []


def test_gripper_nudge_only_writes_positive_five_degrees_to_gripper():
    robot = FakeRobot()
    waits = []

    result = nudge_gripper_open_once(
        robot,
        "right",
        5.0,
        sleep_func=waits.append,
    )

    assert result.start_degrees == -4.75
    assert result.target_degrees == 0.25
    assert result.final_degrees == 0.25
    assert robot.bus.write_calls == [
        ("Goal_Position", [0.25], "gripper_joint")
    ]
    assert robot.bus.read_calls == [
        ("Present_Position", "gripper_joint"),
        ("Present_Position", "gripper_joint"),
    ]
    assert waits == [0.5]
    assert robot.disconnect_calls == 1
    assert robot.bus.disconnect_calls == 1


@pytest.mark.parametrize("delta_degrees", [-1.0, 0.0, 5.01])
def test_gripper_nudge_rejects_unsafe_delta_before_connecting(delta_degrees):
    robot = FakeRobot()

    with pytest.raises(ValueError, match="不超过 5.0 度"):
        nudge_gripper_open_once(robot, "right", delta_degrees)

    assert robot.connect_calls == 0
    assert robot.bus.read_calls == []
    assert robot.bus.write_calls == []


def test_gripper_test_target_only_writes_ten_degrees_to_gripper():
    robot = FakeRobot()
    waits = []

    result = move_gripper_to_test_target_once(
        robot,
        "right",
        10.0,
        sleep_func=waits.append,
    )

    assert result.start_degrees == -4.75
    assert result.target_degrees == 10.0
    assert result.final_degrees == 10.0
    assert robot.bus.write_calls == [
        ("Goal_Position", [10.0], "gripper_joint")
    ]
    assert waits == [2.5]
    assert robot.disconnect_calls == 1


def test_gripper_test_target_rejects_more_than_ten_degrees_before_connecting():
    robot = FakeRobot()

    with pytest.raises(ValueError, match="不能超过 10.0 度"):
        move_gripper_to_test_target_once(robot, "right", 10.01)

    assert robot.connect_calls == 0
    assert robot.bus.read_calls == []
    assert robot.bus.write_calls == []


def test_gripper_test_target_rejects_more_than_twenty_degrees_of_travel():
    robot = FakeRobot()
    robot.bus.gripper_degrees = -15.0

    with pytest.raises(ValueError, match="单次行程不能超过 20.0 度"):
        move_gripper_to_test_target_once(robot, "right", 10.0)

    assert robot.connect_calls == 1
    assert robot.bus.write_calls == []
    assert robot.disconnect_calls == 1


def test_gripper_cycle_opens_closes_and_reopens_in_one_connection():
    robot = FakeRobot()
    waits = []
    reported_steps = []

    result = validate_gripper_cycle_once(
        robot,
        "right",
        sleep_func=waits.append,
        on_step=reported_steps.append,
    )

    assert result.start_degrees == -4.75
    assert [step.target_degrees for step in result.steps] == list(GRIPPER_CYCLE_TARGETS)
    assert [call[1][0] for call in robot.bus.write_calls] == list(GRIPPER_CYCLE_TARGETS)
    assert all(call[2] == "gripper_joint" for call in robot.bus.write_calls)
    assert waits == [2.5] * len(GRIPPER_CYCLE_TARGETS)
    assert reported_steps == list(result.steps)
    assert result.steps[-1].position_degrees == 10.0
    assert robot.connect_calls == 1
    assert robot.disconnect_calls == 1


def test_gripper_cycle_holds_current_position_and_stops_on_high_load():
    robot = FakeRobot()

    def set_overload_after_wait(_seconds):
        if robot.bus.gripper_degrees >= 20.0:
            robot.bus.gripper_load = 301.0

    with pytest.raises(GripperCycleSafetyError, match="负载 301.0 超过限制"):
        validate_gripper_cycle_once(
            robot,
            "right",
            sleep_func=set_overload_after_wait,
        )

    goal_writes = [call[1][0] for call in robot.bus.write_calls]
    assert goal_writes == [0.0, 10.0, 20.0, 20.0]
    assert robot.disconnect_calls == 1
