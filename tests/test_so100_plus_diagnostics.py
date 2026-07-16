import math

import pytest

from rosclaw_mini.arm.so100_plus_diagnostics import (
    MotionPreviewSafetyError,
    build_symmetric_local_grid_offsets,
    preview_local_motion_grid_once,
)


MOTOR_NAMES = (
    "shoulder_rotation_joint",
    "shoulder_pitch_joint",
    "ellbow_joint",
    "wrist_pitch_joint",
    "wrist_jaw_joint",
    "wrist_roll_joint",
    "gripper_joint",
)


class FakeFollowerBus:
    def __init__(self, positions=None, torque=None):
        self.motor_names = MOTOR_NAMES
        self.positions = positions or [30.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.0]
        self.torque = torque or [0] * 7
        self.is_connected = False
        self.read_calls = []
        self.write_calls = []

    def read(self, register, motor_name=None):
        self.read_calls.append((register, motor_name))
        return {
            "Torque_Enable": list(self.torque),
            "Present_Position": list(self.positions),
        }[register]

    def disconnect(self):
        self.is_connected = False


class FakeRobot:
    def __init__(self, **bus_kwargs):
        self.bus = FakeFollowerBus(**bus_kwargs)
        self.follower_arms = {"right": self.bus}
        self.is_connected = False
        self.connect_calls = 0
        self.disconnect_calls = 0

    def connect(self):
        self.connect_calls += 1
        self.bus.is_connected = True
        self.is_connected = True

    def disconnect(self):
        self.disconnect_calls += 1
        self.bus.disconnect()
        self.is_connected = False


class FakeKinematics:
    current_position_m = (0.2, 0.0, 0.1)

    def driver_degrees_to_model_radians(self, driver_degrees):
        return tuple(math.radians(value) for value in driver_degrees)

    def forward_position(self, joint_radians):
        return self.current_position_m

    def solve_position(self, current_joint_radians, target_position_m):
        target = list(current_joint_radians)
        if target_position_m[0] > self.current_position_m[0]:
            target[0] += math.radians(1.0)
        if target_position_m[1] > self.current_position_m[1]:
            target[0] += math.radians(2.0)
        return tuple(target)

    def model_radians_to_driver_degrees(self, model_radians):
        return tuple(math.degrees(value) for value in model_radians)


def test_grid_reads_position_once_and_filters_base_limit():
    robot = FakeRobot()

    result = preview_local_motion_grid_once(
        robot,
        "right",
        FakeKinematics(),
        offsets_m=((0.001, 0.0, 0.0), (0.0, 0.001, 0.0)),
    )

    assert result.current_position_m == (0.2, 0.0, 0.1)
    assert result.points[0].is_candidate is True
    assert result.points[0].preview.target_driver_degrees[0] == pytest.approx(
        31.0
    )
    assert result.points[1].is_candidate is False
    assert "底座关节" in result.points[1].rejection_reason
    assert robot.bus.read_calls.count(("Present_Position", None)) == 1
    assert robot.bus.write_calls == []
    assert robot.disconnect_calls == 1


def test_grid_keeps_large_delta_as_unapproved_math_candidate():
    class LargeDeltaKinematics(FakeKinematics):
        def solve_position(self, current_joint_radians, target_position_m):
            target = list(current_joint_radians)
            target[1] += 0.2
            return tuple(target)

    robot = FakeRobot(
        positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -5.0]
    )
    result = preview_local_motion_grid_once(
        robot,
        "right",
        LargeDeltaKinematics(),
        offsets_m=((0.001, 0.0, 0.0),),
    )

    preview = result.points[0].preview
    assert preview.max_joint_delta_radians == pytest.approx(0.2)
    assert preview.model_step_count == 2
    assert preview.is_approved_for_execution is False


def test_grid_requires_torque_to_be_disabled_and_closes_connection():
    robot = FakeRobot(torque=[1] * 7)

    with pytest.raises(MotionPreviewSafetyError, match="扭矩全部关闭"):
        preview_local_motion_grid_once(
            robot,
            "right",
            FakeKinematics(),
            offsets_m=((0.001, 0.0, 0.0),),
        )

    assert robot.bus.write_calls == []
    assert robot.disconnect_calls == 1


def test_build_symmetric_grid_uses_explicit_extent_and_step():
    offsets = build_symmetric_local_grid_offsets(
        half_extent_mm=(10.0, 10.0, 0.0),
        step_mm=10.0,
    )

    assert len(offsets) == 8
    assert (0.01, 0.01, 0.0) in offsets
    assert (0.0, 0.0, 0.0) not in offsets


def test_build_symmetric_grid_rejects_non_integral_step():
    with pytest.raises(ValueError, match="整数倍"):
        build_symmetric_local_grid_offsets(
            half_extent_mm=(10.0, 0.0, 0.0),
            step_mm=6.0,
        )
