from dataclasses import dataclass
import math

import numpy as np
import pytest

from rosclaw_mini.arm.kinematics import (
    InverseKinematicsError,
    SO100PlusKinematics,
    SO100_PLUS_GRIPPER_TCP_OFFSET_M,
    SO100_PLUS_JOYCON_REST_RADIANS,
)
from rosclaw_mini.safety.limits import (
    AxisLimits,
    JointLimits,
    LimitViolationError,
    MotionLimits,
    WorkspaceLimits,
)


def test_joycon_plus_rest_pose_matches_controller_source():
    assert SO100_PLUS_JOYCON_REST_RADIANS == (
        0.0,
        -3.1,
        3.0,
        0.0,
        0.0,
        1.57,
    )


class FakeTransform:
    def __init__(self, matrix):
        self.A = matrix


@dataclass
class FakeSolution:
    q: np.ndarray
    success: bool = True
    reason: str = "Success"


class FakeKinematicsRobot:
    """只模拟数学接口，不包含串口或电机。"""

    def __init__(self, rotation=None):
        self.solve_calls = []
        self.fail = False
        self.rotation = (
            np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
        )

    def fkine(self, q):
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = np.asarray(q[:3], dtype=float)
        return FakeTransform(matrix)

    def ikine_LM(self, *, Tep, q0, **kwargs):
        self.solve_calls.append((np.array(Tep), np.array(q0), kwargs))
        if self.fail:
            return FakeSolution(
                q=np.zeros(6),
                success=False,
                reason="test solver failure",
            )
        q = np.asarray(q0, dtype=float).copy()
        q[:3] = Tep[:3, 3]
        # 模拟第三方求解器把一个周期角折到另一圈。
        q[5] += 2 * math.pi
        return FakeSolution(q=q)


def make_motion_limits(max_step=0.1):
    return MotionLimits(
        workspace=WorkspaceLimits(
            x=AxisLimits(-1.0, 1.0),
            y=AxisLimits(-1.0, 1.0),
            z=AxisLimits(-1.0, 1.0),
        ),
        joints=JointLimits(
            joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
            lower_radians=(-2.0,) * 6,
            upper_radians=(2.0,) * 6,
            max_step_radians=(max_step,) * 6,
        ),
    )


def test_driver_and_model_joint_conversions_round_trip():
    kinematics = SO100PlusKinematics(robot=FakeKinematicsRobot())
    driver_degrees = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)

    model_radians = kinematics.driver_degrees_to_model_radians(driver_degrees)
    restored = kinematics.model_radians_to_driver_degrees(model_radians)

    assert restored == pytest.approx(driver_degrees)
    assert model_radians == pytest.approx(
        tuple(
            math.radians(value * sign)
            for value, sign in zip(
                driver_degrees,
                (-1, -1, 1, -1, 1, 1),
            )
        )
    )


def test_forward_position_applies_tcp_offset_along_tool_local_x_axis():
    quarter_turn_about_z = np.asarray(
        (
            (0.0, -1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    kinematics = SO100PlusKinematics(
        robot=FakeKinematicsRobot(rotation=quarter_turn_about_z)
    )

    tcp_position = kinematics.forward_position(
        (0.2, -0.1, 0.3, 0.0, 0.0, 0.0)
    )

    assert SO100_PLUS_GRIPPER_TCP_OFFSET_M == (
        0.10127,
        -0.00690,
        0.00118,
    )
    assert tcp_position == pytest.approx((0.20690, 0.00127, 0.30118))


def test_same_position_returns_current_joints_without_calling_solver():
    robot = FakeKinematicsRobot()
    kinematics = SO100PlusKinematics(robot=robot)
    current = (0.2, -0.1, 0.3, 0.0, 0.0, 0.0)

    solved = kinematics.solve_position(
        current,
        kinematics.forward_position(current),
    )

    assert solved == pytest.approx(current)
    assert robot.solve_calls == []


def test_solver_disables_broken_backend_limits_and_unwraps_near_current():
    robot = FakeKinematicsRobot()
    kinematics = SO100PlusKinematics(robot=robot)
    current = (0.0,) * 6

    solved = kinematics.solve_position(current, (0.2, -0.1, 0.3))

    assert solved == pytest.approx(
        (0.09873, -0.09310, 0.29882, 0.0, 0.0, 0.0)
    )
    assert kinematics.forward_position(solved) == pytest.approx(
        (0.2, -0.1, 0.3)
    )
    assert robot.solve_calls[0][0][:3, 3] == pytest.approx(
        (0.09873, -0.09310, 0.29882)
    )
    assert robot.solve_calls[0][2]["joint_limits"] is False
    assert robot.solve_calls[0][2]["seed"] == 0


def test_failed_inverse_kinematics_is_rejected():
    robot = FakeKinematicsRobot()
    robot.fail = True
    kinematics = SO100PlusKinematics(robot=robot)

    with pytest.raises(InverseKinematicsError, match="test solver failure"):
        kinematics.solve_position((0.0,) * 6, (0.2, 0.0, 0.2))


def test_plan_position_validates_limits_and_splits_joint_path():
    kinematics = SO100PlusKinematics(robot=FakeKinematicsRobot())
    limits = make_motion_limits(max_step=0.1)

    plan = kinematics.plan_position(
        current_joint_radians=(0.0,) * 6,
        target_position_m=(0.25, 0.0, 0.0),
        limits=limits,
    )

    assert len(plan.waypoints_radians) == 2
    assert plan.target_joint_radians == pytest.approx(
        (0.14873, 0.00690, -0.00118, 0.0, 0.0, 0.0)
    )
    previous = (0.0,) * 6
    for waypoint in plan.waypoints_radians:
        limits.validate_joint_step(previous, waypoint)
        previous = waypoint


def test_plan_position_rejects_target_outside_configured_workspace():
    kinematics = SO100PlusKinematics(robot=FakeKinematicsRobot())

    with pytest.raises(LimitViolationError, match="x"):
        kinematics.plan_position(
            current_joint_radians=(0.0,) * 6,
            target_position_m=(1.01, 0.0, 0.0),
            limits=make_motion_limits(),
        )
