import math

import pytest

from rosclaw_mini.safety.limits import (
    AxisLimits,
    JointLimits,
    LimitConfigurationError,
    LimitViolationError,
    MotionLimits,
    WorkspaceLimits,
)


def make_workspace() -> WorkspaceLimits:
    return WorkspaceLimits(
        x=AxisLimits(0.1, 0.5),
        y=AxisLimits(-0.3, 0.3),
        z=AxisLimits(0.0, 0.4),
    )


def make_joint_limits() -> JointLimits:
    return JointLimits(
        joint_names=("j1", "j2"),
        lower_radians=(-1.0, -2.0),
        upper_radians=(1.0, 2.0),
        max_step_radians=(0.1, 0.2),
    )


def test_axis_limits_reject_invalid_configuration():
    with pytest.raises(LimitConfigurationError, match="有限数值"):
        AxisLimits(0.0, math.inf)

    with pytest.raises(LimitConfigurationError, match="小于或等于"):
        AxisLimits(1.0, 0.0)


def test_workspace_accepts_zero_and_negative_coordinates_when_configured():
    workspace = make_workspace()

    workspace.validate_position(0.2, -0.1, 0.0)


def test_workspace_rejects_non_finite_and_out_of_range_coordinates():
    workspace = make_workspace()

    with pytest.raises(LimitViolationError, match="x.*有限数值"):
        workspace.validate_position(math.nan, 0.0, 0.2)

    with pytest.raises(LimitViolationError, match="y"):
        workspace.validate_position(0.2, 0.31, 0.2)


def test_joint_limits_validate_position_and_step():
    limits = make_joint_limits()

    limits.validate_position((1.0, -2.0))
    limits.validate_step((0.0, 0.0), (0.1, -0.2))

    with pytest.raises(LimitViolationError, match="j1"):
        limits.validate_position((1.01, 0.0))

    with pytest.raises(LimitViolationError, match="j2.*单步变化"):
        limits.validate_step((0.0, 0.0), (0.0, 0.21))


def test_joint_limits_reject_dimension_mismatch():
    limits = make_joint_limits()

    with pytest.raises(LimitViolationError, match="需要 2 个关节值"):
        limits.validate_position((0.0,))


def test_motion_limits_combines_workspace_and_joint_limits():
    limits = MotionLimits(
        workspace=make_workspace(),
        joints=make_joint_limits(),
    )

    limits.validate_target_position((0.2, 0.0, 0.2))
    limits.validate_joint_step((0.0, 0.0), (0.05, 0.1))
