import math

import pytest

from rosclaw_mini.safety.limits import (
    AxisLimits,
    JointLimits,
    LimitConfigurationError,
    LimitViolationError,
    MotionLimits,
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS,
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    WorkspaceLimits,
    build_so100_plus_right_follower_execution_joint_limits,
    build_so100_plus_right_follower_local_joint_limits,
    build_so100_plus_right_follower_motion_limits,
    choose_so100_plus_right_follower_base_test_target,
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


def test_formal_right_follower_workspace_accepts_closed_boundaries():
    workspace = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS

    assert workspace.validate_position(
        workspace.x.minimum,
        workspace.y.minimum,
        workspace.z.minimum,
    ) == pytest.approx(
        (
            0.3135714232672181,
            -0.041185494280163625,
            0.17932848288990053,
        )
    )
    assert workspace.validate_position(
        workspace.x.maximum,
        workspace.y.maximum,
        workspace.z.maximum,
    ) == pytest.approx(
        (
            0.4335714232672181,
            0.018814505719836373,
            0.29932848288990055,
        )
    )


@pytest.mark.parametrize(
    ("position", "axis"),
    [
        ((0.3135714232672181 - 1e-9, 0.0, 0.2), "x"),
        ((0.35, 0.018814505719836373 + 1e-9, 0.2), "y"),
        ((0.35, 0.0, 0.29932848288990055 + 1e-9), "z"),
    ],
)
def test_formal_right_follower_workspace_rejects_outside(position, axis):
    with pytest.raises(LimitViolationError, match=axis):
        SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS.validate_position(
            *position
        )


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


def test_measured_right_follower_shoulder_rotation_limits_accept_endpoints():
    limits = SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS

    assert limits.validate(-19.599609, "shoulder_rotation_joint") == pytest.approx(
        -19.599609
    )
    assert limits.validate(31.201172, "shoulder_rotation_joint") == pytest.approx(
        31.201172
    )


@pytest.mark.parametrize("angle", [-19.6875, 31.2890625])
def test_measured_right_follower_shoulder_rotation_limits_reject_one_tick_beyond(
    angle,
):
    limits = SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS

    with pytest.raises(LimitViolationError, match="shoulder_rotation_joint"):
        limits.validate(angle, "shoulder_rotation_joint")


def test_right_follower_local_limits_allow_cross_turn_current_joint():
    current = (0.0, math.radians(-191.0), 0.0, 0.0, 0.0, 0.0)

    limits = build_so100_plus_right_follower_local_joint_limits(current)

    assert limits.validate_position(current) == pytest.approx(current)
    assert limits.lower_radians[1] == pytest.approx(current[1] - 0.1)
    assert limits.upper_radians[1] == pytest.approx(current[1] + 0.1)


def test_right_follower_local_limits_can_use_smaller_internal_steps():
    current = (0.0,) * 6

    limits = build_so100_plus_right_follower_local_joint_limits(
        current,
        max_delta_radians=0.1,
        max_step_radians=math.radians(1.0),
    )

    assert limits.max_step_radians == pytest.approx((math.radians(1.0),) * 6)
    assert limits.lower_radians == pytest.approx((-0.1,) * 6)
    assert limits.upper_radians == pytest.approx((0.1,) * 6)


def test_right_follower_local_limits_keep_measured_base_boundary():
    current = (math.radians(-30.0), 0.0, 0.0, 0.0, 0.0, 0.0)

    limits = build_so100_plus_right_follower_local_joint_limits(current)

    assert math.degrees(limits.lower_radians[0]) == pytest.approx(-31.201172)
    assert math.degrees(limits.upper_radians[0]) == pytest.approx(
        math.degrees(current[0] + 0.1)
    )


def test_right_follower_local_limits_reject_base_outside_measured_range():
    current = (math.radians(-32.0), 0.0, 0.0, 0.0, 0.0, 0.0)

    with pytest.raises(LimitViolationError, match="当前底座关节"):
        build_so100_plus_right_follower_local_joint_limits(current)


def test_execution_limits_remove_total_delta_cap_but_keep_internal_step():
    current = (0.0, math.radians(-191.0), 3.0, 0.0, 0.0, 1.57)
    limits = build_so100_plus_right_follower_execution_joint_limits(
        current,
        max_step_radians=math.radians(2.0),
    )

    target = (0.0, math.radians(-170.0), 2.5, 0.2, 0.1, 1.2)
    assert limits.validate_position(target) == pytest.approx(target)
    with pytest.raises(LimitViolationError, match="单步变化"):
        limits.validate_step(current, target)
    assert limits.max_step_radians == pytest.approx(
        (math.radians(2.0),) * 6
    )


def test_execution_limits_only_extend_out_of_model_current_toward_model():
    current = (0.0, math.radians(-191.0), 3.0, 0.0, 0.0, 1.57)
    limits = build_so100_plus_right_follower_execution_joint_limits(
        current,
        max_step_radians=math.radians(2.0),
    )

    assert limits.lower_radians[1] == pytest.approx(current[1])
    limits.validate_position(
        (0.0, math.radians(-180.0), 3.0, 0.0, 0.0, 1.57)
    )
    with pytest.raises(LimitViolationError, match="shoulder_pitch_joint"):
        limits.validate_position(
            (0.0, math.radians(-192.0), 3.0, 0.0, 0.0, 1.57)
        )


def test_formal_right_follower_motion_limits_reuse_registered_workspace():
    current = (0.0, math.radians(-191.0), 3.0, 0.0, 0.0, 1.57)

    limits = build_so100_plus_right_follower_motion_limits(current)

    assert limits.workspace is SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS
    assert limits.joints.max_step_radians == pytest.approx(
        (math.radians(2.0),) * 6
    )


def test_base_test_target_chooses_side_with_more_headroom():
    assert choose_so100_plus_right_follower_base_test_target(0.0) == pytest.approx(
        8.0
    )
    assert choose_so100_plus_right_follower_base_test_target(25.0) == pytest.approx(
        17.0
    )


def test_base_test_target_rejects_unmeasured_current_position():
    with pytest.raises(LimitViolationError, match="当前底座关节"):
        choose_so100_plus_right_follower_base_test_target(40.0)
