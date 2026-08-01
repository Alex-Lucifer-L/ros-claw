import math

import pytest

from rosclaw_mini.arm.kinematics import JointMotionPlan
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
    SO100PlusTrajectoryValidationError,
    SO100PlusTrajectoryValidationUnavailableError,
    StorageTransitionDirection,
    so100_plus_gripper_driver_degrees_to_mujoco_qpos,
)


TEST_GRIPPER_QPOS = math.radians(-9.0)


class LinearTrajectoryKinematics:
    """用第一关节表示 TCP 高度，不加载运动学或硬件依赖。"""

    @staticmethod
    def forward_position(joint_radians):
        height = float(joint_radians[0])
        return (0.3, 0.0, height)


def _joints(height: float) -> tuple[float, ...]:
    return (height, 0.0, 0.0, 0.0, 0.0, 0.0)


def _plan(
    start: float,
    target: float,
    *waypoint_heights: float,
) -> JointMotionPlan:
    controls = (start, *waypoint_heights)
    dense_heights = []
    for begin, end in zip(controls, controls[1:]):
        steps = max(1, math.ceil(abs(end - begin) / math.radians(1.0)))
        dense_heights.extend(
            begin + (end - begin) * index / steps
            for index in range(1, steps + 1)
        )
    return JointMotionPlan(
        target_position_m=(0.3, 0.0, target),
        current_joint_radians=_joints(start),
        target_joint_radians=_joints(target),
        waypoints_radians=tuple(_joints(height) for height in dense_heights),
        is_final_execution_plan=True,
        held_gripper_driver_degrees=-9.0,
    )


def _unfold_plans() -> tuple[JointMotionPlan, JointMotionPlan]:
    return (
        _plan(0.0, 0.2, 0.1, 0.2),
        _plan(0.2, 1.0, 0.6, 1.0),
    )


def _validator_with_contacts(contact_factory):
    # 绕过构造函数只为单元测试注入内存接触结果；不会加载 MuJoCo 模型。
    validator = object.__new__(SO100PlusMuJoCoTrajectoryValidator)
    validator._gripper_qpos_range = (-0.2, 2.0)

    def sample_contacts(samples, gripper_qpos):
        assert gripper_qpos == TEST_GRIPPER_QPOS
        return tuple(
            frozenset(contact_factory(index, joints, len(samples)))
            for index, joints in enumerate(samples)
        )

    validator._sample_contacts = sample_contacts
    return validator


def test_unfold_checks_dense_complete_path_and_returns_same_plans():
    plans = _unfold_plans()
    sampled_heights = []

    def contacts(_index, joints, _sample_count):
        sampled_heights.append(joints[0])
        if math.isclose(joints[0], 0.0, abs_tol=1e-12):
            return {("arm", "storage_support")}
        return set()

    validator = _validator_with_contacts(contacts)

    verified = validator.verify_storage_transition(
        plans,
        escape_joint_radians=_joints(0.2),
        kinematics=LinearTrajectoryKinematics(),
        direction=StorageTransitionDirection.UNFOLD,
        gripper_qpos=TEST_GRIPPER_QPOS,
    )

    assert verified.plans[0] is plans[0]
    assert verified.plans[1] is plans[1]
    assert len(sampled_heights) > 4
    assert sampled_heights[0] == 0.0
    assert sampled_heights[-1] == 1.0
    assert verified.report.sample_count == len(sampled_heights)
    assert verified.report.max_joint_sample_step_degrees <= 1.0 + 1e-9
    assert verified.report.initial_contact_pairs == frozenset(
        {("arm", "storage_support")}
    )
    assert verified.report.final_contact_pairs == frozenset()


def test_unfold_rejects_new_collision_before_storage_escape():
    def contacts(_index, joints, _sample_count):
        if math.isclose(joints[0], 0.0, abs_tol=1e-12):
            return {("arm", "storage_support")}
        if 0.08 < joints[0] < 0.12:
            return {
                ("arm", "storage_support"),
                ("arm", "unexpected_obstacle"),
            }
        return set()

    validator = _validator_with_contacts(contacts)

    with pytest.raises(
        SO100PlusTrajectoryValidationError,
        match="新接触",
    ):
        validator.verify_storage_transition(
            _unfold_plans(),
            escape_joint_radians=_joints(0.2),
            kinematics=LinearTrajectoryKinematics(),
            direction=StorageTransitionDirection.UNFOLD,
            gripper_qpos=TEST_GRIPPER_QPOS,
        )


def test_unfold_rejects_storage_escape_that_still_has_contact():
    def contacts(_index, joints, _sample_count):
        if joints[0] <= 0.2 + 1e-12:
            return {("arm", "storage_support")}
        return set()

    validator = _validator_with_contacts(contacts)

    with pytest.raises(
        SO100PlusTrajectoryValidationError,
        match="storage_escape 尚未脱离",
    ):
        validator.verify_storage_transition(
            _unfold_plans(),
            escape_joint_radians=_joints(0.2),
            kinematics=LinearTrajectoryKinematics(),
            direction=StorageTransitionDirection.UNFOLD,
            gripper_qpos=TEST_GRIPPER_QPOS,
        )


def test_unfold_rejects_work_initial_pose_that_still_has_contact():
    def contacts(index, joints, sample_count):
        if math.isclose(joints[0], 0.0, abs_tol=1e-12):
            return {("arm", "storage_support")}
        if index == sample_count - 1:
            return {("gripper", "unexpected_obstacle")}
        return set()

    validator = _validator_with_contacts(contacts)

    with pytest.raises(
        SO100PlusTrajectoryValidationError,
        match="工作初始姿态仍存在接触",
    ):
        validator.verify_storage_transition(
            _unfold_plans(),
            escape_joint_radians=_joints(0.2),
            kinematics=LinearTrajectoryKinematics(),
            direction=StorageTransitionDirection.UNFOLD,
            gripper_qpos=TEST_GRIPPER_QPOS,
        )


def test_return_to_work_initial_checks_every_dense_sample_for_collision():
    plan = _plan(1.0, 0.4, 0.7, 0.4)
    visited = []

    def contacts(_index, joints, _sample_count):
        visited.append(joints)
        if 0.68 < joints[0] < 0.72:
            return {("arm", "unexpected_obstacle")}
        return set()

    validator = _validator_with_contacts(contacts)

    with pytest.raises(
        SO100PlusTrajectoryValidationError,
        match="返回工作初始姿态路径.*存在接触",
    ):
        validator.verify_collision_free_sequence(
            (plan,),
            LinearTrajectoryKinematics(),
            gripper_qpos=TEST_GRIPPER_QPOS,
        )

    assert len(visited) == len(plan.waypoints_radians) + 1


def test_fold_allows_only_final_storage_contacts_after_escape():
    plans = (
        _plan(1.0, 0.2, 0.6, 0.2),
        _plan(0.2, 0.0, 0.1, 0.0),
    )

    def contacts(_index, joints, _sample_count):
        if joints[0] < 0.08:
            return {("arm", "storage_support")}
        return set()

    validator = _validator_with_contacts(contacts)

    verified = validator.verify_storage_transition(
        plans,
        escape_joint_radians=_joints(0.2),
        kinematics=LinearTrajectoryKinematics(),
        direction=StorageTransitionDirection.FOLD,
        gripper_qpos=TEST_GRIPPER_QPOS,
    )

    assert verified.plans == plans
    assert verified.report.final_contact_pairs == frozenset(
        {("arm", "storage_support")}
    )


def test_missing_mujoco_model_fails_closed(tmp_path):
    missing_model = tmp_path / "missing-scene.xml"

    with pytest.raises(
        SO100PlusTrajectoryValidationUnavailableError,
        match="模型不可用",
    ):
        SO100PlusMuJoCoTrajectoryValidator(model_path=missing_model)


@pytest.mark.parametrize(
    ("driver_degrees", "expected_qpos"),
    [(-9.0, math.radians(-9.0)), (60.0, math.radians(60.0))],
)
def test_gripper_driver_degrees_map_directly_to_mujoco_radians(
    driver_degrees,
    expected_qpos,
):
    assert so100_plus_gripper_driver_degrees_to_mujoco_qpos(
        driver_degrees
    ) == pytest.approx(expected_qpos)


@pytest.mark.parametrize("invalid", [None, float("nan"), float("inf"), True])
def test_invalid_gripper_feedback_fails_mapping(invalid):
    with pytest.raises(
        SO100PlusTrajectoryValidationError,
        match="夹爪反馈",
    ):
        so100_plus_gripper_driver_degrees_to_mujoco_qpos(invalid)


def test_gripper_feedback_outside_loaded_model_range_fails_closed():
    validator = object.__new__(SO100PlusMuJoCoTrajectoryValidator)
    validator._gripper_qpos_range = (-0.2, 2.0)

    with pytest.raises(
        SO100PlusTrajectoryValidationError,
        match="超出 MuJoCo gripper_joint 范围",
    ):
        validator.gripper_driver_degrees_to_qpos(120.0)
