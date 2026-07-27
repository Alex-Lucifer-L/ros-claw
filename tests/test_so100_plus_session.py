import math
from threading import Event, Thread

import pytest

from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus import SO100_PLUS_REAL_HARDWARE_PROFILE
from rosclaw_mini.arm.so100_plus_session import (
    ArmSessionState,
    SO100PlusArmSession,
    SO100PlusPoseSnapshot,
    validate_work_initial_pose,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusTrajectoryValidationError,
    SO100PlusTrajectoryValidationUnavailableError,
    SO100PlusTrajectoryValidationReport,
    StorageTransitionDirection,
    VerifiedJointMotionSequence,
)
from rosclaw_mini.command_schema.commands import Command


class LinearFakeKinematics:
    """只为状态机测试提供单调的内存 FK，不加载机械臂模型。"""

    def __init__(self, event_log=None) -> None:
        self.event_log = event_log if event_log is not None else []
        self.storage = SO100PlusKinematics.driver_degrees_to_model_radians(
            SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
        )
        self.work = SO100_PLUS_JOYCON_INITIAL_RADIANS
        self._delta = tuple(
            target - start
            for start, target in zip(
                self.storage,
                self.work,
                strict=True,
            )
        )
        self._delta_norm_squared = sum(value * value for value in self._delta)

    @staticmethod
    def driver_degrees_to_model_radians(values):
        return SO100PlusKinematics.driver_degrees_to_model_radians(values)

    def forward_position(self, values):
        values = tuple(float(value) for value in values)
        fraction = sum(
            (value - start) * delta
            for value, start, delta in zip(
                values,
                self.storage,
                self._delta,
                strict=True,
            )
        ) / self._delta_norm_squared
        return (
            0.2035714232672181 + 0.1 * fraction,
            -0.0011854942801636243,
            0.04932848288990053 + 0.13 * fraction,
        )

    def _plan(self, current, target):
        current = tuple(current)
        target = tuple(target)
        plan = JointMotionPlan(
            target_position_m=self.forward_position(target),
            current_joint_radians=current,
            target_joint_radians=target,
            waypoints_radians=() if current == target else (target,),
        )
        self.event_log.append(("plan", plan))
        return plan

    def plan_joint_pose(
        self,
        current_joint_radians,
        target_joint_radians,
        limits,
    ):
        del limits
        return self._plan(current_joint_radians, target_joint_radians)

    def plan_position(
        self,
        current_joint_radians,
        target_position_m,
        limits,
    ):
        del target_position_m, limits
        return self._plan(current_joint_radians, self.work)


class RecordingSessionAdapter:
    def __init__(
        self,
        *,
        block_operation: str | None = None,
        event_log=None,
    ) -> None:
        self.calls: list[tuple] = []
        self.event_log = event_log if event_log is not None else []
        self.block_operation = block_operation
        self.operation_started = Event()
        self.release_operation = Event()

    def _record(self, name: str, *values) -> None:
        call = (name, *values)
        self.calls.append(call)
        self.event_log.append(call)
        if self.block_operation == name:
            self.operation_started.set()
            self.release_operation.wait(timeout=2.0)

    def move_to(self, x, y, z) -> None:
        self._record("move_to", x, y, z)

    def move_joints(self, joint_radians) -> None:
        self._record("move_joints", tuple(joint_radians))

    def execute_joint_plan(self, plan) -> None:
        self._record("execute_joint_plan", plan)

    def open_gripper(self) -> None:
        self._record("open_gripper")

    def close_gripper(self) -> None:
        self._record("close_gripper")

    def stop(self) -> None:
        self.calls.append(("stop",))
        self.event_log.append(("stop",))
        self.release_operation.set()


class RecordingTrajectoryValidator:
    def __init__(
        self,
        *,
        event_log=None,
        storage_error: Exception | None = None,
        return_error: Exception | None = None,
    ) -> None:
        self.event_log = event_log if event_log is not None else []
        self.storage_error = storage_error
        self.return_error = return_error
        self.storage_calls = []
        self.return_calls = []

    @staticmethod
    def _verified(plans):
        plans = tuple(plans)
        samples = (plans[0].current_joint_radians,) + tuple(
            waypoint
            for plan in plans
            for waypoint in plan.waypoints_radians
        )
        return VerifiedJointMotionSequence(
            plans=plans,
            sampled_joint_radians=samples,
            report=SO100PlusTrajectoryValidationReport(
                sample_count=len(samples),
                max_joint_sample_step_degrees=1.0,
                minimum_tcp_z_m=0.0,
                initial_contact_pairs=frozenset(),
                final_contact_pairs=frozenset(),
                last_contact_sample=-1,
            ),
        )

    def verify_storage_transition(
        self,
        plans,
        *,
        escape_joint_radians,
        kinematics,
        direction,
    ):
        del escape_joint_radians, kinematics
        plans = tuple(plans)
        self.storage_calls.append((plans, direction))
        self.event_log.append(("validate_storage", plans, direction))
        if self.storage_error is not None:
            raise self.storage_error
        return self._verified(plans)

    def verify_collision_free_sequence(self, plans, kinematics):
        del kinematics
        plans = tuple(plans)
        self.return_calls.append(plans)
        self.event_log.append(("validate_return", plans))
        if self.return_error is not None:
            raise self.return_error
        return self._verified(plans)


class PoseQueue:
    def __init__(self, *snapshots: SO100PlusPoseSnapshot) -> None:
        self.snapshots = list(snapshots)

    def __call__(self) -> SO100PlusPoseSnapshot:
        if not self.snapshots:
            raise AssertionError("测试没有为本次姿态读取准备反馈")
        return self.snapshots.pop(0)


def _rest_snapshot(kinematics) -> SO100PlusPoseSnapshot:
    return SO100PlusPoseSnapshot(
        driver_degrees=(
            SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
        ),
        joint_radians=kinematics.storage,
        tcp_position_m=kinematics.forward_position(kinematics.storage),
        torque_enabled=(1,) * 7,
    )


def _work_snapshot(kinematics) -> SO100PlusPoseSnapshot:
    return SO100PlusPoseSnapshot(
        driver_degrees=SO100PlusKinematics.model_radians_to_driver_degrees(
            kinematics.work
        ),
        joint_radians=kinematics.work,
        tcp_position_m=kinematics.forward_position(kinematics.work),
        torque_enabled=(1,) * 7,
    )


def _unknown_snapshot(kinematics) -> SO100PlusPoseSnapshot:
    driver = list(
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
    )
    driver[1] += 30.0
    joints = list(kinematics.work)
    joints[2] += math.radians(10.0)
    return SO100PlusPoseSnapshot(
        driver_degrees=tuple(driver),
        joint_radians=tuple(joints),
        tcp_position_m=kinematics.forward_position(joints),
        torque_enabled=(1,) * 7,
    )


def _formal_work_snapshot(kinematics) -> SO100PlusPoseSnapshot:
    joints = list(kinematics.work)
    joints[0] += math.radians(8.0)
    return SO100PlusPoseSnapshot(
        driver_degrees=SO100PlusKinematics.model_radians_to_driver_degrees(
            joints
        ),
        joint_radians=tuple(joints),
        tcp_position_m=(0.35, 0.0, 0.22),
        torque_enabled=(1,) * 7,
    )


def _command(skill_name: str, params=None) -> Command:
    return Command(
        command_id=f"test-{skill_name}",
        skill_name=skill_name,
        params={} if params is None else params,
        source="test",
    )


def _session(
    initial_snapshot,
    *operation_snapshots,
    work_adapter=None,
    transition_adapter=None,
    trajectory_validator=None,
    event_log=None,
):
    log = event_log if event_log is not None else []
    kinematics = LinearFakeKinematics(log)
    work = work_adapter or RecordingSessionAdapter(event_log=log)
    transition = transition_adapter or RecordingSessionAdapter(
        event_log=log
    )
    validator = trajectory_validator or RecordingTrajectoryValidator(
        event_log=log
    )
    session = SO100PlusArmSession(
        work_adapter=work,
        transition_adapter=transition,
        pose_reader=PoseQueue(*operation_snapshots),
        kinematics=kinematics,
        initial_snapshot=initial_snapshot(kinematics),
        storage_joint_radians=kinematics.storage,
        transition_motion_limits=object(),
        trajectory_validator=validator,
    )
    return session, kinematics, work, transition


@pytest.mark.parametrize(
    ("snapshot_factory", "expected_state"),
    (
        (_rest_snapshot, ArmSessionState.REST),
        (_work_snapshot, ArmSessionState.WORK),
        (_unknown_snapshot, ArmSessionState.UNVERIFIED),
    ),
)
def test_startup_feedback_is_classified(snapshot_factory, expected_state):
    session, _kinematics, _work, _transition = _session(snapshot_factory)

    assert session.state is expected_state


@pytest.mark.parametrize("offset", (2 * math.pi, -2 * math.pi))
def test_work_gate_rejects_positive_and_negative_full_turn_alias(offset):
    kinematics = LinearFakeKinematics()
    snapshot = _work_snapshot(kinematics)
    joints = list(snapshot.joint_radians)
    joints[5] += offset
    wrapped_snapshot = SO100PlusPoseSnapshot(
        driver_degrees=snapshot.driver_degrees,
        joint_radians=tuple(joints),
        tcp_position_m=snapshot.tcp_position_m,
        torque_enabled=snapshot.torque_enabled,
    )

    with pytest.raises(RuntimeError, match="360.000°"):
        validate_work_initial_pose(
            wrapped_snapshot,
            kinematics.forward_position(kinematics.work),
        )


def test_unfold_uses_escape_then_work_and_marks_work_after_feedback_gate():
    kinematics = LinearFakeKinematics()
    session, _kinematics, _work, transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        _work_snapshot(kinematics),
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is True
    assert session.state is ArmSessionState.WORK
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "execute_joint_plan",
    ]
    assert transition.calls[0][1].target_joint_radians == (
        session.transition.escape_joint_radians
    )
    assert transition.calls[1][1].target_joint_radians == (
        SO100_PLUS_JOYCON_INITIAL_RADIANS
    )


def test_unfold_gate_failure_marks_unverified():
    kinematics = LinearFakeKinematics()
    session, _kinematics, _work, transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        _unknown_snapshot(kinematics),
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is False
    assert "展开完成后的 TCP 和关节门禁" in result.message
    assert session.state is ArmSessionState.UNVERIFIED
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "execute_joint_plan",
    ]


def test_unfold_validates_complete_plans_before_executing_same_objects():
    kinematics = LinearFakeKinematics()
    event_log = []
    transition = RecordingSessionAdapter(event_log=event_log)
    validator = RecordingTrajectoryValidator(event_log=event_log)
    session, _kinematics, _work, _transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        _work_snapshot(kinematics),
        transition_adapter=transition,
        trajectory_validator=validator,
        event_log=event_log,
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is True
    planned = tuple(
        event[1] for event in event_log if event[0] == "plan"
    )
    validated = validator.storage_calls[0][0]
    executed = tuple(
        event[1]
        for event in event_log
        if event[0] == "execute_joint_plan"
    )
    assert [event[0] for event in event_log] == [
        "plan",
        "plan",
        "validate_storage",
        "execute_joint_plan",
        "execute_joint_plan",
    ]
    assert validated == planned
    assert executed == validated
    assert all(
        executed_plan is validated_plan
        for executed_plan, validated_plan in zip(
            executed,
            validated,
            strict=True,
        )
    )


def test_unfold_collision_precheck_fails_before_motion_and_keeps_rest():
    kinematics = LinearFakeKinematics()
    event_log = []
    transition = RecordingSessionAdapter(event_log=event_log)
    validator = RecordingTrajectoryValidator(
        event_log=event_log,
        storage_error=SO100PlusTrajectoryValidationError(
            "模拟新增碰撞"
        ),
    )
    session, _kinematics, _work, _transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        transition_adapter=transition,
        trajectory_validator=validator,
        event_log=event_log,
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is False
    assert "完整展开轨迹 MuJoCo 碰撞和接触预检查" in result.message
    assert "模拟新增碰撞" in result.message
    assert session.state is ArmSessionState.REST
    assert transition.calls == []
    assert [event[0] for event in event_log] == [
        "plan",
        "plan",
        "validate_storage",
    ]


def test_unfold_unavailable_mujoco_fails_closed_without_motion():
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter()
    validator = RecordingTrajectoryValidator(
        storage_error=SO100PlusTrajectoryValidationUnavailableError(
            "模拟 MuJoCo 不可用"
        ),
    )
    session, _kinematics, _work, _transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        transition_adapter=transition,
        trajectory_validator=validator,
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is False
    assert "完整展开轨迹 MuJoCo 碰撞和接触预检查" in result.message
    assert "模拟 MuJoCo 不可用" in result.message
    assert session.state is ArmSessionState.REST
    assert transition.calls == []


@pytest.mark.parametrize(
    "snapshot_factory",
    (_work_snapshot, _unknown_snapshot),
)
def test_unfold_is_rejected_outside_rest(snapshot_factory):
    session, _kinematics, _work, transition = _session(snapshot_factory)

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is False
    assert "只允许在 REST" in result.message
    assert transition.calls == []


def test_fold_returns_to_work_initial_then_escape_then_storage():
    kinematics = LinearFakeKinematics()
    session, _kinematics, _work, transition = _session(
        _work_snapshot,
        _work_snapshot(kinematics),
        _work_snapshot(kinematics),
        _rest_snapshot(kinematics),
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is True
    assert session.state is ArmSessionState.REST
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "execute_joint_plan",
    ]
    assert transition.calls[0][1].target_joint_radians == (
        session.transition.escape_joint_radians
    )
    assert transition.calls[1][1].target_joint_radians == (
        session.transition.storage_joint_radians
    )


def test_fold_storage_gate_failure_marks_unverified():
    kinematics = LinearFakeKinematics()
    session, _kinematics, _work, transition = _session(
        _work_snapshot,
        _work_snapshot(kinematics),
        _work_snapshot(kinematics),
        _unknown_snapshot(kinematics),
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is False
    assert "收纳完成后的 follower_rest 门禁" in result.message
    assert session.state is ArmSessionState.UNVERIFIED
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "execute_joint_plan",
    ]


def test_fold_plans_validates_and_executes_same_return_and_storage_plans():
    kinematics = LinearFakeKinematics()
    event_log = []
    transition = RecordingSessionAdapter(event_log=event_log)
    validator = RecordingTrajectoryValidator(event_log=event_log)
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        _formal_work_snapshot(kinematics),
        _work_snapshot(kinematics),
        _rest_snapshot(kinematics),
        transition_adapter=transition,
        trajectory_validator=validator,
        event_log=event_log,
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is True
    assert [event[0] for event in event_log] == [
        "plan",
        "validate_return",
        "execute_joint_plan",
        "plan",
        "plan",
        "validate_storage",
        "execute_joint_plan",
        "execute_joint_plan",
    ]
    executed = tuple(
        event[1]
        for event in event_log
        if event[0] == "execute_joint_plan"
    )
    validated_return = validator.return_calls[0]
    validated_storage = validator.storage_calls[0][0]
    assert executed == validated_return + validated_storage
    assert all(
        executed_plan is validated_plan
        for executed_plan, validated_plan in zip(
            executed,
            validated_return + validated_storage,
            strict=True,
        )
    )
    assert all(call[0] != "move_to" for call in transition.calls)


def test_fold_return_collision_precheck_does_not_move_and_keeps_work():
    kinematics = LinearFakeKinematics()
    event_log = []
    transition = RecordingSessionAdapter(event_log=event_log)
    validator = RecordingTrajectoryValidator(
        event_log=event_log,
        return_error=SO100PlusTrajectoryValidationError(
            "模拟返回路径碰撞"
        ),
    )
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        _formal_work_snapshot(kinematics),
        transition_adapter=transition,
        trajectory_validator=validator,
        event_log=event_log,
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is False
    assert "返回工作初始姿态轨迹 MuJoCo 碰撞预检查" in result.message
    assert "模拟返回路径碰撞" in result.message
    assert session.state is ArmSessionState.WORK
    assert transition.calls == []
    assert [event[0] for event in event_log] == [
        "plan",
        "validate_return",
    ]


def test_fold_storage_collision_precheck_does_not_move_and_keeps_work():
    kinematics = LinearFakeKinematics()
    event_log = []
    transition = RecordingSessionAdapter(event_log=event_log)
    validator = RecordingTrajectoryValidator(
        event_log=event_log,
        storage_error=SO100PlusTrajectoryValidationError(
            "模拟反向收纳路径碰撞"
        ),
    )
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        _work_snapshot(kinematics),
        _work_snapshot(kinematics),
        transition_adapter=transition,
        trajectory_validator=validator,
        event_log=event_log,
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is False
    assert "完整反向收纳轨迹 MuJoCo 碰撞和接触预检查" in result.message
    assert "模拟反向收纳路径碰撞" in result.message
    assert session.state is ArmSessionState.WORK
    assert transition.calls == []
    assert [event[0] for event in event_log] == [
        "plan",
        "validate_return",
        "plan",
        "plan",
        "validate_storage",
    ]


@pytest.mark.parametrize(
    "snapshot_factory",
    (_rest_snapshot, _unknown_snapshot),
)
def test_fold_is_rejected_outside_work(snapshot_factory):
    session, _kinematics, _work, transition = _session(snapshot_factory)

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is False
    assert "只允许在 WORK" in result.message
    assert transition.calls == []


def test_move_and_gripper_actions_are_rejected_outside_work():
    for snapshot_factory in (_rest_snapshot, _unknown_snapshot):
        session, _kinematics, work, _transition = _session(snapshot_factory)

        move_result = session.move_arm(
            _command(
                "move_arm",
                {"x": 0.35, "y": 0.0, "z": 0.22},
            )
        )
        open_result = session.open_gripper(_command("open_gripper"))
        close_result = session.close_gripper(_command("close_gripper"))

        assert move_result.success is False
        assert open_result.success is False
        assert close_result.success is False
        assert work.calls == []


def test_transition_rejects_move_and_stop_interrupts_unfold():
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter(
        block_operation="execute_joint_plan"
    )
    session, _kinematics, work, _transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        _work_snapshot(kinematics),
        transition_adapter=transition,
    )
    results = []
    worker = Thread(
        target=lambda: results.append(
            session.unfold_arm(_command("unfold_arm"))
        )
    )
    worker.start()
    assert transition.operation_started.wait(timeout=1.0) is True
    assert session.state is ArmSessionState.TRANSITION

    move_result = session.move_arm(
        _command(
            "move_arm",
            {"x": 0.35, "y": 0.0, "z": 0.22},
        )
    )
    stop_result = session.stop(_command("stop"))
    worker.join(timeout=1.0)

    assert move_result.success is False
    assert stop_result.success is True
    assert worker.is_alive() is False
    assert results[0].success is False
    assert session.state is ArmSessionState.UNVERIFIED
    assert work.calls == []
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "stop",
    ]


def test_stop_interrupts_fold_return_and_marks_unverified():
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter(
        block_operation="execute_joint_plan"
    )
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        _formal_work_snapshot(kinematics),
        _work_snapshot(kinematics),
        _rest_snapshot(kinematics),
        transition_adapter=transition,
    )
    results = []
    worker = Thread(
        target=lambda: results.append(session.fold_arm(_command("fold_arm")))
    )
    worker.start()
    assert transition.operation_started.wait(timeout=1.0) is True

    stop_result = session.stop(_command("stop"))
    worker.join(timeout=1.0)

    assert stop_result.success is True
    assert worker.is_alive() is False
    assert results[0].success is False
    assert session.state is ArmSessionState.UNVERIFIED
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "stop",
    ]


def test_stop_interrupts_fold_storage_path_and_marks_unverified():
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter(
        block_operation="execute_joint_plan"
    )
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        _work_snapshot(kinematics),
        _work_snapshot(kinematics),
        _rest_snapshot(kinematics),
        transition_adapter=transition,
    )
    results = []
    worker = Thread(
        target=lambda: results.append(session.fold_arm(_command("fold_arm")))
    )
    worker.start()
    assert transition.operation_started.wait(timeout=1.0) is True

    stop_result = session.stop(_command("stop"))
    worker.join(timeout=1.0)

    assert stop_result.success is True
    assert worker.is_alive() is False
    assert results[0].success is False
    assert session.state is ArmSessionState.UNVERIFIED
    assert [call[0] for call in transition.calls] == [
        "execute_joint_plan",
        "stop",
    ]
