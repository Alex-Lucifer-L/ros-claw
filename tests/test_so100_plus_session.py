from dataclasses import replace
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
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.safety.limits import (
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
)


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
        self.action_active = False
        self.motion_waypoint_written = False
        self.materialized_plans = []

    def begin_motion_action(self) -> None:
        if self.action_active:
            raise RuntimeError("测试动作重复注册")
        self.motion_waypoint_written = False
        self.action_active = True

    def end_motion_action(self) -> None:
        self.action_active = False

    def materialize_joint_plan(
        self,
        plan,
        *,
        held_gripper_driver_degrees=None,
    ):
        finalized = replace(
            plan,
            is_final_execution_plan=True,
            held_gripper_driver_degrees=held_gripper_driver_degrees,
        )
        self.materialized_plans.append(finalized)
        return finalized

    def _record(self, name: str, *values) -> None:
        call = (name, *values)
        self.calls.append(call)
        self.event_log.append(call)
        if self.block_operation == name:
            self.operation_started.set()
            self.release_operation.wait(timeout=2.0)

    def move_to(self, x, y, z) -> None:
        self._record("move_to", x, y, z)

    def plan_move_to(self, x, y, z):
        self._record("plan_move_to", x, y, z)
        current = (0.0,) * 6
        target = (0.01,) * 6
        return JointMotionPlan(
            target_position_m=(x, y, z),
            current_joint_radians=current,
            target_joint_radians=target,
            waypoints_radians=(target,),
        )

    def move_joints(self, joint_radians) -> None:
        self._record("move_joints", tuple(joint_radians))

    def execute_joint_plan(self, plan) -> None:
        self.motion_waypoint_written = True
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
    def gripper_driver_degrees_to_qpos(driver_degrees):
        if driver_degrees is None or not math.isfinite(driver_degrees):
            raise SO100PlusTrajectoryValidationError("模拟夹爪映射失败")
        return math.radians(driver_degrees)

    @staticmethod
    def _verified(plans, gripper_qpos):
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
            gripper_qpos=gripper_qpos,
        )

    def verify_storage_transition(
        self,
        plans,
        *,
        escape_joint_radians,
        kinematics,
        direction,
        gripper_qpos,
    ):
        del escape_joint_radians, kinematics
        plans = tuple(plans)
        self.storage_calls.append((plans, direction, gripper_qpos))
        self.event_log.append(("validate_storage", plans, direction))
        if self.storage_error is not None:
            raise self.storage_error
        return self._verified(plans, gripper_qpos)

    def verify_collision_free_sequence(
        self,
        plans,
        kinematics,
        *,
        gripper_qpos,
    ):
        del kinematics
        plans = tuple(plans)
        self.return_calls.append((plans, gripper_qpos))
        self.event_log.append(("validate_return", plans))
        if self.return_error is not None:
            raise self.return_error
        return self._verified(plans, gripper_qpos)


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
        gripper_driver_degrees=-9.0,
        torque_enabled=(1,) * 7,
    )


def _work_snapshot(kinematics) -> SO100PlusPoseSnapshot:
    return SO100PlusPoseSnapshot(
        driver_degrees=SO100PlusKinematics.model_radians_to_driver_degrees(
            kinematics.work
        ),
        joint_radians=kinematics.work,
        tcp_position_m=kinematics.forward_position(kinematics.work),
        gripper_driver_degrees=-9.0,
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
        gripper_driver_degrees=-9.0,
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
        gripper_driver_degrees=-9.0,
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
    assert len(planned) == len(validated)
    assert validated == tuple(transition.materialized_plans)
    assert all(plan.is_final_execution_plan for plan in validated)
    assert executed == validated
    assert all(
        executed_plan is validated_plan
        for executed_plan, validated_plan in zip(
            executed,
            validated,
            strict=True,
        )
    )


@pytest.mark.parametrize("gripper_degrees", (-9.0, 30.0, 60.0))
def test_unfold_precheck_uses_actual_gripper_and_execution_holds_it(
    gripper_degrees,
):
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter()
    validator = RecordingTrajectoryValidator()

    def rest_with_gripper(_kinematics):
        return replace(
            _rest_snapshot(_kinematics),
            gripper_driver_degrees=gripper_degrees,
        )

    session, _kinematics, _work, _transition = _session(
        rest_with_gripper,
        rest_with_gripper(kinematics),
        replace(
            _work_snapshot(kinematics),
            gripper_driver_degrees=gripper_degrees,
        ),
        transition_adapter=transition,
        trajectory_validator=validator,
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is True
    assert validator.storage_calls[0][2] == pytest.approx(
        math.radians(gripper_degrees)
    )
    assert all(
        plan.held_gripper_driver_degrees == gripper_degrees
        for plan in transition.materialized_plans
    )
    assert all(
        call[1].held_gripper_driver_degrees == gripper_degrees
        for call in transition.calls
        if call[0] == "execute_joint_plan"
    )


@pytest.mark.parametrize("invalid_gripper", (None, float("nan")))
def test_unfold_invalid_gripper_feedback_fails_before_any_motion(
    invalid_gripper,
):
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter()
    invalid_snapshot = replace(
        _rest_snapshot(kinematics),
        gripper_driver_degrees=invalid_gripper,
    )
    session, _kinematics, _work, _transition = _session(
        _rest_snapshot,
        invalid_snapshot,
        transition_adapter=transition,
    )

    result = session.unfold_arm(_command("unfold_arm"))

    assert result.success is False
    assert "展开前 follower_rest 复核" in result.message
    assert "gripper_joint" in result.message
    assert session.state is ArmSessionState.REST
    assert transition.calls == []
    assert transition.materialized_plans == []


def test_stop_after_controller_submit_before_skill_entry_is_not_lost():
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter()
    session, _kinematics, _work, _transition = _session(
        _rest_snapshot,
        _rest_snapshot(kinematics),
        transition_adapter=transition,
    )
    worker_reached_runner = Event()
    allow_skill_entry = Event()

    def runner(command):
        if command.skill_name == "stop":
            return session.stop(command)
        worker_reached_runner.set()
        assert allow_skill_entry.wait(timeout=1.0)
        return session.unfold_arm(command)

    controller = ExecutionController(
        runner,
        before_submit=session.prepare_command,
        after_finish=session.finish_command,
    )
    assert controller.submit(_command("unfold_arm")) is True
    assert worker_reached_runner.wait(timeout=1.0)

    stop_result = controller.request_stop(_command("stop"))
    allow_skill_entry.set()
    result = controller.wait(timeout=1.0)

    assert stop_result.success is True
    assert result is not None
    assert result.success is False
    assert "首条运动指令前已被 stop 中断" in result.message
    assert session.state is ArmSessionState.UNVERIFIED
    assert transition.calls == [("stop",)]
    assert transition.action_active is False


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
    validated_return = validator.return_calls[0][0]
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


def test_fold_return_and_storage_prechecks_share_actual_held_gripper():
    kinematics = LinearFakeKinematics()
    gripper_degrees = 35.0
    transition = RecordingSessionAdapter()
    validator = RecordingTrajectoryValidator()

    def with_gripper(snapshot):
        return replace(
            snapshot,
            gripper_driver_degrees=gripper_degrees,
        )

    session, _kinematics, _work, _transition = _session(
        lambda inner: with_gripper(_work_snapshot(inner)),
        with_gripper(_formal_work_snapshot(kinematics)),
        with_gripper(_work_snapshot(kinematics)),
        with_gripper(_rest_snapshot(kinematics)),
        transition_adapter=transition,
        trajectory_validator=validator,
    )

    result = session.fold_arm(_command("fold_arm"))

    expected_qpos = math.radians(gripper_degrees)
    assert result.success is True
    assert validator.return_calls[0][1] == pytest.approx(expected_qpos)
    assert validator.storage_calls[0][2] == pytest.approx(expected_qpos)
    assert all(
        plan.held_gripper_driver_degrees == gripper_degrees
        for plan in transition.materialized_plans
    )


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


def test_move_relative_uses_execution_time_tcp_and_verified_absolute_plan():
    kinematics = LinearFakeKinematics()
    current_snapshot = replace(
        _work_snapshot(kinematics),
        tcp_position_m=(0.35, -0.01, 0.24),
    )
    work = RecordingSessionAdapter()
    validator = RecordingTrajectoryValidator()
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        current_snapshot,
        work_adapter=work,
        trajectory_validator=validator,
    )

    result = session.move_relative(
        _command(
            "move_relative",
            {"dx": 0.0, "dy": 0.0, "dz": 0.02},
        )
    )

    assert result.success is True
    assert work.calls[0][0] == "plan_move_to"
    assert work.calls[0][1:] == pytest.approx((0.35, -0.01, 0.26))
    assert work.calls[1][0] == "execute_joint_plan"
    validated_plan = validator.return_calls[0][0][0]
    executed_plan = work.calls[1][1]
    assert executed_plan is validated_plan
    assert executed_plan is work.materialized_plans[0]
    assert all(call[0] != "move_to" for call in work.calls)
    assert "dx/dy/dz=(0.0, 0.0, 0.02)" in result.message
    assert "最终位置 (0.35, -0.01, 0.26)" in result.message


def test_move_relative_rejects_workspace_target_before_plan_or_motion():
    kinematics = LinearFakeKinematics()
    workspace = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS
    current = (
        workspace.x.maximum - 0.001,
        0.0,
        0.22,
    )
    work = RecordingSessionAdapter()
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        replace(_work_snapshot(kinematics), tcp_position_m=current),
        work_adapter=work,
    )

    result = session.move_relative(
        _command(
            "move_relative",
            {"dx": 0.01, "dy": 0.0, "dz": 0.0},
        )
    )

    assert result.success is False
    assert session.state is ArmSessionState.WORK
    assert work.calls == []
    assert work.materialized_plans == []
    assert "当前 TCP=" in result.message
    assert "请求位移 dx/dy/dz=" in result.message
    assert "最终目标=" in result.message
    assert "x=" in result.message
    assert "超出允许范围" in result.message


def test_move_relative_zero_displacement_fails_before_plan_or_motion():
    kinematics = LinearFakeKinematics()
    work = RecordingSessionAdapter()
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        replace(
            _work_snapshot(kinematics),
            tcp_position_m=(0.35, -0.01, 0.24),
        ),
        work_adapter=work,
    )

    result = session.move_relative(
        _command(
            "move_relative",
            {"dx": 0.0, "dy": 0.0, "dz": 0.0},
        )
    )

    assert result.success is False
    assert session.state is ArmSessionState.WORK
    assert work.calls == []
    assert "dx/dy/dz 不能全部为 0" in result.message


def test_move_relative_collision_precheck_prevents_execution():
    kinematics = LinearFakeKinematics()
    work = RecordingSessionAdapter()
    validator = RecordingTrajectoryValidator(
        return_error=SO100PlusTrajectoryValidationError(
            "模拟工作区路径碰撞"
        )
    )
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        replace(
            _work_snapshot(kinematics),
            tcp_position_m=(0.35, -0.01, 0.24),
        ),
        work_adapter=work,
        trajectory_validator=validator,
    )

    result = session.move_relative(
        _command(
            "move_relative",
            {"dx": 0.0, "dy": 0.0, "dz": 0.02},
        )
    )

    assert result.success is False
    assert session.state is ArmSessionState.WORK
    assert "工作区完整轨迹 MuJoCo 预检查失败" in result.message
    assert "模拟工作区路径碰撞" in result.message
    assert [call[0] for call in work.calls] == ["plan_move_to"]


@pytest.mark.parametrize(
    ("skill_name", "params"),
    (
        ("move_arm", {"x": 0.36, "y": -0.01, "z": 0.25}),
        ("move_relative", {"dx": 0.01, "dy": 0.0, "dz": 0.0}),
    ),
)
def test_work_motion_failure_after_motor_write_marks_unverified(
    skill_name,
    params,
):
    class FailingAfterWriteAdapter(RecordingSessionAdapter):
        def execute_joint_plan(self, plan) -> None:
            self.motion_waypoint_written = True
            self._record("execute_joint_plan", plan)
            raise RuntimeError("模拟已写入后跟踪失败")

    kinematics = LinearFakeKinematics()
    work = FailingAfterWriteAdapter()
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        replace(
            _work_snapshot(kinematics),
            tcp_position_m=(0.35, -0.01, 0.24),
        ),
        work_adapter=work,
    )

    result = getattr(session, skill_name)(_command(skill_name, params))

    assert result.success is False
    assert session.state is ArmSessionState.UNVERIFIED
    assert "模拟已写入后跟踪失败" in result.message
    assert "会话已标记为 UNVERIFIED" in result.message


def test_work_execution_failure_before_first_write_preserves_work_state():
    class FailingBeforeWriteAdapter(RecordingSessionAdapter):
        def execute_joint_plan(self, plan) -> None:
            self._record("execute_joint_plan", plan)
            raise RuntimeError("模拟首条写入前起点门禁失败")

    kinematics = LinearFakeKinematics()
    work = FailingBeforeWriteAdapter()
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        replace(
            _work_snapshot(kinematics),
            tcp_position_m=(0.35, -0.01, 0.24),
        ),
        work_adapter=work,
    )

    result = session.move_relative(
        _command(
            "move_relative",
            {"dx": 0.01, "dy": 0.0, "dz": 0.0},
        )
    )

    assert result.success is False
    assert session.state is ArmSessionState.WORK
    assert "模拟首条写入前起点门禁失败" in result.message
    assert "UNVERIFIED" not in result.message


def test_move_arm_keeps_absolute_target_semantics_with_shared_work_path():
    kinematics = LinearFakeKinematics()
    work = RecordingSessionAdapter()
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
        replace(
            _work_snapshot(kinematics),
            tcp_position_m=(0.35, -0.01, 0.24),
        ),
        work_adapter=work,
    )

    result = session.move_arm(
        _command(
            "move_arm",
            {"x": 0.36, "y": -0.01, "z": 0.25},
        )
    )

    assert result.success is True
    assert result.message == "夹爪 TCP 已完成工作区移动"
    assert work.calls[0] == ("plan_move_to", 0.36, -0.01, 0.25)
    assert work.calls[1][0] == "execute_joint_plan"


def test_move_and_gripper_actions_are_rejected_outside_work():
    for snapshot_factory in (_rest_snapshot, _unknown_snapshot):
        session, _kinematics, work, _transition = _session(snapshot_factory)

        move_result = session.move_arm(
            _command(
                "move_arm",
                {"x": 0.35, "y": 0.0, "z": 0.22},
            )
        )
        relative_result = session.move_relative(
            _command(
                "move_relative",
                {"dx": 0.0, "dy": 0.0, "dz": 0.02},
            )
        )
        open_result = session.open_gripper(_command("open_gripper"))
        close_result = session.close_gripper(_command("close_gripper"))

        assert move_result.success is False
        assert relative_result.success is False
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
