import math
from threading import Event, Thread

import pytest

from rosclaw_mini.arm.kinematics import (
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
from rosclaw_mini.command_schema.commands import Command


class LinearFakeKinematics:
    """只为状态机测试提供单调的内存 FK，不加载机械臂模型。"""

    def __init__(self) -> None:
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


class RecordingSessionAdapter:
    def __init__(self, *, block_operation: str | None = None) -> None:
        self.calls: list[tuple] = []
        self.block_operation = block_operation
        self.operation_started = Event()
        self.release_operation = Event()

    def _record(self, name: str, *values) -> None:
        self.calls.append((name, *values))
        if self.block_operation == name:
            self.operation_started.set()
            self.release_operation.wait(timeout=2.0)

    def move_to(self, x, y, z) -> None:
        self._record("move_to", x, y, z)

    def move_joints(self, joint_radians) -> None:
        self._record("move_joints", tuple(joint_radians))

    def open_gripper(self) -> None:
        self._record("open_gripper")

    def close_gripper(self) -> None:
        self._record("close_gripper")

    def stop(self) -> None:
        self.calls.append(("stop",))
        self.release_operation.set()


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
):
    kinematics = LinearFakeKinematics()
    work = work_adapter or RecordingSessionAdapter()
    transition = transition_adapter or RecordingSessionAdapter()
    session = SO100PlusArmSession(
        work_adapter=work,
        transition_adapter=transition,
        pose_reader=PoseQueue(*operation_snapshots),
        kinematics=kinematics,
        initial_snapshot=initial_snapshot(kinematics),
        storage_joint_radians=kinematics.storage,
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
    assert transition.calls == [
        ("move_joints", session.transition.escape_joint_radians),
        ("move_joints", SO100_PLUS_JOYCON_INITIAL_RADIANS),
    ]


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
        "move_joints",
        "move_joints",
    ]


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
        _rest_snapshot(kinematics),
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is True
    assert session.state is ArmSessionState.REST
    assert transition.calls == [
        ("move_to", *session.work_tcp_position_m),
        ("move_joints", session.transition.escape_joint_radians),
        ("move_joints", session.transition.storage_joint_radians),
    ]


def test_fold_storage_gate_failure_marks_unverified():
    kinematics = LinearFakeKinematics()
    session, _kinematics, _work, transition = _session(
        _work_snapshot,
        _work_snapshot(kinematics),
        _unknown_snapshot(kinematics),
    )

    result = session.fold_arm(_command("fold_arm"))

    assert result.success is False
    assert "收纳完成后的 follower_rest 门禁" in result.message
    assert session.state is ArmSessionState.UNVERIFIED
    assert [call[0] for call in transition.calls] == [
        "move_to",
        "move_joints",
        "move_joints",
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
    transition = RecordingSessionAdapter(block_operation="move_joints")
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
        "move_joints",
        "stop",
    ]


def test_stop_interrupts_fold_return_and_marks_unverified():
    kinematics = LinearFakeKinematics()
    transition = RecordingSessionAdapter(block_operation="move_to")
    session, _kinematics, _work, _transition = _session(
        _work_snapshot,
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
    assert [call[0] for call in transition.calls] == ["move_to", "stop"]
