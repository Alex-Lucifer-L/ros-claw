import math

import numpy as np
import pytest

from rosclaw_mini.arm.so100_plus_session import (
    SO100PlusArmSession,
    SO100PlusPoseSnapshot,
    SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
    SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
)
from rosclaw_mini.arm.kinematics import SO100PlusKinematics
from rosclaw_mini.arm.so100_plus import SO100_PLUS_REAL_HARDWARE_PROFILE
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
)
from rosclaw_mini.command_schema.commands import Command
from rosclaw_mini.safety.limits import (
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
)
from rosclaw_mini.workspace_scan.irregular_workspace import (
    DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH,
    DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_SHA256,
    IrregularWorkspaceError,
    SO100PlusIrregularWorkspace,
    load_default_so100_plus_irregular_workspace,
)


class OfflineRecordingAdapter:
    """只记录最终计划；不实例化 Robot，也没有硬件能力。"""

    def __init__(self) -> None:
        self.executed = []
        self.motion_waypoint_written = False
        self.active = False

    def begin_motion_action(self):
        self.active = True
        self.motion_waypoint_written = False

    def end_motion_action(self):
        self.active = False

    def materialize_joint_plan(
        self,
        plan,
        *,
        held_gripper_driver_degrees=None,
    ):
        from dataclasses import replace

        return replace(
            plan,
            is_final_execution_plan=True,
            held_gripper_driver_degrees=held_gripper_driver_degrees,
        )

    def execute_joint_plan(self, plan):
        self.motion_waypoint_written = bool(plan.waypoints_radians)
        self.executed.append(plan)

    def stop(self):
        return None


class SnapshotQueue:
    def __init__(self, *snapshots):
        self.snapshots = list(snapshots)

    def __call__(self):
        if not self.snapshots:
            raise AssertionError("缺少离线姿态快照")
        return self.snapshots.pop(0)


def test_default_irregular_workspace_loads_bound_scan_snapshot():
    workspace = load_default_so100_plus_irregular_workspace()

    assert workspace.source_path == (
        DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH
    )
    assert workspace.source_sha256 == (
        DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_SHA256
    )
    assert workspace.grid_step_m == pytest.approx(0.01)
    assert workspace.valid_point_count == 10_974
    assert workspace.valid_cell_count == 9_044
    assert workspace.validate_position(
        *SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M
    ) == pytest.approx(SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M)


def test_irregular_workspace_accepts_continuous_full_cell_outside_old_box():
    workspace = load_default_so100_plus_irregular_workspace()
    target = (0.45, -0.08, 0.15)

    with pytest.raises(ValueError):
        SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS.validate_position(*target)
    assert workspace.validate_position(*target) == pytest.approx(target)


def test_irregular_workspace_rejects_invalid_hole_inside_aabb():
    workspace = load_default_so100_plus_irregular_workspace()
    target = (0.52, 0.05, 0.15)

    workspace.endpoint_aabb.validate_position(*target)
    with pytest.raises(
        IrregularWorkspaceError,
        match="必要角点无效",
    ):
        workspace.validate_position(*target)


def test_every_saved_valid_grid_point_is_accepted_and_has_saved_joints():
    workspace = load_default_so100_plus_irregular_workspace()
    with np.load(
        DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH,
        allow_pickle=False,
    ) as data:
        points = np.asarray(data["valid_tcp_points_m"], dtype=float)

    for point in points:
        assert workspace.validate_position(*point) == pytest.approx(point)
        joints = workspace.target_joint_radians_at_grid_point(point)
        assert joints is not None
        assert len(joints) == 6
        assert all(math.isfinite(value) for value in joints)


@pytest.mark.parametrize(
    "target",
    (
        (True, 0.0, 0.2),
        (math.nan, 0.0, 0.2),
        (math.inf, 0.0, 0.2),
        (1.0, 0.0, 0.2),
    ),
)
def test_irregular_workspace_rejects_invalid_or_outside_target(target):
    workspace = load_default_so100_plus_irregular_workspace()

    with pytest.raises(IrregularWorkspaceError):
        workspace.validate_position(*target)


def test_relative_target_reports_current_displacement_and_final_target():
    workspace = load_default_so100_plus_irregular_workspace()
    current = (0.45, -0.08, 0.15)
    displacement = (0.07, 0.13, 0.0)

    with pytest.raises(IrregularWorkspaceError) as captured:
        workspace.resolve_relative_target(current, displacement)

    message = str(captured.value)
    assert f"当前 TCP={current}" in message
    assert f"请求位移 dx/dy/dz={displacement}" in message
    assert "最终目标=(0.52, 0.05, 0.15)" in message


def test_default_grid_hash_mismatch_fails_closed():
    with pytest.raises(IrregularWorkspaceError, match="SHA-256 不匹配"):
        SO100PlusIrregularWorkspace.from_npz(
            DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH,
            expected_sha256="0" * 64,
        )


def test_real_kinematics_and_mujoco_accept_saved_point_outside_old_box():
    workspace = load_default_so100_plus_irregular_workspace()
    kinematics = SO100PlusKinematics()
    with np.load(
        DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH,
        allow_pickle=False,
    ) as data:
        points = np.asarray(data["valid_tcp_points_m"], dtype=float)
        joints = np.asarray(
            data["target_joint_radians"],
            dtype=float,
        )[np.asarray(data["status"]) == int(data["valid_status_code"])]
    old = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS
    outside = np.asarray(
        [
            not (
                old.x.minimum <= point[0] <= old.x.maximum
                and old.y.minimum <= point[1] <= old.y.maximum
                and old.z.minimum <= point[2] <= old.z.maximum
            )
            for point in points
        ],
        dtype=bool,
    )
    distances = np.linalg.norm(
        points - np.asarray(SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M),
        axis=1,
    )
    candidate_index = int(np.argmin(np.where(outside, distances, np.inf)))
    target = tuple(float(value) for value in points[candidate_index])
    target_joints = tuple(float(value) for value in joints[candidate_index])

    middle_snapshot = SO100PlusPoseSnapshot(
        driver_degrees=(
            SO100PlusKinematics.model_radians_to_driver_degrees(
                SO100_PLUS_MIDDLE_INTERNAL_RADIANS
            )
        ),
        joint_radians=SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
        tcp_position_m=SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
        gripper_driver_degrees=-5.0,
        torque_enabled=(1,) * 7,
    )
    target_snapshot = SO100PlusPoseSnapshot(
        driver_degrees=(
            SO100PlusKinematics.model_radians_to_driver_degrees(target_joints)
        ),
        joint_radians=target_joints,
        tcp_position_m=target,
        gripper_driver_degrees=-5.0,
        torque_enabled=(1,) * 7,
    )
    work = OfflineRecordingAdapter()
    transition = OfflineRecordingAdapter()
    storage = SO100PlusKinematics.driver_degrees_to_model_radians(
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
    )
    session = SO100PlusArmSession(
        work_adapter=work,
        transition_adapter=transition,
        pose_reader=SnapshotQueue(middle_snapshot, target_snapshot),
        kinematics=kinematics,
        initial_snapshot=middle_snapshot,
        storage_joint_radians=storage,
        transition_motion_limits=workspace.build_motion_limits(
            storage,
            max_step_radians=math.radians(1.0),
        ),
        trajectory_validator=SO100PlusMuJoCoTrajectoryValidator(),
        work_workspace=workspace,
    )

    result = session.move_arm(
        Command(
            command_id="offline-irregular-move",
            skill_name="move_arm",
            params={"x": target[0], "y": target[1], "z": target[2]},
            source="test",
        )
    )

    assert result.success is True
    assert len(work.executed) == 2
    assert work.executed[-1].target_joint_radians == pytest.approx(
        target_joints
    )
    assert all(plan.is_final_execution_plan for plan in work.executed)
    assert transition.executed == []
