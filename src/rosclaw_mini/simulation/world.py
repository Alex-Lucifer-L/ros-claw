"""A deterministic MuJoCo-backed kinematic world for offline research.

The SO-100 Plus MJCF and collision mesh are loaded through the existing
``SO100PlusMuJoCoTrajectoryValidator``.  Simple movable tabletop objects are
represented in this layer so the research loop can remain fast and headless.
Their coordinates are never exposed to normal perception/planning code; they
only create RGB-D pixels and evaluate experiment results.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus_session import (
    SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
    SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
    SO100PlusTrajectoryValidationError,
)
from rosclaw_mini.safety.limits import MotionLimits
from rosclaw_mini.simulation.config import (
    SimulationObjectSpec,
    SimulationSceneConfig,
)
from rosclaw_mini.workspace_scan.irregular_workspace import (
    SO100PlusIrregularWorkspace,
    load_default_so100_plus_irregular_workspace,
)


class SimulationMotionError(RuntimeError):
    """The offline simulator cannot safely plan, validate, or execute a move."""


@dataclass
class SimulatedObjectState:
    """Mutable physical state kept inside the simulation world only."""

    spec: SimulationObjectSpec
    grasp_point_m: tuple[float, float, float]
    initial_grasp_point_m: tuple[float, float, float]
    attached: bool = False
    slipped: bool = False

    @property
    def name(self) -> str:
        return self.spec.name


@dataclass(frozen=True)
class SimulationMotionRecord:
    target_tcp_m: tuple[float, float, float]
    actual_tcp_m: tuple[float, float, float]
    waypoint_count: int
    max_tcp_error_m: float
    collision_checked: bool


class SimulationWorld:
    """Simulation-only world state shared by adapter, camera and evaluator."""

    def __init__(
        self,
        scene: SimulationSceneConfig,
        *,
        kinematics: SO100PlusKinematics | None = None,
        workspace: SO100PlusIrregularWorkspace | None = None,
        trajectory_validator: SO100PlusMuJoCoTrajectoryValidator | None = None,
    ) -> None:
        if scene.simulation_only is not True:
            raise SimulationMotionError("拒绝加载未标记 simulation_only 的场景。")
        self.scene = scene
        self.kinematics = kinematics or SO100PlusKinematics()
        self.workspace = workspace or load_default_so100_plus_irregular_workspace()
        self.trajectory_validator = (
            trajectory_validator or SO100PlusMuJoCoTrajectoryValidator()
        )
        self._rng = np.random.default_rng(scene.seed)
        self._objects = self._build_objects(scene)
        self.joint_radians: tuple[float, ...] = tuple(
            SO100_PLUS_MIDDLE_INTERNAL_RADIANS
        )
        self.gripper_is_open = True
        self.session_pose = "WORK"
        self.motion_records: list[SimulationMotionRecord] = []
        self.collision_events: list[str] = []

    def _build_objects(
        self,
        scene: SimulationSceneConfig,
    ) -> dict[str, SimulatedObjectState]:
        objects: dict[str, SimulatedObjectState] = {}
        for spec in scene.objects:
            point = spec.grasp_point_m
            if scene.name == "randomized":
                # Keep randomisation small enough to stay inside the certified
                # local grid around middle_internal; exact membership is still
                # verified when the plan is previewed.
                offset = self._rng.uniform(-0.02, 0.02, size=3)
                offset[2] = self._rng.uniform(-0.01, 0.02)
                point = tuple(float(value) for value in (np.asarray(point) + offset))
            objects[spec.name] = SimulatedObjectState(
                spec=spec,
                grasp_point_m=point,
                initial_grasp_point_m=point,
            )
        return objects

    @property
    def objects(self) -> tuple[SimulatedObjectState, ...]:
        return tuple(self._objects.values())

    @property
    def tcp_position_m(self) -> tuple[float, float, float]:
        return tuple(self.kinematics.forward_position(self.joint_radians))

    @property
    def gripper_driver_degrees(self) -> float:
        # Same mapping used by the existing MuJoCo trajectory validator.
        return 60.0 if self.gripper_is_open else -5.0

    def object_state(self, name: str) -> SimulatedObjectState:
        try:
            return self._objects[name]
        except KeyError as error:
            raise SimulationMotionError(f"仿真场景不存在物体：{name}。") from error

    def build_motion_limits(self) -> MotionLimits:
        return self.workspace.build_motion_limits(
            self.joint_radians,
            max_step_radians=SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS,
        )

    def plan_tcp_motion(self, target_m: Sequence[float]) -> JointMotionPlan:
        """Plan only after exact irregular-workspace membership is confirmed."""

        try:
            target = self.workspace.validate_position(*target_m)
            plan = self.kinematics.plan_position(
                self.joint_radians,
                target,
                self.build_motion_limits(),
            )
        except Exception as error:
            raise SimulationMotionError(f"仿真 TCP 规划失败：{error}") from error
        return JointMotionPlan(
            target_position_m=plan.target_position_m,
            current_joint_radians=plan.current_joint_radians,
            target_joint_radians=plan.target_joint_radians,
            waypoints_radians=plan.waypoints_radians,
            is_final_execution_plan=True,
            waypoint_interval_seconds=1.0 / 30.0,
            held_gripper_driver_degrees=self.gripper_driver_degrees,
        )

    def validate_plan(self, plan: JointMotionPlan) -> None:
        """Run the same local MuJoCo robot collision checker before motion."""

        try:
            verified = self.trajectory_validator.verify_collision_free_sequence(
                (plan,),
                self.kinematics,
                gripper_qpos=self.trajectory_validator.gripper_driver_degrees_to_qpos(
                    self.gripper_driver_degrees
                ),
            )
        except SO100PlusTrajectoryValidationError as error:
            raise SimulationMotionError(
                f"仿真 MuJoCo 轨迹预检查失败：{error}"
            ) from error
        # The existing validator has the real robot mesh/self contacts.  This
        # simulation adds its own table plane conservatively without claiming
        # it is the real laboratory collision geometry.
        for index, joints in enumerate(verified.sampled_joint_radians):
            tcp = tuple(self.kinematics.forward_position(joints))
            if tcp[2] < self.scene.table_z_m - 1e-9:
                raise SimulationMotionError(
                    "仿真桌面碰撞预检查失败："
                    f"第 {index} 个轨迹点 TCP Z={tcp[2]:.6f} m 低于"
                    f"simulation table Z={self.scene.table_z_m:.6f} m。"
                )

    def apply_joint_waypoint(self, joints: Sequence[float]) -> None:
        values = tuple(float(value) for value in joints)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise SimulationMotionError("仿真关节轨迹点必须是六个有限弧度值。")
        self.joint_radians = values
        tcp = self.tcp_position_m
        for item in self._objects.values():
            if item.attached:
                item.grasp_point_m = tcp

    def record_completed_plan(self, plan: JointMotionPlan) -> SimulationMotionRecord:
        actual = self.tcp_position_m
        target = tuple(plan.target_position_m)
        error = math.dist(actual, target)
        record = SimulationMotionRecord(
            target_tcp_m=target,
            actual_tcp_m=actual,
            waypoint_count=len(plan.waypoints_radians),
            max_tcp_error_m=error,
            collision_checked=True,
        )
        self.motion_records.append(record)
        return record

    def close_gripper(self, *, grasp_tolerance_m: float = 0.035) -> bool:
        """Attach one nearby virtual object; no truth is exposed to planner."""

        self.gripper_is_open = False
        tcp = self.tcp_position_m
        candidates = sorted(
            self._objects.values(),
            key=lambda item: math.dist(item.grasp_point_m, tcp),
        )
        if not candidates:
            return False
        target = candidates[0]
        if math.dist(target.grasp_point_m, tcp) <= grasp_tolerance_m:
            target.attached = True
            target.slipped = False
            target.grasp_point_m = tcp
            return True
        return False

    def open_gripper(self) -> None:
        self.gripper_is_open = True
        for item in self._objects.values():
            item.attached = False

    def verify_lift(
        self,
        object_name: str,
        *,
        minimum_lift_m: float,
    ) -> tuple[bool, str]:
        item = self.object_state(object_name)
        achieved = item.grasp_point_m[2] - item.initial_grasp_point_m[2]
        if not item.attached:
            return False, "夹爪未保持目标物体，判定抓取失败。"
        if achieved < minimum_lift_m:
            return (
                False,
                f"目标仅抬升 {achieved:.4f} m，小于要求 {minimum_lift_m:.4f} m。",
            )
        return True, f"目标已抬升 {achieved:.4f} m，仿真抓取验证通过。"

    def reset_to_work(self) -> None:
        self.session_pose = "WORK"
        self.joint_radians = tuple(SO100_PLUS_MIDDLE_INTERNAL_RADIANS)

    def reset_to_rest(self) -> None:
        # ``REST`` is an explicit simulation session label.  The real storage
        # trajectory is not claimed to be re-certified by this offline model.
        self.session_pose = "REST"

    def camera_object_truth(self) -> tuple[SimulatedObjectState, ...]:
        """Internal renderer/evaluator input; never pass this to a planner."""

        return self.objects

    def reference_tcp(self) -> tuple[float, float, float]:
        return tuple(SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M)
