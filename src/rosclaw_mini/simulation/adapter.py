"""ArmAdapter implementation that executes only inside :mod:`simulation`."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import replace
from threading import Event, Lock
import time

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.kinematics import JointMotionPlan
from rosclaw_mini.simulation.world import SimulationMotionError, SimulationWorld


class SimulationMotionStoppedError(RuntimeError):
    """A virtual trajectory was interrupted before all waypoints were sent."""


WaypointHook = Callable[[int, JointMotionPlan], None]


class SimulatedArmAdapter(ArmAdapter):
    """Headless SO-100 adapter with the normal ArmAdapter surface.

    It never imports a robot SDK.  Its waypoint trace is intentionally public
    for experiment metrics and tests, not as a real-hardware telemetry API.
    """

    def __init__(
        self,
        world: SimulationWorld,
        *,
        step_delay_seconds: float = 0.0,
        waypoint_hook: WaypointHook | None = None,
    ) -> None:
        if step_delay_seconds < 0.0:
            raise ValueError("step_delay_seconds 不能为负数。")
        self.world = world
        self.step_delay_seconds = float(step_delay_seconds)
        self.waypoint_hook = waypoint_hook
        self._connected = False
        self._torque_enabled = False
        self._lock = Lock()
        self._is_moving = False
        self._stop_event = Event()
        self._motion_waypoint_written = False
        self.last_motion_plan: JointMotionPlan | None = None
        self.executed_waypoints: list[tuple[float, ...]] = []

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_moving(self) -> bool:
        with self._lock:
            return self._is_moving

    @property
    def motion_waypoint_written(self) -> bool:
        with self._lock:
            return self._motion_waypoint_written

    @property
    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def connect(self) -> None:
        self._connected = True
        self._torque_enabled = True

    def disconnect(self) -> None:
        if self.is_moving:
            raise SimulationMotionError("仿真后台动作仍在运行，拒绝提前 disconnect。")
        self._connected = False
        self._torque_enabled = False

    def begin_motion_action(self) -> None:
        with self._lock:
            if self._is_moving:
                raise SimulationMotionError("已有仿真动作正在执行。")
            # Stop only belongs to an already registered action.  Clearing it
            # while idle cannot erase a stop that arrived after registration.
            self._stop_event.clear()
            self._is_moving = True
            self._motion_waypoint_written = False

    def end_motion_action(self) -> None:
        with self._lock:
            self._is_moving = False

    def _require_ready(self) -> None:
        if not self._connected:
            raise SimulationMotionError("仿真机械臂尚未连接。")
        if not self._torque_enabled:
            raise SimulationMotionError("仿真机械臂力矩已关闭。")

    def _raise_if_stopped(self) -> None:
        if self._stop_event.is_set():
            raise SimulationMotionStoppedError("仿真轨迹被 stop 中断。")

    def plan_move_to(self, x: float, y: float, z: float) -> JointMotionPlan:
        self._require_ready()
        self._raise_if_stopped()
        return self.world.plan_tcp_motion((x, y, z))

    def materialize_joint_plan(
        self,
        plan: JointMotionPlan,
        *,
        held_gripper_driver_degrees: float | None = None,
    ) -> JointMotionPlan:
        """Freeze final points once; execution never interpolates again."""

        if held_gripper_driver_degrees is None:
            held_gripper_driver_degrees = self.world.gripper_driver_degrees
        if plan.is_final_execution_plan:
            return plan
        return replace(
            plan,
            is_final_execution_plan=True,
            waypoint_interval_seconds=1.0 / 30.0,
            held_gripper_driver_degrees=float(held_gripper_driver_degrees),
        )

    def execute_joint_plan(self, plan: JointMotionPlan) -> None:
        self._require_ready()
        if not plan.is_final_execution_plan:
            raise SimulationMotionError("仿真执行拒绝未固化的 JointMotionPlan。")
        self._raise_if_stopped()
        self.world.validate_plan(plan)
        self.last_motion_plan = plan
        for index, waypoint in enumerate(plan.waypoints_radians):
            self._raise_if_stopped()
            if self.waypoint_hook is not None:
                self.waypoint_hook(index, plan)
            self._raise_if_stopped()
            self.world.apply_joint_waypoint(waypoint)
            with self._lock:
                self._motion_waypoint_written = True
            self.executed_waypoints.append(tuple(waypoint))
            if self.step_delay_seconds:
                self._stop_event.wait(self.step_delay_seconds)
        self._raise_if_stopped()
        self.world.record_completed_plan(plan)

    def move_to(self, x: float, y: float, z: float) -> None:
        self.begin_motion_action()
        try:
            self._raise_if_stopped()
            plan = self.plan_move_to(x, y, z)
            self.execute_joint_plan(plan)
        finally:
            self.end_motion_action()

    def move_joints(self, joint_radians: Sequence[float]) -> None:
        self._require_ready()
        current = self.world.joint_radians
        target = tuple(float(value) for value in joint_radians)
        if len(target) != 6:
            raise SimulationMotionError("仿真关节目标必须有六个值。")
        limits = self.world.build_motion_limits()
        raw_plan = self.world.kinematics.plan_joint_pose(current, target, limits)
        plan = self.materialize_joint_plan(raw_plan)
        self.begin_motion_action()
        try:
            self.execute_joint_plan(plan)
        finally:
            self.end_motion_action()

    def read_tcp_position(self) -> tuple[float, float, float]:
        self._require_ready()
        return self.world.tcp_position_m

    def read_joint_radians(self) -> tuple[float, ...]:
        self._require_ready()
        return self.world.joint_radians

    def open_gripper(self) -> None:
        self._require_ready()
        self._raise_if_stopped()
        self.world.open_gripper()

    def close_gripper(self) -> None:
        self._require_ready()
        self._raise_if_stopped()
        self.world.close_gripper()

    def stop(self) -> None:
        with self._lock:
            if self._is_moving:
                self._stop_event.set()

    def disable_torque(self, *, emergency: bool = False) -> None:
        del emergency
        self._torque_enabled = False

    def sleep_for_visualization(self, seconds: float) -> None:
        """Optional helper used by a local human-facing demo, never by tests."""

        if seconds > 0:
            time.sleep(seconds)
