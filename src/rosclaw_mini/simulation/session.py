"""Simulation-only counterpart of the existing REST/WORK session gate."""

from __future__ import annotations

from dataclasses import replace

from rosclaw_mini.arm.so100_plus_session import ArmSessionState
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.simulation.adapter import SimulatedArmAdapter
from rosclaw_mini.simulation.world import SimulationMotionError
from rosclaw_mini.skills.arm_handler import ArmHandlers
from rosclaw_mini.skills.arm_skills import build_arm_skills
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.workspace_scan.irregular_workspace import SO100PlusIrregularWorkspace


class SimulatedArmSession:
    """Use the real project state names without representing a real arm pose."""

    def __init__(
        self,
        adapter: SimulatedArmAdapter,
        workspace: SO100PlusIrregularWorkspace,
        *,
        start_state: ArmSessionState = ArmSessionState.WORK,
    ) -> None:
        self.adapter = adapter
        self.workspace = workspace
        self._handlers = ArmHandlers(adapter, workspace_limits=workspace.endpoint_aabb)
        self._state = start_state
        self._state_reason = "仿真启动状态；不代表真机姿态认证。"
        if start_state is ArmSessionState.REST:
            adapter.world.reset_to_rest()
        elif start_state is ArmSessionState.WORK:
            adapter.world.reset_to_work()

    @property
    def state(self) -> ArmSessionState:
        return self._state

    @property
    def state_reason(self) -> str:
        return self._state_reason

    @property
    def work_workspace_aabb(self):
        return self.workspace.endpoint_aabb

    def prepare_command(self, command: Command) -> None:
        del command

    def finish_command(self, command: Command) -> None:
        del command

    def _failure(self, command: Command, message: str) -> ExecutionResult:
        return ExecutionResult(command.command_id, command.skill_name, False, message)

    def _success(self, command: Command, message: str) -> ExecutionResult:
        return ExecutionResult(command.command_id, command.skill_name, True, message)

    def _require_work(self, command: Command) -> ExecutionResult | None:
        if self._state is not ArmSessionState.WORK:
            return self._failure(command, f"{command.skill_name} 只允许在 WORK 状态执行；当前为 {self._state.value}。")
        return None

    def move_arm(self, command: Command) -> ExecutionResult:
        rejected = self._require_work(command)
        if rejected is not None:
            return rejected
        try:
            self.workspace.validate_position(command.params["x"], command.params["y"], command.params["z"])
            return self._handlers.move_arm(command)
        except Exception as error:
            return self._motion_failure(command, error)

    def move_relative(self, command: Command) -> ExecutionResult:
        rejected = self._require_work(command)
        if rejected is not None:
            return rejected
        try:
            current = self.adapter.read_tcp_position()
            displacement = (command.params["dx"], command.params["dy"], command.params["dz"])
            target = self.workspace.resolve_relative_target(current, displacement)
            self.adapter.move_to(*target)
        except Exception as error:
            return self._motion_failure(command, error)
        return self._success(command, f"仿真 TCP 已从 {current} m 移动至 {target} m。")

    def open_gripper(self, command: Command) -> ExecutionResult:
        rejected = self._require_work(command)
        if rejected is not None:
            return rejected
        try:
            return self._handlers.open_gripper(command)
        except Exception as error:
            return self._failure(command, f"仿真 open_gripper 失败：{error}")

    def close_gripper(self, command: Command) -> ExecutionResult:
        rejected = self._require_work(command)
        if rejected is not None:
            return rejected
        try:
            return self._handlers.close_gripper(command)
        except Exception as error:
            return self._failure(command, f"仿真 close_gripper 失败：{error}")

    def _motion_failure(self, command: Command, error: Exception) -> ExecutionResult:
        message = f"仿真 WORK 运动失败：{error}"
        if self.adapter.motion_waypoint_written:
            self._state = ArmSessionState.UNVERIFIED
            self._state_reason = message + "；已写入轨迹点，会话标记为 UNVERIFIED。"
            message = self._state_reason
        return self._failure(command, message)

    def unfold_arm(self, command: Command) -> ExecutionResult:
        if self._state is not ArmSessionState.REST:
            return self._failure(command, f"unfold_arm 只允许在 REST 状态执行；当前为 {self._state.value}。")
        self._state = ArmSessionState.TRANSITION
        try:
            # This is a logical state handoff in a virtual environment.  It is
            # deliberately documented as not re-validating the real storage path.
            self.adapter.world.reset_to_work()
        except Exception as error:
            self._state = ArmSessionState.UNVERIFIED
            self._state_reason = f"仿真展开失败：{error}"
            return self._failure(command, self._state_reason)
        self._state = ArmSessionState.WORK
        self._state_reason = "仿真会话已进入 WORK；不代表真机展开已验收。"
        return self._success(command, self._state_reason)

    def fold_arm(self, command: Command) -> ExecutionResult:
        if self._state is not ArmSessionState.WORK:
            return self._failure(command, f"fold_arm 只允许在 WORK 状态执行；当前为 {self._state.value}。")
        self._state = ArmSessionState.TRANSITION
        try:
            self.adapter.world.reset_to_rest()
        except Exception as error:
            self._state = ArmSessionState.UNVERIFIED
            self._state_reason = f"仿真收纳失败：{error}"
            return self._failure(command, self._state_reason)
        self._state = ArmSessionState.REST
        self._state_reason = "仿真会话已进入 REST；不代表真机收纳已验收。"
        return self._success(command, self._state_reason)

    def revalidate_state(self, command: Command) -> ExecutionResult:
        if self._state is not ArmSessionState.UNVERIFIED:
            return self._failure(command, f"revalidate_state 只允许在 UNVERIFIED；当前为 {self._state.value}。")
        if self.adapter.world.session_pose == "REST":
            self._state = ArmSessionState.REST
        else:
            self._state = ArmSessionState.WORK
        self._state_reason = "仿真状态重新认证完成；不读取真实硬件。"
        return self._success(command, self._state_reason)

    def request_stop(self) -> None:
        self.adapter.stop()
        if self._state is ArmSessionState.TRANSITION:
            self._state = ArmSessionState.UNVERIFIED
            self._state_reason = "仿真状态转换被 stop 中断。"

    def stop(self, command: Command) -> ExecutionResult:
        self.request_stop()
        return self._success(command, "已向仿真 Adapter 请求停止。")


def build_simulation_arm_skills(
    adapter: SimulatedArmAdapter,
    session: SimulatedArmSession,
) -> dict[str, SkillDefinition]:
    """Reuse normal Skill schemas/Gateway, replacing only session handlers."""

    skills = build_arm_skills(adapter, workspace_limits=session.workspace.endpoint_aabb)
    skills = dict(skills)
    for name, handler in (
        ("move_arm", session.move_arm),
        ("move_relative", session.move_relative),
        ("open_gripper", session.open_gripper),
        ("close_gripper", session.close_gripper),
        ("stop", session.stop),
    ):
        skills[name] = replace(skills[name], handler=handler)
    skills["unfold_arm"] = SkillDefinition("unfold_arm", "仿真 REST → WORK 状态转换", "high", True, {}, session.unfold_arm)
    skills["fold_arm"] = SkillDefinition("fold_arm", "仿真 WORK → REST 状态转换", "high", True, {}, session.fold_arm)
    skills["revalidate_state"] = SkillDefinition("revalidate_state", "只读重新认证仿真状态，不产生运动", "low", True, {}, session.revalidate_state)
    return skills
