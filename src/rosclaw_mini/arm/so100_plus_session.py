"""SO-100 Plus 已认证收纳/工作会话的状态和编排。

本模块只组合现有 Adapter 原子动作，并读取调用方提供的 follower 反馈。
它不创建设备、不连接串口，也不替代 Adapter、运动学或 Safety Checker。
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
import math
from threading import RLock
from typing import Protocol

from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus import (
    GRIPPER_MOTOR_NAME,
    SO100_PLUS_REAL_HARDWARE_PROFILE,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
    StorageTransitionDirection,
    VerifiedJointMotionSequence,
)
from rosclaw_mini.command_schema.commands import Command, ExecutionResult
from rosclaw_mini.safety.limits import (
    AxisLimits,
    LimitViolationError,
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    WorkspaceLimits,
    build_so100_plus_right_follower_execution_joint_limits,
    resolve_relative_tcp_target,
)


SO100_PLUS_WORK_INITIAL_MAX_JOINT_ERROR_DEGREES = 5.0
SO100_PLUS_WORK_INITIAL_MAX_TCP_ERROR_M = 0.03
SO100_PLUS_STORAGE_ESCAPE_FRACTION = 0.20
SO100_PLUS_TRANSITION_SIMULATION_STEP_RADIANS = math.radians(1.0)
SO100_PLUS_TRANSITION_EXECUTION_STEP_RADIANS = math.radians(2.0)
SO100_PLUS_TRANSITION_WORKSPACE_MARGIN_M = 0.005


class ArmSessionState(str, Enum):
    """同一进程内对机械臂当前可信姿态的唯一分类。"""

    REST = "REST"
    TRANSITION = "TRANSITION"
    WORK = "WORK"
    UNVERIFIED = "UNVERIFIED"


@dataclass(frozen=True)
class SO100PlusPoseSnapshot:
    """一次由真实 follower 反馈和 FK 得到的姿态快照。"""

    driver_degrees: tuple[float, ...]
    joint_radians: tuple[float, ...]
    tcp_position_m: tuple[float, float, float]
    gripper_driver_degrees: float | None = None
    torque_enabled: tuple[int, ...] = ()


@dataclass(frozen=True)
class SO100PlusStorageTransition:
    """原验收流程使用的 follower_rest → 20% 脱离点 → 工作初始姿态。"""

    storage_joint_radians: tuple[float, ...]
    escape_joint_radians: tuple[float, ...]
    work_joint_radians: tuple[float, ...]
    path_joint_radians: tuple[tuple[float, ...], ...]
    path_positions_m: tuple[tuple[float, float, float], ...]


class SO100PlusSessionAdapter(Protocol):
    """会话所需的现有 Adapter 原子操作子集。"""

    def move_to(self, x: float, y: float, z: float) -> None: ...

    def plan_move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> JointMotionPlan: ...

    def move_joints(self, joint_radians: Sequence[float]) -> None: ...

    def execute_joint_plan(self, plan: JointMotionPlan) -> None: ...

    def materialize_joint_plan(
        self,
        plan: JointMotionPlan,
        *,
        held_gripper_driver_degrees: float | None = None,
    ) -> JointMotionPlan: ...

    def begin_motion_action(self) -> None: ...

    def end_motion_action(self) -> None: ...

    def open_gripper(self) -> None: ...

    def close_gripper(self) -> None: ...

    def stop(self) -> None: ...


PoseReader = Callable[[], SO100PlusPoseSnapshot]


def _finite_tuple(
    values: Sequence[float],
    *,
    length: int,
    label: str,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label}需要 {length} 个有限数值。")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label}需要 {length} 个有限数值。") from error
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label}需要 {length} 个有限数值。")
    return result


def read_so100_plus_pose_snapshot(
    follower_bus,
    kinematics: SO100PlusKinematics,
    *,
    include_torque: bool = True,
) -> SO100PlusPoseSnapshot:
    """按关节名称读取位置，并计算当前模型关节角和 TCP。"""

    motor_names = tuple(follower_bus.motor_names)
    positions = tuple(
        float(value) for value in follower_bus.read("Present_Position")
    )
    if len(motor_names) != len(positions):
        raise RuntimeError("电机名称与位置数量不一致。")
    if len(set(motor_names)) != len(motor_names):
        raise RuntimeError("follower 返回了重复的电机名称。")

    position_by_name = dict(zip(motor_names, positions, strict=True))
    missing = tuple(
        name
        for name in (*SO100_PLUS_ARM_JOINT_NAMES, GRIPPER_MOTOR_NAME)
        if name not in position_by_name
    )
    if missing:
        raise RuntimeError(f"缺少手臂关节：{', '.join(missing)}。")

    driver_degrees = tuple(
        position_by_name[name] for name in SO100_PLUS_ARM_JOINT_NAMES
    )
    gripper_driver_degrees = position_by_name[GRIPPER_MOTOR_NAME]
    if not math.isfinite(gripper_driver_degrees):
        raise RuntimeError("夹爪位置反馈不是有限数值。")
    joint_radians = tuple(
        kinematics.driver_degrees_to_model_radians(driver_degrees)
    )
    torque_enabled = (
        tuple(int(value) for value in follower_bus.read("Torque_Enable"))
        if include_torque
        else ()
    )
    return SO100PlusPoseSnapshot(
        driver_degrees=driver_degrees,
        joint_radians=joint_radians,
        tcp_position_m=tuple(kinematics.forward_position(joint_radians)),
        gripper_driver_degrees=gripper_driver_degrees,
        torque_enabled=torque_enabled,
    )


def validate_storage_rest_start(
    snapshot: SO100PlusPoseSnapshot,
    *,
    require_torque_disabled: bool = True,
) -> float:
    """复用真机验收采用的 follower_rest 逐关节容差。"""

    if (
        require_torque_disabled
        and snapshot.torque_enabled
        and any(snapshot.torque_enabled)
    ):
        raise RuntimeError(
            f"只读预检发现力矩仍开启：{snapshot.torque_enabled}；"
            "已停止，未发送任何写命令。"
        )

    expected = SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
    tolerances = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_tolerances_degrees
    )
    try:
        joint_errors = tuple(
            abs(actual - target)
            for actual, target in zip(
                snapshot.driver_degrees,
                expected,
                strict=True,
            )
        )
    except ValueError as error:
        raise RuntimeError("follower_rest 反馈关节数量不一致。") from error

    violating_indices = tuple(
        index
        for index, (error, tolerance) in enumerate(
            zip(joint_errors, tolerances, strict=True)
        )
        if error > tolerance
    )
    if violating_indices:
        violating_index = max(
            violating_indices,
            key=joint_errors.__getitem__,
        )
        raise RuntimeError(
            "当前位置不是本机已测的 follower_rest："
            f"{SO100_PLUS_ARM_JOINT_NAMES[violating_index]} 偏差 "
            f"{joint_errors[violating_index]:.3f}° 超过 "
            f"{tolerances[violating_index]:.1f}°。"
        )
    return max(joint_errors)


def validate_work_initial_pose(
    snapshot: SO100PlusPoseSnapshot,
    expected_tcp_position_m: Sequence[float],
    *,
    expected_joint_radians: Sequence[float] = (
        SO100_PLUS_JOYCON_INITIAL_RADIANS
    ),
    max_joint_error_degrees: float = (
        SO100_PLUS_WORK_INITIAL_MAX_JOINT_ERROR_DEGREES
    ),
    max_tcp_error_m: float = SO100_PLUS_WORK_INITIAL_MAX_TCP_ERROR_M,
) -> tuple[float, float]:
    """复用原真机验收的 JoyCon 初始 TCP 和真实关节角门槛。

    关节角直接比较 ``abs(actual - expected)``，不会把相差 2π 的
    物理反馈折叠成同一姿态。
    """

    expected_tcp = _finite_tuple(
        expected_tcp_position_m,
        length=3,
        label="JoyCon 初始 TCP",
    )
    expected_joints = _finite_tuple(
        expected_joint_radians,
        length=len(SO100_PLUS_ARM_JOINT_NAMES),
        label="JoyCon 初始关节姿态",
    )
    try:
        joint_errors = tuple(
            abs(math.degrees(actual - expected))
            for actual, expected in zip(
                snapshot.joint_radians,
                expected_joints,
                strict=True,
            )
        )
    except ValueError as error:
        raise RuntimeError("当前关节数与已认证工作初始姿态不一致。") from error

    max_joint_error = max(joint_errors)
    tcp_error = math.dist(snapshot.tcp_position_m, expected_tcp)
    if max_joint_error > max_joint_error_degrees:
        index = joint_errors.index(max_joint_error)
        raise RuntimeError(
            "没有到达 JoyCon 初始工作姿态："
            f"{SO100_PLUS_ARM_JOINT_NAMES[index]} 偏差 "
            f"{max_joint_error:.3f}° 超过 {max_joint_error_degrees:.1f}°。"
        )
    if tcp_error > max_tcp_error_m:
        raise RuntimeError(
            "当前 TCP 与仿真初始工作姿态相差 "
            f"{tcp_error * 100:.3f} cm，超过 "
            f"{max_tcp_error_m * 100:.1f} cm。"
        )
    return max_joint_error, tcp_error


def classify_so100_plus_startup_pose(
    snapshot: SO100PlusPoseSnapshot,
    expected_work_tcp_position_m: Sequence[float],
) -> tuple[ArmSessionState, str]:
    """按 REST → WORK → UNVERIFIED 的优先级分类启动反馈。"""

    try:
        validate_storage_rest_start(
            snapshot,
            require_torque_disabled=False,
        )
    except RuntimeError as rest_error:
        try:
            validate_work_initial_pose(
                snapshot,
                expected_work_tcp_position_m,
            )
        except RuntimeError as work_error:
            return (
                ArmSessionState.UNVERIFIED,
                "当前姿态既不能认证为 follower_rest，也不能认证为 "
                f"JoyCon 工作初始姿态。REST 检查：{rest_error}"
                f"；WORK 检查：{work_error}",
            )
        return (
            ArmSessionState.WORK,
            "启动反馈符合 JoyCon 工作初始姿态门槛。",
        )
    return (
        ArmSessionState.REST,
        "启动反馈符合 follower_rest 逐关节容差。",
    )


def build_so100_plus_storage_transition(
    storage_joint_radians: Sequence[float],
    kinematics: SO100PlusKinematics,
) -> SO100PlusStorageTransition:
    """提取原验收脚本采用的直线关节路径和 20% storage_escape。"""

    storage = _finite_tuple(
        storage_joint_radians,
        length=len(SO100_PLUS_ARM_JOINT_NAMES),
        label="收纳姿态关节角",
    )
    work = tuple(float(value) for value in SO100_PLUS_JOYCON_INITIAL_RADIANS)
    delta = tuple(
        target - start
        for start, target in zip(storage, work, strict=True)
    )
    max_change = max(abs(value) for value in delta)
    step_count = max(
        1,
        math.ceil(
            max_change / SO100_PLUS_TRANSITION_SIMULATION_STEP_RADIANS
        ),
    )
    path = tuple(
        tuple(
            start + change * (step_index / step_count)
            for start, change in zip(storage, delta, strict=True)
        )
        for step_index in range(step_count + 1)
    )
    positions = tuple(
        tuple(kinematics.forward_position(joints)) for joints in path
    )
    if any(
        current[2] < previous[2] - 1e-9
        for previous, current in zip(positions, positions[1:])
    ):
        raise RuntimeError("收纳姿态展开路径的 TCP 不是单调上升。")

    escape = tuple(
        start + SO100_PLUS_STORAGE_ESCAPE_FRACTION * change
        for start, change in zip(storage, delta, strict=True)
    )
    return SO100PlusStorageTransition(
        storage_joint_radians=storage,
        escape_joint_radians=escape,
        work_joint_radians=work,
        path_joint_radians=path,
        path_positions_m=positions,
    )


def build_so100_plus_transition_motion_limits(
    transition: SO100PlusStorageTransition,
) -> MotionLimits:
    """为固定展开/收纳路径构造内部限制，不改变正式 move_arm 工作框。"""

    xs = tuple(position[0] for position in transition.path_positions_m)
    ys = tuple(position[1] for position in transition.path_positions_m)
    zs = tuple(position[2] for position in transition.path_positions_m)
    margin = SO100_PLUS_TRANSITION_WORKSPACE_MARGIN_M
    workspace = WorkspaceLimits(
        x=AxisLimits(min(xs) - margin, max(xs) + margin),
        y=AxisLimits(min(ys) - margin, max(ys) + margin),
        z=AxisLimits(min(zs) - margin, max(zs) + margin),
    )
    return MotionLimits(
        workspace=workspace,
        joints=build_so100_plus_right_follower_execution_joint_limits(
            transition.storage_joint_radians,
            max_step_radians=(
                SO100_PLUS_TRANSITION_EXECUTION_STEP_RADIANS
            ),
        ),
    )


class SO100PlusArmSession:
    """持有唯一会话状态，并编排工作 Adapter 与过渡 Adapter。"""

    def __init__(
        self,
        *,
        work_adapter: SO100PlusSessionAdapter,
        transition_adapter: SO100PlusSessionAdapter,
        pose_reader: PoseReader,
        kinematics: SO100PlusKinematics,
        initial_snapshot: SO100PlusPoseSnapshot,
        storage_joint_radians: Sequence[float],
        transition_motion_limits: MotionLimits,
        trajectory_validator: SO100PlusMuJoCoTrajectoryValidator,
    ) -> None:
        self._work_adapter = work_adapter
        self._transition_adapter = transition_adapter
        self._pose_reader = pose_reader
        self._kinematics = kinematics
        self._transition_motion_limits = transition_motion_limits
        self._trajectory_validator = trajectory_validator
        self._work_tcp_position_m = tuple(
            kinematics.forward_position(
                SO100_PLUS_JOYCON_INITIAL_RADIANS
            )
        )
        self._transition = build_so100_plus_storage_transition(
            storage_joint_radians,
            kinematics,
        )
        self._lock = RLock()
        self._active_adapter: SO100PlusSessionAdapter | None = None
        self._active_transition: str | None = None
        self._prepared_command_id: str | None = None
        self._transition_interrupted = False
        self._state, self._state_reason = classify_so100_plus_startup_pose(
            initial_snapshot,
            self._work_tcp_position_m,
        )

    @property
    def state(self) -> ArmSessionState:
        with self._lock:
            return self._state

    @property
    def state_reason(self) -> str:
        with self._lock:
            return self._state_reason

    @property
    def work_tcp_position_m(self) -> tuple[float, float, float]:
        return self._work_tcp_position_m

    @property
    def transition(self) -> SO100PlusStorageTransition:
        return self._transition

    def _set_state(self, state: ArmSessionState, reason: str) -> None:
        with self._lock:
            self._state = state
            self._state_reason = reason

    def _failure(self, command: Command, message: str) -> ExecutionResult:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=False,
            message=message,
        )

    def _success(self, command: Command, message: str) -> ExecutionResult:
        return ExecutionResult(
            command_id=command.command_id,
            skill_name=command.skill_name,
            success=True,
            message=message,
        )

    def _require_state(
        self,
        command: Command,
        allowed: ArmSessionState,
    ) -> ExecutionResult | None:
        with self._lock:
            current = self._state
            interrupted = (
                self._transition_interrupted
                and self._prepared_command_id == command.command_id
            )
        if current is allowed:
            return None
        if interrupted:
            return self._failure(
                command,
                f"{command.skill_name} 在执行首条运动指令前"
                "已被 stop 中断；当前状态为 UNVERIFIED。",
            )
        return self._failure(
            command,
            f"{command.skill_name} 只允许在 {allowed.value} 状态执行；"
            f"当前状态为 {current.value}。",
        )

    def _activate(
        self,
        adapter: SO100PlusSessionAdapter,
        *,
        transition_name: str | None = None,
        command: Command | None = None,
    ) -> None:
        with self._lock:
            if self._active_adapter is not None:
                if (
                    self._active_adapter is adapter
                    and command is not None
                    and self._prepared_command_id == command.command_id
                ):
                    return
                raise RuntimeError("会话中已有注册的机械臂动作。")
            adapter.begin_motion_action()
            self._active_adapter = adapter
            self._active_transition = transition_name
            self._prepared_command_id = (
                command.command_id if command is not None else None
            )
            if transition_name is not None:
                self._transition_interrupted = False

    def _deactivate(self, adapter: SO100PlusSessionAdapter) -> None:
        should_end = False
        with self._lock:
            if self._active_adapter is adapter:
                self._active_adapter = None
                self._active_transition = None
                self._prepared_command_id = None
                should_end = True
        if should_end:
            adapter.end_motion_action()

    def prepare_command(self, command: Command) -> None:
        """Controller 在标记提交前注册动作的 stop 世代。"""

        if command.skill_name in {
            "move_arm",
            "move_relative",
            "open_gripper",
            "close_gripper",
        }:
            if self.state is ArmSessionState.WORK:
                self._activate(self._work_adapter, command=command)
            return
        if command.skill_name == "unfold_arm":
            if self.state is ArmSessionState.REST:
                self._activate(
                    self._transition_adapter,
                    transition_name="unfold_arm",
                    command=command,
                )
            return
        if command.skill_name == "fold_arm" and self.state is ArmSessionState.WORK:
            self._activate(
                self._transition_adapter,
                transition_name="fold_arm",
                command=command,
            )

    def finish_command(self, command: Command) -> None:
        """无论 Gateway 是否进入 Skill，都结束预注册的动作。"""

        with self._lock:
            adapter = (
                self._active_adapter
                if self._prepared_command_id == command.command_id
                else None
            )
        if adapter is not None:
            self._deactivate(adapter)

    def _raise_if_transition_interrupted(self) -> None:
        with self._lock:
            if self._transition_interrupted:
                raise RuntimeError("动作已被 stop 中断")

    def _finish_transition(
        self,
        state: ArmSessionState,
        reason: str,
    ) -> None:
        with self._lock:
            if self._transition_interrupted:
                raise RuntimeError("动作已被 stop 中断")
            self._state = state
            self._state_reason = reason

    def _transition_was_interrupted(self) -> bool:
        with self._lock:
            return self._transition_interrupted

    def _snapshot_gripper_qpos(
        self,
        snapshot: SO100PlusPoseSnapshot,
    ) -> tuple[float, float]:
        driver_degrees = snapshot.gripper_driver_degrees
        if driver_degrees is None:
            raise RuntimeError("当前姿态快照缺少 gripper_joint 实测反馈。")
        if not math.isfinite(driver_degrees):
            raise RuntimeError("gripper_joint 实测反馈不是有限数值。")
        qpos = self._trajectory_validator.gripper_driver_degrees_to_qpos(
            driver_degrees
        )
        return float(driver_degrees), float(qpos)

    @staticmethod
    def _validate_gripper_held(
        snapshot: SO100PlusPoseSnapshot,
        expected_driver_degrees: float,
    ) -> None:
        actual = snapshot.gripper_driver_degrees
        if actual is None or not math.isfinite(actual):
            raise RuntimeError("转换后无法读取有限的 gripper_joint 反馈。")
        tolerance = (
            SO100_PLUS_REAL_HARDWARE_PROFILE
            .gripper_position_tolerance_degrees
        )
        if abs(actual - expected_driver_degrees) > tolerance:
            raise RuntimeError(
                f"转换期间夹爪未保持预检姿态：预检 "
                f"{expected_driver_degrees:.6f}°，实测 {actual:.6f}°，"
                f"超过 {tolerance:.1f}° 容差。"
            )

    @staticmethod
    def _sequence_has_motion(
        sequence: VerifiedJointMotionSequence,
    ) -> bool:
        return any(plan.waypoints_radians for plan in sequence.plans)

    def _execute_verified_sequence(
        self,
        sequence: VerifiedJointMotionSequence,
    ) -> None:
        for plan in sequence.plans:
            self._raise_if_transition_interrupted()
            self._transition_adapter.execute_joint_plan(plan)

    def _plan_storage_sequence(
        self,
        start_joint_radians: Sequence[float],
        transition: SO100PlusStorageTransition,
        *,
        direction: StorageTransitionDirection,
        held_gripper_driver_degrees: float,
    ) -> tuple[JointMotionPlan, JointMotionPlan]:
        if direction is StorageTransitionDirection.UNFOLD:
            first_target = transition.escape_joint_radians
            second_target = transition.work_joint_radians
        else:
            first_target = transition.escape_joint_radians
            second_target = transition.storage_joint_radians

        first = self._kinematics.plan_joint_pose(
            current_joint_radians=start_joint_radians,
            target_joint_radians=first_target,
            limits=self._transition_motion_limits,
        )
        second = self._kinematics.plan_joint_pose(
            current_joint_radians=first.target_joint_radians,
            target_joint_radians=second_target,
            limits=self._transition_motion_limits,
        )
        return (
            self._transition_adapter.materialize_joint_plan(
                first,
                held_gripper_driver_degrees=(
                    held_gripper_driver_degrees
                ),
            ),
            self._transition_adapter.materialize_joint_plan(
                second,
                held_gripper_driver_degrees=(
                    held_gripper_driver_degrees
                ),
            ),
        )

    def _validate_fold_start(
        self,
        snapshot: SO100PlusPoseSnapshot,
    ) -> None:
        try:
            SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS.validate_position(
                *snapshot.tcp_position_m
            )
        except LimitViolationError:
            # JoyCon 工作初始点位于正式工作框 X 下限外约 1 cm，只能
            # 作为 unfold/fold 的固定转换端点。
            validate_work_initial_pose(
                snapshot,
                self._work_tcp_position_m,
            )

    def _execute_work_target(
        self,
        target_position_m: Sequence[float],
        current_snapshot: SO100PlusPoseSnapshot,
    ) -> tuple[float, float, float]:
        """规划、固化、MuJoCo 验证并执行同一个正式工作区计划。"""

        target = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS.validate_position(
            *target_position_m
        )
        gripper_degrees, gripper_qpos = self._snapshot_gripper_qpos(
            current_snapshot
        )
        try:
            plan = self._work_adapter.plan_move_to(*target)
        except Exception as error:
            raise RuntimeError(f"工作区绝对目标规划失败：{error}") from error
        try:
            final_plan = self._work_adapter.materialize_joint_plan(
                plan,
                held_gripper_driver_degrees=gripper_degrees,
            )
            verified = (
                self._trajectory_validator.verify_collision_free_sequence(
                    (final_plan,),
                    self._kinematics,
                    gripper_qpos=gripper_qpos,
                )
            )
        except Exception as error:
            raise RuntimeError(
                f"工作区完整轨迹 MuJoCo 预检查失败：{error}"
            ) from error
        try:
            for verified_plan in verified.plans:
                self._work_adapter.execute_joint_plan(verified_plan)
        except Exception as error:
            raise RuntimeError(f"工作区已验证轨迹执行失败：{error}") from error
        return target

    def move_arm(self, command: Command) -> ExecutionResult:
        rejected = self._require_state(command, ArmSessionState.WORK)
        if rejected is not None:
            return rejected
        self._activate(self._work_adapter, command=command)
        try:
            current_snapshot = self._pose_reader()
            self._execute_work_target(
                (
                    command.params["x"],
                    command.params["y"],
                    command.params["z"],
                ),
                current_snapshot,
            )
        except Exception as error:
            return self._failure(
                command,
                f"move_arm 工作区运动阶段失败：{error}",
            )
        finally:
            self._deactivate(self._work_adapter)
        return self._success(command, "夹爪 TCP 已完成工作区移动")

    def move_relative(self, command: Command) -> ExecutionResult:
        rejected = self._require_state(command, ArmSessionState.WORK)
        if rejected is not None:
            return rejected
        self._activate(self._work_adapter, command=command)
        try:
            current_snapshot = self._pose_reader()
            displacement = (
                command.params["dx"],
                command.params["dy"],
                command.params["dz"],
            )
            target = resolve_relative_tcp_target(
                current_snapshot.tcp_position_m,
                displacement,
                SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
            )
            self._execute_work_target(target, current_snapshot)
        except Exception as error:
            return self._failure(
                command,
                f"move_relative 工作区运动阶段失败：{error}",
            )
        finally:
            self._deactivate(self._work_adapter)
        return self._success(
            command,
            (
                f"夹爪 TCP 已从 {current_snapshot.tcp_position_m} m 相对移动 "
                f"dx/dy/dz={displacement} m，最终位置 {target} m"
            ),
        )

    def open_gripper(self, command: Command) -> ExecutionResult:
        rejected = self._require_state(command, ArmSessionState.WORK)
        if rejected is not None:
            return rejected
        try:
            self._activate(self._work_adapter, command=command)
            self._work_adapter.open_gripper()
        except Exception as error:
            return self._failure(command, f"open_gripper 执行失败：{error}")
        finally:
            self._deactivate(self._work_adapter)
        return self._success(command, "机械臂夹爪已打开")

    def close_gripper(self, command: Command) -> ExecutionResult:
        rejected = self._require_state(command, ArmSessionState.WORK)
        if rejected is not None:
            return rejected
        try:
            self._activate(self._work_adapter, command=command)
            self._work_adapter.close_gripper()
        except Exception as error:
            return self._failure(command, f"close_gripper 执行失败：{error}")
        finally:
            self._deactivate(self._work_adapter)
        return self._success(command, "机械臂夹爪已关闭")

    def unfold_arm(self, command: Command) -> ExecutionResult:
        rejected = self._require_state(command, ArmSessionState.REST)
        if rejected is not None:
            return rejected

        stage = "展开前 follower_rest 复核"
        motion_started = False
        self._activate(
            self._transition_adapter,
            transition_name="unfold_arm",
            command=command,
        )
        try:
            current_snapshot = self._pose_reader()
            validate_storage_rest_start(
                current_snapshot,
                require_torque_disabled=False,
            )
            gripper_degrees, gripper_qpos = self._snapshot_gripper_qpos(
                current_snapshot
            )
            self._raise_if_transition_interrupted()
            stage = "按当次 follower_rest 规划完整展开轨迹"
            transition = build_so100_plus_storage_transition(
                current_snapshot.joint_radians,
                self._kinematics,
            )
            plans = self._plan_storage_sequence(
                current_snapshot.joint_radians,
                transition,
                direction=StorageTransitionDirection.UNFOLD,
                held_gripper_driver_degrees=gripper_degrees,
            )
            stage = "完整展开轨迹 MuJoCo 碰撞和接触预检查"
            verified = self._trajectory_validator.verify_storage_transition(
                plans,
                escape_joint_radians=transition.escape_joint_radians,
                kinematics=self._kinematics,
                direction=StorageTransitionDirection.UNFOLD,
                gripper_qpos=gripper_qpos,
            )
            self._raise_if_transition_interrupted()
            self._transition = transition
            self._set_state(
                ArmSessionState.TRANSITION,
                "正在沿已验收路径展开机械臂。",
            )
            motion_started = self._sequence_has_motion(verified)
            stage = "执行已验证的完整展开轨迹"
            self._execute_verified_sequence(verified)
            self._raise_if_transition_interrupted()
            stage = "展开完成后的 TCP 和关节门禁"
            final_snapshot = self._pose_reader()
            validate_work_initial_pose(
                final_snapshot,
                self._work_tcp_position_m,
            )
            self._validate_gripper_held(final_snapshot, gripper_degrees)
            self._finish_transition(
                ArmSessionState.WORK,
                "已到达并认证 JoyCon 工作初始姿态。",
            )
        except Exception as error:
            message = f"unfold_arm 在“{stage}”失败：{error}"
            if motion_started or self._transition_was_interrupted():
                self._set_state(ArmSessionState.UNVERIFIED, message)
            return self._failure(command, message)
        finally:
            self._deactivate(self._transition_adapter)

        return self._success(
            command,
            "展开完成；会话状态已变为 WORK，机械臂停留在工作初始姿态。",
        )

    def fold_arm(self, command: Command) -> ExecutionResult:
        rejected = self._require_state(command, ArmSessionState.WORK)
        if rejected is not None:
            return rejected

        stage = "收纳前读取并认证当前 WORK 起点"
        motion_started = False
        self._activate(
            self._transition_adapter,
            transition_name="fold_arm",
            command=command,
        )
        try:
            current_snapshot = self._pose_reader()
            self._validate_fold_start(current_snapshot)
            gripper_degrees, gripper_qpos = self._snapshot_gripper_qpos(
                current_snapshot
            )
            self._raise_if_transition_interrupted()
            stage = "规划当前位置到 JoyCon 工作初始姿态的完整轨迹"
            return_plan = self._kinematics.plan_position(
                current_joint_radians=current_snapshot.joint_radians,
                target_position_m=self._work_tcp_position_m,
                limits=self._transition_motion_limits,
            )
            return_plan = self._transition_adapter.materialize_joint_plan(
                return_plan,
                held_gripper_driver_degrees=gripper_degrees,
            )
            stage = "返回工作初始姿态轨迹 MuJoCo 碰撞预检查"
            verified_return = (
                self._trajectory_validator.verify_collision_free_sequence(
                    (return_plan,),
                    self._kinematics,
                    gripper_qpos=gripper_qpos,
                )
            )
            self._raise_if_transition_interrupted()
            if self._sequence_has_motion(verified_return):
                self._set_state(
                    ArmSessionState.TRANSITION,
                    "正在返回 JoyCon 工作初始姿态。",
                )
                motion_started = True
                stage = "执行已验证的工作初始姿态返回轨迹"
                self._execute_verified_sequence(verified_return)
                self._raise_if_transition_interrupted()

            stage = "返回后的工作初始姿态门禁"
            work_snapshot = self._pose_reader()
            validate_work_initial_pose(
                work_snapshot,
                self._work_tcp_position_m,
            )
            self._validate_gripper_held(work_snapshot, gripper_degrees)
            self._raise_if_transition_interrupted()
            stage = "规划 JoyCon 初始姿态到 follower_rest 的完整反向轨迹"
            storage_plans = self._plan_storage_sequence(
                work_snapshot.joint_radians,
                self._transition,
                direction=StorageTransitionDirection.FOLD,
                held_gripper_driver_degrees=gripper_degrees,
            )
            stage = "完整反向收纳轨迹 MuJoCo 碰撞和接触预检查"
            verified_storage = (
                self._trajectory_validator.verify_storage_transition(
                    storage_plans,
                    escape_joint_radians=(
                        self._transition.escape_joint_radians
                    ),
                    kinematics=self._kinematics,
                    direction=StorageTransitionDirection.FOLD,
                    gripper_qpos=gripper_qpos,
                )
            )
            self._raise_if_transition_interrupted()
            if not motion_started:
                self._set_state(
                    ArmSessionState.TRANSITION,
                    "正在沿已验收反向路径收纳机械臂。",
                )
            motion_started = (
                motion_started
                or self._sequence_has_motion(verified_storage)
            )
            stage = "执行已验证的完整反向收纳轨迹"
            self._execute_verified_sequence(verified_storage)
            self._raise_if_transition_interrupted()
            stage = "收纳完成后的 follower_rest 门禁"
            final_snapshot = self._pose_reader()
            validate_storage_rest_start(
                final_snapshot,
                require_torque_disabled=False,
            )
            self._validate_gripper_held(final_snapshot, gripper_degrees)
            self._finish_transition(
                ArmSessionState.REST,
                "收纳完成，实际反馈符合 follower_rest 逐关节容差。",
            )
        except Exception as error:
            message = f"fold_arm 在“{stage}”失败：{error}"
            if motion_started or self._transition_was_interrupted():
                self._set_state(ArmSessionState.UNVERIFIED, message)
            return self._failure(command, message)
        finally:
            self._deactivate(self._transition_adapter)

        return self._success(
            command,
            "收纳完成；会话状态已变为 REST。",
        )

    def request_stop(self) -> None:
        """停止当前原子动作；中断会话转换时立即失去姿态认证。"""

        with self._lock:
            adapter = self._active_adapter
            transition_name = self._active_transition
            if adapter is None:
                adapter = (
                    self._transition_adapter
                    if self._state is ArmSessionState.REST
                    else self._work_adapter
                )
            if (
                transition_name is not None
                or self._state is ArmSessionState.TRANSITION
            ):
                self._transition_interrupted = True
                self._state = ArmSessionState.UNVERIFIED
                self._state_reason = (
                    f"{transition_name or '状态转换'} 被 stop 中断；"
                    "当前姿态不再可信。"
                )
        adapter.stop()

    def stop(self, command: Command) -> ExecutionResult:
        try:
            self.request_stop()
        except Exception as error:
            return self._failure(command, f"stop 执行失败：{error}")
        return self._success(
            command,
            "机械臂已请求停止；被中断的展开或收纳状态已标记为 UNVERIFIED。",
        )
