"""应用运行时装配：把 Adapter、Skills 和 Controller 连接起来。"""

from collections.abc import Callable
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.kinematics import SO100PlusKinematics
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.arm.so100_plus import (
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionConfig,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.execution.controller import ExecutionController
from rosclaw_mini.gateway.command.gateway import run_command
from rosclaw_mini.safety.limits import (
    AxisLimits,
    LimitViolationError,
    SO100_PLUS_ARM_JOINT_NAMES,
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    WorkspaceLimits,
    build_so100_plus_right_follower_motion_limits,
)
from rosclaw_mini.skills.arm_skills import (
    build_arm_skills,
    build_so100_plus_right_follower_arm_skills,
)
from rosclaw_mini.skills.base import SkillDefinition


DEFAULT_SO100_PLUS_PORT = Path("/dev/lerobot_right")
DEFAULT_SO100_PLUS_CALIBRATION_DIR = Path(
    "lerobot-joycon_plus/.cache/calibration/so100_plus"
)
DEFAULT_SO100_PLUS_FOLLOWER_NAME = "right"
DEFAULT_SO100_PLUS_GRIPPER_OPEN_DEGREES = 60.0
DEFAULT_SO100_PLUS_GRIPPER_CLOSE_DEGREES = -5.0

# 这里只属于 Mock 演示，不代表任何真实机械臂的物理工作空间。
DEFAULT_MOCK_WORKSPACE = WorkspaceLimits(
    x=AxisLimits(-1.0, 1.0),
    y=AxisLimits(-1.0, 1.0),
    z=AxisLimits(-1.0, 1.0),
)


@dataclass
class ArmRuntime:
    """一次应用运行所共享的 Adapter、Skills 和 Controller。"""

    adapter: ArmAdapter
    skills: dict[str, SkillDefinition]
    controller: ExecutionController
    current_tcp_position_m: tuple[float, float, float] | None = None
    move_arm_disabled_reason: str | None = None
    _is_shutdown: bool = field(default=False, init=False, repr=False)

    def shutdown(self) -> None:
        """先停止并等待后台动作结束，再断开；不会自动关闭力矩。"""

        if self._is_shutdown:
            return

        try:
            if self.adapter.is_connected:
                self.adapter.stop()
                if self.controller.is_running():
                    self.controller.wait()
        finally:
            try:
                if self.adapter.is_connected:
                    self.adapter.disconnect()
            finally:
                self._is_shutdown = True


def _build_controller(
    skills: dict[str, SkillDefinition],
) -> ExecutionController:
    def execute_command(command):
        return run_command(command, skills)

    return ExecutionController(execute_command)


def build_mock_runtime(
    *,
    move_duration_seconds: float = 5.0,
) -> ArmRuntime:
    """创建并连接默认 Mock 运行时；不会导入或访问真机驱动。"""

    adapter = MockArmAdapter(move_duration_seconds=move_duration_seconds)
    adapter.connect()
    skills = build_arm_skills(
        adapter,
        workspace_limits=DEFAULT_MOCK_WORKSPACE,
    )
    return ArmRuntime(
        adapter=adapter,
        skills=skills,
        controller=_build_controller(skills),
    )


def _read_current_arm_joint_radians(
    robot: Any,
    follower_name: str,
    kinematics: SO100PlusKinematics,
) -> tuple[float, ...]:
    """读取已连接 follower 的六关节位置，供正式 MotionLimits 装配。"""

    try:
        follower_bus = robot.follower_arms[follower_name]
    except (AttributeError, KeyError) as error:
        raise RuntimeError(
            f"真机 Robot 缺少 follower {follower_name!r}。"
        ) from error

    motor_names = tuple(follower_bus.motor_names)
    positions = tuple(float(value) for value in follower_bus.read("Present_Position"))
    if len(motor_names) != len(positions):
        raise RuntimeError("follower 返回的电机名称数量与当前位置数量不一致。")
    if len(set(motor_names)) != len(motor_names):
        raise RuntimeError("follower 返回了重复的电机名称。")

    position_by_name = dict(zip(motor_names, positions, strict=True))
    missing_names = tuple(
        name
        for name in SO100_PLUS_ARM_JOINT_NAMES
        if name not in position_by_name
    )
    if missing_names:
        raise RuntimeError(
            f"follower 缺少手臂关节：{', '.join(missing_names)}。"
        )

    driver_degrees = tuple(
        position_by_name[name]
        for name in SO100_PLUS_ARM_JOINT_NAMES
    )
    return tuple(
        kinematics.driver_degrees_to_model_radians(driver_degrees)
    )


def _cleanup_connected_adapter(adapter: ArmAdapter) -> None:
    """装配中途失败时尽力停止并断开，但不调用 disable_torque。"""

    try:
        if adapter.is_connected:
            adapter.stop()
    finally:
        if adapter.is_connected:
            adapter.disconnect()


def _apply_so100_plus_startup_workspace_gate(
    skills: dict[str, SkillDefinition],
    current_tcp_position_m: tuple[float, float, float],
) -> tuple[dict[str, SkillDefinition], str | None]:
    """当前 TCP 不在正式工作空间时，只失败关闭 move_arm Skill。"""

    move_arm_skill = skills.get("move_arm")
    if move_arm_skill is None:
        raise RuntimeError("right_follower Skill 注册表缺少 move_arm。")

    try:
        SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS.validate_position(
            *current_tcp_position_m
        )
    except LimitViolationError as error:
        gated_skills = dict(skills)
        gated_skills["move_arm"] = replace(
            move_arm_skill,
            enabled=False,
        )
        position = ", ".join(
            f"{value:.6f}" for value in current_tcp_position_m
        )
        reason = (
            f"move_arm 已失败关闭：启动 TCP ({position}) m 不在当前 "
            f"right_follower 正式工作空间内（{error}）。"
            "请退出统一入口，使用经过验证的收纳展开流程进入工作区后重新启动；"
            "不要从 follower_rest 直接发送工作空间目标。"
        )
        return gated_skills, reason

    return skills, None


def build_so100_plus_runtime(
    robot_config: SO100PlusRobotConfig,
    *,
    risk_acknowledged: bool,
    gripper_open_degrees: float = DEFAULT_SO100_PLUS_GRIPPER_OPEN_DEGREES,
    gripper_close_degrees: float = DEFAULT_SO100_PLUS_GRIPPER_CLOSE_DEGREES,
    robot_factory: Callable[[SO100PlusRobotConfig], Any] = create_so100_plus_robot,
    kinematics_factory: Callable[[], SO100PlusKinematics] = SO100PlusKinematics,
    adapter_factory: Callable[..., SO100PlusAdapter] = SO100PlusAdapter,
    motion_limits_builder: Callable[..., Any] = (
        build_so100_plus_right_follower_motion_limits
    ),
    skill_builder: Callable[
        [ArmAdapter], dict[str, SkillDefinition]
    ] = build_so100_plus_right_follower_arm_skills,
) -> ArmRuntime:
    """装配并连接已登记的 right_follower 真机运行时。"""

    if not risk_acknowledged:
        raise PermissionError(
            "SO-100 Plus 真机模式必须显式确认连接、上力和运动风险。"
        )
    if robot_config.follower_name != DEFAULT_SO100_PLUS_FOLLOWER_NAME:
        raise ValueError(
            "当前正式工作空间只登记给 follower 'right'；"
            "已拒绝套用到其他 follower。"
        )

    robot = robot_factory(robot_config)
    kinematics = kinematics_factory()
    gripper_config = SO100PlusGripperConfig(
        follower_name=robot_config.follower_name,
        open_degrees=gripper_open_degrees,
        close_degrees=gripper_close_degrees,
    )
    active_adapter = adapter_factory(robot, gripper_config)

    try:
        # connect() 会启用力矩并恢复正式运行参数，因此只能在风险确认后执行。
        active_adapter.connect()
        current_joint_radians = _read_current_arm_joint_radians(
            robot,
            robot_config.follower_name,
            kinematics,
        )
        current_tcp_position_m = tuple(
            kinematics.forward_position(current_joint_radians)
        )
        motion_limits = motion_limits_builder(current_joint_radians)

        # 复用同一个已连接 Robot；这个最终 Adapter 才交给 Skills 和 Controller。
        active_adapter = adapter_factory(
            robot,
            gripper_config,
            kinematics=kinematics,
            motion_limits=motion_limits,
            motion_config=SO100PlusMotionConfig(),
        )
        skills, move_arm_disabled_reason = (
            _apply_so100_plus_startup_workspace_gate(
                skill_builder(active_adapter),
                current_tcp_position_m,
            )
        )
        return ArmRuntime(
            adapter=active_adapter,
            skills=skills,
            controller=_build_controller(skills),
            current_tcp_position_m=current_tcp_position_m,
            move_arm_disabled_reason=move_arm_disabled_reason,
        )
    except BaseException:
        try:
            _cleanup_connected_adapter(active_adapter)
        except Exception:
            # 保留原始装配异常；清理已经尽力完成。
            pass
        raise
