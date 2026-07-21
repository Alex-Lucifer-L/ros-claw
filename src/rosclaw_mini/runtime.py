"""应用运行时装配：把 Adapter、Skills 和 Controller 连接起来。"""

from collections.abc import Callable
from dataclasses import dataclass, field, replace
import hashlib
import math
from pathlib import Path
from threading import Lock, Thread
from typing import Any

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.mock_arm import MockArmAdapter
from rosclaw_mini.arm.so100_plus import (
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionConfig,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusConfigurationError,
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
DEFAULT_SHUTDOWN_WAIT_TIMEOUT_SECONDS = 5.0


@dataclass(frozen=True)
class SO100PlusWorkspaceCertification:
    """把正式工作空间绑定到已验收的 follower、校准和启动姿态。"""

    port: Path
    follower_name: str
    calibration_filename: str
    calibration_sha256: str
    startup_joint_radians: tuple[float, ...]
    startup_joint_tolerance_degrees: float


SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_CERTIFICATION = (
    SO100PlusWorkspaceCertification(
        port=Path("/dev/lerobot_right"),
        follower_name="right",
        calibration_filename="right_follower.json",
        calibration_sha256=(
            "ac7b9877020da10aa6f886347bedf6b105aaeaf01493b2a65830c628c35837de"
        ),
        startup_joint_radians=SO100_PLUS_JOYCON_INITIAL_RADIANS,
        startup_joint_tolerance_degrees=5.0,
    )
)

# 这里只属于 Mock 演示，不代表任何真实机械臂的物理工作空间。
DEFAULT_MOCK_WORKSPACE = WorkspaceLimits(
    x=AxisLimits(-1.0, 1.0),
    y=AxisLimits(-1.0, 1.0),
    z=AxisLimits(-1.0, 1.0),
)


class ArmRuntimeShutdownError(RuntimeError):
    """运行时无法确认后台动作已结束并安全断开。"""


@dataclass
class ArmRuntime:
    """一次应用运行所共享的 Adapter、Skills 和 Controller。"""

    adapter: ArmAdapter
    skills: dict[str, SkillDefinition]
    controller: ExecutionController
    current_tcp_position_m: tuple[float, float, float] | None = None
    move_arm_disabled_reason: str | None = None
    shutdown_wait_timeout_seconds: float = (
        DEFAULT_SHUTDOWN_WAIT_TIMEOUT_SECONDS
    )
    _is_shutdown: bool = field(default=False, init=False, repr=False)
    _shutdown_lock: Lock = field(
        default_factory=Lock,
        init=False,
        repr=False,
    )
    _deferred_cleanup_thread: Thread | None = field(
        default=None,
        init=False,
        repr=False,
    )
    _deferred_cleanup_error: ArmRuntimeShutdownError | None = field(
        default=None,
        init=False,
        repr=False,
    )

    def __post_init__(self) -> None:
        if (
            isinstance(self.shutdown_wait_timeout_seconds, bool)
            or not math.isfinite(self.shutdown_wait_timeout_seconds)
            or self.shutdown_wait_timeout_seconds <= 0
        ):
            raise ValueError("关闭等待超时必须是有限正数。")

    def shutdown(self) -> None:
        """停止后限时等待后台动作；线程结束前绝不主动断开。"""

        with self._shutdown_lock:
            if self._is_shutdown:
                return
            if (
                self._deferred_cleanup_thread is not None
                and self._deferred_cleanup_thread.is_alive()
            ):
                raise ArmRuntimeShutdownError(
                    "后台动作仍在运行；延后 disconnect 已安排，尚未完成。"
                )
            if self._deferred_cleanup_error is not None:
                raise self._deferred_cleanup_error

            errors: list[str] = []
            connected_at_start = self.adapter.is_connected

            if not connected_at_start:
                errors.append(
                    "Adapter 在关闭开始前已经意外断开，无法确认 stop 已送达"
                )
            else:
                try:
                    self.adapter.stop()
                except Exception as error:
                    errors.append(f"stop 失败：{error}")

            if self.controller.is_running():
                try:
                    self.controller.wait(
                        timeout=self.shutdown_wait_timeout_seconds,
                    )
                except Exception as error:
                    errors.append(f"等待后台 Controller 失败：{error}")

            if self.controller.is_running():
                self._start_deferred_cleanup()
                details = "；".join(errors)
                if details:
                    details = f"；此前还发生：{details}"
                raise ArmRuntimeShutdownError(
                    "后台动作在 "
                    f"{self.shutdown_wait_timeout_seconds:.3f} 秒内未结束；"
                    "线程结束前未执行 disconnect，已安排结束后的延后 disconnect"
                    f"{details}。"
                )

            if connected_at_start and not self.adapter.is_connected:
                errors.append("Adapter 在关闭过程中意外断开")
            elif self.adapter.is_connected:
                try:
                    self.adapter.disconnect()
                except Exception as error:
                    errors.append(f"disconnect 失败：{error}")

            if not self.adapter.is_connected:
                self._is_shutdown = True

            if errors:
                raise ArmRuntimeShutdownError("；".join(errors) + "。")

    def _start_deferred_cleanup(self) -> None:
        """安排后台动作结束后的最终断开；调用方必须持有关闭锁。"""

        if (
            self._deferred_cleanup_thread is not None
            and self._deferred_cleanup_thread.is_alive()
        ):
            return
        cleanup_thread = Thread(
            target=self._disconnect_after_controller_stops,
            name="rosclaw-arm-runtime-cleanup",
            daemon=False,
        )
        self._deferred_cleanup_thread = cleanup_thread
        cleanup_thread.start()

    def _disconnect_after_controller_stops(self) -> None:
        """以有界等待轮询 Controller，结束后完成最终 disconnect。"""

        try:
            while self.controller.is_running():
                self.controller.wait(
                    timeout=self.shutdown_wait_timeout_seconds,
                )

            with self._shutdown_lock:
                if self._is_shutdown:
                    return
                if self.adapter.is_connected:
                    self.adapter.disconnect()
                if not self.adapter.is_connected:
                    self._is_shutdown = True
        except Exception as error:
            with self._shutdown_lock:
                self._deferred_cleanup_error = ArmRuntimeShutdownError(
                    f"延后 disconnect 失败：{error}。"
                )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as input_file:
            for chunk in iter(lambda: input_file.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise SO100PlusConfigurationError(
            f"无法读取已认证校准文件：{path}"
        ) from error
    return digest.hexdigest()


def _validate_so100_plus_workspace_certification(
    robot_config: SO100PlusRobotConfig,
    certification: SO100PlusWorkspaceCertification,
) -> None:
    """在创建 Robot 前确认正式工作空间所绑定的校准文件。"""

    if Path(robot_config.port) != certification.port:
        raise SO100PlusConfigurationError(
            "正式工作空间和校准认证只允许端口 "
            f"{str(certification.port)!r}。"
        )
    if robot_config.follower_name != certification.follower_name:
        raise SO100PlusConfigurationError(
            "正式工作空间只认证给 follower "
            f"{certification.follower_name!r}。"
        )

    calibration_path = robot_config.calibration_path
    if calibration_path.name != certification.calibration_filename:
        raise SO100PlusConfigurationError(
            "正式工作空间要求校准文件 "
            f"{certification.calibration_filename!r}。"
        )
    if not calibration_path.is_file():
        raise SO100PlusConfigurationError(
            f"已认证校准文件不存在：{calibration_path}。"
        )

    actual_sha256 = _sha256_file(calibration_path)
    if actual_sha256 != certification.calibration_sha256:
        raise SO100PlusConfigurationError(
            "校准文件与正式工作空间认证指纹不一致；"
            f"期望 {certification.calibration_sha256}，实际 {actual_sha256}。"
        )


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


def _apply_so100_plus_startup_certification_gate(
    skills: dict[str, SkillDefinition],
    current_tcp_position_m: tuple[float, float, float],
    current_joint_radians: tuple[float, ...],
    certification: SO100PlusWorkspaceCertification,
) -> tuple[dict[str, SkillDefinition], str | None]:
    """启动 TCP 或关节姿态不符合认证时，失败关闭 move_arm Skill。"""

    move_arm_skill = skills.get("move_arm")
    if move_arm_skill is None:
        raise RuntimeError("right_follower Skill 注册表缺少 move_arm。")

    violations: list[str] = []
    try:
        SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS.validate_position(
            *current_tcp_position_m
        )
    except LimitViolationError as error:
        position = ", ".join(
            f"{value:.6f}" for value in current_tcp_position_m
        )
        violations.append(
            f"启动 TCP ({position}) m 不在当前 right_follower "
            f"正式工作空间内（{error}）"
        )

    try:
        joint_errors_degrees = tuple(
            abs(math.degrees(actual - expected))
            for actual, expected in zip(
                current_joint_radians,
                certification.startup_joint_radians,
                strict=True,
            )
        )
    except ValueError as error:
        raise RuntimeError("当前关节数与已认证启动姿态不一致。") from error

    max_joint_error_degrees = max(joint_errors_degrees)
    if max_joint_error_degrees > certification.startup_joint_tolerance_degrees:
        max_error_index = joint_errors_degrees.index(max_joint_error_degrees)
        violations.append(
            "启动关节姿态不在已认证范围内："
            f"{SO100_PLUS_ARM_JOINT_NAMES[max_error_index]} 偏差 "
            f"{max_joint_error_degrees:.3f}°，超过 "
            f"{certification.startup_joint_tolerance_degrees:.1f}°"
        )

    if violations:
        gated_skills = dict(skills)
        gated_skills["move_arm"] = replace(
            move_arm_skill,
            enabled=False,
        )
        reason = (
            "move_arm 已失败关闭："
            + "；".join(violations)
            + "。"
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

    certification = SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_CERTIFICATION
    _validate_so100_plus_workspace_certification(
        robot_config,
        certification,
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
            _apply_so100_plus_startup_certification_gate(
                skill_builder(active_adapter),
                current_tcp_position_m,
                current_joint_radians,
                certification,
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
