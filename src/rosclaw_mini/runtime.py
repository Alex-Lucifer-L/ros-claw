"""应用运行时装配：把 Adapter、Skills 和 Controller 连接起来。"""

from collections.abc import Callable
from dataclasses import dataclass, field
import hashlib
import inspect
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
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionConfig,
)
from rosclaw_mini.arm.so100_plus_session import (
    ArmSessionState,
    SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
    SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
    SO100PlusArmSession,
    build_so100_plus_storage_transition,
    build_so100_plus_transition_motion_limits,
    classify_so100_plus_startup_pose,
    read_so100_plus_pose_snapshot,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
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
    WorkspaceLimits,
    build_so100_plus_right_follower_motion_limits,
)
from rosclaw_mini.skills.arm_skills import (
    bind_so100_plus_arm_session,
    build_arm_skills,
    build_so100_plus_right_follower_arm_skills,
)
from rosclaw_mini.skills.base import SkillDefinition
from rosclaw_mini.workspace_scan.irregular_workspace import (
    SO100PlusIrregularWorkspace,
    load_default_so100_plus_irregular_workspace,
)


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
    session: SO100PlusArmSession | None = None
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
    _torque_disabled_on_shutdown: bool = field(
        default=False,
        init=False,
        repr=False,
    )
    _deferred_emergency_shutdown: bool = field(
        default=False,
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

        self._shutdown(emergency=False)

    def emergency_shutdown(self) -> None:
        """停止并等待动作结束，然后紧急关闭力矩并断开。"""

        self._shutdown(emergency=True)

    def _shutdown(self, *, emergency: bool) -> None:
        """实现普通/紧急关闭；两者都不会在线程运行时断开。"""

        with self._shutdown_lock:
            if self._deferred_cleanup_error is not None:
                raise self._deferred_cleanup_error
            if self._is_shutdown:
                return
            if (
                self._deferred_cleanup_thread is not None
                and self._deferred_cleanup_thread.is_alive()
            ):
                self._deferred_emergency_shutdown = (
                    self._deferred_emergency_shutdown or emergency
                )
                raise ArmRuntimeShutdownError(
                    "后台动作仍在运行；延后 disconnect 已安排，尚未完成。"
                )
            errors: list[str] = []
            connected_at_start = self.adapter.is_connected

            if not connected_at_start:
                errors.append(
                    "Adapter 在关闭开始前已经意外断开，无法确认 stop 已送达"
                )
            else:
                try:
                    if self.session is None:
                        self.adapter.stop()
                    else:
                        self.session.request_stop()
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
                self._start_deferred_cleanup(emergency=emergency)
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
                if emergency:
                    self._disable_torque_emergency(errors)
                else:
                    self._disable_torque_if_rest(errors)
                try:
                    if self.adapter.is_connected:
                        self.adapter.disconnect()
                except Exception as error:
                    errors.append(f"disconnect 失败：{error}")

            if not self.adapter.is_connected:
                self._is_shutdown = True

            if errors:
                raise ArmRuntimeShutdownError("；".join(errors) + "。")

    @property
    def session_state(self) -> ArmSessionState | None:
        """返回真机会话的唯一状态；Mock 后端没有姿态状态机。"""

        return self.session.state if self.session is not None else None

    @property
    def torque_disabled_on_shutdown(self) -> bool:
        """正常关闭是否已验证全部电机力矩关闭。"""

        return self._torque_disabled_on_shutdown

    @property
    def exit_pose_warning(self) -> str | None:
        """退出前提示未处于认证收纳姿态，但不阻止安全断开。"""

        state = self.session_state
        if state is None or state is ArmSessionState.REST:
            return None
        return (
            f"退出提示：当前会话状态为 {state.value}，机械臂未处于"
            "认证 follower_rest；不会自动展开、收纳或关闭力矩，"
            "将只停止、等待并断开。"
        )

    def _disable_torque_if_rest(self, errors: list[str]) -> None:
        """仅在软件状态和真实反馈都确认收纳时正常卸力。"""

        if self.session_state is not ArmSessionState.REST:
            return
        try:
            self.adapter.disable_torque()
        except Exception as error:
            errors.append(f"REST 状态关闭力矩失败：{error}")
        else:
            self._torque_disabled_on_shutdown = True

    def _disable_torque_emergency(self, errors: list[str]) -> None:
        """紧急退出明确请求卸力；不把当前姿态误报为 REST。"""

        try:
            self.adapter.disable_torque(emergency=True)
        except Exception as error:
            errors.append(f"紧急关闭力矩失败：{error}")
        else:
            self._torque_disabled_on_shutdown = True

    def _start_deferred_cleanup(self, *, emergency: bool = False) -> None:
        """安排后台动作结束后的最终断开；调用方必须持有关闭锁。"""

        self._deferred_emergency_shutdown = (
            self._deferred_emergency_shutdown or emergency
        )
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
                cleanup_errors: list[str] = []
                if self.adapter.is_connected:
                    if self._deferred_emergency_shutdown:
                        self._disable_torque_emergency(cleanup_errors)
                    else:
                        self._disable_torque_if_rest(cleanup_errors)
                    if self.adapter.is_connected:
                        self.adapter.disconnect()
                if not self.adapter.is_connected:
                    self._is_shutdown = True
                if cleanup_errors:
                    self._deferred_cleanup_error = ArmRuntimeShutdownError(
                        "；".join(cleanup_errors) + "。"
                    )
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
    *,
    session: SO100PlusArmSession | None = None,
) -> ExecutionController:
    def execute_command(command):
        return run_command(command, skills)

    return ExecutionController(
        execute_command,
        before_submit=(
            session.prepare_command if session is not None else None
        ),
        after_finish=(
            session.finish_command if session is not None else None
        ),
    )


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


def _cleanup_connected_adapter(adapter: ArmAdapter) -> None:
    """装配中途失败时尽力停止并断开，但不调用 disable_torque。"""

    try:
        if adapter.is_connected:
            adapter.stop()
    finally:
        if adapter.is_connected:
            adapter.disconnect()


def _build_so100_plus_work_motion_limits(
    builder: Callable[..., Any],
    current_joint_radians,
    workspace: WorkspaceLimits,
):
    """兼容旧的一参数测试/扩展 builder，同时给正式 builder 传规划框。"""

    try:
        parameters = inspect.signature(builder).parameters.values()
    except (TypeError, ValueError):
        parameters = ()
    supports_workspace = any(
        parameter.name == "workspace"
        or parameter.kind is inspect.Parameter.VAR_KEYWORD
        for parameter in parameters
    )
    if supports_workspace:
        return builder(
            current_joint_radians,
            workspace=workspace,
        )
    return builder(current_joint_radians)


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
    transition_motion_limits_builder: Callable[..., Any] = (
        build_so100_plus_transition_motion_limits
    ),
    trajectory_validator_factory: Callable[
        [], SO100PlusMuJoCoTrajectoryValidator
    ] = SO100PlusMuJoCoTrajectoryValidator,
    irregular_workspace_factory: Callable[
        [], SO100PlusIrregularWorkspace
    ] = load_default_so100_plus_irregular_workspace,
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
    # 网格、MuJoCo 或模型不可用时必须在创建、连接 Robot 之前失败关闭。
    irregular_workspace = irregular_workspace_factory()
    trajectory_validator = trajectory_validator_factory()

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
        try:
            follower_bus = robot.follower_arms[robot_config.follower_name]
        except (AttributeError, KeyError) as error:
            raise RuntimeError(
                f"真机 Robot 缺少 follower "
                f"{robot_config.follower_name!r}。"
            ) from error

        initial_snapshot = read_so100_plus_pose_snapshot(
            follower_bus,
            kinematics,
            include_torque=False,
        )
        work_tcp_position_m = SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M
        startup_state, _startup_reason = classify_so100_plus_startup_pose(
            initial_snapshot,
            work_tcp_position_m,
            SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
        )
        storage_joint_radians = (
            initial_snapshot.joint_radians
            if startup_state is ArmSessionState.REST
            else SO100PlusKinematics.driver_degrees_to_model_radians(
                SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
            )
        )
        transition = build_so100_plus_storage_transition(
            storage_joint_radians,
            kinematics,
        )
        motion_limits = _build_so100_plus_work_motion_limits(
            motion_limits_builder,
            SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
            irregular_workspace.planning_envelope,
        )
        transition_motion_limits = transition_motion_limits_builder(
            transition
        )

        # 两个 Adapter 复用同一个已连接 Robot。普通 Skill 只能看到正式
        # 工作区 Adapter；固定展开/收纳通道只由会话内部使用。
        work_adapter = adapter_factory(
            robot,
            gripper_config,
            kinematics=kinematics,
            motion_limits=motion_limits,
            motion_config=SO100PlusMotionConfig(),
        )
        active_adapter = work_adapter
        transition_adapter = adapter_factory(
            robot,
            gripper_config,
            kinematics=kinematics,
            motion_limits=transition_motion_limits,
            motion_config=SO100PlusMotionConfig(),
        )

        def pose_reader():
            return read_so100_plus_pose_snapshot(
                follower_bus,
                kinematics,
                include_torque=False,
            )

        session = SO100PlusArmSession(
            work_adapter=work_adapter,
            transition_adapter=transition_adapter,
            pose_reader=pose_reader,
            kinematics=kinematics,
            initial_snapshot=initial_snapshot,
            storage_joint_radians=storage_joint_radians,
            transition_motion_limits=transition_motion_limits,
            trajectory_validator=trajectory_validator,
            work_workspace=irregular_workspace,
        )
        skills = bind_so100_plus_arm_session(
            skill_builder(work_adapter),
            session,
        )
        move_arm_disabled_reason = (
            "当前姿态无法认证，所有运动动作已失败关闭："
            + session.state_reason
            if session.state is ArmSessionState.UNVERIFIED
            else None
        )
        return ArmRuntime(
            adapter=work_adapter,
            skills=skills,
            controller=_build_controller(skills, session=session),
            current_tcp_position_m=initial_snapshot.tcp_position_m,
            move_arm_disabled_reason=move_arm_disabled_reason,
            session=session,
        )
    except BaseException:
        try:
            _cleanup_connected_adapter(active_adapter)
        except Exception:
            # 保留原始装配异常；清理已经尽力完成。
            pass
        raise
