"""SO-100 Plus 机械臂适配器。"""

from dataclasses import dataclass, replace
import math
from threading import Event, Lock
from typing import Callable, Mapping, Sequence

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.safety.limits import (
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
)


GRIPPER_MOTOR_NAME = "gripper_joint"
SHOULDER_ROTATION_MOTOR_NAME = "shoulder_rotation_joint"
ELBOW_MOTOR_NAME = "ellbow_joint"
WRIST_PITCH_MOTOR_NAME = "wrist_pitch_joint"
TUNED_PID_MOTOR_NAMES = (
    SHOULDER_ROTATION_MOTOR_NAME,
    ELBOW_MOTOR_NAME,
    WRIST_PITCH_MOTOR_NAME,
)


class SO100PlusGripperSafetyError(RuntimeError):
    """夹爪在执行途中超出已验证的安全条件。"""


class SO100PlusMotionStoppedError(RuntimeError):
    """适配器正在执行的动作已被 stop() 取消。"""


class SO100PlusMotionPlanningDisabledError(RuntimeError):
    """缺少明确的运动学或安全限制，因此规划保持禁用。"""


class SO100PlusMotionExecutionDisabledError(RuntimeError):
    """缺少明确的执行参数，因此物理移动保持禁用。"""


class SO100PlusArmSafetyError(RuntimeError):
    """手臂轨迹执行途中超出已确认的安全条件。"""


class SO100PlusMotionConvergenceError(SO100PlusArmSafetyError):
    """轨迹安全完成，但最终位置或稳定时间没有达到验收门槛。"""


class SO100PlusTorqueReleaseSafetyError(RuntimeError):
    """普通力矩释放不满足 follower_rest 前置条件。"""


@dataclass(frozen=True)
class SO100PlusRealHardwareProfile:
    """上一次 right_follower 真机验证后保存的运行配置。"""

    other_motor_p_coefficient: int = 16
    elbow_p_coefficient: int = 64
    wrist_pitch_p_coefficient: int = 24
    tuned_motor_i_coefficient: int = 2
    tuned_motor_d_coefficient: int = 32
    storage_rest_driver_degrees: tuple[float, ...] = (
        2.900,
        193.096,
        178.418,
        71.719,
        -1.318,
        101.162,
    )
    storage_rest_tolerances_degrees: tuple[float, ...] = (
        5.0,
        8.0,
        5.0,
        5.0,
        20.0,
        15.0,
    )
    runtime_acceleration: int = 35
    gripper_max_step_degrees: float = 10.0
    gripper_settle_seconds: float = 2.5
    gripper_load_limit: float = 300.0
    gripper_position_tolerance_degrees: float = 3.0
    waypoint_timeout_seconds: float = 8.0
    waypoint_poll_interval_seconds: float = 0.25
    final_settle_seconds: float = 0.75
    joint_position_tolerance_degrees: float = 5.0
    cartesian_tolerance_m: float = 0.012
    load_limit: float = 450.0
    critical_load_limit: float = 700.0
    load_confirmation_samples: int = 2
    max_temperature_celsius: float = 60.0
    critical_temperature_celsius: float = 70.0
    temperature_confirmation_samples: int = 2
    stream_frequency_hz: float = 30.0
    # 5° 是流式推进的反馈节流线，不是单次立即停止线。
    # 超过时暂停发送后续 waypoint，等待实测关节追上；
    # 超时、stop、过载、过温或通信异常仍会停止。
    stream_max_joint_speed_degrees_per_second: float = 12.0
    stream_tracking_error_limit_degrees: float = 5.0
    stream_critical_tracking_error_limit_degrees: float = 8.0
    stream_telemetry_interval_seconds: float = 0.25


SO100_PLUS_REAL_HARDWARE_PROFILE = SO100PlusRealHardwareProfile()
DEFAULT_ELBOW_P_COEFFICIENT = (
    SO100_PLUS_REAL_HARDWARE_PROFILE.elbow_p_coefficient
)
DEFAULT_WRIST_PITCH_P_COEFFICIENT = (
    SO100_PLUS_REAL_HARDWARE_PROFILE.wrist_pitch_p_coefficient
)


@dataclass(frozen=True)
class SO100PlusTelemetry:
    """一次 SO-100 Plus 全部 follower 舍机的原始遥测快照。"""

    phase: str
    motor_names: tuple[str, ...]
    voltage_raw: tuple[float, ...]
    current_raw: tuple[float, ...]
    load_magnitude: tuple[float, ...]
    temperature_raw: tuple[float, ...]


@dataclass(frozen=True)
class SO100PlusSettleReport:
    """最终关节进入容差后连续稳定观察得到的位置统计。"""

    motor_names: tuple[str, ...]
    position_samples_degrees: tuple[tuple[float, ...], ...]
    position_span_degrees: tuple[float, ...]
    tcp_samples_m: tuple[tuple[float, float, float], ...]
    tcp_min_m: tuple[float, float, float]
    tcp_max_m: tuple[float, float, float]
    tcp_mean_m: tuple[float, float, float]
    duration_seconds: float


@dataclass(frozen=True)
class SO100PlusPIDGains:
    """一个 STS3215 位置环的 EPROM PID 参数。"""

    p: int
    i: int
    d: int

    def __post_init__(self) -> None:
        for name, value in (("P", self.p), ("I", self.i), ("D", self.d)):
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not 0 <= value <= 254
            ):
                raise ValueError(f"{name} 增益必须是 0 到 254 之间的整数。")


@dataclass(frozen=True)
class SO100PlusGripperConfig:
    """真实夹爪已经验证过的运行参数。"""

    follower_name: str
    open_degrees: float
    close_degrees: float
    max_step_degrees: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.gripper_max_step_degrees
    )
    settle_seconds: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.gripper_settle_seconds
    )
    load_limit: float = SO100_PLUS_REAL_HARDWARE_PROFILE.gripper_load_limit
    position_tolerance_degrees: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.gripper_position_tolerance_degrees
    )
    runtime_acceleration: int = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.runtime_acceleration
    )

    def __post_init__(self) -> None:
        numeric_values = (
            self.open_degrees,
            self.close_degrees,
            self.max_step_degrees,
            self.settle_seconds,
            self.load_limit,
            self.position_tolerance_degrees,
        )
        if not self.follower_name:
            raise ValueError("follower_name 不能为空。")
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("夹爪配置必须是有限数值。")
        if self.open_degrees <= self.close_degrees:
            raise ValueError("夹爪张开角度必须大于闭合角度。")
        if self.max_step_degrees <= 0:
            raise ValueError("夹爪单步角度必须大于 0。")
        if self.settle_seconds < 0:
            raise ValueError("夹爪等待时间不能为负数。")
        if self.load_limit <= 0:
            raise ValueError("夹爪负载限制必须大于 0。")
        if self.position_tolerance_degrees < 0:
            raise ValueError("夹爪位置容差不能为负数。")
        if (
            isinstance(self.runtime_acceleration, bool)
            or not isinstance(self.runtime_acceleration, int)
            or not 0 <= self.runtime_acceleration <= 254
        ):
            raise ValueError("运行时加速度必须是 0 到 254 之间的整数。")


@dataclass(frozen=True)
class SO100PlusMotionConfig:
    """手臂轨迹的显式到位、误差和遥测限制。"""

    waypoint_timeout_seconds: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.waypoint_timeout_seconds
    )
    waypoint_poll_interval_seconds: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.waypoint_poll_interval_seconds
    )
    final_settle_seconds: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.final_settle_seconds
    )
    joint_position_tolerance_degrees: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.joint_position_tolerance_degrees
    )
    cartesian_tolerance_m: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.cartesian_tolerance_m
    )
    load_limit: float = SO100_PLUS_REAL_HARDWARE_PROFILE.load_limit
    critical_load_limit: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.critical_load_limit
    )
    load_confirmation_samples: int = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.load_confirmation_samples
    )
    max_temperature_celsius: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.max_temperature_celsius
    )
    critical_temperature_celsius: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.critical_temperature_celsius
    )
    temperature_confirmation_samples: int = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.temperature_confirmation_samples
    )
    stream_frequency_hz: float | None = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.stream_frequency_hz
    )
    stream_max_joint_speed_degrees_per_second: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.stream_max_joint_speed_degrees_per_second
    )
    stream_tracking_error_limit_degrees: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.stream_tracking_error_limit_degrees
    )
    stream_critical_tracking_error_limit_degrees: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE
        .stream_critical_tracking_error_limit_degrees
    )
    stream_telemetry_interval_seconds: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.stream_telemetry_interval_seconds
    )

    def __post_init__(self) -> None:
        values = (
            self.waypoint_timeout_seconds,
            self.waypoint_poll_interval_seconds,
            self.final_settle_seconds,
            self.joint_position_tolerance_degrees,
            self.cartesian_tolerance_m,
            self.load_limit,
            self.critical_load_limit,
            self.max_temperature_celsius,
            self.critical_temperature_celsius,
            self.stream_max_joint_speed_degrees_per_second,
            self.stream_tracking_error_limit_degrees,
            self.stream_critical_tracking_error_limit_degrees,
            self.stream_telemetry_interval_seconds,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("运动执行参数必须是有限数值。")
        if self.waypoint_timeout_seconds <= 0:
            raise ValueError("轨迹点到位超时时间必须大于 0。")
        if self.waypoint_poll_interval_seconds <= 0:
            raise ValueError("轨迹点轮询间隔必须大于 0。")
        if (
            self.waypoint_poll_interval_seconds
            > self.waypoint_timeout_seconds
        ):
            raise ValueError("轨迹点轮询间隔不能大于到位超时时间。")
        if self.final_settle_seconds < 0:
            raise ValueError("最终稳定观察时间不能为负数。")
        if self.final_settle_seconds > self.waypoint_timeout_seconds:
            raise ValueError("最终稳定观察时间不能大于到位超时时间。")
        if self.joint_position_tolerance_degrees < 0:
            raise ValueError("关节位置容差不能为负数。")
        if self.cartesian_tolerance_m <= 0:
            raise ValueError("夹爪 TCP 位置容差必须大于 0。")
        if self.load_limit <= 0:
            raise ValueError("手臂负载限制必须大于 0。")
        if self.critical_load_limit <= self.load_limit:
            raise ValueError("手臂紧急负载上限必须大于普通负载上限。")
        if (
            isinstance(self.load_confirmation_samples, bool)
            or not isinstance(self.load_confirmation_samples, int)
            or not 2 <= self.load_confirmation_samples <= 5
        ):
            raise ValueError("普通负载连续确认次数必须是 2 到 5。")
        if self.max_temperature_celsius <= 0:
            raise ValueError("电机温度上限必须大于 0°C。")
        if (
            self.critical_temperature_celsius
            <= self.max_temperature_celsius
        ):
            raise ValueError("电机紧急温度上限必须大于普通温度上限。")
        if (
            isinstance(self.temperature_confirmation_samples, bool)
            or not isinstance(self.temperature_confirmation_samples, int)
            or not 2 <= self.temperature_confirmation_samples <= 5
        ):
            raise ValueError("普通过温连续确认次数必须是 2 到 5。")
        if self.stream_frequency_hz is not None:
            if (
                isinstance(self.stream_frequency_hz, bool)
                or not math.isfinite(self.stream_frequency_hz)
                or not 5.0 <= self.stream_frequency_hz <= 50.0
            ):
                raise ValueError("流式轨迹频率必须在 5 到 50 Hz 之间。")
        if not 0 < self.stream_max_joint_speed_degrees_per_second <= 60.0:
            raise ValueError("流式轨迹最大关节速度必须大于 0 且不超过 60°/s。")
        if (
            self.stream_tracking_error_limit_degrees
            < self.joint_position_tolerance_degrees
        ):
            raise ValueError("流式跟踪误差上限不能小于最终关节位置容差。")
        if (
            self.stream_critical_tracking_error_limit_degrees
            <= self.stream_tracking_error_limit_degrees
        ):
            raise ValueError("流式紧急跟踪误差线必须大于普通记录线。")
        if self.stream_telemetry_interval_seconds <= 0:
            raise ValueError("流式遥测间隔必须大于 0。")


class SO100PlusAdapter(ArmAdapter):
    """SO-100 Plus 的统一原子操作适配器。"""

    def __init__(
        self,
        robot,
        gripper_config: SO100PlusGripperConfig,
        *,
        wait_func: Callable[[float], bool | None] | None = None,
        on_telemetry: Callable[[SO100PlusTelemetry], None] | None = None,
        kinematics: SO100PlusKinematics | None = None,
        motion_limits: MotionLimits | None = None,
        motion_config: SO100PlusMotionConfig | None = None,
        elbow_p_coefficient: int = DEFAULT_ELBOW_P_COEFFICIENT,
        wrist_pitch_p_coefficient: int = DEFAULT_WRIST_PITCH_P_COEFFICIENT,
        cameras: Mapping[str, object] | None = None,
    ):
        if (kinematics is None) != (motion_limits is None):
            raise ValueError("运动学和运动限制必须同时配置。")
        if motion_config is not None and kinematics is None:
            raise ValueError("运动执行参数必须与运动学和运动限制一起配置。")
        if (
            isinstance(elbow_p_coefficient, bool)
            or not isinstance(elbow_p_coefficient, int)
            or not 32 <= elbow_p_coefficient <= 64
        ):
            raise ValueError("肘关节 P 增益必须是 32 到 64 之间的整数。")
        if (
            isinstance(wrist_pitch_p_coefficient, bool)
            or not isinstance(wrist_pitch_p_coefficient, int)
            or not 16 <= wrist_pitch_p_coefficient <= 32
        ):
            raise ValueError("腕部俯仰关节 P 增益必须是 16 到 32 之间的整数。")
        camera_map = dict(cameras or {})
        if getattr(robot, "cameras", {}):
            raise ValueError(
                "LeRobot Robot 内嵌摄像头会重新耦合机械臂和摄像头"
                "生命周期；请改为传给 SO100PlusAdapter(cameras=...)。"
            )
        invalid_camera_names = tuple(
            name
            for name in camera_map
            if not isinstance(name, str) or not name.isidentifier()
        )
        if invalid_camera_names:
            raise ValueError(
                f"摄像头名称必须是简单标识符：{invalid_camera_names}。"
            )

        self.robot = robot
        self.gripper_config = gripper_config
        self.kinematics = kinematics
        self.motion_limits = motion_limits
        self.motion_config = motion_config
        self.elbow_p_coefficient = elbow_p_coefficient
        self.wrist_pitch_p_coefficient = wrist_pitch_p_coefficient
        self.cameras = camera_map
        self._stop_requested = Event()
        self._action_state_lock = Lock()
        self._motion_action_active = False
        self._motion_waypoint_written = False
        self._bus_lock = Lock()
        self._motion_lock = Lock()
        self._wait = wait_func or self._stop_requested.wait
        self._on_telemetry = on_telemetry
        self._telemetry_history: list[SO100PlusTelemetry] = []
        self._last_motion_plan: JointMotionPlan | None = None
        self._last_settle_report: SO100PlusSettleReport | None = None
        self._last_cartesian_target_m: tuple[float, float, float] | None = None
        self._last_cartesian_actual_m: tuple[float, float, float] | None = None
        self._last_cartesian_error_m: float | None = None
        self._load_limit_streak: dict[str, int] = {}
        self._temperature_limit_streak: dict[str, int] = {}

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    @property
    def telemetry_history(self) -> tuple[SO100PlusTelemetry, ...]:
        return tuple(self._telemetry_history)

    @property
    def last_motion_plan(self) -> JointMotionPlan | None:
        return self._last_motion_plan

    @property
    def last_settle_report(self) -> SO100PlusSettleReport | None:
        return self._last_settle_report

    @property
    def last_cartesian_target_m(self) -> tuple[float, float, float] | None:
        return self._last_cartesian_target_m

    @property
    def last_cartesian_actual_m(self) -> tuple[float, float, float] | None:
        return self._last_cartesian_actual_m

    @property
    def last_cartesian_error_m(self) -> float | None:
        return self._last_cartesian_error_m

    @property
    def motion_waypoint_written(self) -> bool:
        """当前注册动作是否已向电机写过轨迹目标。"""

        with self._action_state_lock:
            return self._motion_waypoint_written

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(self.cameras)

    @property
    def has_cameras(self) -> bool:
        return bool(self.cameras)

    @property
    def cameras_connected(self) -> bool:
        return self.has_cameras and all(
            bool(getattr(camera, "is_connected", False))
            for camera in self.cameras.values()
        )

    def connect(self) -> None:
        if not self.robot.is_connected:
            follower_bus = self._follower_bus()
            try:
                self.robot.connect()
                with self._bus_lock:
                    # 提高肘关节和腕部俯仰关节刚度前先把目标同步到
                    # 实测位置，避免旧目标在 P 增益提高后引起跳动。
                    motor_names, present_positions = self._read_all_positions_locked(
                        follower_bus
                    )
                    follower_bus.write(
                        "Goal_Position",
                        list(present_positions),
                    )
                    # 旧版 LeRobot 预设会在每次连接时写回 P=16、
                    # I=0、D=0 并打开 EEPROM 写锁。先统一上锁，再只
                    # 恢复与正式实机配置不同的寄存器。
                    follower_bus.write("Lock", 1)
                    self._raise_unless_eprom_locked(
                        follower_bus,
                        motor_names,
                    )
                    self._write_pid_gains_locked(
                        follower_bus,
                        self._saved_arm_pid_gains(),
                    )
                    # 只覆盖本次运行的 RAM 加速度，不写 Lock 或
                    # Maximum_Acceleration。
                    follower_bus.write(
                        "Acceleration",
                        self.gripper_config.runtime_acceleration,
                    )
                    telemetry = self._capture_telemetry_locked(
                        follower_bus,
                        phase="connected",
                    )
            except Exception:
                try:
                    if bool(
                        getattr(follower_bus, "is_connected", False)
                        or getattr(self.robot, "is_connected", False)
                    ):
                        follower_bus.write("Torque_Enable", 0)
                        follower_bus.write("Lock", 1)
                finally:
                    if getattr(self.robot, "is_connected", False):
                        self.robot.disconnect()
                    elif getattr(follower_bus, "is_connected", False):
                        follower_bus.disconnect()
                raise
            self._notify_telemetry(telemetry)

    def disconnect(self) -> None:
        if self.robot.is_connected:
            self.robot.disconnect()

    def connect_cameras(self) -> None:
        """按需连接已配置摄像头，不读取或改变机械臂状态。"""

        if not self.cameras:
            raise RuntimeError("未配置摄像头，无法执行摄像头连接。")

        connected_now: list[object] = []
        for name, camera in self.cameras.items():
            if getattr(camera, "is_connected", False):
                continue
            try:
                camera.connect()
            except Exception as exc:
                for connected_camera in reversed(connected_now):
                    try:
                        connected_camera.disconnect()
                    except Exception:
                        pass
                raise RuntimeError(
                    f"摄像头 {name!r} 连接失败；机械臂状态未改变。"
                ) from exc
            connected_now.append(camera)

    def disconnect_cameras(self) -> None:
        """断开已配置摄像头，不读取或改变机械臂状态。"""

        errors: list[str] = []
        for name, camera in reversed(tuple(self.cameras.items())):
            if not getattr(camera, "is_connected", False):
                continue
            try:
                camera.disconnect()
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError(
                "摄像头断开失败：" + "; ".join(errors)
            )

    def capture_camera_images(
        self,
        *,
        asynchronous: bool = True,
    ) -> dict[str, object]:
        """抓取 RGB/BGR HWC 图像，键名与 LeRobot observation 一致。"""

        if not self.cameras:
            raise RuntimeError("未配置摄像头，无法抓取图像。")

        images: dict[str, object] = {}
        for name, camera in self.cameras.items():
            if not getattr(camera, "is_connected", False):
                raise RuntimeError(f"摄像头 {name!r} 未连接。")
            image = camera.async_read() if asynchronous else camera.read()
            actual_shape = tuple(getattr(image, "shape", ()))
            expected_shape = (
                int(camera.height),
                int(camera.width),
                int(camera.channels),
            )
            if actual_shape != expected_shape:
                raise RuntimeError(
                    f"摄像头 {name!r} 返回图像形状 {actual_shape}，"
                    f"期望 {expected_shape}。"
                )
            images[f"observation.images.{name}"] = image
        return images

    def read_pid_gains(
        self,
        motor_names: Sequence[str] = SO100_PLUS_ARM_JOINT_NAMES,
    ) -> dict[str, SO100PlusPIDGains]:
        """读取位置环 PID；不写入 EPROM。"""

        if not self.is_connected:
            raise RuntimeError("读取 PID 前必须先显式连接机械臂。")
        names = self._validated_pid_motor_names(motor_names)
        follower_bus = self._follower_bus()
        with self._bus_lock:
            return self._read_pid_gains_locked(follower_bus, names)

    def set_pid_gains(
        self,
        gains_by_motor: Mapping[str, SO100PlusPIDGains],
        *,
        acknowledge_eprom_write: bool = False,
    ) -> dict[str, SO100PlusPIDGains]:
        """有限次更新位置环 PID，并在每个电机写后重新锁定 EPROM。"""

        if not self.is_connected:
            raise RuntimeError("设置 PID 前必须先显式连接机械臂。")
        if not acknowledge_eprom_write:
            raise PermissionError(
                "PID 位于 EPROM；必须显式确认 acknowledge_eprom_write=True。"
            )
        names = self._validated_pid_motor_names(tuple(gains_by_motor))
        requested = {
            name: gains_by_motor[name]
            for name in names
        }
        if not all(
            isinstance(gains, SO100PlusPIDGains)
            for gains in requested.values()
        ):
            raise TypeError("PID 配置值必须是 SO100PlusPIDGains。")

        follower_bus = self._follower_bus()
        telemetry = None
        with self._motion_lock:
            with self._bus_lock:
                _motor_names, present_positions = (
                    self._read_all_positions_locked(follower_bus)
                )
                self._hold_all_positions(follower_bus, present_positions)
                previous = self._write_pid_gains_locked(
                    follower_bus,
                    requested,
                )
                telemetry = self._capture_telemetry_locked(
                    follower_bus,
                    phase="pid_updated",
                )
        self._notify_telemetry(telemetry)
        return previous

    def begin_motion_action(self) -> None:
        """在命令正式提交前注册动作并初始化 stop 世代。"""

        with self._action_state_lock:
            if self._motion_action_active:
                raise RuntimeError("已有机械臂动作注册。")
            self._stop_requested.clear()
            self._motion_waypoint_written = False
            self._motion_action_active = True

    def end_motion_action(self) -> None:
        """结束已注册动作；不清除动作期间到达的 stop。"""

        with self._action_state_lock:
            self._motion_action_active = False

    def _begin_motion_action_if_needed(self) -> bool:
        """为直接 Adapter 调用注册动作，返回是否由本层持有。"""

        with self._action_state_lock:
            if self._motion_action_active:
                return False
            self._stop_requested.clear()
            self._motion_waypoint_written = False
            self._motion_action_active = True
            return True

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """将夹爪尖端之间的 TCP 移到给定底座系绝对坐标。"""

        if not self.is_connected:
            raise RuntimeError("手臂操作前必须先显式连接机械臂。")
        self._raise_if_motion_planning_disabled()
        if self.motion_config is None:
            raise SO100PlusMotionExecutionDisabledError(
                "未配置经过确认的运动执行参数，物理移动保持禁用。"
            )

        owns_action = self._begin_motion_action_if_needed()
        try:
            with self._motion_lock:
                plan = self._plan_move_to_locked(x, y, z)
                execution_plan = self.materialize_joint_plan(plan)
                self._last_motion_plan = execution_plan
                self._execute_motion_plan_locked(execution_plan)
        finally:
            if owns_action:
                self.end_motion_action()

    def read_tcp_position(self) -> tuple[float, float, float]:
        """从六个真实关节反馈复算当前 TCP，不使用启动缓存。"""

        if not self.is_connected:
            raise RuntimeError("读取 TCP 前必须先显式连接机械臂。")
        self._raise_if_motion_planning_disabled()
        with self._motion_lock:
            follower_bus = self._follower_bus()
            with self._bus_lock:
                driver_degrees = self._read_arm_driver_degrees_locked(
                    follower_bus
                )
            joint_radians = self.kinematics.driver_degrees_to_model_radians(
                driver_degrees
            )
            return tuple(self.kinematics.forward_position(joint_radians))

    def move_joints(
        self,
        joint_radians: Sequence[float],
    ) -> None:
        """将六个模型关节移到显式姿态；夹爪位置保持不变。"""

        if not self.is_connected:
            raise RuntimeError("手臂操作前必须先显式连接机械臂。")
        self._raise_if_motion_planning_disabled()
        if self.motion_config is None:
            raise SO100PlusMotionExecutionDisabledError(
                "未配置经过确认的运动执行参数，物理移动保持禁用。"
            )

        owns_action = self._begin_motion_action_if_needed()
        try:
            with self._motion_lock:
                plan = self._plan_joints_locked(joint_radians)
                execution_plan = self.materialize_joint_plan(plan)
                self._last_motion_plan = execution_plan
                self._execute_motion_plan_locked(execution_plan)
        finally:
            if owns_action:
                self.end_motion_action()

    def plan_move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> JointMotionPlan:
        """读取当前 TCP 并生成安全轨迹，但绝不向电机写目标位置。"""

        if not self.is_connected:
            raise RuntimeError("运动规划前必须先显式连接机械臂。")
        self._raise_if_motion_planning_disabled()

        with self._motion_lock:
            return self._plan_move_to_locked(x, y, z)

    def plan_joints(
        self,
        joint_radians: Sequence[float],
    ) -> JointMotionPlan:
        """读取当前姿态并规划六关节目标，但绝不写入电机。"""

        if not self.is_connected:
            raise RuntimeError("运动规划前必须先显式连接机械臂。")
        self._raise_if_motion_planning_disabled()

        with self._motion_lock:
            return self._plan_joints_locked(joint_radians)

    def execute_joint_plan(self, plan: JointMotionPlan) -> None:
        """执行调用方已经完整预检查的同一个关节计划，不重新规划。

        这个入口只供受控会话使用。普通用户动作仍应调用 ``move_to``；
        本方法不会扩大其工作空间，也不会绕过现有关节单步、遥测、
        跟踪误差、负载、温度和最终到位检查。
        """

        if not self.is_connected:
            raise RuntimeError("手臂操作前必须先显式连接机械臂。")
        self._raise_if_motion_planning_disabled()
        if self.motion_config is None:
            raise SO100PlusMotionExecutionDisabledError(
                "未配置经过确认的运动执行参数，物理移动保持禁用。"
            )
        if not isinstance(plan, JointMotionPlan):
            raise TypeError("execute_joint_plan 需要 JointMotionPlan。")
        if not plan.is_final_execution_plan:
            raise ValueError(
                "execute_joint_plan 只接受预先固化并完成预检的"
                "最终执行计划。"
            )

        owns_action = self._begin_motion_action_if_needed()
        try:
            with self._motion_lock:
                self._raise_if_stop_requested()
                self._validate_final_execution_plan(plan)
                self._last_motion_plan = plan
                self._execute_motion_plan_locked(plan)
        finally:
            if owns_action:
                self.end_motion_action()

    def materialize_joint_plan(
        self,
        plan: JointMotionPlan,
        *,
        held_gripper_driver_degrees: float | None = None,
    ) -> JointMotionPlan:
        """在预检前一次性固化将写给电机的全部目标点。"""

        if not isinstance(plan, JointMotionPlan):
            raise TypeError("materialize_joint_plan 需要 JointMotionPlan。")
        if self.motion_config is None:
            raise SO100PlusMotionExecutionDisabledError(
                "固化执行计划前必须配置运动执行参数。"
            )
        if plan.is_final_execution_plan:
            if (
                held_gripper_driver_degrees is not None
                and plan.held_gripper_driver_degrees
                != float(held_gripper_driver_degrees)
            ):
                raise ValueError("不允许改写已固化计划的夹爪保持姿态。")
            return plan

        gripper_degrees = None
        if held_gripper_driver_degrees is not None:
            if (
                isinstance(held_gripper_driver_degrees, bool)
                or not math.isfinite(float(held_gripper_driver_degrees))
            ):
                raise ValueError("夹爪保持姿态必须是有限驱动角。")
            gripper_degrees = float(held_gripper_driver_degrees)

        raw_has_motion = any(
            abs(target - current) > 1e-12
            for current, target in zip(
                plan.current_joint_radians,
                plan.target_joint_radians,
                strict=True,
            )
        )
        if plan.waypoints_radians:
            if any(
                abs(actual - expected) > 1e-12
                for actual, expected in zip(
                    plan.waypoints_radians[-1],
                    plan.target_joint_radians,
                    strict=True,
                )
            ):
                raise ValueError("待固化计划的最后轨迹点不是计划终点。")
        elif raw_has_motion:
            raise ValueError("待固化的非零关节计划缺少轨迹点。")

        if not raw_has_motion:
            execution_plan = replace(
                plan,
                waypoints_radians=(),
                is_final_execution_plan=True,
                waypoint_interval_seconds=None,
                held_gripper_driver_degrees=gripper_degrees,
            )
            self._validate_final_execution_plan(execution_plan)
            return execution_plan

        frequency_hz = self.motion_config.stream_frequency_hz
        if frequency_hz is None:
            route = (
                tuple(plan.current_joint_radians),
                *tuple(plan.waypoints_radians),
            )
            dense_waypoints: list[tuple[float, ...]] = []
            for start, target in zip(route, route[1:]):
                delta = tuple(
                    end - begin
                    for begin, end in zip(start, target, strict=True)
                )
                step_count = max(
                    1,
                    math.ceil(
                        max(abs(value) for value in delta)
                        / SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS
                    ),
                )
                dense_waypoints.extend(
                    tuple(
                        begin + change * (index / step_count)
                        for begin, change in zip(
                            start,
                            delta,
                            strict=True,
                        )
                    )
                    for index in range(1, step_count + 1)
                )
            waypoints = tuple(dense_waypoints)
            interval_seconds = None
        else:
            max_delta_degrees = max(
                abs(math.degrees(target - current))
                for current, target in zip(
                    plan.current_joint_radians,
                    plan.target_joint_radians,
                    strict=True,
                )
            )
            duration_seconds = (
                max_delta_degrees
                * math.pi
                / (
                    2.0
                    * self.motion_config.stream_max_joint_speed_degrees_per_second
                )
            )
            sample_count = max(
                1,
                math.ceil(duration_seconds * frequency_hz),
            )
            while True:
                waypoints = tuple(
                    tuple(
                        current
                        + (target - current)
                        * (
                            0.5
                            - 0.5
                            * math.cos(
                                math.pi * (sample_index / sample_count)
                            )
                        )
                        for current, target in zip(
                            plan.current_joint_radians,
                            plan.target_joint_radians,
                            strict=True,
                        )
                    )
                    for sample_index in range(1, sample_count + 1)
                )
                previous = plan.current_joint_radians
                largest_step = 0.0
                for waypoint in waypoints:
                    largest_step = max(
                        largest_step,
                        *(
                            abs(target - current)
                            for current, target in zip(
                                previous,
                                waypoint,
                                strict=True,
                            )
                        ),
                    )
                    previous = waypoint
                if (
                    largest_step
                    <= SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS
                    + 1e-12
                ):
                    break
                sample_count += 1
            interval_seconds = 1.0 / frequency_hz

        execution_plan = replace(
            plan,
            waypoints_radians=waypoints,
            is_final_execution_plan=True,
            waypoint_interval_seconds=interval_seconds,
            held_gripper_driver_degrees=gripper_degrees,
        )
        self._validate_final_execution_plan(execution_plan)
        return execution_plan

    def _validate_final_execution_plan(self, plan: JointMotionPlan) -> None:
        """复核固化计划，不生成或修改任何 waypoint。"""

        if not plan.is_final_execution_plan:
            raise ValueError("关节计划尚未固化为最终执行点。")
        interval = plan.waypoint_interval_seconds
        if interval is not None and (
            isinstance(interval, bool)
            or not math.isfinite(interval)
            or interval <= 0
        ):
            raise ValueError("最终执行计划的 waypoint 间隔必须为有限正数。")
        if plan.held_gripper_driver_degrees is not None and (
            isinstance(plan.held_gripper_driver_degrees, bool)
            or not math.isfinite(plan.held_gripper_driver_degrees)
        ):
            raise ValueError("最终执行计划的夹爪保持角必须为有限数值。")

        self.motion_limits.validate_target_position(plan.target_position_m)
        previous = self.motion_limits.joints.validate_position(
            plan.current_joint_radians
        )
        for waypoint in plan.waypoints_radians:
            previous = self.motion_limits.joints.validate_step(
                previous,
                waypoint,
            )
        target = self.motion_limits.joints.validate_position(
            plan.target_joint_radians
        )
        if plan.waypoints_radians:
            final_waypoint = tuple(plan.waypoints_radians[-1])
            if any(
                abs(actual - expected) > 1e-12
                for actual, expected in zip(
                    final_waypoint,
                    target,
                    strict=True,
                )
            ):
                raise ValueError("关节计划的最后轨迹点不是计划终点。")
        elif any(
            abs(actual - expected) > 1e-12
            for actual, expected in zip(
                previous,
                target,
                strict=True,
            )
        ):
            raise ValueError("非零关节计划缺少轨迹点。")

    def _raise_if_motion_planning_disabled(self) -> None:
        if self.kinematics is None or self.motion_limits is None:
            raise SO100PlusMotionPlanningDisabledError(
                "未配置运动学和经过确认的运动限制，规划功能保持禁用。"
            )

    def _plan_move_to_locked(
        self,
        x: float,
        y: float,
        z: float,
    ) -> JointMotionPlan:
        follower_bus = self._follower_bus()
        with self._bus_lock:
            driver_degrees = self._read_arm_driver_degrees_locked(
                follower_bus
            )

        current_joint_radians = (
            self.kinematics.driver_degrees_to_model_radians(driver_degrees)
        )
        plan = self.kinematics.plan_position(
            current_joint_radians=current_joint_radians,
            target_position_m=(x, y, z),
            limits=self.motion_limits,
        )
        self._last_motion_plan = plan
        return plan

    def _plan_joints_locked(
        self,
        joint_radians: Sequence[float],
    ) -> JointMotionPlan:
        follower_bus = self._follower_bus()
        with self._bus_lock:
            driver_degrees = self._read_arm_driver_degrees_locked(
                follower_bus
            )

        current_joint_radians = (
            self.kinematics.driver_degrees_to_model_radians(driver_degrees)
        )
        plan = self.kinematics.plan_joint_pose(
            current_joint_radians=current_joint_radians,
            target_joint_radians=joint_radians,
            limits=self.motion_limits,
        )
        self._last_motion_plan = plan
        return plan

    def _execute_motion_plan_locked(self, plan: JointMotionPlan) -> None:
        if not plan.waypoints_radians:
            return

        self._raise_if_stop_requested()
        self._validate_final_execution_plan(plan)
        self._last_settle_report = None
        self._last_cartesian_target_m = None
        self._last_cartesian_actual_m = None
        self._last_cartesian_error_m = None
        self._load_limit_streak.clear()
        self._temperature_limit_streak.clear()
        follower_bus = self._follower_bus()
        with self._bus_lock:
            motor_names, held_positions = self._read_all_positions_locked(
                follower_bus
            )
            actual_arm_driver_degrees = self._arm_values_by_name(
                motor_names,
                held_positions,
            )
            actual_joint_radians = (
                self.kinematics.driver_degrees_to_model_radians(
                    actual_arm_driver_degrees
                )
            )
            start_errors_degrees = tuple(
                abs(math.degrees(actual - expected))
                for actual, expected in zip(
                    actual_joint_radians,
                    plan.current_joint_radians,
                    strict=True,
                )
            )
            start_tolerance_degrees = (
                self.motion_config.joint_position_tolerance_degrees
            )
            violating_start_indices = tuple(
                index
                for index, error in enumerate(start_errors_degrees)
                if error > start_tolerance_degrees
            )
            if violating_start_indices:
                index = max(
                    violating_start_indices,
                    key=start_errors_degrees.__getitem__,
                )
                raise SO100PlusArmSafetyError(
                    "关节计划执行前起点复核失败："
                    f"{SO100_PLUS_ARM_JOINT_NAMES[index]} 计划 "
                    f"{math.degrees(plan.current_joint_radians[index]):.6f}°、"
                    f"实测 {math.degrees(actual_joint_radians[index]):.6f}°，"
                    f"偏差 {start_errors_degrees[index]:.6f}° 超过现有 "
                    f"{start_tolerance_degrees:.1f}° 关节位置容差；"
                    "未发送任何运动目标。"
                )
            if plan.held_gripper_driver_degrees is not None:
                try:
                    gripper_index = motor_names.index(GRIPPER_MOTOR_NAME)
                except ValueError as error:
                    raise SO100PlusArmSafetyError(
                        "执行计划时缺少 gripper_joint 反馈。"
                    ) from error
                actual_gripper_degrees = held_positions[gripper_index]
                if (
                    abs(
                        actual_gripper_degrees
                        - plan.held_gripper_driver_degrees
                    )
                    > self.gripper_config.position_tolerance_degrees
                ):
                    raise SO100PlusArmSafetyError(
                        "夹爪实测姿态与 MuJoCo 预检姿态不一致："
                        f"预检 {plan.held_gripper_driver_degrees:.6f}°，"
                        f"实测 {actual_gripper_degrees:.6f}°，超过 "
                        f"{self.gripper_config.position_tolerance_degrees:.1f}° "
                        "保持容差。"
                    )
                held = list(held_positions)
                held[gripper_index] = plan.held_gripper_driver_degrees
                held_positions = tuple(held)
            start_telemetry = self._capture_telemetry_locked(
                follower_bus,
                phase="arm_start",
            )
        self._notify_telemetry(start_telemetry)
        self._raise_if_arm_telemetry_unsafe(start_telemetry, start_telemetry)

        if plan.waypoint_interval_seconds is not None:
            final_positions = self._execute_streaming_motion_plan(
                follower_bus,
                motor_names,
                held_positions,
                plan,
                start_telemetry,
            )
        else:
            final_positions = held_positions
            for waypoint in plan.waypoints_radians:
                self._raise_if_stop_requested()
                target_positions = self._compose_arm_target(
                    motor_names,
                    held_positions,
                    waypoint,
                )
                self._write_motion_target(
                    follower_bus,
                    target_positions,
                )

                final_positions = self._wait_for_arm_waypoint(
                    follower_bus,
                    motor_names,
                    target_positions,
                    start_telemetry,
                )

        self._record_final_tcp_position(
            motor_names,
            final_positions,
            plan,
        )

    def _execute_streaming_motion_plan(
        self,
        follower_bus,
        motor_names: tuple[str, ...],
        held_positions: tuple[float, ...],
        plan: JointMotionPlan,
        start_telemetry: SO100PlusTelemetry,
    ) -> tuple[float, ...]:
        """原样发送预检前固化的 30 Hz 关节目标点。"""

        interval_seconds = plan.waypoint_interval_seconds
        if interval_seconds is None:
            raise ValueError("流式执行计划缺少 waypoint 时间间隔。")
        telemetry_stride = max(
            1,
            round(
                self.motion_config.stream_telemetry_interval_seconds
                / interval_seconds
            ),
        )
        final_positions = held_positions
        final_target_positions = held_positions
        sample_count = len(plan.waypoints_radians)

        for sample_index, waypoint in enumerate(
            plan.waypoints_radians,
            start=1,
        ):
            self._raise_if_stop_requested()
            final_target_positions = self._compose_arm_target(
                motor_names,
                held_positions,
                waypoint,
            )
            self._write_motion_target(
                follower_bus,
                final_target_positions,
            )

            stopped_while_waiting = self._wait(interval_seconds)
            if stopped_while_waiting or self._stop_requested.is_set():
                if not self._stop_requested.is_set():
                    with self._bus_lock:
                        _, present_positions = self._read_all_positions_locked(
                            follower_bus
                        )
                        self._hold_all_positions(
                            follower_bus,
                            present_positions,
                        )
                raise SO100PlusMotionStoppedError(
                    "手臂动作已被 stop() 取消。"
                )

            with self._bus_lock:
                _, final_positions = self._read_all_positions_locked(
                    follower_bus
                )
                *_, max_error = self._largest_arm_position_error(
                    motor_names,
                    final_positions,
                    final_target_positions,
                )

            if max_error > (
                self.motion_config
                .stream_critical_tracking_error_limit_degrees
            ):
                final_positions = self._wait_for_stream_tracking_catchup(
                    follower_bus,
                    motor_names,
                    final_positions,
                    final_target_positions,
                    start_telemetry,
                    poll_interval_seconds=interval_seconds,
                )

            step_telemetry = None
            try:
                if (
                    sample_index % telemetry_stride == 0
                    or sample_index == sample_count
                ):
                    with self._bus_lock:
                        step_telemetry = self._capture_telemetry_locked(
                            follower_bus,
                            phase="arm_stream",
                        )
                        self._raise_if_arm_telemetry_unsafe(
                            start_telemetry,
                            step_telemetry,
                            follower_bus=follower_bus,
                            present_positions=final_positions,
                        )
            finally:
                if step_telemetry is not None:
                    self._notify_telemetry(step_telemetry)
        return self._wait_for_arm_waypoint(
            follower_bus,
            motor_names,
            final_target_positions,
            start_telemetry,
        )

    def _wait_for_stream_tracking_catchup(
        self,
        follower_bus,
        motor_names: tuple[str, ...],
        present_positions: tuple[float, ...],
        target_positions: tuple[float, ...],
        start_telemetry: SO100PlusTelemetry,
        *,
        poll_interval_seconds: float,
    ) -> tuple[float, ...]:
        """暂停轨迹推进，让电机追上最后一个已验证目标。

        这里只读反馈，不重发目标、不插值、不重规划，因此后续
        写给电机的 waypoint 仍与 MuJoCo 预检通过的计划一致。
        """

        elapsed_seconds = 0.0
        telemetry_elapsed_seconds = (
            self.motion_config.stream_telemetry_interval_seconds
        )
        latest_positions = present_positions
        while True:
            (
                max_error_joint,
                max_error_measured,
                max_error_target,
                max_error,
            ) = self._largest_arm_position_error(
                motor_names,
                latest_positions,
                target_positions,
            )
            if (
                max_error
                <= self.motion_config
                .stream_critical_tracking_error_limit_degrees
            ):
                return latest_positions

            if (
                telemetry_elapsed_seconds
                >= self.motion_config.stream_telemetry_interval_seconds
            ):
                catchup_telemetry = None
                try:
                    with self._bus_lock:
                        catchup_telemetry = self._capture_telemetry_locked(
                            follower_bus,
                            phase="arm_stream_catchup",
                        )
                        self._raise_if_arm_telemetry_unsafe(
                            start_telemetry,
                            catchup_telemetry,
                            follower_bus=follower_bus,
                            present_positions=latest_positions,
                        )
                finally:
                    if catchup_telemetry is not None:
                        self._notify_telemetry(catchup_telemetry)
                telemetry_elapsed_seconds = 0.0

            if elapsed_seconds >= self.motion_config.waypoint_timeout_seconds:
                with self._bus_lock:
                    self._hold_all_positions(
                        follower_bus,
                        latest_positions,
                    )
                raise SO100PlusMotionConvergenceError(
                    f"流式轨迹关节 {max_error_joint} 在 "
                    f"{self.motion_config.waypoint_timeout_seconds:.1f} 秒内"
                    "未追上最后一个已验证目标："
                    f"目标 {max_error_target:.6f}°、实测 "
                    f"{max_error_measured:.6f}°，跟踪误差 "
                    f"{max_error:.6f}° 仍超过 "
                    f"{self.motion_config.stream_critical_tracking_error_limit_degrees:.1f}°，"
                    "已保持当前位置。"
                )

            wait_seconds = min(
                poll_interval_seconds,
                self.motion_config.waypoint_timeout_seconds
                - elapsed_seconds,
            )
            stopped_while_waiting = self._wait(wait_seconds)
            elapsed_seconds += wait_seconds
            telemetry_elapsed_seconds += wait_seconds
            if stopped_while_waiting or self._stop_requested.is_set():
                if not self._stop_requested.is_set():
                    with self._bus_lock:
                        _, latest_positions = (
                            self._read_all_positions_locked(follower_bus)
                        )
                        self._hold_all_positions(
                            follower_bus,
                            latest_positions,
                        )
                raise SO100PlusMotionStoppedError(
                    "手臂动作已被 stop() 取消。"
                )

            with self._bus_lock:
                _, latest_positions = self._read_all_positions_locked(
                    follower_bus
                )

    def _write_motion_target(
        self,
        follower_bus,
        target_positions: Sequence[float],
    ) -> None:
        """stop 与第一条/后续电机目标在同一动作锁上排序。"""

        with self._action_state_lock:
            self._raise_if_stop_requested()
            with self._bus_lock:
                self._raise_if_stop_requested()
                follower_bus.write(
                    "Goal_Position",
                    list(target_positions),
                )
            self._motion_waypoint_written = True

    def _record_final_tcp_position(
        self,
        motor_names: tuple[str, ...],
        final_positions: tuple[float, ...],
        plan: JointMotionPlan,
    ) -> None:
        """记录真实 TCP 精度；安全去留由会话层的真实姿态门禁决定。"""

        final_arm_degrees = self._arm_values_by_name(
            motor_names,
            final_positions,
        )
        final_joint_radians = (
            self.kinematics.driver_degrees_to_model_radians(
                final_arm_degrees
            )
        )
        final_position_m = self.kinematics.forward_position(
            final_joint_radians
        )
        target_position_m = tuple(float(value) for value in plan.target_position_m)
        actual_position_m = tuple(float(value) for value in final_position_m)
        cartesian_error_m = math.sqrt(
            sum(
                (actual - target) ** 2
                for actual, target in zip(
                    actual_position_m,
                    target_position_m,
                    strict=True,
                )
            )
        )
        self._last_cartesian_target_m = target_position_m
        self._last_cartesian_actual_m = actual_position_m
        self._last_cartesian_error_m = cartesian_error_m

    def _wait_for_arm_waypoint(
        self,
        follower_bus,
        motor_names: tuple[str, ...],
        target_positions: tuple[float, ...],
        start_telemetry: SO100PlusTelemetry,
    ) -> tuple[float, ...]:
        """轮询真实关节，全部到位后才允许进入下一个轨迹点。"""

        elapsed_seconds = 0.0
        settle_elapsed_seconds = 0.0
        settle_position_samples: list[tuple[float, ...]] = []
        while True:
            self._raise_if_stop_requested()
            step_telemetry = None
            try:
                with self._bus_lock:
                    _, present_positions = self._read_all_positions_locked(
                        follower_bus
                    )
                    step_telemetry = self._capture_telemetry_locked(
                        follower_bus,
                        phase="arm_step",
                    )
                    (
                        max_error_joint,
                        max_error_measured,
                        max_error_target,
                        max_error,
                    ) = self._largest_arm_position_error(
                        motor_names,
                        present_positions,
                        target_positions,
                    )
                    self._raise_if_arm_telemetry_unsafe(
                        start_telemetry,
                        step_telemetry,
                        follower_bus=follower_bus,
                        present_positions=present_positions,
                    )
            finally:
                if step_telemetry is not None:
                    self._notify_telemetry(step_telemetry)

            within_joint_tolerance = (
                max_error
                <= self.motion_config.joint_position_tolerance_degrees
            )
            if within_joint_tolerance:
                settle_position_samples.append(present_positions)
                if (
                    settle_elapsed_seconds
                    >= self.motion_config.final_settle_seconds
                ):
                    self._last_settle_report = self._build_settle_report(
                        motor_names,
                        settle_position_samples,
                        settle_elapsed_seconds,
                    )
                    return present_positions
            else:
                settle_elapsed_seconds = 0.0
                settle_position_samples.clear()

            if (
                elapsed_seconds
                >= self.motion_config.waypoint_timeout_seconds
            ):
                with self._bus_lock:
                    _, present_positions = self._read_all_positions_locked(
                        follower_bus
                    )
                    self._hold_all_positions(
                        follower_bus,
                        present_positions,
                    )
                (
                    max_error_joint,
                    max_error_measured,
                    max_error_target,
                    max_error,
                ) = self._largest_arm_position_error(
                    motor_names,
                    present_positions,
                    target_positions,
                )
                if within_joint_tolerance:
                    raise SO100PlusMotionConvergenceError(
                        f"关节已进入 "
                        f"{self.motion_config.joint_position_tolerance_degrees:.1f}°"
                        "容差，但在 "
                        f"{self.motion_config.waypoint_timeout_seconds:.1f} 秒内"
                        "没有连续稳定 "
                        f"{self.motion_config.final_settle_seconds:.2f} 秒，"
                        "已保持当前位置。"
                    )
                raise SO100PlusMotionConvergenceError(
                    f"关节 {max_error_joint} 在 "
                    f"{self.motion_config.waypoint_timeout_seconds:.1f} 秒内未到位："
                    f"目标 {max_error_target:.6f}°、实测 "
                    f"{max_error_measured:.6f}°，跟踪误差 "
                    f"{max_error:.6f}° 超过 "
                    f"{self.motion_config.joint_position_tolerance_degrees:.1f}°，"
                    "已保持当前位置。"
                )

            wait_seconds = min(
                self.motion_config.waypoint_poll_interval_seconds,
                self.motion_config.waypoint_timeout_seconds
                - elapsed_seconds,
            )
            stopped_while_waiting = self._wait(wait_seconds)
            elapsed_seconds += wait_seconds
            if within_joint_tolerance:
                settle_elapsed_seconds += wait_seconds
            if stopped_while_waiting or self._stop_requested.is_set():
                if not self._stop_requested.is_set():
                    with self._bus_lock:
                        _, present_positions = self._read_all_positions_locked(
                            follower_bus
                        )
                        self._hold_all_positions(
                            follower_bus,
                            present_positions,
                        )
                raise SO100PlusMotionStoppedError(
                    "手臂动作已被 stop() 取消。"
                )

    def _build_settle_report(
        self,
        motor_names: tuple[str, ...],
        position_samples: Sequence[tuple[float, ...]],
        duration_seconds: float,
    ) -> SO100PlusSettleReport:
        arm_samples = tuple(
            self._arm_values_by_name(motor_names, positions)
            for positions in position_samples
        )
        position_span_degrees = tuple(
            max(sample[index] for sample in arm_samples)
            - min(sample[index] for sample in arm_samples)
            for index in range(len(SO100_PLUS_ARM_JOINT_NAMES))
        )
        tcp_samples = tuple(
            self.kinematics.forward_position(
                self.kinematics.driver_degrees_to_model_radians(sample)
            )
            for sample in arm_samples
        )
        tcp_min_m = tuple(
            min(sample[index] for sample in tcp_samples)
            for index in range(3)
        )
        tcp_max_m = tuple(
            max(sample[index] for sample in tcp_samples)
            for index in range(3)
        )
        tcp_mean_m = tuple(
            sum(sample[index] for sample in tcp_samples) / len(tcp_samples)
            for index in range(3)
        )
        return SO100PlusSettleReport(
            motor_names=SO100_PLUS_ARM_JOINT_NAMES,
            position_samples_degrees=arm_samples,
            position_span_degrees=position_span_degrees,
            tcp_samples_m=tcp_samples,
            tcp_min_m=tcp_min_m,
            tcp_max_m=tcp_max_m,
            tcp_mean_m=tcp_mean_m,
            duration_seconds=duration_seconds,
        )

    def open_gripper(self) -> None:
        self._move_gripper_to(self.gripper_config.open_degrees)

    def close_gripper(self) -> None:
        self._move_gripper_to(self.gripper_config.close_degrees)

    def stop(self) -> None:
        if not self.is_connected:
            raise RuntimeError("停止操作前必须先显式连接机械臂。")

        with self._action_state_lock:
            self._stop_requested.set()
            # stop 在首条动作指令前到达时，不能把“保持”
            # 写入误计为动作；事件会使后续首条指令失败。
            should_write_hold = (
                not self._motion_action_active
                or self._motion_waypoint_written
            )
        follower_bus = self._follower_bus()
        with self._bus_lock:
            present_positions = follower_bus.read("Present_Position")
            if should_write_hold:
                follower_bus.write("Goal_Position", present_positions)
            telemetry = self._capture_telemetry_locked(
                follower_bus,
                phase="stopped",
            )
        self._notify_telemetry(telemetry)

    def disable_torque(self, *, emergency: bool = False) -> None:
        """在 follower_rest 正常释放力矩，或显式执行紧急释放。"""

        if not self.is_connected:
            raise RuntimeError("关闭力矩前必须先显式连接机械臂。")
        if not isinstance(emergency, bool):
            raise TypeError("emergency 必须是布尔值。")

        self._stop_requested.set()
        follower_bus = self._follower_bus()
        with self._bus_lock:
            if not emergency:
                self._raise_unless_at_storage_rest_locked(follower_bus)
            follower_bus.write("Torque_Enable", 0)
            follower_bus.write("Lock", 1)
            self._raise_unless_eprom_locked(
                follower_bus,
                tuple(follower_bus.motor_names),
            )
            torque_enabled = tuple(
                int(value)
                for value in follower_bus.read("Torque_Enable")
            )
            if any(torque_enabled):
                raise RuntimeError(
                    f"力矩未能全部关闭：{torque_enabled}；请立即物理断电。"
                )
            telemetry = self._capture_telemetry_locked(
                follower_bus,
                phase=(
                    "torque_disabled_emergency"
                    if emergency
                    else "torque_disabled"
                ),
            )
        self._notify_telemetry(telemetry)

    def _move_gripper_to(self, final_target_degrees: float) -> None:
        if not self.is_connected:
            raise RuntimeError("夹爪操作前必须先显式连接机械臂。")

        owns_action = self._begin_motion_action_if_needed()
        try:
            with self._motion_lock:
                self._raise_if_stop_requested()
                self._move_gripper_to_locked(final_target_degrees)
        finally:
            if owns_action:
                self.end_motion_action()

    def _move_gripper_to_locked(self, final_target_degrees: float) -> None:
        follower_bus = self._follower_bus()
        start_telemetry = None
        try:
            with self._bus_lock:
                position_degrees = _single_value(
                    follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
                )
                start_telemetry = self._capture_telemetry_locked(
                    follower_bus,
                    phase="gripper_start",
                )
                load = self._gripper_load(start_telemetry)
                self._raise_if_overloaded(follower_bus, position_degrees, load)
        finally:
            if start_telemetry is not None:
                self._notify_telemetry(start_telemetry)

        while (
            abs(final_target_degrees - position_degrees)
            > self.gripper_config.position_tolerance_degrees
        ):
            distance = final_target_degrees - position_degrees
            step = math.copysign(
                min(abs(distance), self.gripper_config.max_step_degrees),
                distance,
            )
            step_target_degrees = position_degrees + step
            with self._action_state_lock:
                self._raise_if_stop_requested()
                with self._bus_lock:
                    self._raise_if_stop_requested()
                    follower_bus.write(
                        "Goal_Position",
                        [step_target_degrees],
                        GRIPPER_MOTOR_NAME,
                    )
                self._motion_waypoint_written = True

            stopped_while_waiting = self._wait(
                self.gripper_config.settle_seconds
            )
            if stopped_while_waiting or self._stop_requested.is_set():
                raise SO100PlusMotionStoppedError("夹爪动作已被 stop() 取消。")

            step_telemetry = None
            try:
                with self._bus_lock:
                    position_degrees = _single_value(
                        follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
                    )
                    step_telemetry = self._capture_telemetry_locked(
                        follower_bus,
                        phase="gripper_step",
                    )
                    load = self._gripper_load(step_telemetry)
                    self._raise_if_overloaded(follower_bus, position_degrees, load)
                    if (
                        abs(position_degrees - step_target_degrees)
                        > self.gripper_config.position_tolerance_degrees
                    ):
                        self._hold_position(follower_bus, position_degrees)
                        raise SO100PlusGripperSafetyError(
                            f"夹爪目标 {step_target_degrees:.1f}° 与实测 "
                            f"{position_degrees:.1f}° 差距过大，已保持当前位置。"
                        )
            finally:
                if step_telemetry is not None:
                    self._notify_telemetry(step_telemetry)

    def _raise_if_stop_requested(self) -> None:
        if self._stop_requested.is_set():
            raise SO100PlusMotionStoppedError("夹爪动作已被 stop() 取消。")

    def _follower_bus(self):
        try:
            return self.robot.follower_arms[self.gripper_config.follower_name]
        except KeyError as error:
            raise RuntimeError(
                f"机器人中没有 follower "
                f"{self.gripper_config.follower_name!r}。"
            ) from error

    @staticmethod
    def _validated_pid_motor_names(
        motor_names: Sequence[str],
    ) -> tuple[str, ...]:
        names = tuple(motor_names)
        if not names:
            raise ValueError("PID 电机列表不能为空。")
        if len(set(names)) != len(names):
            raise ValueError("PID 电机列表不能包含重复名称。")
        invalid = tuple(
            name
            for name in names
            if name not in SO100_PLUS_ARM_JOINT_NAMES
        )
        if invalid:
            raise ValueError(
                f"PID 只允许配置六个手臂关节：{invalid}。"
            )
        return names

    @staticmethod
    def _read_pid_gains_locked(
        follower_bus,
        motor_names: Sequence[str],
    ) -> dict[str, SO100PlusPIDGains]:
        return {
            name: SO100PlusPIDGains(
                p=int(
                    _single_value(
                        follower_bus.read("P_Coefficient", name)
                    )
                ),
                i=int(
                    _single_value(
                        follower_bus.read("I_Coefficient", name)
                    )
                ),
                d=int(
                    _single_value(
                        follower_bus.read("D_Coefficient", name)
                    )
                ),
            )
            for name in motor_names
        }

    def _saved_arm_pid_gains(self) -> dict[str, SO100PlusPIDGains]:
        profile = SO100_PLUS_REAL_HARDWARE_PROFILE
        return {
            name: SO100PlusPIDGains(
                p=(
                    self.elbow_p_coefficient
                    if name == ELBOW_MOTOR_NAME
                    else (
                        self.wrist_pitch_p_coefficient
                        if name == WRIST_PITCH_MOTOR_NAME
                        else profile.other_motor_p_coefficient
                    )
                ),
                i=(
                    profile.tuned_motor_i_coefficient
                    if name in TUNED_PID_MOTOR_NAMES
                    else 0
                ),
                d=(
                    profile.tuned_motor_d_coefficient
                    if name in TUNED_PID_MOTOR_NAMES
                    else 0
                ),
            )
            for name in SO100_PLUS_ARM_JOINT_NAMES
        }

    def _write_pid_gains_locked(
        self,
        follower_bus,
        requested: Mapping[str, SO100PlusPIDGains],
    ) -> dict[str, SO100PlusPIDGains]:
        names = tuple(requested)
        previous = self._read_pid_gains_locked(follower_bus, names)
        touched: list[str] = []
        try:
            for name in names:
                before = previous[name]
                after = requested[name]
                if before == after:
                    continue
                follower_bus.write("Lock", 0, name)
                touched.append(name)
                for register, old_value, new_value in (
                    ("P_Coefficient", before.p, after.p),
                    ("I_Coefficient", before.i, after.i),
                    ("D_Coefficient", before.d, after.d),
                ):
                    if old_value != new_value:
                        follower_bus.write(register, new_value, name)
                follower_bus.write("Lock", 1, name)
                touched.remove(name)
        finally:
            for name in touched:
                follower_bus.write("Lock", 1, name)

        actual = self._read_pid_gains_locked(follower_bus, names)
        if actual != requested:
            raise RuntimeError(
                f"PID 写入失败：期望 {requested}，实测 {actual}。"
            )
        self._raise_unless_eprom_locked(follower_bus, names)
        return previous

    @staticmethod
    def _raise_unless_eprom_locked(
        follower_bus,
        motor_names: Sequence[str],
    ) -> None:
        unlocked = tuple(
            name
            for name in motor_names
            if int(_single_value(follower_bus.read("Lock", name))) != 1
        )
        if unlocked:
            raise RuntimeError(
                f"电机 EEPROM 写锁未能关闭：{unlocked}；"
                "已停止后续操作。"
            )

    def _raise_unless_at_storage_rest_locked(self, follower_bus) -> None:
        motor_names, positions = self._read_all_positions_locked(follower_bus)
        arm_positions = self._arm_values_by_name(motor_names, positions)
        profile = SO100_PLUS_REAL_HARDWARE_PROFILE
        errors = tuple(
            abs(actual - expected)
            for actual, expected in zip(
                arm_positions,
                profile.storage_rest_driver_degrees,
                strict=True,
            )
        )
        violating = tuple(
            index
            for index, (error, tolerance) in enumerate(
                zip(
                    errors,
                    profile.storage_rest_tolerances_degrees,
                    strict=True,
                )
            )
            if error > tolerance
        )
        if not violating:
            return

        index = max(violating, key=errors.__getitem__)
        name = SO100_PLUS_ARM_JOINT_NAMES[index]
        raise SO100PlusTorqueReleaseSafetyError(
            f"普通关闭力矩已拒绝：当前位置不是已验证的 follower_rest；"
            f"{name} 偏差 {errors[index]:.3f}° 超过 "
            f"{profile.storage_rest_tolerances_degrees[index]:.1f}°。"
            "请先受控返回 follower_rest；仅在过温、过载、碰撞或人工"
            "急停时显式使用 emergency=True，并托住机械臂。"
        )

    @staticmethod
    def _read_arm_driver_degrees_locked(
        follower_bus,
    ) -> tuple[float, ...]:
        motor_names, positions = SO100PlusAdapter._read_all_positions_locked(
            follower_bus
        )
        return SO100PlusAdapter._arm_values_by_name(
            motor_names,
            positions,
        )

    @staticmethod
    def _read_all_positions_locked(
        follower_bus,
    ) -> tuple[tuple[str, ...], tuple[float, ...]]:
        motor_names = tuple(follower_bus.motor_names)
        positions = _values_tuple(follower_bus.read("Present_Position"))
        if len(motor_names) != len(positions):
            raise RuntimeError(
                "follower 返回的电机名称数量与当前位置数量不一致。"
            )
        if len(set(motor_names)) != len(motor_names):
            raise RuntimeError("follower 返回了重复的电机名称。")

        required_names = SO100_PLUS_ARM_JOINT_NAMES + (GRIPPER_MOTOR_NAME,)
        missing_names = tuple(
            name
            for name in required_names
            if name not in motor_names
        )
        if missing_names:
            raise RuntimeError(
                f"follower 缺少关节：{', '.join(missing_names)}。"
            )
        return motor_names, positions

    @staticmethod
    def _arm_values_by_name(
        motor_names: tuple[str, ...],
        values: tuple[float, ...],
    ) -> tuple[float, ...]:
        value_by_name = dict(zip(motor_names, values, strict=True))
        return tuple(
            value_by_name[name]
            for name in SO100_PLUS_ARM_JOINT_NAMES
        )

    def _compose_arm_target(
        self,
        motor_names: tuple[str, ...],
        held_positions: tuple[float, ...],
        waypoint_radians: tuple[float, ...],
    ) -> tuple[float, ...]:
        validated_waypoint = self.motion_limits.joints.validate_position(
            waypoint_radians
        )
        arm_driver_degrees = _values_tuple(
            self.kinematics.model_radians_to_driver_degrees(
                validated_waypoint
            )
        )
        if len(arm_driver_degrees) != len(SO100_PLUS_ARM_JOINT_NAMES):
            raise SO100PlusArmSafetyError(
                "运动学转换没有返回六个手臂关节目标。"
            )
        if not all(math.isfinite(value) for value in arm_driver_degrees):
            raise SO100PlusArmSafetyError(
                "运动学转换返回了非有限手臂关节目标。"
            )

        target_by_name = dict(
            zip(motor_names, held_positions, strict=True)
        )
        target_by_name.update(
            zip(
                SO100_PLUS_ARM_JOINT_NAMES,
                arm_driver_degrees,
                strict=True,
            )
        )
        return tuple(target_by_name[name] for name in motor_names)

    @staticmethod
    def _max_arm_position_error(
        motor_names: tuple[str, ...],
        measured_positions: tuple[float, ...],
        target_positions: tuple[float, ...],
    ) -> float:
        return SO100PlusAdapter._largest_arm_position_error(
            motor_names,
            measured_positions,
            target_positions,
        )[3]

    @staticmethod
    def _largest_arm_position_error(
        motor_names: tuple[str, ...],
        measured_positions: tuple[float, ...],
        target_positions: tuple[float, ...],
    ) -> tuple[str, float, float, float]:
        measured_arm = SO100PlusAdapter._arm_values_by_name(
            motor_names,
            measured_positions,
        )
        target_arm = SO100PlusAdapter._arm_values_by_name(
            motor_names,
            target_positions,
        )
        errors = tuple(
            (name, measured, target, abs(measured - target))
            for name, measured, target in zip(
                SO100_PLUS_ARM_JOINT_NAMES,
                measured_arm,
                target_arm,
                strict=True,
            )
        )
        return max(errors, key=lambda item: item[3])

    def _raise_if_arm_telemetry_unsafe(
        self,
        _start: SO100PlusTelemetry,
        current: SO100PlusTelemetry,
        *,
        follower_bus=None,
        present_positions: tuple[float, ...] | None = None,
    ) -> None:
        reason = None
        critical_load_samples = tuple(
            (load, motor_name)
            for motor_name, load in zip(
                current.motor_names,
                current.load_magnitude,
                strict=True,
            )
            if load >= self.motion_config.critical_load_limit
        )
        if critical_load_samples:
            maximum_load, motor_name = max(critical_load_samples)
            reason = (
                f"手臂电机 {motor_name} 负载 "
                f"{maximum_load:.1f} 达到紧急限制 "
                f"{self.motion_config.critical_load_limit:.1f}"
            )
        else:
            next_load_streak: dict[str, int] = {}
            confirmed_load_samples: list[tuple[float, str, int]] = []
            for motor_name, load in zip(
                current.motor_names,
                current.load_magnitude,
                strict=True,
            ):
                if load < self.motion_config.load_limit:
                    continue
                count = self._load_limit_streak.get(motor_name, 0) + 1
                next_load_streak[motor_name] = count
                if count >= self.motion_config.load_confirmation_samples:
                    confirmed_load_samples.append(
                        (load, motor_name, count)
                    )
            self._load_limit_streak = next_load_streak
            if confirmed_load_samples:
                maximum_load, motor_name, count = max(
                    confirmed_load_samples
                )
                reason = (
                    f"手臂电机 {motor_name} 负载 "
                    f"{maximum_load:.1f} 连续 {count} 次达到限制 "
                    f"{self.motion_config.load_limit:.1f}"
                )

        if reason is None:
            critical_samples = tuple(
                (temperature, motor_name)
                for motor_name, temperature in zip(
                    current.motor_names,
                    current.temperature_raw,
                    strict=True,
                )
                if (
                    temperature
                    >= self.motion_config.critical_temperature_celsius
                )
            )
            if critical_samples:
                maximum_temperature, motor_name = max(critical_samples)
                reason = (
                    f"手臂电机 {motor_name} 温度 "
                    f"{maximum_temperature:.1f}°C 达到紧急限制 "
                    f"{self.motion_config.critical_temperature_celsius:.1f}°C"
                )
            else:
                next_streak: dict[str, int] = {}
                confirmed_samples: list[tuple[float, str, int]] = []
                for motor_name, temperature in zip(
                    current.motor_names,
                    current.temperature_raw,
                    strict=True,
                ):
                    if (
                        temperature
                        < self.motion_config.max_temperature_celsius
                    ):
                        continue
                    count = (
                        self._temperature_limit_streak.get(motor_name, 0)
                        + 1
                    )
                    next_streak[motor_name] = count
                    if (
                        count
                        >= self.motion_config.temperature_confirmation_samples
                    ):
                        confirmed_samples.append(
                            (temperature, motor_name, count)
                        )
                self._temperature_limit_streak = next_streak
                if confirmed_samples:
                    maximum_temperature, motor_name, count = max(
                        confirmed_samples
                    )
                    reason = (
                        f"手臂电机 {motor_name} 温度 "
                        f"{maximum_temperature:.1f}°C 连续 {count} 次达到限制 "
                        f"{self.motion_config.max_temperature_celsius:.1f}°C"
                    )
        if reason is None:
            return
        if follower_bus is not None and present_positions is not None:
            self._hold_all_positions(follower_bus, present_positions)
            reason += "，已保持当前位置"
        raise SO100PlusArmSafetyError(reason + "。")

    @staticmethod
    def _hold_all_positions(
        follower_bus,
        positions: tuple[float, ...],
    ) -> None:
        follower_bus.write("Goal_Position", list(positions))

    def _capture_telemetry_locked(
        self,
        follower_bus,
        *,
        phase: str,
    ) -> SO100PlusTelemetry:
        telemetry = SO100PlusTelemetry(
            phase=phase,
            motor_names=tuple(follower_bus.motor_names),
            voltage_raw=_values_tuple(follower_bus.read("Present_Voltage")),
            current_raw=_values_tuple(follower_bus.read("Present_Current")),
            load_magnitude=tuple(
                _load_magnitude(value)
                for value in _values_tuple(follower_bus.read("Present_Load"))
            ),
            temperature_raw=_values_tuple(
                follower_bus.read("Present_Temperature")
            ),
        )
        self._telemetry_history.append(telemetry)
        return telemetry

    def _notify_telemetry(self, telemetry: SO100PlusTelemetry) -> None:
        if self._on_telemetry is not None:
            self._on_telemetry(telemetry)

    @staticmethod
    def _gripper_load(telemetry: SO100PlusTelemetry) -> float:
        try:
            gripper_index = telemetry.motor_names.index(GRIPPER_MOTOR_NAME)
        except ValueError as error:
            raise RuntimeError(
                f"follower 中没有夹爪舍机 {GRIPPER_MOTOR_NAME!r}。"
            ) from error
        return telemetry.load_magnitude[gripper_index]

    def _raise_if_overloaded(self, follower_bus, position_degrees: float, load: float) -> None:
        if load <= self.gripper_config.load_limit:
            return
        self._hold_position(follower_bus, position_degrees)
        raise SO100PlusGripperSafetyError(
            f"夹爪负载 {load:.1f} 超过限制 "
            f"{self.gripper_config.load_limit:.1f}，已保持当前位置。"
        )

    @staticmethod
    def _hold_position(follower_bus, position_degrees: float) -> None:
        follower_bus.write(
            "Goal_Position",
            [position_degrees],
            GRIPPER_MOTOR_NAME,
        )


def _single_value(values) -> float:
    if hasattr(values, "item"):
        return float(values.item())
    return float(values[0])


def _values_tuple(values) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def _load_magnitude(value: float) -> float:
    magnitude = abs(value)
    if magnitude >= 1024:
        magnitude = abs(1024 - magnitude)
    return magnitude
