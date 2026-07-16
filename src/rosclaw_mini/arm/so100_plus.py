"""SO-100 Plus 机械臂适配器。"""

from dataclasses import dataclass
import math
from threading import Event, Lock
from typing import Callable, Mapping

from rosclaw_mini.arm.base import ArmAdapter
from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100PlusKinematics,
)
from rosclaw_mini.safety.limits import (
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
)


GRIPPER_MOTOR_NAME = "gripper_joint"
ELBOW_MOTOR_NAME = "ellbow_joint"


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


@dataclass(frozen=True)
class SO100PlusRealHardwareProfile:
    """上一次 right_follower 真机验证后保存的运行配置。"""

    other_motor_p_coefficient: int = 16
    elbow_p_coefficient: int = 64
    runtime_acceleration: int = 35
    gripper_max_step_degrees: float = 10.0
    gripper_settle_seconds: float = 2.5
    gripper_position_tolerance_degrees: float = 3.0
    waypoint_timeout_seconds: float = 8.0
    waypoint_poll_interval_seconds: float = 0.25
    joint_position_tolerance_degrees: float = 1.5
    cartesian_tolerance_m: float = 0.006
    load_limit: float = 300.0
    max_temperature_celsius: float = 60.0
    stream_frequency_hz: float = 20.0
    stream_max_joint_speed_degrees_per_second: float = 20.0
    stream_tracking_error_limit_degrees: float = 5.0
    stream_telemetry_interval_seconds: float = 0.25


SO100_PLUS_REAL_HARDWARE_PROFILE = SO100PlusRealHardwareProfile()
DEFAULT_ELBOW_P_COEFFICIENT = (
    SO100_PLUS_REAL_HARDWARE_PROFILE.elbow_p_coefficient
)
RESTORED_ELBOW_P_COEFFICIENT = (
    SO100_PLUS_REAL_HARDWARE_PROFILE.other_motor_p_coefficient
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
    load_limit: float = SO100_PLUS_REAL_HARDWARE_PROFILE.load_limit
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
    joint_position_tolerance_degrees: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.joint_position_tolerance_degrees
    )
    cartesian_tolerance_m: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.cartesian_tolerance_m
    )
    load_limit: float = SO100_PLUS_REAL_HARDWARE_PROFILE.load_limit
    max_temperature_celsius: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.max_temperature_celsius
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
    stream_telemetry_interval_seconds: float = (
        SO100_PLUS_REAL_HARDWARE_PROFILE.stream_telemetry_interval_seconds
    )

    def __post_init__(self) -> None:
        values = (
            self.waypoint_timeout_seconds,
            self.waypoint_poll_interval_seconds,
            self.joint_position_tolerance_degrees,
            self.cartesian_tolerance_m,
            self.load_limit,
            self.max_temperature_celsius,
            self.stream_max_joint_speed_degrees_per_second,
            self.stream_tracking_error_limit_degrees,
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
        if self.joint_position_tolerance_degrees < 0:
            raise ValueError("关节位置容差不能为负数。")
        if self.cartesian_tolerance_m <= 0:
            raise ValueError("夹爪 TCP 位置容差必须大于 0。")
        if self.load_limit <= 0:
            raise ValueError("手臂负载限制必须大于 0。")
        if self.max_temperature_celsius <= 0:
            raise ValueError("电机温度上限必须大于 0°C。")
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
        camera_map = dict(cameras or {})
        if getattr(robot, "cameras", {}):
            raise ValueError(
                "请将摄像头传给 SO100PlusAdapter(cameras=...)，"
                "不要直接放入 LeRobot Robot；否则机械臂会先于"
                "摄像头上力。"
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
        self.cameras = camera_map
        self._stop_requested = Event()
        self._bus_lock = Lock()
        self._motion_lock = Lock()
        self._wait = wait_func or self._stop_requested.wait
        self._on_telemetry = on_telemetry
        self._telemetry_history: list[SO100PlusTelemetry] = []
        self._last_motion_plan: JointMotionPlan | None = None

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
            self._connect_cameras_before_arm()
            try:
                self.robot.connect()
                with self._bus_lock:
                    # 提高肘关节刚度前先把目标同步到实测位置，避免旧目标
                    # 在 P 增益提高后引起跳动。
                    motor_names, present_positions = self._read_all_positions_locked(
                        follower_bus
                    )
                    follower_bus.write(
                        "Goal_Position",
                        list(present_positions),
                    )
                    # 明确恢复上次实机配置：除肘关节外 P=16，
                    # 肘关节 P=64。不依赖驱动库当前的隐式默认值。
                    follower_bus.write(
                        "P_Coefficient",
                        SO100_PLUS_REAL_HARDWARE_PROFILE.other_motor_p_coefficient,
                    )
                    follower_bus.write(
                        "P_Coefficient",
                        self.elbow_p_coefficient,
                        ELBOW_MOTOR_NAME,
                    )
                    actual_p_coefficients = _values_tuple(
                        follower_bus.read("P_Coefficient")
                    )
                    expected_p_coefficients = tuple(
                        self.elbow_p_coefficient
                        if name == ELBOW_MOTOR_NAME
                        else SO100_PLUS_REAL_HARDWARE_PROFILE.other_motor_p_coefficient
                        for name in motor_names
                    )
                    if actual_p_coefficients != expected_p_coefficients:
                        raise RuntimeError(
                            f"电机 P 增益写入失败：期望 "
                            f"{expected_p_coefficients}，实测 "
                            f"{actual_p_coefficients}。"
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
                        follower_bus.write(
                            "P_Coefficient",
                            SO100_PLUS_REAL_HARDWARE_PROFILE.other_motor_p_coefficient,
                        )
                finally:
                    if getattr(self.robot, "is_connected", False):
                        self.robot.disconnect()
                    elif getattr(follower_bus, "is_connected", False):
                        follower_bus.disconnect()
                    self._disconnect_cameras_after_arm()
                raise
            self._notify_telemetry(telemetry)

    def disconnect(self) -> None:
        try:
            if self.robot.is_connected:
                self.robot.disconnect()
        finally:
            self._disconnect_cameras_after_arm()

    def capture_camera_images(
        self,
        *,
        asynchronous: bool = True,
    ) -> dict[str, object]:
        """抓取 RGB/BGR HWC 图像，键名与 LeRobot observation 一致。"""

        if not self.robot.is_connected:
            raise RuntimeError("抓取图像前必须先显式连接适配器。")

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

    def _connect_cameras_before_arm(self) -> None:
        connected: list[object] = []
        for name, camera in self.cameras.items():
            if getattr(camera, "is_connected", False):
                raise RuntimeError(
                    f"摄像头 {name!r} 已被连接，适配器拒绝重复接管。"
                )
            try:
                camera.connect()
            except Exception as exc:
                for connected_camera in reversed(connected):
                    try:
                        connected_camera.disconnect()
                    except Exception:
                        pass
                raise RuntimeError(
                    f"摄像头 {name!r} 连接失败，机械臂保持未连接。"
                ) from exc
            connected.append(camera)

    def _disconnect_cameras_after_arm(self) -> None:
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

        with self._motion_lock:
            self._stop_requested.clear()
            plan = self._plan_move_to_locked(x, y, z)
            self._execute_motion_plan_locked(plan)

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

    def _execute_motion_plan_locked(self, plan: JointMotionPlan) -> None:
        if not plan.waypoints_radians:
            return

        follower_bus = self._follower_bus()
        with self._bus_lock:
            motor_names, held_positions = self._read_all_positions_locked(
                follower_bus
            )
            start_telemetry = self._capture_telemetry_locked(
                follower_bus,
                phase="arm_start",
            )
        self._notify_telemetry(start_telemetry)
        self._raise_if_arm_telemetry_unsafe(start_telemetry, start_telemetry)

        if self.motion_config.stream_frequency_hz is not None:
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
                with self._bus_lock:
                    self._raise_if_stop_requested()
                    follower_bus.write("Goal_Position", list(target_positions))

                final_positions = self._wait_for_arm_waypoint(
                    follower_bus,
                    motor_names,
                    target_positions,
                    start_telemetry,
                )

        self._validate_final_tcp_position(
            follower_bus,
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
        """用余弦缓入缓出连续发送关节目标，最后再等待精确到位。"""

        frequency_hz = self.motion_config.stream_frequency_hz
        interval_seconds = 1.0 / frequency_hz
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
        sample_count = max(1, math.ceil(duration_seconds * frequency_hz))
        telemetry_stride = max(
            1,
            round(
                self.motion_config.stream_telemetry_interval_seconds
                * frequency_hz
            ),
        )
        previous_waypoint = plan.current_joint_radians
        final_positions = held_positions
        final_target_positions = held_positions

        for sample_index in range(1, sample_count + 1):
            self._raise_if_stop_requested()
            fraction = sample_index / sample_count
            smooth_fraction = 0.5 - 0.5 * math.cos(math.pi * fraction)
            waypoint = tuple(
                current + (target - current) * smooth_fraction
                for current, target in zip(
                    plan.current_joint_radians,
                    plan.target_joint_radians,
                    strict=True,
                )
            )
            self.motion_limits.joints.validate_step(
                previous_waypoint,
                waypoint,
            )
            final_target_positions = self._compose_arm_target(
                motor_names,
                held_positions,
                waypoint,
            )
            with self._bus_lock:
                self._raise_if_stop_requested()
                follower_bus.write(
                    "Goal_Position",
                    list(final_target_positions),
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

            step_telemetry = None
            try:
                with self._bus_lock:
                    _, final_positions = self._read_all_positions_locked(
                        follower_bus
                    )
                    (
                        max_error_joint,
                        max_error_measured,
                        max_error_target,
                        max_error,
                    ) = self._largest_arm_position_error(
                        motor_names,
                        final_positions,
                        final_target_positions,
                    )
                    if (
                        max_error
                        > self.motion_config.stream_tracking_error_limit_degrees
                    ):
                        self._hold_all_positions(
                            follower_bus,
                            final_positions,
                        )
                        raise SO100PlusArmSafetyError(
                            f"流式轨迹关节 {max_error_joint} 目标 "
                            f"{max_error_target:.6f}°、实测 "
                            f"{max_error_measured:.6f}°，跟踪误差 "
                            f"{max_error:.6f}° 超过 "
                            f"{self.motion_config.stream_tracking_error_limit_degrees:.1f}°，"
                            "已保持当前位置。"
                        )
                    if (
                        sample_index % telemetry_stride == 0
                        or sample_index == sample_count
                    ):
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
            previous_waypoint = waypoint

        return self._wait_for_arm_waypoint(
            follower_bus,
            motor_names,
            final_target_positions,
            start_telemetry,
        )

    def _validate_final_tcp_position(
        self,
        follower_bus,
        motor_names: tuple[str, ...],
        final_positions: tuple[float, ...],
        plan: JointMotionPlan,
    ) -> None:
        """复算真实夹爪 TCP，并在最终误差超限时保持当前位置。"""

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
        cartesian_error_m = math.sqrt(
            sum(
                (actual - target) ** 2
                for actual, target in zip(
                    final_position_m,
                    plan.target_position_m,
                    strict=True,
                )
            )
        )
        if cartesian_error_m > self.motion_config.cartesian_tolerance_m:
            with self._bus_lock:
                self._hold_all_positions(follower_bus, final_positions)
            raise SO100PlusArmSafetyError(
                f"夹爪 TCP 位置误差 {cartesian_error_m * 1000:.6f} mm 超过 "
                f"{self.motion_config.cartesian_tolerance_m * 1000:.1f} mm，"
                "已保持当前位置。"
            )

    def _wait_for_arm_waypoint(
        self,
        follower_bus,
        motor_names: tuple[str, ...],
        target_positions: tuple[float, ...],
        start_telemetry: SO100PlusTelemetry,
    ) -> tuple[float, ...]:
        """轮询真实关节，全部到位后才允许进入下一个轨迹点。"""

        elapsed_seconds = 0.0
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

            if (
                max_error
                <= self.motion_config.joint_position_tolerance_degrees
            ):
                return present_positions

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
                raise SO100PlusArmSafetyError(
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

    def open_gripper(self) -> None:
        self._move_gripper_to(self.gripper_config.open_degrees)

    def close_gripper(self) -> None:
        self._move_gripper_to(self.gripper_config.close_degrees)

    def stop(self) -> None:
        if not self.is_connected:
            raise RuntimeError("停止操作前必须先显式连接机械臂。")

        self._stop_requested.set()
        follower_bus = self._follower_bus()
        with self._bus_lock:
            present_positions = follower_bus.read("Present_Position")
            follower_bus.write("Goal_Position", present_positions)
            telemetry = self._capture_telemetry_locked(
                follower_bus,
                phase="stopped",
            )
        self._notify_telemetry(telemetry)

    def disable_torque(self) -> None:
        """关闭全部 follower 力矩；机械臂会立即变软。"""

        if not self.is_connected:
            raise RuntimeError("关闭力矩前必须先显式连接机械臂。")

        self._stop_requested.set()
        follower_bus = self._follower_bus()
        with self._bus_lock:
            follower_bus.write("Torque_Enable", 0)
            follower_bus.write(
                "P_Coefficient",
                SO100_PLUS_REAL_HARDWARE_PROFILE.other_motor_p_coefficient,
            )
            torque_enabled = tuple(
                int(value)
                for value in follower_bus.read("Torque_Enable")
            )
            restored_p_coefficients = _values_tuple(
                follower_bus.read("P_Coefficient")
            )
            if any(torque_enabled):
                raise RuntimeError(
                    f"力矩未能全部关闭：{torque_enabled}；请立即物理断电。"
                )
            if any(
                value != RESTORED_ELBOW_P_COEFFICIENT
                for value in restored_p_coefficients
            ):
                raise RuntimeError(
                    f"电机 P 增益未全部恢复为 "
                    f"{RESTORED_ELBOW_P_COEFFICIENT}：实测 "
                    f"{restored_p_coefficients}；请立即物理断电。"
                )
            telemetry = self._capture_telemetry_locked(
                follower_bus,
                phase="torque_disabled",
            )
        self._notify_telemetry(telemetry)

    def _move_gripper_to(self, final_target_degrees: float) -> None:
        if not self.is_connected:
            raise RuntimeError("夹爪操作前必须先显式连接机械臂。")

        with self._motion_lock:
            self._stop_requested.clear()
            self._move_gripper_to_locked(final_target_degrees)

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
            with self._bus_lock:
                self._raise_if_stop_requested()
                follower_bus.write(
                    "Goal_Position",
                    [step_target_degrees],
                    GRIPPER_MOTOR_NAME,
                )

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
        start: SO100PlusTelemetry,
        current: SO100PlusTelemetry,
        *,
        follower_bus=None,
        present_positions: tuple[float, ...] | None = None,
    ) -> None:
        reason = None
        maximum_load = max(current.load_magnitude)
        if maximum_load > self.motion_config.load_limit:
            reason = (
                f"手臂负载 {maximum_load:.1f} 超过限制 "
                f"{self.motion_config.load_limit:.1f}"
            )
        else:
            maximum_temperature = max(current.temperature_raw)
            if maximum_temperature >= self.motion_config.max_temperature_celsius:
                reason = (
                    f"手臂温度 {maximum_temperature:.1f}°C 达到限制 "
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
