"""SO-100 Plus 机械臂适配器。"""

from dataclasses import dataclass
import math
from threading import Event, Lock
from typing import Callable

from rosclaw_mini.arm.base import ArmAdapter


GRIPPER_MOTOR_NAME = "gripper_joint"


class SO100PlusGripperSafetyError(RuntimeError):
    """夹爪在执行途中超出已验证的安全条件。"""


class SO100PlusMotionStoppedError(RuntimeError):
    """适配器正在执行的动作已被 stop() 取消。"""


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
    max_step_degrees: float = 10.0
    settle_seconds: float = 2.5
    load_limit: float = 300.0
    position_tolerance_degrees: float = 3.0
    runtime_acceleration: int = 35

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


class SO100PlusAdapter(ArmAdapter):
    """SO-100 Plus 的统一原子操作适配器。"""

    def __init__(
        self,
        robot,
        gripper_config: SO100PlusGripperConfig,
        *,
        wait_func: Callable[[float], bool | None] | None = None,
        on_telemetry: Callable[[SO100PlusTelemetry], None] | None = None,
    ):
        self.robot = robot
        self.gripper_config = gripper_config
        self._stop_requested = Event()
        self._bus_lock = Lock()
        self._motion_lock = Lock()
        self._wait = wait_func or self._stop_requested.wait
        self._on_telemetry = on_telemetry
        self._telemetry_history: list[SO100PlusTelemetry] = []

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    @property
    def telemetry_history(self) -> tuple[SO100PlusTelemetry, ...]:
        return tuple(self._telemetry_history)

    def connect(self) -> None:
        if not self.robot.is_connected:
            self.robot.connect()
            follower_bus = self._follower_bus()
            with self._bus_lock:
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
            self._notify_telemetry(telemetry)

    def disconnect(self) -> None:
        if self.robot.is_connected:
            self.robot.disconnect()

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
    ) -> None:
        raise NotImplementedError("SO100 Plus 驱动接口的移动功能尚未实现。")

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
