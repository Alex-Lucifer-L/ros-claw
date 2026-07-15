"""SO-100 Plus 手动硬件检查逻辑。"""

from dataclasses import dataclass
import time


GRIPPER_MOTOR_NAME = "gripper_joint"
MAX_GRIPPER_NUDGE_DEGREES = 5.0
MAX_GRIPPER_TEST_TARGET_DEGREES = 10.0
MAX_GRIPPER_TEST_TRAVEL_DEGREES = 20.0
GRIPPER_CYCLE_TARGETS = (
    0.0,
    10.0,
    20.0,
    30.0,
    40.0,
    50.0,
    60.0,
    50.0,
    40.0,
    30.0,
    20.0,
    10.0,
    0.0,
    -10.0,
    0.0,
    10.0,
)
GRIPPER_LOAD_LIMIT = 300.0
GRIPPER_POSITION_TOLERANCE_DEGREES = 3.0


@dataclass(frozen=True)
class GripperNudgeResult:
    start_degrees: float
    target_degrees: float
    final_degrees: float


@dataclass(frozen=True)
class GripperCycleStep:
    target_degrees: float
    position_degrees: float
    load: float
    current: float
    temperature: float


@dataclass(frozen=True)
class GripperCycleResult:
    start_degrees: float
    steps: tuple[GripperCycleStep, ...]


class GripperCycleSafetyError(RuntimeError):
    """夹爪循环触发安全停止。"""


def _single_value(values) -> float:
    if hasattr(values, "item"):
        return float(values.item())
    return float(values[0])


def _load_magnitude(value: float) -> float:
    magnitude = abs(value)
    if magnitude >= 1024:
        magnitude = abs(1024 - magnitude)
    return magnitude


def _close_communication(robot, follower_bus) -> None:
    if getattr(robot, "is_connected", False):
        robot.disconnect()
    elif getattr(follower_bus, "is_connected", False):
        # connect() 中途失败时，Robot 可能还没来得及标记为已连接。
        follower_bus.disconnect()


def read_present_positions_once(robot, follower_name: str):
    """连接一次、读取一次位置，并在结束时关闭通信。"""

    follower_bus = robot.follower_arms[follower_name]
    try:
        robot.connect()
        return follower_bus.read("Present_Position")
    finally:
        _close_communication(robot, follower_bus)


def nudge_gripper_open_once(
    robot,
    follower_name: str,
    delta_degrees: float,
    *,
    settle_seconds: float = 0.5,
    sleep_func=time.sleep,
) -> GripperNudgeResult:
    """只让夹爪向已确认的打开方向微动最多 5 度。"""

    if not 0 < delta_degrees <= MAX_GRIPPER_NUDGE_DEGREES:
        raise ValueError(
            f"夹爪微动必须大于 0 且不超过 {MAX_GRIPPER_NUDGE_DEGREES} 度。"
        )
    if settle_seconds < 0:
        raise ValueError("settle_seconds 不能为负数。")

    follower_bus = robot.follower_arms[follower_name]
    try:
        robot.connect()
        start_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        target_degrees = start_degrees + delta_degrees
        follower_bus.write(
            "Goal_Position",
            [target_degrees],
            GRIPPER_MOTOR_NAME,
        )
        sleep_func(settle_seconds)
        final_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        return GripperNudgeResult(
            start_degrees=start_degrees,
            target_degrees=target_degrees,
            final_degrees=final_degrees,
        )
    finally:
        _close_communication(robot, follower_bus)


def move_gripper_to_test_target_once(
    robot,
    follower_name: str,
    target_degrees: float,
    *,
    settle_seconds: float = 2.5,
    sleep_func=time.sleep,
) -> GripperNudgeResult:
    """只让夹爪向打开方向移动到不超过 10 度的测试目标。"""

    if target_degrees > MAX_GRIPPER_TEST_TARGET_DEGREES:
        raise ValueError(
            f"夹爪测试目标不能超过 {MAX_GRIPPER_TEST_TARGET_DEGREES} 度。"
        )
    if settle_seconds < 0:
        raise ValueError("settle_seconds 不能为负数。")

    follower_bus = robot.follower_arms[follower_name]
    try:
        robot.connect()
        start_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        travel_degrees = target_degrees - start_degrees
        if not 0 < travel_degrees <= MAX_GRIPPER_TEST_TRAVEL_DEGREES:
            raise ValueError(
                "夹爪测试只能向打开方向移动，且单次行程不能超过 20.0 度。"
            )

        follower_bus.write(
            "Goal_Position",
            [target_degrees],
            GRIPPER_MOTOR_NAME,
        )
        sleep_func(settle_seconds)
        final_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        return GripperNudgeResult(
            start_degrees=start_degrees,
            target_degrees=target_degrees,
            final_degrees=final_degrees,
        )
    finally:
        _close_communication(robot, follower_bus)


def validate_gripper_cycle_once(
    robot,
    follower_name: str,
    *,
    settle_seconds: float = 2.5,
    sleep_func=time.sleep,
    on_step=None,
) -> GripperCycleResult:
    """一次连接内逐级验证夹爪打开、关闭和重复打开。"""

    if settle_seconds < 0:
        raise ValueError("settle_seconds 不能为负数。")

    follower_bus = robot.follower_arms[follower_name]
    steps = []
    try:
        robot.connect()
        start_degrees = _single_value(
            follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
        )
        if not -10.0 <= start_degrees <= 10.0:
            raise GripperCycleSafetyError(
                f"夹爪起点 {start_degrees:.4f}° 不在预期 rest 范围 [-10°, 10°]。"
            )

        baseline_load = _load_magnitude(
            _single_value(follower_bus.read("Present_Load", GRIPPER_MOTOR_NAME))
        )
        if baseline_load > GRIPPER_LOAD_LIMIT:
            raise GripperCycleSafetyError(
                f"夹爪初始负载 {baseline_load:.1f} 超过限制 {GRIPPER_LOAD_LIMIT:.1f}。"
            )

        # 当起点已大于 0 度时，从下一个更大的打开目标开始；
        # 完整保留后续关闭和重复打开阶段。
        opening_targets = tuple(
            target
            for target in GRIPPER_CYCLE_TARGETS[:7]
            if target > start_degrees
        )
        targets = opening_targets + GRIPPER_CYCLE_TARGETS[7:]

        for target_degrees in targets:
            follower_bus.write(
                "Goal_Position",
                [target_degrees],
                GRIPPER_MOTOR_NAME,
            )
            sleep_func(settle_seconds)

            position_degrees = _single_value(
                follower_bus.read("Present_Position", GRIPPER_MOTOR_NAME)
            )
            load = _load_magnitude(
                _single_value(follower_bus.read("Present_Load", GRIPPER_MOTOR_NAME))
            )
            current = _single_value(
                follower_bus.read("Present_Current", GRIPPER_MOTOR_NAME)
            )
            temperature = _single_value(
                follower_bus.read("Present_Temperature", GRIPPER_MOTOR_NAME)
            )
            step = GripperCycleStep(
                target_degrees=target_degrees,
                position_degrees=position_degrees,
                load=load,
                current=current,
                temperature=temperature,
            )
            steps.append(step)
            if on_step is not None:
                on_step(step)

            if load > GRIPPER_LOAD_LIMIT:
                follower_bus.write(
                    "Goal_Position",
                    [position_degrees],
                    GRIPPER_MOTOR_NAME,
                )
                raise GripperCycleSafetyError(
                    f"目标 {target_degrees:.1f}° 时负载 {load:.1f} 超过限制。"
                )
            if abs(position_degrees - target_degrees) > GRIPPER_POSITION_TOLERANCE_DEGREES:
                follower_bus.write(
                    "Goal_Position",
                    [position_degrees],
                    GRIPPER_MOTOR_NAME,
                )
                raise GripperCycleSafetyError(
                    f"目标 {target_degrees:.1f}° 与实测 {position_degrees:.1f}° 相差超过 3°。"
                )

        return GripperCycleResult(
            start_degrees=start_degrees,
            steps=tuple(steps),
        )
    finally:
        _close_communication(robot, follower_bus)
