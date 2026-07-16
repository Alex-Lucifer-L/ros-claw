"""SO-100 Plus 只读运动学诊断，不具备硬件写入能力。"""

from dataclasses import dataclass
from itertools import product
import math
from numbers import Real

from rosclaw_mini.arm.kinematics import KinematicsError
from rosclaw_mini.safety.limits import (
    LimitViolationError,
    SO100_PLUS_ARM_JOINT_NAMES,
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS,
)


GRIPPER_MOTOR_NAME = "gripper_joint"
MODEL_PREVIEW_MAX_STEP_RADIANS = 0.1


@dataclass(frozen=True)
class MotionPreviewResult:
    """只读计算候选，永远不代表已经批准执行。"""

    current_driver_degrees: tuple[float, ...]
    current_all_driver_degrees: tuple[float, ...]
    target_driver_degrees: tuple[float, ...]
    current_joint_radians: tuple[float, ...]
    target_joint_radians: tuple[float, ...]
    joint_delta_radians: tuple[float, ...]
    current_position_m: tuple[float, float, float]
    target_position_m: tuple[float, float, float]
    max_joint_delta_radians: float
    model_step_count: int

    @property
    def is_approved_for_execution(self) -> bool:
        return False


@dataclass(frozen=True)
class LocalMotionGridPoint:
    offset_m: tuple[float, float, float]
    preview: MotionPreviewResult | None = None
    rejection_reason: str | None = None

    @property
    def is_candidate(self) -> bool:
        return self.preview is not None and self.rejection_reason is None


@dataclass(frozen=True)
class LocalMotionGridResult:
    current_position_m: tuple[float, float, float]
    points: tuple[LocalMotionGridPoint, ...]

    @property
    def is_approved_for_execution(self) -> bool:
        return False


class MotionPreviewSafetyError(RuntimeError):
    """当前状态或候选违反已确认的只读诊断边界。"""


def _finite_values(values, *, expected_length: int, label: str):
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label} 必须包含 {expected_length} 个有限数值。")
    try:
        result = tuple(values)
    except TypeError as error:
        raise ValueError(
            f"{label} 必须包含 {expected_length} 个有限数值。"
        ) from error
    if len(result) != expected_length:
        raise ValueError(f"{label} 必须包含 {expected_length} 个有限数值。")
    if any(
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(float(value))
        for value in result
    ):
        raise ValueError(f"{label} 必须包含 {expected_length} 个有限数值。")
    return tuple(float(value) for value in result)


def build_symmetric_local_grid_offsets(
    *,
    half_extent_mm,
    step_mm: float,
) -> tuple[tuple[float, float, float], ...]:
    """根据显式半宽和步长生成不包含中心点的对称局部网格。"""

    half_extents = _finite_values(
        half_extent_mm,
        expected_length=3,
        label="网格半宽",
    )
    if any(value < 0 for value in half_extents) or not any(half_extents):
        raise ValueError("网格半宽不能为负数，并且至少一个方向必须大于 0。")
    if (
        isinstance(step_mm, bool)
        or not isinstance(step_mm, Real)
        or not math.isfinite(float(step_mm))
        or float(step_mm) <= 0
    ):
        raise ValueError("网格步长必须是有限正数。")
    step = float(step_mm)

    axes_mm = []
    for half_extent in half_extents:
        if half_extent == 0:
            axes_mm.append((0.0,))
            continue
        step_count = half_extent / step
        rounded_step_count = round(step_count)
        if not math.isclose(step_count, rounded_step_count, abs_tol=1e-9):
            raise ValueError("每个非零网格半宽必须是步长的整数倍。")
        axes_mm.append(
            tuple(
                index * step
                for index in range(
                    -rounded_step_count,
                    rounded_step_count + 1,
                )
            )
        )

    offsets = tuple(
        tuple(value / 1000.0 for value in offset_mm)
        for offset_mm in product(*axes_mm)
        if any(value != 0 for value in offset_mm)
    )
    if len(offsets) > 5000:
        raise ValueError("网格点数量不能超过 5000。")
    return offsets


def _validated_current_driver_degrees(motor_names, positions):
    names = tuple(motor_names)
    values = tuple(float(value) for value in positions)
    if len(names) != len(values):
        raise RuntimeError("电机名称数量与当前位置数量不一致。")
    if len(set(names)) != len(names):
        raise RuntimeError("电机名称不能重复。")

    position_by_name = dict(zip(names, values, strict=True))
    required_names = SO100_PLUS_ARM_JOINT_NAMES + (GRIPPER_MOTOR_NAME,)
    missing_names = tuple(
        name for name in required_names if name not in position_by_name
    )
    if missing_names:
        raise RuntimeError(f"缺少关节：{', '.join(missing_names)}。")

    current = tuple(
        position_by_name[name] for name in SO100_PLUS_ARM_JOINT_NAMES
    )
    try:
        SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.validate(
            current[0],
            "当前底座关节",
        )
    except LimitViolationError as error:
        raise MotionPreviewSafetyError(str(error)) from error
    return names, values, current


def _calculate_candidate(
    motor_names,
    positions,
    kinematics,
    offset_m,
) -> MotionPreviewResult:
    names, values, current_driver = _validated_current_driver_degrees(
        motor_names,
        positions,
    )
    offset = _finite_values(
        offset_m,
        expected_length=3,
        label="网格偏移",
    )
    current_joints = tuple(
        kinematics.driver_degrees_to_model_radians(current_driver)
    )
    current_position = tuple(kinematics.forward_position(current_joints))
    target_position = tuple(
        current + delta
        for current, delta in zip(current_position, offset, strict=True)
    )
    target_joints = tuple(
        kinematics.solve_position(current_joints, target_position)
    )
    joint_deltas = tuple(
        target - current
        for current, target in zip(
            current_joints,
            target_joints,
            strict=True,
        )
    )
    max_joint_delta = max(abs(delta) for delta in joint_deltas)
    target_driver = tuple(
        kinematics.model_radians_to_driver_degrees(target_joints)
    )
    if len(target_driver) != len(SO100_PLUS_ARM_JOINT_NAMES):
        raise MotionPreviewSafetyError("运动学没有返回六个关节驱动目标。")
    try:
        SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.validate(
            target_driver[0],
            "底座关节候选",
        )
    except LimitViolationError as error:
        raise MotionPreviewSafetyError(str(error)) from error

    step_count = (
        math.ceil(max_joint_delta / MODEL_PREVIEW_MAX_STEP_RADIANS)
        if max_joint_delta > 0
        else 0
    )
    return MotionPreviewResult(
        current_driver_degrees=current_driver,
        current_all_driver_degrees=values,
        target_driver_degrees=target_driver,
        current_joint_radians=current_joints,
        target_joint_radians=target_joints,
        joint_delta_radians=joint_deltas,
        current_position_m=current_position,
        target_position_m=target_position,
        max_joint_delta_radians=max_joint_delta,
        model_step_count=step_count,
    )


def _close_communication(robot, follower_bus) -> None:
    if getattr(robot, "is_connected", False):
        robot.disconnect()
    elif getattr(follower_bus, "is_connected", False):
        follower_bus.disconnect()


def preview_local_motion_grid_once(
    robot,
    follower_name: str,
    kinematics,
    *,
    offsets_m,
) -> LocalMotionGridResult:
    """只读一次当前位置，在内存中计算多个候选并关闭通信。"""

    try:
        offsets = tuple(
            _finite_values(
                offset,
                expected_length=3,
                label="网格偏移",
            )
            for offset in offsets_m
        )
    except TypeError as error:
        raise ValueError("网格偏移必须是至少一个三维有限向量。") from error
    if not offsets:
        raise ValueError("网格偏移必须至少包含一个点。")

    follower_bus = robot.follower_arms[follower_name]
    try:
        robot.connect()
        torque_enabled = tuple(
            int(value) for value in follower_bus.read("Torque_Enable")
        )
        if any(torque_enabled):
            raise MotionPreviewSafetyError(
                f"只读网格预览要求扭矩全部关闭，当前为 {torque_enabled}。"
            )
        motor_names = tuple(follower_bus.motor_names)
        positions = tuple(
            float(value)
            for value in follower_bus.read("Present_Position")
        )
        _, _, current_driver = _validated_current_driver_degrees(
            motor_names,
            positions,
        )
        current_joints = kinematics.driver_degrees_to_model_radians(
            current_driver
        )
        current_position = tuple(
            kinematics.forward_position(current_joints)
        )

        points = []
        for offset in offsets:
            try:
                preview = _calculate_candidate(
                    motor_names,
                    positions,
                    kinematics,
                    offset,
                )
            except (KinematicsError, MotionPreviewSafetyError) as error:
                points.append(
                    LocalMotionGridPoint(
                        offset_m=offset,
                        rejection_reason=str(error),
                    )
                )
            else:
                points.append(
                    LocalMotionGridPoint(
                        offset_m=offset,
                        preview=preview,
                    )
                )
        return LocalMotionGridResult(
            current_position_m=current_position,
            points=tuple(points),
        )
    finally:
        _close_communication(robot, follower_bus)
