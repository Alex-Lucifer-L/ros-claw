"""机械臂运动限制的数据结构和纯计算检查。

本模块不读取机械臂、不写电机，也不把示例工作空间当成实机安全范围。
真实运动必须由调用方显式提供经过确认的 ``WorkspaceLimits`` 和
``JointLimits``。
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from numbers import Real


class LimitConfigurationError(ValueError):
    """限制配置自身无效。"""


class LimitViolationError(ValueError):
    """目标位置或关节运动违反已配置的限制。"""


def _finite_number(
    value: object,
    label: str,
    error_type: type[ValueError],
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(float(value))
    ):
        raise error_type(f"{label} 必须是有限数值。")
    return float(value)


def _finite_vector(
    values: Sequence[float],
    *,
    expected_length: int,
    label: str,
    error_type: type[ValueError],
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise error_type(f"{label} 需要 {expected_length} 个关节值。")

    try:
        vector = tuple(values)
    except TypeError as error:
        raise error_type(
            f"{label} 需要 {expected_length} 个关节值。"
        ) from error

    if len(vector) != expected_length:
        raise error_type(f"{label} 需要 {expected_length} 个关节值。")

    return tuple(
        _finite_number(value, f"{label}[{index}]", error_type)
        for index, value in enumerate(vector)
    )


@dataclass(frozen=True)
class AxisLimits:
    """一个笛卡尔坐标轴的闭区间，单位由调用场景决定。"""

    minimum: float
    maximum: float

    def __post_init__(self) -> None:
        minimum = _finite_number(
            self.minimum,
            "轴下限",
            LimitConfigurationError,
        )
        maximum = _finite_number(
            self.maximum,
            "轴上限",
            LimitConfigurationError,
        )
        if minimum > maximum:
            raise LimitConfigurationError("轴下限必须小于或等于轴上限。")
        object.__setattr__(self, "minimum", minimum)
        object.__setattr__(self, "maximum", maximum)

    def validate(self, value: float, axis_name: str) -> float:
        numeric_value = _finite_number(
            value,
            axis_name,
            LimitViolationError,
        )
        if not self.minimum <= numeric_value <= self.maximum:
            raise LimitViolationError(
                f"{axis_name}={numeric_value} 超出允许范围 "
                f"[{self.minimum}, {self.maximum}]。"
            )
        return numeric_value


@dataclass(frozen=True)
class WorkspaceLimits:
    """末端位置的三轴闭区间，统一使用米。"""

    x: AxisLimits
    y: AxisLimits
    z: AxisLimits

    def validate_position(
        self,
        x: float,
        y: float,
        z: float,
    ) -> tuple[float, float, float]:
        return (
            self.x.validate(x, "x"),
            self.y.validate(y, "y"),
            self.z.validate(z, "z"),
        )


@dataclass(frozen=True)
class JointLimits:
    """关节角和关节单步变化限制，单位统一使用弧度。"""

    joint_names: tuple[str, ...]
    lower_radians: tuple[float, ...]
    upper_radians: tuple[float, ...]
    max_step_radians: tuple[float, ...]

    def __post_init__(self) -> None:
        names = tuple(self.joint_names)
        if not names or any(not name for name in names):
            raise LimitConfigurationError("关节名称不能为空。")
        if len(set(names)) != len(names):
            raise LimitConfigurationError("关节名称不能重复。")

        count = len(names)
        lower = _finite_vector(
            self.lower_radians,
            expected_length=count,
            label="关节下限",
            error_type=LimitConfigurationError,
        )
        upper = _finite_vector(
            self.upper_radians,
            expected_length=count,
            label="关节上限",
            error_type=LimitConfigurationError,
        )
        max_step = _finite_vector(
            self.max_step_radians,
            expected_length=count,
            label="关节最大单步变化",
            error_type=LimitConfigurationError,
        )

        for index, name in enumerate(names):
            if lower[index] > upper[index]:
                raise LimitConfigurationError(
                    f"关节 {name} 的下限必须小于或等于上限。"
                )
            if max_step[index] <= 0:
                raise LimitConfigurationError(
                    f"关节 {name} 的最大单步变化必须大于 0。"
                )

        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "lower_radians", lower)
        object.__setattr__(self, "upper_radians", upper)
        object.__setattr__(self, "max_step_radians", max_step)

    @property
    def count(self) -> int:
        return len(self.joint_names)

    def validate_position(
        self,
        joint_radians: Sequence[float],
    ) -> tuple[float, ...]:
        values = _finite_vector(
            joint_radians,
            expected_length=self.count,
            label="关节位置",
            error_type=LimitViolationError,
        )
        for index, name in enumerate(self.joint_names):
            if not (
                self.lower_radians[index]
                <= values[index]
                <= self.upper_radians[index]
            ):
                raise LimitViolationError(
                    f"关节 {name}={values[index]} rad 超出允许范围 "
                    f"[{self.lower_radians[index]}, {self.upper_radians[index]}]。"
                )
        return values

    def validate_step(
        self,
        current_radians: Sequence[float],
        target_radians: Sequence[float],
    ) -> tuple[float, ...]:
        current = self.validate_position(current_radians)
        target = self.validate_position(target_radians)
        for index, name in enumerate(self.joint_names):
            change = abs(target[index] - current[index])
            if change > self.max_step_radians[index] + 1e-12:
                raise LimitViolationError(
                    f"关节 {name} 单步变化 {change} rad 超出上限 "
                    f"{self.max_step_radians[index]} rad。"
                )
        return target


@dataclass(frozen=True)
class MotionLimits:
    """一次笛卡尔运动规划所需的完整显式限制。"""

    workspace: WorkspaceLimits
    joints: JointLimits

    def validate_target_position(
        self,
        position_m: Sequence[float],
    ) -> tuple[float, float, float]:
        if isinstance(position_m, (str, bytes)):
            raise LimitViolationError("末端目标需要 3 个坐标值。")
        try:
            position = tuple(position_m)
        except TypeError as error:
            raise LimitViolationError("末端目标需要 3 个坐标值。") from error
        if len(position) != 3:
            raise LimitViolationError("末端目标需要 3 个坐标值。")
        return self.workspace.validate_position(*position)

    def validate_joint_step(
        self,
        current_radians: Sequence[float],
        target_radians: Sequence[float],
    ) -> tuple[float, ...]:
        return self.joints.validate_step(current_radians, target_radians)


# 以下数值逐字来自 lerobot-kinematics-plus 的 SO-100 Plus 模型。
# 它们只描述第三方运动学模型，不是 right_follower 实机认证的物理安全范围。
SO100_PLUS_ARM_JOINT_NAMES = (
    "shoulder_rotation_joint",
    "shoulder_pitch_joint",
    "ellbow_joint",
    "wrist_pitch_joint",
    "wrist_jaw_joint",
    "wrist_roll_joint",
)

SO100_PLUS_MODEL_JOINT_LIMITS = JointLimits(
    joint_names=SO100_PLUS_ARM_JOINT_NAMES,
    lower_radians=(-2.2, -3.14158, -0.2, -2.2, -2.2, -3.14158),
    upper_radians=(2.2, 0.2, 3.14158, 1.8, 1.5, 3.14158),
    max_step_radians=(0.1,) * 6,
)
