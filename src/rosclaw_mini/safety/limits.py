"""机械臂运动限制的数据结构和纯计算检查。

本模块不读取机械臂、不写电机。真实运动必须由调用方显式提供经过确认的
``WorkspaceLimits`` 和 ``JointLimits``；带具体机械臂名称的常量只适用于
注释中写明的实机、校准、底座和 TCP 姿态条件。
通俗来说，这个模块是用来检查机械臂运动命令是否在安全空间运行范围内的工具，而不是直接控制机械臂的代码。
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
    """
    检查数值是否为有限数值，并返回浮点数表示。
    """
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
    """
    检查向量是否为有限数值，并返回浮点数元组表示。
    """
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
    """
    一个笛卡尔坐标轴的闭区间，单位由调用场景决定。
    通俗解释：这个类表示机械臂在某个轴上的运动范围，包含最小值和最大值。如果设置的最小值大于最大值，会抛出配置错误异常。
    """

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
    """夹爪 TCP 绝对位置的三轴闭区间，统一使用米。"""

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
            raise LimitViolationError("夹爪 TCP 目标需要 3 个坐标值。")
        try:
            position = tuple(position_m)
        except TypeError as error:
            raise LimitViolationError(
                "夹爪 TCP 目标需要 3 个坐标值。"
            ) from error
        if len(position) != 3:
            raise LimitViolationError("夹爪 TCP 目标需要 3 个坐标值。")
        return self.workspace.validate_position(*position)

    def validate_joint_step(
        self,
        current_radians: Sequence[float],
        target_radians: Sequence[float],
    ) -> tuple[float, ...]:
        return self.joints.validate_step(current_radians, target_radians)


def resolve_relative_tcp_target(
    current_position_m: Sequence[float],
    displacement_m: Sequence[float],
    workspace: WorkspaceLimits,
) -> tuple[float, float, float]:
    """将执行时 TCP 与基座系位移合成绝对目标，并验证最终工作空间。"""

    current = _finite_vector(
        current_position_m,
        expected_length=3,
        label="当前 TCP",
        error_type=LimitViolationError,
    )
    displacement = _finite_vector(
        displacement_m,
        expected_length=3,
        label="相对位移",
        error_type=LimitViolationError,
    )
    if all(value == 0.0 for value in displacement):
        raise LimitViolationError(
            "相对位移 dx/dy/dz 不能全部为 0；"
            "未向机械臂发送运动。"
        )
    target = tuple(
        current_value + delta
        for current_value, delta in zip(
            current,
            displacement,
            strict=True,
        )
    )
    try:
        return workspace.validate_position(*target)
    except LimitViolationError as error:
        raise LimitViolationError(
            "相对移动最终目标违反工作空间："
            f"当前 TCP={current} m；"
            f"请求位移 dx/dy/dz={displacement} m；"
            f"最终目标={target} m；{error}"
        ) from error


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

# 2026-07-16 在 right_follower 扭矩全部关闭时，由用户手动移动安装了
# 底座的 shoulder_rotation_joint，并选择日常使用不会碰到底座或拉扯
# 线缆的两侧位置。这里保存的是 LeRobot 校准后的驱动角度，不是模型
# 弧度；它只认证这一个关节，不能冒充完整的实机 JointLimits。
SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS = AxisLimits(
    minimum=-19.599609,
    maximum=31.201172,
)

# 2026-07-18 将 right_follower 的 JoyCon 初始 TCP 姿态候选框内缩
# 一个 1 cm 仿真网格后，14 个边界代表点均完成真机运动测试。
# 12 点满足 12 mm 到位门槛；X 最大面中心和 X/Y/Z 最大角分别有
# 约 24.8 mm、14.78 mm 到位误差，但路径、负载和温度均无异常。
# 用户确认把这个内缩长方体作为正式“可达工作空间”使用。
#
# 这组范围不承诺全域 12 mm 定位精度，也不代表任意 TCP 姿态、其他
# follower、其他校准文件、其他底座或存在障碍物的工位仍然有效。
SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS = WorkspaceLimits(
    x=AxisLimits(
        minimum=0.3135714232672181,
        maximum=0.4335714232672181,
    ),
    y=AxisLimits(
        minimum=-0.041185494280163625,
        maximum=0.018814505719836373,
    ),
    z=AxisLimits(
        minimum=0.17932848288990053,
        maximum=0.29932848288990055,
    ),
)


def choose_so100_plus_right_follower_base_test_target(
    current_driver_degrees: float,
    *,
    delta_degrees: float = 8.0,
    boundary_margin_degrees: float = 2.0,
) -> float:
    """在实测底座范围内选择空余更大一侧的明显诊断目标。"""

    current = SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.validate(
        current_driver_degrees,
        "当前底座关节",
    )
    delta = _finite_number(
        delta_degrees,
        "底座诊断变化",
        LimitConfigurationError,
    )
    margin = _finite_number(
        boundary_margin_degrees,
        "底座边界余量",
        LimitConfigurationError,
    )
    if not 0 < delta <= 8.0:
        raise LimitConfigurationError(
            "底座诊断变化必须大于 0 且不超过 8 度。"
        )
    if margin < 0:
        raise LimitConfigurationError("底座边界余量不能为负数。")

    limits = SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS
    positive_headroom = limits.maximum - margin - current
    negative_headroom = current - (limits.minimum + margin)
    if positive_headroom >= delta and positive_headroom >= negative_headroom:
        return current + delta
    if negative_headroom >= delta:
        return current - delta
    if positive_headroom >= delta:
        return current + delta
    raise LimitViolationError(
        "当前底座位置没有足够空间完成带边界余量的诊断动作。"
    )


def build_so100_plus_right_follower_local_joint_limits(
    current_joint_radians: Sequence[float],
    *,
    max_delta_radians: float = 0.1,
    max_step_radians: float | None = None,
) -> JointLimits:
    """为一次局部验证创建相对当前位置的小范围关节限制。

    只有底座关节额外使用了真机实测绝对范围；其余关节只允许围绕当前
    校准位置小幅变化，不能把返回值当作完整的实机绝对关节范围。
    """

    current = _finite_vector(
        current_joint_radians,
        expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
        label="当前关节位置",
        error_type=LimitViolationError,
    )
    max_delta = _finite_number(
        max_delta_radians,
        "局部关节最大变化",
        LimitConfigurationError,
    )
    if not 0 < max_delta <= 0.1:
        raise LimitConfigurationError(
            "局部关节最大变化必须大于 0 且不超过 0.1 rad。"
        )
    step = (
        max_delta
        if max_step_radians is None
        else _finite_number(
            max_step_radians,
            "局部关节单步变化",
            LimitConfigurationError,
        )
    )
    if not 0 < step <= max_delta:
        raise LimitConfigurationError(
            "局部关节单步变化必须大于 0 且不超过局部最大变化。"
        )

    current_base_driver_degrees = -math.degrees(current[0])
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.validate(
        current_base_driver_degrees,
        "当前底座关节",
    )
    measured_base_lower_radians = math.radians(
        -SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.maximum
    )
    measured_base_upper_radians = math.radians(
        -SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.minimum
    )

    lower = [value - max_delta for value in current]
    upper = [value + max_delta for value in current]
    lower[0] = max(lower[0], measured_base_lower_radians)
    upper[0] = min(upper[0], measured_base_upper_radians)
    limits = JointLimits(
        joint_names=SO100_PLUS_ARM_JOINT_NAMES,
        lower_radians=tuple(lower),
        upper_radians=tuple(upper),
        max_step_radians=(step,) * len(SO100_PLUS_ARM_JOINT_NAMES),
    )
    limits.validate_position(current)
    return limits


def build_so100_plus_right_follower_execution_joint_limits(
    current_joint_radians: Sequence[float],
    *,
    max_step_radians: float,
) -> JointLimits:
    """创建不限制整段总变化、但保留绝对边界和单步限制的执行范围。

    底座继续使用真机实测范围。其余关节使用第三方模型范围；如果当前
    校准角略在模型范围外，只把当前位置扩展为边界，并仅允许向模型
    范围内移动，不能继续向外扩大。
    """

    current = _finite_vector(
        current_joint_radians,
        expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
        label="当前关节位置",
        error_type=LimitViolationError,
    )
    step = _finite_number(
        max_step_radians,
        "关节单步变化",
        LimitConfigurationError,
    )
    if not 0 < step <= 0.1:
        raise LimitConfigurationError(
            "关节单步变化必须大于 0 且不超过 0.1 rad。"
        )

    current_base_driver_degrees = -math.degrees(current[0])
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.validate(
        current_base_driver_degrees,
        "当前底座关节",
    )
    measured_base_lower_radians = math.radians(
        -SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.maximum
    )
    measured_base_upper_radians = math.radians(
        -SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.minimum
    )

    lower = [
        min(model_lower, current_value)
        for model_lower, current_value in zip(
            SO100_PLUS_MODEL_JOINT_LIMITS.lower_radians,
            current,
            strict=True,
        )
    ]
    upper = [
        max(model_upper, current_value)
        for model_upper, current_value in zip(
            SO100_PLUS_MODEL_JOINT_LIMITS.upper_radians,
            current,
            strict=True,
        )
    ]
    lower[0] = measured_base_lower_radians
    upper[0] = measured_base_upper_radians
    limits = JointLimits(
        joint_names=SO100_PLUS_ARM_JOINT_NAMES,
        lower_radians=tuple(lower),
        upper_radians=tuple(upper),
        max_step_radians=(step,) * len(SO100_PLUS_ARM_JOINT_NAMES),
    )
    limits.validate_position(current)
    return limits


def build_so100_plus_right_follower_motion_limits(
    current_joint_radians: Sequence[float],
    *,
    max_step_radians: float = math.radians(2.0),
) -> MotionLimits:
    """构造正式工作空间与 right_follower 执行关节范围的组合限制。

    当前关节位置仍是必需输入，因为收纳姿态可能略超第三方模型边界；
    执行关节范围只会为当次当前位置向模型内部提供过渡，不会继续向外扩张。
    """

    return MotionLimits(
        workspace=SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
        joints=build_so100_plus_right_follower_execution_joint_limits(
            current_joint_radians,
            max_step_radians=max_step_radians,
        ),
    )
