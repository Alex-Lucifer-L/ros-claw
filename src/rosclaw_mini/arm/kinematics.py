"""SO-100 Plus 的纯运动学计算和安全轨迹规划。

本模块只处理内存中的数值，不导入厂商电机驱动，也不发送串口命令。
具体通俗来说，这个模块是用来计算机械臂的运动学和规划安全的轨迹，而不是直接控制机械臂的代码。
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from numbers import Real
from typing import Any

import numpy as np

from rosclaw_mini.safety.limits import (
    JointLimits,
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
)


SO100_PLUS_DRIVER_TO_MODEL_SIGNS = (-1, -1, 1, -1, 1, 1)
# JoyConController_plus.init_qpos 中的“控制器初始工作姿态”。它不是
# README 校准流程照片里的 follower_rest 收纳姿态。
SO100_PLUS_JOYCON_INITIAL_RADIANS = (0.0, -3.1, 3.0, 0.0, 0.0, 1.57)
# 保留旧名称，避免外部调用立即失效；新代码应使用 INITIAL 名称。
SO100_PLUS_JOYCON_REST_RADIANS = SO100_PLUS_JOYCON_INITIAL_RADIANS
# X 来自 lerobot_kinematics 源码中预留但未启用的 E18=tx(0.10127)。
# Y/Z 来自 MuJoCo 两根夹指最前端内侧接触面的共同高度中心和间隙中点。
# 三者共同描述第六关节运动学末端到固定夹持中心的工具局部坐标。
SO100_PLUS_GRIPPER_TCP_OFFSET_M = (0.10127, -0.00690, 0.00118)
# MuJoCo 碰撞预检与执行器共用的最大相邻执行点距离。最终
# JointMotionPlan 在预检前就必须满足它，执行期间不再插值。
SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS = math.radians(1.0)


class KinematicsError(RuntimeError):
    """运动学计算无法生成可验证的结果。"""


class KinematicsDependencyError(KinematicsError):
    """纯运动学依赖尚未安装或模型无法创建。"""


class InverseKinematicsError(KinematicsError):
    """逆运动学失败或解的复算误差不可接受。"""


@dataclass(frozen=True)
class JointMotionPlan:
    """只包含数值的关节轨迹，不具备执行硬件的能力。"""

    target_position_m: tuple[float, float, float]
    current_joint_radians: tuple[float, ...]
    target_joint_radians: tuple[float, ...]
    waypoints_radians: tuple[tuple[float, ...], ...]
    is_final_execution_plan: bool = False
    waypoint_interval_seconds: float | None = None
    held_gripper_driver_degrees: float | None = None


def _finite_vector(
    values: Sequence[float],
    *,
    expected_length: int,
    label: str,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise KinematicsError(f"{label} 需要 {expected_length} 个有限数值。")
    try:
        vector = tuple(values)
    except TypeError as error:
        raise KinematicsError(
            f"{label} 需要 {expected_length} 个有限数值。"
        ) from error
    if len(vector) != expected_length:
        raise KinematicsError(f"{label} 需要 {expected_length} 个有限数值。")

    result = []
    for value in vector:
        if (
            isinstance(value, bool)
            or not isinstance(value, Real)
            or not math.isfinite(float(value))
        ):
            raise KinematicsError(f"{label} 需要 {expected_length} 个有限数值。")
        result.append(float(value))
    return tuple(result)


class SO100PlusKinematics:
    """封装 SO-100 Plus 的 6 轴 FK、IK 和关节路径插值。"""

    def __init__(
        self,
        robot: Any | None = None,
        *,
        tcp_offset_m: Sequence[float] = SO100_PLUS_GRIPPER_TCP_OFFSET_M,
        position_tolerance_m: float = 0.0001,
        orientation_matrix_tolerance: float = 0.0001,
    ) -> None:
        if position_tolerance_m <= 0 or not math.isfinite(position_tolerance_m):
            raise ValueError("位置复算容差必须是有限正数。")
        if (
            orientation_matrix_tolerance <= 0
            or not math.isfinite(orientation_matrix_tolerance)
        ):
            raise ValueError("姿态复算容差必须是有限正数。")

        self.robot = robot if robot is not None else self._load_default_robot()
        self.tcp_offset_m = _finite_vector(
            tcp_offset_m,
            expected_length=3,
            label="夹爪 TCP 偏移",
        )
        self._flange_to_tcp_transform = np.eye(4, dtype=float)
        self._flange_to_tcp_transform[:3, 3] = self.tcp_offset_m
        self._tcp_to_flange_transform = np.eye(4, dtype=float)
        self._tcp_to_flange_transform[:3, 3] = tuple(
            -value for value in self.tcp_offset_m
        )
        self.position_tolerance_m = float(position_tolerance_m)
        self.orientation_matrix_tolerance = float(
            orientation_matrix_tolerance
        )

    @staticmethod
    def _load_default_robot() -> Any:
        try:
            from lerobot_kinematics import get_robot
        except (ImportError, ModuleNotFoundError) as error:
            raise KinematicsDependencyError(
                "缺少 lerobot_kinematics 纯运动学依赖。"
            ) from error

        robot = get_robot("so100_plus")
        if robot is None:
            raise KinematicsDependencyError("无法创建 so100_plus 运动学模型。")
        return robot

    @staticmethod
    def driver_degrees_to_model_radians(
        driver_degrees: Sequence[float],
    ) -> tuple[float, ...]:
        values = _finite_vector(
            driver_degrees,
            expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
            label="驱动关节角",
        )
        return tuple(
            math.radians(value * sign)
            for value, sign in zip(values, SO100_PLUS_DRIVER_TO_MODEL_SIGNS)
        )

    @staticmethod
    def model_radians_to_driver_degrees(
        model_radians: Sequence[float],
    ) -> tuple[float, ...]:
        values = _finite_vector(
            model_radians,
            expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
            label="模型关节角",
        )
        return tuple(
            math.degrees(value) * sign
            for value, sign in zip(values, SO100_PLUS_DRIVER_TO_MODEL_SIGNS)
        )

    def forward_position(
        self,
        joint_radians: Sequence[float],
    ) -> tuple[float, float, float]:
        """返回夹爪尖端之间 TCP 在机械臂底座坐标系中的位置。"""

        joints = _finite_vector(
            joint_radians,
            expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
            label="模型关节角",
        )
        transform = self._forward_transform(joints)
        return tuple(float(value) for value in transform[:3, 3])

    def solve_position(
        self,
        current_joint_radians: Sequence[float],
        target_position_m: Sequence[float],
        *,
        joint_limits: JointLimits | None = None,
    ) -> tuple[float, ...]:
        """保持当前夹爪 TCP 姿态，只求解新的绝对 x/y/z。"""

        current = _finite_vector(
            current_joint_radians,
            expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
            label="当前模型关节角",
        )
        target = _finite_vector(
            target_position_m,
            expected_length=3,
            label="夹爪 TCP 目标",
        )
        current_transform = self._forward_transform(current)

        if (
            np.linalg.norm(current_transform[:3, 3] - np.asarray(target))
            <= self.position_tolerance_m
        ):
            if joint_limits is not None:
                joint_limits.validate_position(current)
            return current

        target_transform = current_transform.copy()
        target_transform[:3, 3] = np.asarray(target)
        target_flange_transform = (
            target_transform @ self._tcp_to_flange_transform
        )

        solution = self.robot.ikine_LM(
            Tep=target_flange_transform,
            q0=np.asarray(current, dtype=float),
            ilimit=100,
            slimit=5,
            tol=1e-8,
            # 第三方实现会先把角度强制折回 [-pi, pi]，再错误检查
            # right_follower 当前跨圈关节。这里关闭它，随后使用我们显式
            # 传入的关节限制检查展开后的结果。
            joint_limits=False,
            seed=0,
            method="chan",
            k=1.0,
        )
        if not bool(getattr(solution, "success", False)):
            reason = getattr(solution, "reason", "未知原因")
            raise InverseKinematicsError(f"逆运动学失败：{reason}")

        raw_solution = _finite_vector(
            solution.q,
            expected_length=len(SO100_PLUS_ARM_JOINT_NAMES),
            label="逆运动学结果",
        )
        unwrapped = tuple(
            reference
            + (value - reference + math.pi) % (2 * math.pi)
            - math.pi
            for value, reference in zip(raw_solution, current)
        )

        if joint_limits is not None:
            joint_limits.validate_position(unwrapped)

        solved_transform = self._forward_transform(unwrapped)
        position_error = float(
            np.linalg.norm(solved_transform[:3, 3] - target_transform[:3, 3])
        )
        orientation_error = float(
            np.max(
                np.abs(
                    solved_transform[:3, :3]
                    - target_transform[:3, :3]
                )
            )
        )
        if position_error > self.position_tolerance_m:
            raise InverseKinematicsError(
                f"逆运动学位置复算误差 {position_error} m 超出容差 "
                f"{self.position_tolerance_m} m。"
            )
        if orientation_error > self.orientation_matrix_tolerance:
            raise InverseKinematicsError(
                f"逆运动学姿态复算误差 {orientation_error} 超出容差 "
                f"{self.orientation_matrix_tolerance}。"
            )
        return unwrapped

    def plan_position(
        self,
        current_joint_radians: Sequence[float],
        target_position_m: Sequence[float],
        limits: MotionLimits,
    ) -> JointMotionPlan:
        """求解并拆分轨迹；返回值仍只是内存中的数值。"""

        target_position = limits.validate_target_position(target_position_m)
        current = limits.joints.validate_position(current_joint_radians)
        target_joints = self.solve_position(
            current,
            target_position,
            joint_limits=limits.joints,
        )
        waypoints = self._interpolate_joint_path(
            current,
            target_joints,
            limits.joints,
        )
        return JointMotionPlan(
            target_position_m=target_position,
            current_joint_radians=current,
            target_joint_radians=target_joints,
            waypoints_radians=waypoints,
        )

    def plan_joint_pose(
        self,
        current_joint_radians: Sequence[float],
        target_joint_radians: Sequence[float],
        limits: MotionLimits,
    ) -> JointMotionPlan:
        """规划到显式模型关节姿态，并复用同一套关节与 TCP 边界。"""

        current = limits.joints.validate_position(current_joint_radians)
        target = limits.joints.validate_position(target_joint_radians)
        target_position = self.forward_position(target)
        limits.validate_target_position(target_position)
        waypoints = self._interpolate_joint_path(
            current,
            target,
            limits.joints,
        )
        for waypoint in waypoints:
            limits.validate_target_position(
                self.forward_position(waypoint)
            )
        return JointMotionPlan(
            target_position_m=target_position,
            current_joint_radians=current,
            target_joint_radians=target,
            waypoints_radians=waypoints,
        )

    def _forward_transform(
        self,
        joint_radians: Sequence[float],
    ) -> np.ndarray:
        """返回底座到夹爪 TCP 的齐次变换。"""

        return (
            self._forward_flange_transform(joint_radians)
            @ self._flange_to_tcp_transform
        )

    def _forward_flange_transform(
        self,
        joint_radians: Sequence[float],
    ) -> np.ndarray:
        """返回第三方六轴模型原始末端的齐次变换。"""

        transform_object = self.robot.fkine(
            np.asarray(joint_radians, dtype=float)
        )
        transform = np.asarray(
            getattr(transform_object, "A", transform_object),
            dtype=float,
        )
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise KinematicsError("正运动学没有返回有限的 4x4 变换矩阵。")
        return transform

    @staticmethod
    def _interpolate_joint_path(
        current: tuple[float, ...],
        target: tuple[float, ...],
        limits: JointLimits,
    ) -> tuple[tuple[float, ...], ...]:
        deltas = tuple(
            target_value - current_value
            for current_value, target_value in zip(current, target)
        )
        if all(abs(delta) <= 1e-15 for delta in deltas):
            return ()

        step_count = max(
            math.ceil(abs(delta) / max_step)
            for delta, max_step in zip(deltas, limits.max_step_radians)
        )
        waypoints = []
        previous = current
        for step_index in range(1, step_count + 1):
            fraction = step_index / step_count
            waypoint = tuple(
                current_value + delta * fraction
                for current_value, delta in zip(current, deltas)
            )
            limits.validate_step(previous, waypoint)
            waypoints.append(waypoint)
            previous = waypoint
        return tuple(waypoints)
