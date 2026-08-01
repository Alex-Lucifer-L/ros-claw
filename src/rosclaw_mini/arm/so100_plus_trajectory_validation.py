"""SO-100 Plus 关节轨迹的生产级 MuJoCo 预检查。

本模块不连接 Robot、不读写串口。它只验证已经规划完成的
``JointMotionPlan``，并把通过验证的同一组 Plan 原样返回给执行层。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Any

from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.safety.limits import SO100_PLUS_ARM_JOINT_NAMES


DEFAULT_SO100_PLUS_MUJOCO_MODEL_PATH = (
    Path(__file__).resolve().parents[3]
    / "lerobot-joycon_plus"
    / "lerobot"
    / "common"
    / "robot_devices"
    / "controllers"
    / "scene_plus.xml"
)
SO100_PLUS_COLLISION_SAMPLE_STEP_RADIANS = (
    SO100_PLUS_COLLISION_EXECUTION_STEP_RADIANS
)
SO100_PLUS_MUJOCO_GRIPPER_JOINT_NAME = "gripper_joint"


class SO100PlusTrajectoryValidationError(RuntimeError):
    """轨迹不能通过离线模型预检查。"""


class SO100PlusTrajectoryValidationUnavailableError(
    SO100PlusTrajectoryValidationError
):
    """MuJoCo 或已认证模型不可用，轨迹验证失败关闭。"""


class StorageTransitionDirection(str, Enum):
    """收纳通道的验证方向。"""

    UNFOLD = "unfold"
    FOLD = "fold"


@dataclass(frozen=True)
class SO100PlusTrajectoryValidationReport:
    """一次完整轨迹预检查的只读统计。"""

    sample_count: int
    max_joint_sample_step_degrees: float
    minimum_tcp_z_m: float
    initial_contact_pairs: frozenset[tuple[str, str]]
    final_contact_pairs: frozenset[tuple[str, str]]
    last_contact_sample: int


@dataclass(frozen=True)
class VerifiedJointMotionSequence:
    """已经通过 MuJoCo 预检查、可原样交给 Adapter 的 Plan。"""

    plans: tuple[JointMotionPlan, ...]
    sampled_joint_radians: tuple[tuple[float, ...], ...]
    report: SO100PlusTrajectoryValidationReport
    gripper_qpos: float


def so100_plus_gripper_driver_degrees_to_mujoco_qpos(
    driver_degrees: float,
) -> float:
    """将已校准驱动反馈映射到 MuJoCo 夹爪 qpos。

    Feetech 总线的 ``Present_Position`` 已经校准为度；
    ``JoyConController_plus`` 仅翻转 0/1/3 号手臂关节，不翻转
    ``gripper_joint``，而 MuJoCo XML 声明角度单位为 radian。
    """

    if (
        isinstance(driver_degrees, bool)
        or not isinstance(driver_degrees, (int, float))
        or not math.isfinite(float(driver_degrees))
    ):
        raise SO100PlusTrajectoryValidationError(
            "夹爪反馈必须是有限的已校准驱动角。"
        )
    return math.radians(float(driver_degrees))


def _finite_joints(
    values: Sequence[float],
    *,
    label: str,
) -> tuple[float, ...]:
    try:
        joints = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise SO100PlusTrajectoryValidationError(
            f"{label}需要六个有限关节角。"
        ) from error
    if (
        len(joints) != len(SO100_PLUS_ARM_JOINT_NAMES)
        or not all(math.isfinite(value) for value in joints)
    ):
        raise SO100PlusTrajectoryValidationError(
            f"{label}需要六个有限关节角。"
        )
    return joints


def _same_joints(
    first: Sequence[float],
    second: Sequence[float],
    *,
    tolerance: float = 1e-12,
) -> bool:
    return all(
        abs(left - right) <= tolerance
        for left, right in zip(first, second, strict=True)
    )


def _plans_to_route(
    plans: Sequence[JointMotionPlan],
) -> tuple[tuple[float, ...], ...]:
    plans_tuple = tuple(plans)
    if not plans_tuple:
        raise SO100PlusTrajectoryValidationError("待验证轨迹不能为空。")

    route: list[tuple[float, ...]] = []
    previous_target: tuple[float, ...] | None = None
    for index, plan in enumerate(plans_tuple):
        if not plan.is_final_execution_plan:
            raise SO100PlusTrajectoryValidationError(
                f"第 {index + 1} 段尚未固化为最终执行计划。"
            )
        current = _finite_joints(
            plan.current_joint_radians,
            label=f"第 {index + 1} 段起点",
        )
        target = _finite_joints(
            plan.target_joint_radians,
            label=f"第 {index + 1} 段终点",
        )
        if (
            previous_target is not None
            and not _same_joints(previous_target, current)
        ):
            raise SO100PlusTrajectoryValidationError(
                f"第 {index + 1} 段起点与上一段终点不连续。"
            )
        if not route:
            route.append(current)

        waypoints = tuple(
            _finite_joints(
                waypoint,
                label=f"第 {index + 1} 段轨迹点",
            )
            for waypoint in plan.waypoints_radians
        )
        if waypoints:
            if not _same_joints(waypoints[-1], target):
                raise SO100PlusTrajectoryValidationError(
                    f"第 {index + 1} 段最后轨迹点不是计划终点。"
                )
            route.extend(waypoints)
        elif not _same_joints(current, target):
            raise SO100PlusTrajectoryValidationError(
                f"第 {index + 1} 段缺少到达非当前终点的轨迹点。"
            )
        previous_target = target
    return tuple(route)


class SO100PlusMuJoCoTrajectoryValidator:
    """使用已登记 MuJoCo 模型检查完整关节轨迹。"""

    def __init__(
        self,
        *,
        model_path: Path = DEFAULT_SO100_PLUS_MUJOCO_MODEL_PATH,
    ) -> None:
        self.model_path = Path(model_path)
        if not self.model_path.is_file():
            raise SO100PlusTrajectoryValidationUnavailableError(
                f"SO-100 Plus MuJoCo 模型不可用：{self.model_path}。"
            )
        try:
            import mujoco
        except (ImportError, ModuleNotFoundError) as error:
            raise SO100PlusTrajectoryValidationUnavailableError(
                "缺少 MuJoCo，SO-100 Plus 运动轨迹保持失败关闭。"
            ) from error
        try:
            self._model = mujoco.MjModel.from_xml_path(str(self.model_path))
        except Exception as error:
            raise SO100PlusTrajectoryValidationUnavailableError(
                f"无法加载 SO-100 Plus MuJoCo 模型：{self.model_path}。"
            ) from error
        self._mujoco = mujoco
        joint_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_JOINT,
            SO100_PLUS_MUJOCO_GRIPPER_JOINT_NAME,
        )
        if joint_id < 0:
            raise SO100PlusTrajectoryValidationUnavailableError(
                "MuJoCo 模型缺少 gripper_joint，无法表达实际夹爪姿态。"
            )
        if int(self._model.jnt_type[joint_id]) != int(
            mujoco.mjtJoint.mjJNT_HINGE
        ):
            raise SO100PlusTrajectoryValidationUnavailableError(
                "MuJoCo gripper_joint 不是单自由度转动关节。"
            )
        if not bool(self._model.jnt_limited[joint_id]):
            raise SO100PlusTrajectoryValidationUnavailableError(
                "MuJoCo gripper_joint 没有可认证的角度范围。"
            )
        self._gripper_qpos_address = int(
            self._model.jnt_qposadr[joint_id]
        )
        self._gripper_qpos_range = tuple(
            float(value) for value in self._model.jnt_range[joint_id]
        )

    def gripper_driver_degrees_to_qpos(
        self,
        driver_degrees: float,
    ) -> float:
        """映射实测夹爪角，并按当前已加载模型范围失败关闭。"""

        qpos = so100_plus_gripper_driver_degrees_to_mujoco_qpos(
            driver_degrees
        )
        lower, upper = self._gripper_qpos_range
        if not lower <= qpos <= upper:
            raise SO100PlusTrajectoryValidationError(
                f"夹爪实测角 {float(driver_degrees):.6f}° 映射为 "
                f"qpos={qpos:.6f} rad，超出 MuJoCo gripper_joint "
                f"范围 [{lower:.6f}, {upper:.6f}]。"
            )
        return qpos

    def _validate_gripper_qpos(self, gripper_qpos: float) -> float:
        if (
            isinstance(gripper_qpos, bool)
            or not isinstance(gripper_qpos, (int, float))
            or not math.isfinite(float(gripper_qpos))
        ):
            raise SO100PlusTrajectoryValidationError(
                "MuJoCo 夹爪 qpos 必须是有限数值。"
            )
        value = float(gripper_qpos)
        lower, upper = self._gripper_qpos_range
        if not lower <= value <= upper:
            raise SO100PlusTrajectoryValidationError(
                f"MuJoCo 夹爪 qpos={value:.6f} 超出模型范围 "
                f"[{lower:.6f}, {upper:.6f}]。"
            )
        return value

    def _contact_pairs(
        self,
        data: Any,
    ) -> frozenset[tuple[str, str]]:
        pairs = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            first_body_id = int(
                self._model.geom_bodyid[int(contact.geom1)]
            )
            second_body_id = int(
                self._model.geom_bodyid[int(contact.geom2)]
            )
            pairs.add(
                (
                    self._mujoco.mj_id2name(
                        self._model,
                        self._mujoco.mjtObj.mjOBJ_BODY,
                        first_body_id,
                    )
                    or "world",
                    self._mujoco.mj_id2name(
                        self._model,
                        self._mujoco.mjtObj.mjOBJ_BODY,
                        second_body_id,
                    )
                    or "world",
                )
            )
        return frozenset(pairs)

    def _sample_contacts(
        self,
        sampled_joint_radians: Sequence[Sequence[float]],
        gripper_qpos: float,
    ) -> tuple[frozenset[tuple[str, str]], ...]:
        data = self._mujoco.MjData(self._model)
        contacts = []
        for joints in sampled_joint_radians:
            data.qpos[:6] = joints
            data.qpos[self._gripper_qpos_address] = gripper_qpos
            self._mujoco.mj_forward(self._model, data)
            contacts.append(self._contact_pairs(data))
        return tuple(contacts)

    @staticmethod
    def _max_sample_step_degrees(
        samples: Sequence[Sequence[float]],
    ) -> float:
        return max(
            (
                math.degrees(
                    max(
                        abs(target - current)
                        for current, target in zip(
                            start,
                            end,
                            strict=True,
                        )
                    )
                )
                for start, end in zip(samples, samples[1:])
            ),
            default=0.0,
        )

    def verify_collision_free_sequence(
        self,
        plans: Sequence[JointMotionPlan],
        kinematics: SO100PlusKinematics,
        *,
        gripper_qpos: float,
    ) -> VerifiedJointMotionSequence:
        """验证普通工作位置返回工作初始姿态的完整计划。"""

        plans_tuple = tuple(plans)
        samples = _plans_to_route(plans_tuple)
        gripper_qpos = self._validate_gripper_qpos(gripper_qpos)
        max_step = self._max_sample_step_degrees(samples)
        if max_step > math.degrees(
            SO100_PLUS_COLLISION_SAMPLE_STEP_RADIANS
        ) + 1e-9:
            raise SO100PlusTrajectoryValidationError(
                f"最终执行计划的相邻关节点距离 {max_step:.6f}° "
                "超过 MuJoCo 逐点预检上限 1.0°。"
            )
        positions = tuple(
            tuple(kinematics.forward_position(joints))
            for joints in samples
        )
        minimum_tcp_z = min(position[2] for position in positions)
        if minimum_tcp_z < 0.0:
            raise SO100PlusTrajectoryValidationError(
                f"返回工作初始姿态路径的 TCP 最低 Z="
                f"{minimum_tcp_z:.6f} m，低于支撑平面。"
            )

        contacts = self._sample_contacts(samples, gripper_qpos)
        for index, pairs in enumerate(contacts):
            if pairs:
                raise SO100PlusTrajectoryValidationError(
                    "返回工作初始姿态路径第 "
                    f"{index}/{len(samples) - 1} 个采样点存在接触："
                    f"{sorted(pairs)}。"
                )
        return VerifiedJointMotionSequence(
            plans=plans_tuple,
            sampled_joint_radians=samples,
            gripper_qpos=gripper_qpos,
            report=SO100PlusTrajectoryValidationReport(
                sample_count=len(samples),
                max_joint_sample_step_degrees=(
                    self._max_sample_step_degrees(samples)
                ),
                minimum_tcp_z_m=minimum_tcp_z,
                initial_contact_pairs=contacts[0],
                final_contact_pairs=contacts[-1],
                last_contact_sample=max(
                    (
                        index
                        for index, pairs in enumerate(contacts)
                        if pairs
                    ),
                    default=-1,
                ),
            ),
        )

    def verify_storage_transition(
        self,
        plans: Sequence[JointMotionPlan],
        *,
        escape_joint_radians: Sequence[float],
        kinematics: SO100PlusKinematics,
        direction: StorageTransitionDirection,
        gripper_qpos: float,
    ) -> VerifiedJointMotionSequence:
        """验证包含固定 storage_escape 的完整展开或反向收纳计划。"""

        plans_tuple = tuple(plans)
        route = _plans_to_route(plans_tuple)
        escape = _finite_joints(
            escape_joint_radians,
            label="storage_escape",
        )
        escape_indices = tuple(
            index
            for index, joints in enumerate(route)
            if _same_joints(joints, escape)
        )
        if len(escape_indices) != 1:
            raise SO100PlusTrajectoryValidationError(
                "执行计划必须且只能经过一次固定 storage_escape。"
            )

        samples = route
        gripper_qpos = self._validate_gripper_qpos(gripper_qpos)
        max_step = self._max_sample_step_degrees(samples)
        if max_step > math.degrees(
            SO100_PLUS_COLLISION_SAMPLE_STEP_RADIANS
        ) + 1e-9:
            raise SO100PlusTrajectoryValidationError(
                f"最终执行计划的相邻关节点距离 {max_step:.6f}° "
                "超过 MuJoCo 逐点预检上限 1.0°。"
            )
        sample_escape_indices = tuple(
            index
            for index, joints in enumerate(samples)
            if _same_joints(joints, escape)
        )
        if len(sample_escape_indices) != 1:
            raise SO100PlusTrajectoryValidationError(
                "碰撞采样轨迹没有唯一包含 storage_escape。"
            )
        escape_index = sample_escape_indices[0]
        positions = tuple(
            tuple(kinematics.forward_position(joints))
            for joints in samples
        )
        contacts = self._sample_contacts(samples, gripper_qpos)
        minimum_tcp_z = min(position[2] for position in positions)

        if contacts[escape_index]:
            raise SO100PlusTrajectoryValidationError(
                "storage_escape 尚未脱离收纳接触："
                f"{sorted(contacts[escape_index])}。"
            )

        if direction is StorageTransitionDirection.UNFOLD:
            if any(
                current[2] < previous[2] - 1e-9
                for previous, current in zip(positions, positions[1:])
            ):
                raise SO100PlusTrajectoryValidationError(
                    "收纳姿态展开路径的 TCP 不是单调上升。"
                )
            allowed_storage_contacts = contacts[0]
            if contacts[-1]:
                raise SO100PlusTrajectoryValidationError(
                    "JoyCon 工作初始姿态仍存在接触："
                    f"{sorted(contacts[-1])}。"
                )
            for index, pairs in enumerate(contacts[:escape_index]):
                new_pairs = pairs - allowed_storage_contacts
                if new_pairs:
                    raise SO100PlusTrajectoryValidationError(
                        "展开路径产生收纳姿态中不存在的新接触："
                        f"采样点 {index}，{sorted(new_pairs)}。"
                    )
            for index, pairs in enumerate(
                contacts[escape_index:],
                start=escape_index,
            ):
                if pairs:
                    raise SO100PlusTrajectoryValidationError(
                        "storage_escape 之后的展开路径仍存在接触："
                        f"采样点 {index}，{sorted(pairs)}。"
                    )
        elif direction is StorageTransitionDirection.FOLD:
            if any(
                current[2] > previous[2] + 1e-9
                for previous, current in zip(positions, positions[1:])
            ):
                raise SO100PlusTrajectoryValidationError(
                    "反向收纳路径的 TCP 不是单调下降。"
                )
            for index, pairs in enumerate(contacts[: escape_index + 1]):
                if pairs:
                    raise SO100PlusTrajectoryValidationError(
                        "到达 storage_escape 前的收纳路径存在接触："
                        f"采样点 {index}，{sorted(pairs)}。"
                    )
            allowed_storage_contacts = contacts[-1]
            for index, pairs in enumerate(
                contacts[escape_index + 1 :],
                start=escape_index + 1,
            ):
                new_pairs = pairs - allowed_storage_contacts
                if new_pairs:
                    raise SO100PlusTrajectoryValidationError(
                        "反向收纳路径产生最终收纳姿态以外的新接触："
                        f"采样点 {index}，{sorted(new_pairs)}。"
                    )
        else:
            raise SO100PlusTrajectoryValidationError(
                f"未知收纳通道方向：{direction!r}。"
            )

        return VerifiedJointMotionSequence(
            plans=plans_tuple,
            sampled_joint_radians=samples,
            gripper_qpos=gripper_qpos,
            report=SO100PlusTrajectoryValidationReport(
                sample_count=len(samples),
                max_joint_sample_step_degrees=(
                    self._max_sample_step_degrees(samples)
                ),
                minimum_tcp_z_m=minimum_tcp_z,
                initial_contact_pairs=contacts[0],
                final_contact_pairs=contacts[-1],
                last_contact_sample=max(
                    (
                        index
                        for index, pairs in enumerate(contacts)
                        if pairs
                    ),
                    default=-1,
                ),
            ),
        )
