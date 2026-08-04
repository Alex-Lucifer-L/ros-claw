"""SO-100 Plus 不规则 WORK 空间的只读运行时表示。

本模块只加载离线扫描产物、验证目标点并构造规划限制。它不连接 Robot、
不访问串口，也不发送任何运动命令。
"""

from __future__ import annotations

from itertools import product
import hashlib
import math
from numbers import Real
from pathlib import Path
from typing import Sequence

import numpy as np

from rosclaw_mini.safety.limits import (
    AxisLimits,
    LimitViolationError,
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
    SO100_PLUS_MODEL_JOINT_LIMITS,
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS,
    SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS,
    WorkspaceLimits,
    build_so100_plus_right_follower_execution_joint_limits,
    resolve_relative_tcp_target,
)


DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH = (
    Path(__file__).resolve().parents[3]
    / "artifacts"
    / "so100_plus_middle_workspace_10mm_final"
    / "rest_workspace_grid.npz"
)
DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_SHA256 = (
    "c139f4e9f75343d01a368fea30dfc3d1b1d40ae13dfbc7f9f84e14f2bb34ad27"
)
SO100_PLUS_IRREGULAR_WORKSPACE_GRID_STEP_M = 0.01
SO100_PLUS_IRREGULAR_WORKSPACE_VALID_POINT_COUNT = 10_974
SO100_PLUS_IRREGULAR_WORKSPACE_GRIPPER_DEGREES = (-5.0, 60.0)
# NPZ 的有效点列表为 float32，坐标轴为 float64；1 µm 仅用于识别同一
# 1 cm 网格节点的序列化误差，不用于吸附用户目标。
_GRID_COORDINATE_TOLERANCE_M = 1e-6


class IrregularWorkspaceError(LimitViolationError):
    """目标或扫描产物不能通过不规则工作空间门禁。"""


def _finite_triplet(
    values: Sequence[float],
    *,
    label: str,
) -> tuple[float, float, float]:
    if isinstance(values, (str, bytes)):
        raise IrregularWorkspaceError(f"{label}需要 3 个有限数值。")
    try:
        raw = tuple(values)
    except TypeError as error:
        raise IrregularWorkspaceError(
            f"{label}需要 3 个有限数值。"
        ) from error
    if len(raw) != 3:
        raise IrregularWorkspaceError(f"{label}需要 3 个有限数值。")
    if any(
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(float(value))
        for value in raw
    ):
        raise IrregularWorkspaceError(f"{label}需要 3 个有限数值。")
    return tuple(float(value) for value in raw)  # type: ignore[return-value]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class RectangularWorkWorkspace:
    """兼容旧测试/Mock 的长方体策略；真机运行时使用不规则实现。"""

    requires_reference_hub = False

    def __init__(self, workspace: WorkspaceLimits) -> None:
        self.endpoint_aabb = workspace
        self.planning_envelope = workspace

    def validate_position(
        self,
        *position_m: float,
    ) -> tuple[float, float, float]:
        return self.endpoint_aabb.validate_position(*position_m)

    def resolve_relative_target(
        self,
        current_position_m: Sequence[float],
        displacement_m: Sequence[float],
    ) -> tuple[float, float, float]:
        return resolve_relative_tcp_target(
            current_position_m,
            displacement_m,
            self.endpoint_aabb,
        )

    def target_joint_radians_at_grid_point(
        self,
        position_m: Sequence[float],
    ) -> tuple[float, ...] | None:
        del position_m
        return None

    def build_motion_limits(
        self,
        current_joint_radians: Sequence[float],
        *,
        max_step_radians: float,
    ) -> MotionLimits:
        return MotionLimits(
            workspace=self.planning_envelope,
            joints=build_so100_plus_right_follower_execution_joint_limits(
                current_joint_radians,
                max_step_radians=max_step_radians,
            ),
        )


class SO100PlusIrregularWorkspace:
    """扫描网格形成的保守连续不规则空间和中心通道配置。"""

    requires_reference_hub = True

    def __init__(
        self,
        *,
        axes_m: tuple[np.ndarray, np.ndarray, np.ndarray],
        valid_mask: np.ndarray,
        target_joint_radians: np.ndarray,
        reference_index: tuple[int, int, int],
        reference_tcp_m: Sequence[float],
        reference_joint_radians: Sequence[float],
        source_path: Path,
        source_sha256: str,
    ) -> None:
        self._axes_m = tuple(
            np.asarray(axis, dtype=float).copy() for axis in axes_m
        )
        self._valid_mask = np.asarray(valid_mask, dtype=bool).copy()
        self._target_joint_radians = np.asarray(
            target_joint_radians,
            dtype=float,
        ).copy()
        self.reference_index = tuple(int(value) for value in reference_index)
        self.reference_tcp_m = _finite_triplet(
            reference_tcp_m,
            label="不规则空间参考 TCP",
        )
        self.reference_joint_radians = tuple(
            float(value) for value in reference_joint_radians
        )
        self.source_path = Path(source_path)
        self.source_sha256 = source_sha256

        valid_positions = self._grid_positions()[self._valid_mask]
        self._valid_tcp_points_m = np.asarray(valid_positions, dtype=float)
        lower = np.min(self._valid_tcp_points_m, axis=0)
        upper = np.max(self._valid_tcp_points_m, axis=0)
        self.endpoint_aabb = WorkspaceLimits(
            x=AxisLimits(float(lower[0]), float(upper[0])),
            y=AxisLimits(float(lower[1]), float(upper[1])),
            z=AxisLimits(float(lower[2]), float(upper[2])),
        )

        # 扫描发现少数 middle→目标关节路径的 TCP 会比端点网格 X 最大值
        # 多约 2.7 mm。规划外包框只供底层数值规划使用，不负责目标放行；
        # 每个用户目标仍必须通过上面的不规则单元门禁和最终 MuJoCo 预检。
        step = self.grid_step_m
        self.planning_envelope = WorkspaceLimits(
            x=AxisLimits(
                float(self._axes_m[0][0] - step),
                float(self._axes_m[0][-1] + step),
            ),
            y=AxisLimits(
                float(self._axes_m[1][0] - step),
                float(self._axes_m[1][-1] + step),
            ),
            z=AxisLimits(
                0.0,
                float(self._axes_m[2][-1] + step),
            ),
        )

    @property
    def grid_step_m(self) -> float:
        return float(self._axes_m[0][1] - self._axes_m[0][0])

    @property
    def valid_point_count(self) -> int:
        return int(np.count_nonzero(self._valid_mask))

    @property
    def valid_cell_count(self) -> int:
        mask = self._valid_mask
        cells = (
            mask[:-1, :-1, :-1]
            & mask[1:, :-1, :-1]
            & mask[:-1, 1:, :-1]
            & mask[:-1, :-1, 1:]
            & mask[1:, 1:, :-1]
            & mask[1:, :-1, 1:]
            & mask[:-1, 1:, 1:]
            & mask[1:, 1:, 1:]
        )
        return int(np.count_nonzero(cells))

    def _grid_positions(self) -> np.ndarray:
        grids = np.meshgrid(*self._axes_m, indexing="ij")
        return np.stack(grids, axis=-1)

    @classmethod
    def from_npz(
        cls,
        path: str | Path,
        *,
        expected_sha256: str | None = None,
        expected_reference_tcp_m: Sequence[float] | None = None,
        expected_reference_joint_radians: Sequence[float] | None = None,
        expected_valid_point_count: int | None = None,
        expected_grid_step_m: float | None = None,
        expected_gripper_driver_degrees: Sequence[float] | None = None,
    ) -> "SO100PlusIrregularWorkspace":
        source_path = Path(path)
        if not source_path.is_file():
            raise IrregularWorkspaceError(
                f"不规则工作空间网格不存在：{source_path}。"
            )
        actual_sha256 = _sha256(source_path)
        if (
            expected_sha256 is not None
            and actual_sha256 != expected_sha256
        ):
            raise IrregularWorkspaceError(
                "不规则工作空间网格 SHA-256 不匹配："
                f"期望 {expected_sha256}，实际 {actual_sha256}。"
            )

        required = {
            "x_m",
            "y_m",
            "z_m",
            "status",
            "target_joint_radians",
            "valid_status_code",
            "rest_index",
            "rest_tcp_m",
            "rest_joint_radians",
            "valid_tcp_points_m",
            "gripper_driver_degrees",
            "gripper_qpos_radians",
        }
        try:
            with np.load(source_path, allow_pickle=False) as data:
                missing = sorted(required.difference(data.files))
                if missing:
                    raise IrregularWorkspaceError(
                        "不规则工作空间网格缺少字段："
                        + ", ".join(missing)
                        + "。"
                    )
                axes = tuple(
                    np.asarray(data[name], dtype=float)
                    for name in ("x_m", "y_m", "z_m")
                )
                status = np.asarray(data["status"])
                valid_code_array = np.asarray(data["valid_status_code"])
                if valid_code_array.size != 1:
                    raise IrregularWorkspaceError(
                        "valid_status_code 必须是单一数值。"
                    )
                valid_code = int(valid_code_array.reshape(-1)[0])
                valid_mask = status == valid_code
                target_joints = np.asarray(
                    data["target_joint_radians"],
                    dtype=float,
                )
                reference_index = tuple(
                    int(value) for value in np.asarray(data["rest_index"])
                )
                reference_tcp = np.asarray(data["rest_tcp_m"], dtype=float)
                reference_joints = np.asarray(
                    data["rest_joint_radians"],
                    dtype=float,
                )
                valid_tcp_points = np.asarray(
                    data["valid_tcp_points_m"],
                    dtype=float,
                )
                gripper_degrees = np.asarray(
                    data["gripper_driver_degrees"],
                    dtype=float,
                )
                gripper_qpos = np.asarray(
                    data["gripper_qpos_radians"],
                    dtype=float,
                )
        except IrregularWorkspaceError:
            raise
        except Exception as error:
            raise IrregularWorkspaceError(
                f"无法读取不规则工作空间网格：{source_path}。"
            ) from error

        cls._validate_arrays(
            axes=axes,
            status=status,
            valid_mask=valid_mask,
            target_joints=target_joints,
            reference_index=reference_index,
            reference_tcp=reference_tcp,
            reference_joints=reference_joints,
            valid_tcp_points=valid_tcp_points,
            gripper_degrees=gripper_degrees,
            gripper_qpos=gripper_qpos,
        )
        workspace = cls(
            axes_m=axes,  # type: ignore[arg-type]
            valid_mask=valid_mask,
            target_joint_radians=target_joints,
            reference_index=reference_index,  # type: ignore[arg-type]
            reference_tcp_m=reference_tcp,
            reference_joint_radians=reference_joints,
            source_path=source_path,
            source_sha256=actual_sha256,
        )
        workspace._validate_expected_certification(
            expected_reference_tcp_m=expected_reference_tcp_m,
            expected_reference_joint_radians=(
                expected_reference_joint_radians
            ),
            expected_valid_point_count=expected_valid_point_count,
            expected_grid_step_m=expected_grid_step_m,
            expected_gripper_driver_degrees=(
                expected_gripper_driver_degrees
            ),
            actual_gripper_driver_degrees=gripper_degrees,
        )
        return workspace

    @staticmethod
    def _validate_arrays(
        *,
        axes: tuple[np.ndarray, ...],
        status: np.ndarray,
        valid_mask: np.ndarray,
        target_joints: np.ndarray,
        reference_index: tuple[int, ...],
        reference_tcp: np.ndarray,
        reference_joints: np.ndarray,
        valid_tcp_points: np.ndarray,
        gripper_degrees: np.ndarray,
        gripper_qpos: np.ndarray,
    ) -> None:
        if len(axes) != 3:
            raise IrregularWorkspaceError("工作空间网格必须包含三个坐标轴。")
        steps = []
        for name, axis in zip(("x", "y", "z"), axes, strict=True):
            if axis.ndim != 1 or len(axis) < 2 or not np.all(np.isfinite(axis)):
                raise IrregularWorkspaceError(
                    f"工作空间 {name} 轴必须是一维有限递增数组。"
                )
            differences = np.diff(axis)
            if np.any(differences <= 0) or not np.allclose(
                differences,
                differences[0],
                atol=1e-12,
                rtol=0.0,
            ):
                raise IrregularWorkspaceError(
                    f"工作空间 {name} 轴必须等间距严格递增。"
                )
            steps.append(float(differences[0]))
        if not np.allclose(steps, steps[0], atol=1e-12, rtol=0.0):
            raise IrregularWorkspaceError("三个工作空间坐标轴步长不一致。")

        shape = tuple(len(axis) for axis in axes)
        if status.shape != shape or valid_mask.shape != shape:
            raise IrregularWorkspaceError("status 形状与三维网格不一致。")
        if target_joints.shape != (*shape, len(SO100_PLUS_ARM_JOINT_NAMES)):
            raise IrregularWorkspaceError(
                "target_joint_radians 形状与三维网格/六关节不一致。"
            )
        if len(reference_index) != 3 or any(
            index < 0 or index >= shape[axis]
            for axis, index in enumerate(reference_index)
        ):
            raise IrregularWorkspaceError("参考网格索引无效。")
        if not valid_mask[reference_index]:
            raise IrregularWorkspaceError("参考网格点不是有效点。")
        if reference_tcp.shape != (3,) or not np.all(np.isfinite(reference_tcp)):
            raise IrregularWorkspaceError("参考 TCP 字段无效。")
        if reference_joints.shape != (6,) or not np.all(
            np.isfinite(reference_joints)
        ):
            raise IrregularWorkspaceError("参考六关节字段无效。")
        valid_targets = target_joints[valid_mask]
        if len(valid_targets) == 0 or not np.all(np.isfinite(valid_targets)):
            raise IrregularWorkspaceError("有效网格点缺少有限六关节解。")

        grid_positions = np.stack(
            np.meshgrid(*axes, indexing="ij"),
            axis=-1,
        )[valid_mask]
        if valid_tcp_points.shape != grid_positions.shape or not np.allclose(
            valid_tcp_points,
            grid_positions,
            atol=1e-6,
            rtol=0.0,
        ):
            raise IrregularWorkspaceError(
                "valid_tcp_points_m 与 status 有效点不一致。"
            )
        if gripper_degrees.ndim != 1 or not np.all(np.isfinite(gripper_degrees)):
            raise IrregularWorkspaceError("夹爪驱动角字段无效。")
        if gripper_qpos.shape != gripper_degrees.shape or not np.allclose(
            gripper_qpos,
            np.radians(gripper_degrees),
            atol=1e-12,
            rtol=0.0,
        ):
            raise IrregularWorkspaceError("夹爪驱动角与模型 qpos 映射不一致。")

        lower = np.asarray(SO100_PLUS_MODEL_JOINT_LIMITS.lower_radians)
        upper = np.asarray(SO100_PLUS_MODEL_JOINT_LIMITS.upper_radians)
        if np.any(valid_targets < lower - 1e-8) or np.any(
            valid_targets > upper + 1e-8
        ):
            raise IrregularWorkspaceError("有效网格包含超出模型范围的关节解。")
        base_driver_degrees = -np.degrees(valid_targets[:, 0])
        base_limits = SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS
        if np.any(base_driver_degrees < base_limits.minimum - 1e-6) or np.any(
            base_driver_degrees > base_limits.maximum + 1e-6
        ):
            raise IrregularWorkspaceError(
                "有效网格包含超出 right_follower 实测底座范围的关节解。"
            )

    def _validate_expected_certification(
        self,
        *,
        expected_reference_tcp_m: Sequence[float] | None,
        expected_reference_joint_radians: Sequence[float] | None,
        expected_valid_point_count: int | None,
        expected_grid_step_m: float | None,
        expected_gripper_driver_degrees: Sequence[float] | None,
        actual_gripper_driver_degrees: np.ndarray,
    ) -> None:
        if expected_reference_tcp_m is not None and not np.allclose(
            self.reference_tcp_m,
            _finite_triplet(
                expected_reference_tcp_m,
                label="期望参考 TCP",
            ),
            atol=1e-8,
            rtol=0.0,
        ):
            raise IrregularWorkspaceError("网格参考 TCP 与登记配置不一致。")
        if expected_reference_joint_radians is not None:
            expected_joints = tuple(
                float(value) for value in expected_reference_joint_radians
            )
            if len(expected_joints) != 6 or not np.allclose(
                self.reference_joint_radians,
                expected_joints,
                atol=1e-8,
                rtol=0.0,
            ):
                raise IrregularWorkspaceError(
                    "网格参考关节姿态与登记配置不一致。"
                )
        if (
            expected_valid_point_count is not None
            and self.valid_point_count != expected_valid_point_count
        ):
            raise IrregularWorkspaceError(
                f"网格有效点数量 {self.valid_point_count} 与登记值 "
                f"{expected_valid_point_count} 不一致。"
            )
        if expected_grid_step_m is not None and not math.isclose(
            self.grid_step_m,
            expected_grid_step_m,
            abs_tol=1e-12,
            rel_tol=0.0,
        ):
            raise IrregularWorkspaceError(
                f"网格步长 {self.grid_step_m} m 与登记值 "
                f"{expected_grid_step_m} m 不一致。"
            )
        if expected_gripper_driver_degrees is not None and not np.allclose(
            actual_gripper_driver_degrees,
            tuple(float(value) for value in expected_gripper_driver_degrees),
            atol=1e-12,
            rtol=0.0,
        ):
            raise IrregularWorkspaceError("网格夹爪姿态交集与登记配置不一致。")

    def _axis_indices(
        self,
        axis: np.ndarray,
        value: float,
        axis_name: str,
    ) -> tuple[int, ...]:
        if value < axis[0] - _GRID_COORDINATE_TOLERANCE_M or value > (
            axis[-1] + _GRID_COORDINATE_TOLERANCE_M
        ):
            raise IrregularWorkspaceError(
                f"{axis_name}={value} 超出扫描网格范围 "
                f"[{axis[0]}, {axis[-1]}]。"
            )
        step = float(axis[1] - axis[0])
        coordinate = (value - float(axis[0])) / step
        nearest = int(round(coordinate))
        nearest = min(max(nearest, 0), len(axis) - 1)
        if abs(value - float(axis[nearest])) <= _GRID_COORDINATE_TOLERANCE_M:
            return (nearest,)
        lower = math.floor(coordinate)
        if lower < 0 or lower + 1 >= len(axis):
            raise IrregularWorkspaceError(
                f"{axis_name}={value} 不能形成完整扫描单元。"
            )
        return (lower, lower + 1)

    def validate_position(
        self,
        *position_m: float,
    ) -> tuple[float, float, float]:
        position = _finite_triplet(position_m, label="夹爪 TCP 目标")
        try:
            brackets = tuple(
                self._axis_indices(axis, value, name)
                for axis, value, name in zip(
                    self._axes_m,
                    position,
                    ("x", "y", "z"),
                    strict=True,
                )
            )
        except IrregularWorkspaceError as error:
            raise self._position_error(position, str(error)) from error

        corners = tuple(product(*brackets))
        invalid = tuple(
            index for index in corners if not self._valid_mask[index]
        )
        if invalid:
            raise self._position_error(
                position,
                f"所在离散单元的 {len(invalid)}/{len(corners)} 个"
                "必要角点无效；不允许跨过空洞或未验证边界。",
            )
        return position

    def _position_error(
        self,
        position: tuple[float, float, float],
        reason: str,
    ) -> IrregularWorkspaceError:
        deltas = self._valid_tcp_points_m - np.asarray(position, dtype=float)
        distances = np.linalg.norm(deltas, axis=1)
        nearest_index = int(np.argmin(distances))
        nearest = tuple(
            float(value) for value in self._valid_tcp_points_m[nearest_index]
        )
        return IrregularWorkspaceError(
            "目标不在 SO-100 Plus 不规则 WORK 空间："
            f"目标={position} m；{reason}最近有效网格点={nearest} m，"
            f"距离={float(distances[nearest_index]) * 1000:.3f} mm。"
        )

    def resolve_relative_target(
        self,
        current_position_m: Sequence[float],
        displacement_m: Sequence[float],
    ) -> tuple[float, float, float]:
        current = _finite_triplet(current_position_m, label="当前 TCP")
        displacement = _finite_triplet(displacement_m, label="相对位移")
        if all(value == 0.0 for value in displacement):
            raise IrregularWorkspaceError(
                "相对位移 dx/dy/dz 不能全部为 0；未发送运动。"
            )
        target = tuple(
            value + delta
            for value, delta in zip(current, displacement, strict=True)
        )
        try:
            return self.validate_position(*target)
        except IrregularWorkspaceError as error:
            raise IrregularWorkspaceError(
                "相对移动最终目标违反不规则工作空间："
                f"当前 TCP={current} m；请求位移 dx/dy/dz={displacement} m；"
                f"最终目标={target} m；{error}"
            ) from error

    def target_joint_radians_at_grid_point(
        self,
        position_m: Sequence[float],
    ) -> tuple[float, ...] | None:
        position = _finite_triplet(position_m, label="夹爪 TCP 目标")
        indices = []
        for axis, value in zip(self._axes_m, position, strict=True):
            step = float(axis[1] - axis[0])
            coordinate = (value - float(axis[0])) / step
            nearest = int(round(coordinate))
            if not 0 <= nearest < len(axis) or abs(
                value - float(axis[nearest])
            ) > _GRID_COORDINATE_TOLERANCE_M:
                return None
            indices.append(nearest)
        index = tuple(indices)
        if not self._valid_mask[index]:
            return None
        return tuple(float(value) for value in self._target_joint_radians[index])

    def build_motion_limits(
        self,
        current_joint_radians: Sequence[float],
        *,
        max_step_radians: float,
    ) -> MotionLimits:
        return MotionLimits(
            workspace=self.planning_envelope,
            joints=build_so100_plus_right_follower_execution_joint_limits(
                current_joint_radians,
                max_step_radians=max_step_radians,
            ),
        )


def load_default_so100_plus_irregular_workspace(
) -> SO100PlusIrregularWorkspace:
    """加载并绑定当前 right_follower 的固定扫描快照。"""

    # 延迟导入，避免 so100_plus_session 导入本模块时形成循环依赖。
    from rosclaw_mini.arm.so100_plus_session import (
        SO100_PLUS_MIDDLE_INTERNAL_RADIANS,
        SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M,
    )

    return SO100PlusIrregularWorkspace.from_npz(
        DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_PATH,
        expected_sha256=(
            DEFAULT_SO100_PLUS_IRREGULAR_WORKSPACE_GRID_SHA256
        ),
        expected_reference_tcp_m=(
            SO100_PLUS_MIDDLE_INTERNAL_TCP_POSITION_M
        ),
        expected_reference_joint_radians=(
            SO100_PLUS_MIDDLE_INTERNAL_RADIANS
        ),
        expected_valid_point_count=(
            SO100_PLUS_IRREGULAR_WORKSPACE_VALID_POINT_COUNT
        ),
        expected_grid_step_m=(
            SO100_PLUS_IRREGULAR_WORKSPACE_GRID_STEP_M
        ),
        expected_gripper_driver_degrees=(
            SO100_PLUS_IRREGULAR_WORKSPACE_GRIPPER_DEGREES
        ),
    )


def default_rectangular_work_workspace() -> RectangularWorkWorkspace:
    """仅供旧调用和无网格测试使用的保守兼容策略。"""

    return RectangularWorkWorkspace(
        SO100_PLUS_RIGHT_FOLLOWER_WORKSPACE_LIMITS
    )
