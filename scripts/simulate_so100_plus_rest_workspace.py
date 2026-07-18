"""离线扫描保持 JoyCon 控制器初始 TCP 姿态时的工作空间。

每个笛卡尔网格点都从控制器初始关节姿态独立求解，并检查端点及从
初始姿态到目标关节角的插值路径。脚本不访问任何真实设备。
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_GRIPPER_TCP_OFFSET_M,
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.safety.limits import (
    JointLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS,
)


DEFAULT_MODEL_PATH = Path(
    "lerobot-joycon_plus/lerobot/common/robot_devices/controllers/scene_plus.xml"
)
DEFAULT_CANDIDATE_REPORT_PATH = Path(
    "artifacts/so100_plus_workspace/workspace_report.json"
)
DEFAULT_OUTPUT_DIR = Path("artifacts/so100_plus_rest_workspace")
ARM_JOINT_COUNT = len(SO100_PLUS_ARM_JOINT_NAMES)
GRIPPER_MODEL_RADIANS = 0.0

STATUS_IK_FAILED = 1
STATUS_ENDPOINT_COLLISION = 2
STATUS_PATH_COLLISION = 3
STATUS_VALID = 4

STATUS_LABELS = {
    STATUS_IK_FAILED: "ik_failed",
    STATUS_ENDPOINT_COLLISION: "endpoint_collision",
    STATUS_PATH_COLLISION: "path_collision",
    STATUS_VALID: "valid",
}


@dataclass(frozen=True)
class RestCenteredBox:
    """包含控制器初始网格点且全部有效的轴对齐长方体。"""

    x_start: int
    x_end: int
    y_start: int
    y_end: int
    z_start: int
    z_end: int

    @property
    def point_count(self) -> int:
        return (
            (self.x_end - self.x_start + 1)
            * (self.y_end - self.y_start + 1)
            * (self.z_end - self.z_start + 1)
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "纯离线扫描保持 JoyCon 控制器初始 TCP 姿态时的工作空间；"
            "不会连接、上力或移动真实机械臂。"
        )
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument(
        "--candidate-report",
        type=Path,
        default=DEFAULT_CANDIDATE_REPORT_PATH,
        help="上一阶段所有姿态候选工作空间报告",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--grid-step-mm",
        type=float,
        default=20.0,
        help="笛卡尔网格步长，单位 mm；默认 20",
    )
    parser.add_argument(
        "--path-step-degrees",
        type=float,
        default=2.0,
        help="从控制器初始姿态到目标的关节路径检查步长；默认 2 度",
    )
    parser.add_argument(
        "--bounds-m",
        nargs=6,
        type=float,
        metavar=(
            "X_MIN",
            "X_MAX",
            "Y_MIN",
            "Y_MAX",
            "Z_MIN",
            "Z_MAX",
        ),
        help=(
            "可选精扫边界，必须包含控制器初始点且位于候选外包围盒内"
        ),
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if (
        not math.isfinite(args.grid_step_mm)
        or not 5.0 <= args.grid_step_mm <= 50.0
    ):
        raise SystemExit("grid-step-mm 必须在 [5, 50] 范围内。")
    if (
        not math.isfinite(args.path_step_degrees)
        or not 0.5 <= args.path_step_degrees <= 5.0
    ):
        raise SystemExit("path-step-degrees 必须在 [0.5, 5] 范围内。")


def build_simulation_joint_limits(
    model: mujoco.MjModel,
) -> JointLimits:
    """底座使用实测范围，其余五关节使用 MuJoCo 范围。"""

    names = tuple(
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            index,
        )
        for index in range(ARM_JOINT_COUNT)
    )
    if names != SO100_PLUS_ARM_JOINT_NAMES:
        raise RuntimeError(
            "MuJoCo 前 6 个关节名称或顺序与 SO-100 Plus 不一致。"
        )

    lower = np.asarray(model.jnt_range[:ARM_JOINT_COUNT, 0], dtype=float).copy()
    upper = np.asarray(model.jnt_range[:ARM_JOINT_COUNT, 1], dtype=float).copy()
    driver_limits = (
        SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS
    )
    lower[0] = math.radians(-driver_limits.maximum)
    upper[0] = math.radians(-driver_limits.minimum)
    return JointLimits(
        joint_names=SO100_PLUS_ARM_JOINT_NAMES,
        lower_radians=tuple(lower),
        upper_radians=tuple(upper),
        max_step_radians=(0.1,) * ARM_JOINT_COUNT,
    )


def centered_axis_values(
    center_m: float,
    minimum_m: float,
    maximum_m: float,
    step_m: float,
) -> tuple[np.ndarray, int]:
    """创建以 center 为一个精确网格点的等间距坐标轴。"""

    values = (center_m, minimum_m, maximum_m, step_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("网格坐标必须是有限数值。")
    if minimum_m > center_m or center_m > maximum_m:
        raise ValueError("网格中心必须位于候选范围内。")
    if step_m <= 0:
        raise ValueError("网格步长必须是正数。")

    minimum_step = math.ceil(
        (minimum_m - center_m) / step_m - 1e-12
    )
    maximum_step = math.floor(
        (maximum_m - center_m) / step_m + 1e-12
    )
    integer_steps = np.arange(minimum_step, maximum_step + 1, dtype=int)
    axis = center_m + integer_steps.astype(float) * step_m
    center_indices = np.flatnonzero(integer_steps == 0)
    if len(center_indices) != 1:
        raise RuntimeError("生成的坐标轴没有唯一 rest 网格点。")
    return axis, int(center_indices[0])


def batch_tcp_positions(
    kinematics: SO100PlusKinematics,
    joint_radians: np.ndarray,
) -> np.ndarray:
    joints = np.asarray(joint_radians, dtype=float)
    if joints.ndim != 2 or joints.shape[1] != ARM_JOINT_COUNT:
        raise ValueError("批量关节角必须是形状为 (N, 6) 的数组。")
    transform_object = kinematics.robot.fkine(joints)
    transforms = np.asarray(
        getattr(transform_object, "A", transform_object),
        dtype=float,
    )
    if transforms.ndim == 2:
        transforms = transforms[np.newaxis, :, :]
    offset = np.asarray(SO100_PLUS_GRIPPER_TCP_OFFSET_M, dtype=float)
    return (
        transforms[:, :3, 3]
        + np.einsum("nij,j->ni", transforms[:, :3, :3], offset)
    )


def pose_has_collision(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    qpos: np.ndarray,
    tcp_position_m: np.ndarray,
) -> bool:
    """检查自碰撞、与 Z=0 支撑平面接触和 TCP 低于平面。"""

    if tcp_position_m[2] < 0.0:
        return True
    data.qpos[:ARM_JOINT_COUNT] = qpos
    data.qpos[ARM_JOINT_COUNT] = GRIPPER_MODEL_RADIANS
    mujoco.mj_forward(model, data)
    return bool(data.ncon)


def path_has_collision(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    kinematics: SO100PlusKinematics,
    start_qpos: np.ndarray,
    target_qpos: np.ndarray,
    *,
    max_step_radians: float,
) -> bool:
    """线性插值关节路径并检查每个中间姿态。"""

    delta = target_qpos - start_qpos
    step_count = max(
        1,
        math.ceil(float(np.max(np.abs(delta))) / max_step_radians),
    )
    fractions = np.arange(1, step_count + 1, dtype=float) / step_count
    waypoints = start_qpos + fractions[:, np.newaxis] * delta
    tcp_positions = batch_tcp_positions(kinematics, waypoints)
    for qpos, tcp in zip(waypoints, tcp_positions, strict=True):
        if pose_has_collision(model, data, qpos, tcp):
            return True
    return False


def largest_valid_box_containing_center(
    valid_mask: np.ndarray,
    center_index: tuple[int, int, int],
) -> RestCenteredBox:
    """暴力搜索包含 center 且全部为 True 的最大体积网格长方体。"""

    mask = np.asarray(valid_mask, dtype=bool)
    if mask.ndim != 3 or any(size == 0 for size in mask.shape):
        raise ValueError("有效网格必须是非空三维布尔数组。")
    cx, cy, cz = center_index
    if not (
        0 <= cx < mask.shape[0]
        and 0 <= cy < mask.shape[1]
        and 0 <= cz < mask.shape[2]
    ):
        raise ValueError("中心索引超出网格。")
    if not mask[cx, cy, cz]:
        raise ValueError("中心网格点必须有效。")

    best = RestCenteredBox(cx, cx, cy, cy, cz, cz)
    best_score = (0, best.point_count)

    for x_start in range(cx + 1):
        common_yz = np.ones(mask.shape[1:], dtype=bool)
        for x_end in range(x_start, mask.shape[0]):
            common_yz &= mask[x_end]
            if not common_yz[cy, cz]:
                break
            if x_end < cx:
                continue

            for y_start in range(cy + 1):
                common_z = np.ones(mask.shape[2], dtype=bool)
                for y_end in range(y_start, mask.shape[1]):
                    common_z &= common_yz[y_end]
                    if not common_z[cz]:
                        break
                    if y_end < cy:
                        continue

                    z_start = cz
                    while z_start > 0 and common_z[z_start - 1]:
                        z_start -= 1
                    z_end = cz
                    while (
                        z_end + 1 < len(common_z)
                        and common_z[z_end + 1]
                    ):
                        z_end += 1

                    candidate = RestCenteredBox(
                        x_start,
                        x_end,
                        y_start,
                        y_end,
                        z_start,
                        z_end,
                    )
                    # 第一项对应物理长方体体积的网格间隔乘积。
                    interval_volume = (
                        (x_end - x_start)
                        * (y_end - y_start)
                        * (z_end - z_start)
                    )
                    score = (interval_volume, candidate.point_count)
                    if score > best_score:
                        best = candidate
                        best_score = score
    return best


def iter_directed_neighbor_edges(
    box: RestCenteredBox,
):
    """生成长方体内三个坐标轴相邻网格点的双向边。"""

    starts = (box.x_start, box.y_start, box.z_start)
    ends = (box.x_end, box.y_end, box.z_end)
    for x_index in range(box.x_start, box.x_end + 1):
        for y_index in range(box.y_start, box.y_end + 1):
            for z_index in range(box.z_start, box.z_end + 1):
                source = (x_index, y_index, z_index)
                for axis in range(3):
                    target_values = [x_index, y_index, z_index]
                    if target_values[axis] >= ends[axis]:
                        continue
                    target_values[axis] += 1
                    target = tuple(target_values)
                    yield source, target
                    yield target, source


def _candidate_bounds_from_report(
    report_path: Path,
) -> tuple[np.ndarray, np.ndarray]:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    bounds = report["tcp_workspace"]["collision_free_sample_aabb_m"]
    lower = np.asarray(
        [bounds[axis]["minimum"] for axis in ("x", "y", "z")],
        dtype=float,
    )
    upper = np.asarray(
        [bounds[axis]["maximum"] for axis in ("x", "y", "z")],
        dtype=float,
    )
    if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
        raise RuntimeError("候选工作空间报告包含非有限边界。")
    if np.any(lower >= upper):
        raise RuntimeError("候选工作空间报告边界无效。")
    return lower, upper


def select_grid_bounds(
    rest_tcp_m: np.ndarray,
    candidate_lower_m: np.ndarray,
    candidate_upper_m: np.ndarray,
    override_bounds_m: list[float] | tuple[float, ...] | None,
) -> tuple[np.ndarray, np.ndarray, str]:
    """选择全候选范围或经过验证的显式精扫范围。"""

    rest = np.asarray(rest_tcp_m, dtype=float)
    candidate_lower = np.asarray(candidate_lower_m, dtype=float)
    candidate_upper = np.asarray(candidate_upper_m, dtype=float)
    if override_bounds_m is None:
        return candidate_lower.copy(), candidate_upper.copy(), "candidate_aabb"

    values = np.asarray(override_bounds_m, dtype=float)
    if values.shape != (6,) or np.any(~np.isfinite(values)):
        raise ValueError("bounds-m 必须包含 6 个有限数值。")
    lower = values[[0, 2, 4]]
    upper = values[[1, 3, 5]]
    if np.any(lower >= upper):
        raise ValueError("bounds-m 每个轴的下限必须小于上限。")
    if np.any(lower < candidate_lower - 1e-12) or np.any(
        upper > candidate_upper + 1e-12
    ):
        raise ValueError("bounds-m 不能超出所有姿态候选外包围盒。")
    if np.any(rest < lower - 1e-12) or np.any(rest > upper + 1e-12):
        raise ValueError("bounds-m 必须包含 JoyCon 控制器初始 TCP。")
    if lower[2] < 0:
        raise ValueError("bounds-m 的 Z 下限不能低于支撑平面。")
    return lower, upper, "explicit_refinement_bounds"


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def _box_report(
    box: RestCenteredBox,
    axes: tuple[np.ndarray, np.ndarray, np.ndarray],
    step_m: float,
) -> dict[str, Any]:
    x_values, y_values, z_values = axes
    starts = (box.x_start, box.y_start, box.z_start)
    ends = (box.x_end, box.y_end, box.z_end)
    values = (x_values, y_values, z_values)
    result: dict[str, Any] = {}
    spans = []
    for axis, axis_values, start, end in zip(
        ("x", "y", "z"),
        values,
        starts,
        ends,
        strict=True,
    ):
        minimum = float(axis_values[start])
        maximum = float(axis_values[end])
        span = maximum - minimum
        spans.append(span)
        result[axis] = {
            "minimum": minimum,
            "maximum": maximum,
            "span": span,
            "grid_points": end - start + 1,
        }
    result["point_count"] = box.point_count
    result["grid_step_m"] = step_m
    result["continuous_box_volume_m3"] = float(np.prod(spans))
    result["warning"] = (
        "All sampled grid points and their rest-to-target paths passed, "
        "but unsampled interior and arbitrary point-to-point paths are not proven."
    )
    return result


def _save_plot(
    output_path: Path,
    valid_positions_m: np.ndarray,
    rest_tcp_m: np.ndarray,
    box_report: dict[str, Any],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    figure = plt.figure(figsize=(14, 11))
    xy = figure.add_subplot(2, 2, 1)
    xz = figure.add_subplot(2, 2, 2)
    yz = figure.add_subplot(2, 2, 3)
    view3d = figure.add_subplot(2, 2, 4, projection="3d")

    color = valid_positions_m[:, 2]
    options: dict[str, Any] = {
        "c": color,
        "s": 7,
        "alpha": 0.55,
        "cmap": "viridis",
        "linewidths": 0,
    }
    xy.scatter(valid_positions_m[:, 0], valid_positions_m[:, 1], **options)
    xz.scatter(valid_positions_m[:, 0], valid_positions_m[:, 2], **options)
    yz.scatter(valid_positions_m[:, 1], valid_positions_m[:, 2], **options)
    view3d.scatter(
        valid_positions_m[:, 0],
        valid_positions_m[:, 1],
        valid_positions_m[:, 2],
        **options,
    )

    projections = (
        (xy, "x", "y", 0, 1, "Top view: XY"),
        (xz, "x", "z", 0, 2, "Side view: XZ"),
        (yz, "y", "z", 1, 2, "Front view: YZ"),
    )
    for axis, first, second, i, j, title in projections:
        axis.scatter(
            rest_tcp_m[i],
            rest_tcp_m[j],
            c="red",
            marker="*",
            s=100,
            zorder=3,
        )
        axis.add_patch(
            Rectangle(
                (
                    box_report[first]["minimum"],
                    box_report[second]["minimum"],
                ),
                box_report[first]["span"],
                box_report[second]["span"],
                fill=False,
                edgecolor="red",
                linewidth=2,
            )
        )
        axis.set(
            xlabel=f"{first.upper()} (m)",
            ylabel=f"{second.upper()} (m)",
            title=title,
        )
        axis.grid(alpha=0.2)
        axis.set_aspect("equal", adjustable="box")

    view3d.scatter(
        rest_tcp_m[0],
        rest_tcp_m[1],
        rest_tcp_m[2],
        c="red",
        marker="*",
        s=100,
        label="JoyCon controller initial TCP",
    )
    view3d.set(
        xlabel="X (m)",
        ylabel="Y (m)",
        zlabel="Z (m)",
        title="JoyCon-initial-orientation path-valid grid",
    )
    view3d.legend(loc="upper right")
    figure.suptitle(
        "SO-100 Plus JoyCon-initial-orientation candidate workspace\n"
        "Red rectangles = largest all-valid grid box containing initial",
        fontsize=14,
    )
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def main() -> int:
    args = build_parser().parse_args()
    _validate_args(args)

    model_path = args.model.resolve()
    candidate_report_path = args.candidate_report.resolve()
    if not model_path.is_file():
        raise SystemExit(f"MuJoCo 模型不存在：{model_path}")
    if not candidate_report_path.is_file():
        raise SystemExit(
            f"候选工作空间报告不存在：{candidate_report_path}"
        )

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "rest_workspace_report.json"
    grid_path = output_dir / "rest_workspace_grid.npz"
    plot_path = output_dir / "rest_workspace_views.png"

    print("模式：纯离线固定姿态 IK；真实硬件访问 0。", flush=True)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    kinematics = SO100PlusKinematics()
    joint_limits = build_simulation_joint_limits(model)
    rest_qpos = np.asarray(SO100_PLUS_JOYCON_INITIAL_RADIANS, dtype=float)
    joint_limits.validate_position(rest_qpos)
    rest_tcp_m = np.asarray(
        kinematics.forward_position(rest_qpos),
        dtype=float,
    )
    if pose_has_collision(model, data, rest_qpos, rest_tcp_m):
        raise RuntimeError("JoyCon 控制器初始姿态在当前模型中发生碰撞。")

    candidate_lower, candidate_upper = _candidate_bounds_from_report(
        candidate_report_path
    )
    grid_lower, grid_upper, bounds_source = select_grid_bounds(
        rest_tcp_m,
        candidate_lower,
        candidate_upper,
        args.bounds_m,
    )
    step_m = args.grid_step_mm / 1000.0
    x_values, rest_x_index = centered_axis_values(
        rest_tcp_m[0],
        grid_lower[0],
        grid_upper[0],
        step_m,
    )
    y_values, rest_y_index = centered_axis_values(
        rest_tcp_m[1],
        grid_lower[1],
        grid_upper[1],
        step_m,
    )
    z_values, rest_z_index = centered_axis_values(
        rest_tcp_m[2],
        max(0.0, grid_lower[2]),
        grid_upper[2],
        step_m,
    )
    shape = (len(x_values), len(y_values), len(z_values))
    total_points = int(np.prod(shape))
    print(
        f"网格：{shape[0]} × {shape[1]} × {shape[2]} "
        f"= {total_points} 点，步长 {args.grid_step_mm:g} mm。",
        flush=True,
    )

    status = np.zeros(shape, dtype=np.uint8)
    target_joint_radians = np.full(
        (*shape, ARM_JOINT_COUNT),
        np.nan,
        dtype=np.float32,
    )
    failure_reasons: Counter[str] = Counter()
    max_path_step_radians = math.radians(args.path_step_degrees)

    for x_index, x in enumerate(x_values):
        for y_index, y in enumerate(y_values):
            for z_index, z in enumerate(z_values):
                target = np.asarray((x, y, z), dtype=float)
                try:
                    solution = np.asarray(
                        kinematics.solve_position(
                            rest_qpos,
                            target,
                            joint_limits=joint_limits,
                        ),
                        dtype=float,
                    )
                except Exception as error:
                    status[x_index, y_index, z_index] = STATUS_IK_FAILED
                    failure_reasons[type(error).__name__] += 1
                    continue

                target_joint_radians[x_index, y_index, z_index] = solution
                if pose_has_collision(model, data, solution, target):
                    status[
                        x_index,
                        y_index,
                        z_index,
                    ] = STATUS_ENDPOINT_COLLISION
                    continue
                if path_has_collision(
                    model,
                    data,
                    kinematics,
                    rest_qpos,
                    solution,
                    max_step_radians=max_path_step_radians,
                ):
                    status[
                        x_index,
                        y_index,
                        z_index,
                    ] = STATUS_PATH_COLLISION
                    continue
                status[x_index, y_index, z_index] = STATUS_VALID

        processed = (x_index + 1) * shape[1] * shape[2]
        print(
            f"已处理 X 平面 {x_index + 1}/{shape[0]}，"
            f"累计 {processed}/{total_points} 点，"
            f"有效 {int(np.count_nonzero(status == STATUS_VALID))}。",
            flush=True,
        )

    rest_index = (rest_x_index, rest_y_index, rest_z_index)
    if status[rest_index] != STATUS_VALID:
        raise RuntimeError("rest 网格点没有通过自身固定姿态检查。")
    valid_mask = status == STATUS_VALID
    box = largest_valid_box_containing_center(valid_mask, rest_index)
    axes = (x_values, y_values, z_values)
    box_summary = _box_report(box, axes, step_m)

    neighbor_counts: Counter[str] = Counter()
    neighbor_failure_types: Counter[str] = Counter()
    maximum_neighbor_joint_delta_radians = 0.0
    directed_edges = list(iter_directed_neighbor_edges(box))
    print(
        f"开始检查候选框内 {len(directed_edges)} 条相邻网格有向边。",
        flush=True,
    )
    for source_index, target_index in directed_edges:
        source_qpos = np.asarray(
            target_joint_radians[source_index],
            dtype=float,
        )
        target_position = np.asarray(
            (
                x_values[target_index[0]],
                y_values[target_index[1]],
                z_values[target_index[2]],
            ),
            dtype=float,
        )
        try:
            neighbor_solution = np.asarray(
                kinematics.solve_position(
                    source_qpos,
                    target_position,
                    joint_limits=joint_limits,
                ),
                dtype=float,
            )
        except Exception as error:
            neighbor_counts["ik_failed"] += 1
            neighbor_failure_types[type(error).__name__] += 1
            continue

        maximum_neighbor_joint_delta_radians = max(
            maximum_neighbor_joint_delta_radians,
            float(np.max(np.abs(neighbor_solution - source_qpos))),
        )
        if pose_has_collision(
            model,
            data,
            neighbor_solution,
            target_position,
        ):
            neighbor_counts["endpoint_collision"] += 1
            continue
        if path_has_collision(
            model,
            data,
            kinematics,
            source_qpos,
            neighbor_solution,
            max_step_radians=max_path_step_radians,
        ):
            neighbor_counts["path_collision"] += 1
            continue
        neighbor_counts["valid"] += 1

    grid_x, grid_y, grid_z = np.meshgrid(
        x_values,
        y_values,
        z_values,
        indexing="ij",
    )
    positions = np.stack((grid_x, grid_y, grid_z), axis=-1)
    valid_positions = positions[valid_mask]
    if len(valid_positions) == 0:
        raise RuntimeError("固定控制器初始姿态没有得到任何有效网格点。")

    np.savez_compressed(
        grid_path,
        x_m=x_values,
        y_m=y_values,
        z_m=z_values,
        status=status,
        target_joint_radians=target_joint_radians,
        valid_status_code=np.asarray(STATUS_VALID, dtype=np.uint8),
        rest_index=np.asarray(rest_index, dtype=np.int32),
        rest_tcp_m=rest_tcp_m,
        rest_joint_radians=rest_qpos,
    )
    _save_plot(
        plot_path,
        valid_positions,
        rest_tcp_m,
        box_summary,
    )

    counts = {
        label: int(np.count_nonzero(status == code))
        for code, label in STATUS_LABELS.items()
    }
    report = {
        "classification": (
            "joycon_initial_orientation_grid_candidate_not_real_hardware_certification"
        ),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": {
            "model": _portable_path(model_path),
            "candidate_report": _portable_path(candidate_report_path),
            "support_plane_z_m": 0.0,
            "base_driver_degrees": {
                "minimum": (
                    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.minimum
                ),
                "maximum": (
                    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.maximum
                ),
            },
            "rest_joint_radians": rest_qpos.tolist(),
            "rest_tcp_m": rest_tcp_m.tolist(),
            "tcp_orientation": "JoyCon controller initial rotation matrix",
        },
        "grid": {
            "bounds_source": bounds_source,
            "step_m": step_m,
            "shape": list(shape),
            "point_count": total_points,
            "x_m": {
                "minimum": float(x_values[0]),
                "maximum": float(x_values[-1]),
            },
            "y_m": {
                "minimum": float(y_values[0]),
                "maximum": float(y_values[-1]),
            },
            "z_m": {
                "minimum": float(z_values[0]),
                "maximum": float(z_values[-1]),
            },
        },
        "path_check": {
            "start": "JoyCon controller initial joint position",
            "max_joint_step_degrees": args.path_step_degrees,
            "checks": [
                "joint limits",
                "IK FK residual",
                "endpoint self/table collision",
                "interpolated path self/table collision",
                "TCP z >= 0",
            ],
        },
        "results": {
            **counts,
            "valid_ratio": float(counts["valid"] / total_points),
            "ik_failure_types": dict(failure_reasons),
            "valid_endpoint_aabb_m": {
                axis: {
                    "minimum": float(np.min(valid_positions[:, index])),
                    "maximum": float(np.max(valid_positions[:, index])),
                }
                for index, axis in enumerate(("x", "y", "z"))
            },
        },
        "largest_all_valid_grid_box_containing_rest": box_summary,
        "box_directed_neighbor_check": {
            "directed_edge_count": len(directed_edges),
            "valid": neighbor_counts["valid"],
            "ik_failed": neighbor_counts["ik_failed"],
            "endpoint_collision": neighbor_counts["endpoint_collision"],
            "path_collision": neighbor_counts["path_collision"],
            "all_valid": neighbor_counts["valid"] == len(directed_edges),
            "maximum_joint_delta_degrees": math.degrees(
                maximum_neighbor_joint_delta_radians
            ),
            "ik_failure_types": dict(neighbor_failure_types),
            "meaning": (
                "Every directed 1-grid-step move is re-solved from its "
                "source pose and collision-checked."
            ),
        },
        "artifacts": {
            "grid": _portable_path(grid_path),
            "plot": _portable_path(plot_path),
            "report": _portable_path(report_path),
        },
        "limitations": [
            "The box is proven only at sampled Cartesian grid points.",
            "Every valid point was checked from the JoyCon controller initial pose.",
            "The gripper orientation is fixed to the JoyCon controller initial pose.",
            "Real backlash, sag, cable routing, payload, and thermal effects are absent.",
            "Do not copy this box into real WorkspaceLimits before refinement and hardware checks.",
        ],
    }
    with report_path.open("w", encoding="utf-8") as report_file:
        json.dump(report, report_file, ensure_ascii=False, indent=2)
        report_file.write("\n")

    print("固定 JoyCon 控制器初始姿态网格结果：", flush=True)
    for label, count in counts.items():
        print(f"  {label}: {count}", flush=True)
    print("包含控制器初始点的最大全通过网格长方体：", flush=True)
    for axis in ("x", "y", "z"):
        bounds = box_summary[axis]
        print(
            f"  {axis.upper()}: {bounds['minimum']:.6f} .. "
            f"{bounds['maximum']:.6f} m",
            flush=True,
        )
    print(
        "候选框相邻网格有向边："
        f"{neighbor_counts['valid']}/{len(directed_edges)} 通过。",
        flush=True,
    )
    print(f"报告：{_portable_path(report_path)}", flush=True)
    print(f"网格：{_portable_path(grid_path)}", flush=True)
    print(f"图片：{_portable_path(plot_path)}", flush=True)
    print("结论等级：仿真候选；尚未写入真机限制。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
