"""通过 MuJoCo 离线采样 SO-100 Plus 的候选 TCP 工作空间。

本脚本不导入 Feetech 驱动，不打开串口，也不具备执行机械臂动作的能力。
输出描述的是仿真模型中的“无碰撞位置并集”，不能直接作为真机安全范围。
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
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
    SO100_PLUS_ARM_JOINT_NAMES,
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS,
)


DEFAULT_MODEL_PATH = Path(
    "lerobot-joycon_plus/lerobot/common/robot_devices/controllers/scene_plus.xml"
)
DEFAULT_OUTPUT_DIR = Path("artifacts/so100_plus_workspace")
ARM_JOINT_COUNT = len(SO100_PLUS_ARM_JOINT_NAMES)
DEFAULT_GRIPPER_MODEL_RADIANS = 0.0


@dataclass(frozen=True)
class CollisionSummary:
    """一个采样批次的 MuJoCo 碰撞过滤结果。"""

    collision_free_mask: np.ndarray
    mujoco_collision_samples: int
    below_floor_samples: int
    floor_contact_samples: int
    self_contact_samples: int
    contact_pair_events: Counter[str]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "纯离线采样 SO-100 Plus 仿真候选工作空间；"
            "不会连接、上力或移动真实机械臂。"
        )
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--samples",
        type=int,
        default=262_144,
        help="关节空间均匀随机样本数；默认 262144",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=32_768,
        help="批量 FK 和碰撞检查的分块大小；默认 32768",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=20260718,
        help="可复现随机种子",
    )
    parser.add_argument(
        "--voxel-size-mm",
        type=float,
        default=10.0,
        help="工作空间占用体素边长，单位 mm；默认 10",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=80_000,
        help="图片中最多绘制多少个无碰撞点",
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if not 1_000 <= args.samples <= 5_000_000:
        raise SystemExit("samples 必须在 [1000, 5000000] 范围内。")
    if not 1 <= args.chunk_size <= args.samples:
        raise SystemExit("chunk-size 必须大于 0 且不超过 samples。")
    if (
        not math.isfinite(args.voxel_size_mm)
        or not 1.0 <= args.voxel_size_mm <= 100.0
    ):
        raise SystemExit("voxel-size-mm 必须在 [1, 100] 范围内。")
    if not 1_000 <= args.max_plot_points <= 500_000:
        raise SystemExit(
            "max-plot-points 必须在 [1000, 500000] 范围内。"
        )


def _model_joint_name(model: mujoco.MjModel, index: int) -> str:
    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        index,
    )
    if name is None:
        raise RuntimeError(f"MuJoCo 第 {index} 个关节没有名称。")
    return name


def build_simulation_joint_ranges(
    model: mujoco.MjModel,
) -> tuple[np.ndarray, np.ndarray]:
    """构造扫描范围；底座使用实测范围，其余关节使用 MuJoCo 范围。"""

    if model.njnt < ARM_JOINT_COUNT:
        raise RuntimeError("MuJoCo 模型不足 6 个手臂关节。")

    actual_names = tuple(
        _model_joint_name(model, index)
        for index in range(ARM_JOINT_COUNT)
    )
    if actual_names != SO100_PLUS_ARM_JOINT_NAMES:
        raise RuntimeError(
            "MuJoCo 前 6 个关节名称或顺序与 SO-100 Plus 不一致："
            f"{actual_names}"
        )

    lower = np.asarray(model.jnt_range[:ARM_JOINT_COUNT, 0], dtype=float).copy()
    upper = np.asarray(model.jnt_range[:ARM_JOINT_COUNT, 1], dtype=float).copy()

    # 驱动角到模型角的第一个符号为 -1，所以端点顺序也需要反转。
    driver_limits = (
        SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS
    )
    lower[0] = math.radians(-driver_limits.maximum)
    upper[0] = math.radians(-driver_limits.minimum)
    if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
        raise RuntimeError("仿真关节范围包含非有限数值。")
    if np.any(lower >= upper):
        raise RuntimeError("仿真关节范围下限必须小于上限。")
    return lower, upper


def sample_joint_positions(
    rng: np.random.Generator,
    count: int,
    lower_radians: np.ndarray,
    upper_radians: np.ndarray,
) -> np.ndarray:
    """在六维关节空间中均匀随机采样。"""

    return rng.uniform(
        lower_radians,
        upper_radians,
        size=(count, ARM_JOINT_COUNT),
    )


def batch_tcp_positions(
    kinematics: SO100PlusKinematics,
    joint_radians: np.ndarray,
) -> np.ndarray:
    """批量计算与 ``SO100PlusKinematics.forward_position`` 相同的 TCP。"""

    joints = np.asarray(joint_radians, dtype=float)
    if joints.ndim != 2 or joints.shape[1] != ARM_JOINT_COUNT:
        raise ValueError("批量关节角必须是形状为 (N, 6) 的数组。")
    if not np.all(np.isfinite(joints)):
        raise ValueError("批量关节角必须全部是有限数值。")

    transform_object = kinematics.robot.fkine(joints)
    transforms = np.asarray(
        getattr(transform_object, "A", transform_object),
        dtype=float,
    )
    if transforms.ndim == 2:
        transforms = transforms[np.newaxis, :, :]
    if transforms.shape != (len(joints), 4, 4):
        raise RuntimeError(
            "批量正运动学没有返回形状为 (N, 4, 4) 的变换。"
        )
    if not np.all(np.isfinite(transforms)):
        raise RuntimeError("批量正运动学结果包含非有限数值。")

    offset = np.asarray(SO100_PLUS_GRIPPER_TCP_OFFSET_M, dtype=float)
    rotated_offset = np.einsum(
        "nij,j->ni",
        transforms[:, :3, :3],
        offset,
    )
    return transforms[:, :3, 3] + rotated_offset


def _contact_labels(
    model: mujoco.MjModel,
    geom_id: int,
) -> tuple[str, str]:
    if geom_id < 0:
        return "unknown_geom", "unknown_body"
    geom_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        geom_id,
    )
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        body_id,
    )
    return geom_name or f"geom#{geom_id}", body_name or "world"


def evaluate_collision_free(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_radians: np.ndarray,
    tcp_positions_m: np.ndarray,
    *,
    gripper_model_radians: float = DEFAULT_GRIPPER_MODEL_RADIANS,
) -> CollisionSummary:
    """使用 MuJoCo 接触和 TCP 地面位置过滤一个采样批次。"""

    joints = np.asarray(joint_radians, dtype=float)
    tcp = np.asarray(tcp_positions_m, dtype=float)
    if joints.ndim != 2 or joints.shape[1] != ARM_JOINT_COUNT:
        raise ValueError("碰撞检查关节角必须是形状为 (N, 6) 的数组。")
    if tcp.shape != (len(joints), 3):
        raise ValueError("碰撞检查 TCP 必须是形状为 (N, 3) 的数组。")
    if not math.isfinite(gripper_model_radians):
        raise ValueError("夹爪模型角必须是有限数值。")

    collision_free = np.ones(len(joints), dtype=bool)
    below_floor = tcp[:, 2] < 0.0
    mujoco_collision_samples = 0
    floor_contact_samples = 0
    self_contact_samples = 0
    contact_pairs: Counter[str] = Counter()

    for sample_index, qpos in enumerate(joints):
        data.qpos[:ARM_JOINT_COUNT] = qpos
        data.qpos[ARM_JOINT_COUNT] = gripper_model_radians
        mujoco.mj_forward(model, data)

        has_floor_contact = False
        has_self_contact = False
        if data.ncon:
            mujoco_collision_samples += 1
            collision_free[sample_index] = False

        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom1, body1 = _contact_labels(model, int(contact.geom1))
            geom2, body2 = _contact_labels(model, int(contact.geom2))
            pair = " ↔ ".join(sorted((body1, body2)))
            contact_pairs[pair] += 1
            if "floor" in (geom1, geom2) or "world" in (body1, body2):
                has_floor_contact = True
            else:
                has_self_contact = True

        floor_contact_samples += int(has_floor_contact)
        self_contact_samples += int(has_self_contact)

    # FK 模型和 MuJoCo 网格存在毫米级坐标差；TCP 已低于场景地面时也拒绝。
    collision_free[below_floor] = False
    return CollisionSummary(
        collision_free_mask=collision_free,
        mujoco_collision_samples=mujoco_collision_samples,
        below_floor_samples=int(np.count_nonzero(below_floor)),
        floor_contact_samples=floor_contact_samples,
        self_contact_samples=self_contact_samples,
        contact_pair_events=contact_pairs,
    )


def _axis_bounds(points: np.ndarray) -> dict[str, dict[str, float]]:
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    return {
        axis: {
            "minimum": float(minimum[index]),
            "maximum": float(maximum[index]),
            "span": float(maximum[index] - minimum[index]),
        }
        for index, axis in enumerate(("x", "y", "z"))
    }


def _quantile_bounds(
    points: np.ndarray,
    lower_quantile: float,
    upper_quantile: float,
) -> dict[str, dict[str, float]]:
    values = np.quantile(
        points,
        (lower_quantile, upper_quantile),
        axis=0,
    )
    return {
        axis: {
            "minimum": float(values[0, index]),
            "maximum": float(values[1, index]),
            "span": float(values[1, index] - values[0, index]),
        }
        for index, axis in enumerate(("x", "y", "z"))
    }


def voxelize_points(
    points_m: np.ndarray,
    voxel_size_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """返回体素原点和已占用体素的整数索引。"""

    if not math.isfinite(voxel_size_m) or voxel_size_m <= 0:
        raise ValueError("体素大小必须是有限正数。")
    points = np.asarray(points_m, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("体素化需要非空的 (N, 3) 点数组。")
    origin = np.floor(np.min(points, axis=0) / voxel_size_m) * voxel_size_m
    indices = np.floor((points - origin) / voxel_size_m).astype(np.int32)
    return origin, np.unique(indices, axis=0)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(Path.cwd().resolve()))
    except ValueError:
        return str(path)


def _save_plot(
    output_path: Path,
    points_m: np.ndarray,
    rest_tcp_m: np.ndarray,
    *,
    max_plot_points: int,
    seed: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if len(points_m) > max_plot_points:
        rng = np.random.default_rng(seed)
        indices = rng.choice(
            len(points_m),
            size=max_plot_points,
            replace=False,
        )
        plot_points = points_m[indices]
    else:
        plot_points = points_m

    figure = plt.figure(figsize=(14, 11))
    xy = figure.add_subplot(2, 2, 1)
    xz = figure.add_subplot(2, 2, 2)
    yz = figure.add_subplot(2, 2, 3)
    view3d = figure.add_subplot(2, 2, 4, projection="3d")

    color = plot_points[:, 2]
    scatter_options: dict[str, Any] = {
        "c": color,
        "s": 1.2,
        "alpha": 0.28,
        "cmap": "viridis",
        "linewidths": 0,
    }
    xy.scatter(plot_points[:, 0], plot_points[:, 1], **scatter_options)
    xy.scatter(rest_tcp_m[0], rest_tcp_m[1], c="red", marker="*", s=90)
    xy.set(xlabel="X (m)", ylabel="Y (m)", title="Top view: XY")

    xz.scatter(plot_points[:, 0], plot_points[:, 2], **scatter_options)
    xz.scatter(rest_tcp_m[0], rest_tcp_m[2], c="red", marker="*", s=90)
    xz.set(xlabel="X (m)", ylabel="Z (m)", title="Side view: XZ")

    yz.scatter(plot_points[:, 1], plot_points[:, 2], **scatter_options)
    yz.scatter(rest_tcp_m[1], rest_tcp_m[2], c="red", marker="*", s=90)
    yz.set(xlabel="Y (m)", ylabel="Z (m)", title="Front view: YZ")

    for axis in (xy, xz, yz):
        axis.grid(alpha=0.2)
        axis.set_aspect("equal", adjustable="box")

    view3d.scatter(
        plot_points[:, 0],
        plot_points[:, 1],
        plot_points[:, 2],
        **scatter_options,
    )
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
        title="Collision-free candidate TCP union",
    )
    view3d.legend(loc="upper right")
    figure.suptitle(
        "SO-100 Plus simulated candidate workspace\n"
        "Measured base rotation range; other joints from MuJoCo",
        fontsize=14,
    )
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def _joint_range_report(
    lower_radians: np.ndarray,
    upper_radians: np.ndarray,
) -> list[dict[str, Any]]:
    result = []
    for index, name in enumerate(SO100_PLUS_ARM_JOINT_NAMES):
        result.append(
            {
                "name": name,
                "source": (
                    "measured right_follower driver limits converted to model"
                    if index == 0
                    else "MuJoCo joint range"
                ),
                "model_radians": {
                    "minimum": float(lower_radians[index]),
                    "maximum": float(upper_radians[index]),
                },
                "model_degrees": {
                    "minimum": float(math.degrees(lower_radians[index])),
                    "maximum": float(math.degrees(upper_radians[index])),
                },
            }
        )
    return result


def main() -> int:
    args = build_parser().parse_args()
    _validate_args(args)

    model_path = args.model.resolve()
    if not model_path.is_file():
        raise SystemExit(f"MuJoCo 模型不存在：{model_path}")

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "workspace_report.json"
    points_path = output_dir / "collision_free_tcp_points.npz"
    plot_path = output_dir / "workspace_views.png"

    print("模式：纯离线仿真；串口写入 0，真实机械臂动作 0。", flush=True)
    print(f"模型：{_portable_path(model_path)}", flush=True)
    print(f"计划样本数：{args.samples}", flush=True)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    lower_radians, upper_radians = build_simulation_joint_ranges(model)
    kinematics = SO100PlusKinematics()
    rng = np.random.default_rng(args.seed)

    accepted_batches: list[np.ndarray] = []
    all_geometric_batches: list[np.ndarray] = []
    mujoco_collision_samples = 0
    below_floor_samples = 0
    floor_contact_samples = 0
    self_contact_samples = 0
    contact_pairs: Counter[str] = Counter()

    processed = 0
    while processed < args.samples:
        batch_count = min(args.chunk_size, args.samples - processed)
        joints = sample_joint_positions(
            rng,
            batch_count,
            lower_radians,
            upper_radians,
        )
        tcp_positions = batch_tcp_positions(kinematics, joints)
        collision = evaluate_collision_free(
            model,
            data,
            joints,
            tcp_positions,
        )
        all_geometric_batches.append(tcp_positions)
        accepted_batches.append(
            tcp_positions[collision.collision_free_mask]
        )
        mujoco_collision_samples += collision.mujoco_collision_samples
        below_floor_samples += collision.below_floor_samples
        floor_contact_samples += collision.floor_contact_samples
        self_contact_samples += collision.self_contact_samples
        contact_pairs.update(collision.contact_pair_events)
        processed += batch_count
        print(
            f"已处理 {processed}/{args.samples}，"
            f"当前批次无碰撞候选 "
            f"{int(np.count_nonzero(collision.collision_free_mask))}/"
            f"{batch_count}",
            flush=True,
        )

    all_geometric_points = np.concatenate(all_geometric_batches, axis=0)
    collision_free_points = np.concatenate(accepted_batches, axis=0)
    if len(collision_free_points) == 0:
        raise RuntimeError("没有得到任何无碰撞候选点。")

    voxel_size_m = args.voxel_size_mm / 1000.0
    voxel_origin_m, occupied_voxels = voxelize_points(
        collision_free_points,
        voxel_size_m,
    )
    rest_tcp_m = np.asarray(
        kinematics.forward_position(SO100_PLUS_JOYCON_INITIAL_RADIANS),
        dtype=float,
    )

    np.savez_compressed(
        points_path,
        tcp_points_m=collision_free_points.astype(np.float32),
        voxel_origin_m=voxel_origin_m,
        occupied_voxel_indices=occupied_voxels,
        voxel_size_m=np.asarray(voxel_size_m),
        rest_tcp_m=rest_tcp_m,
        lower_joint_radians=lower_radians,
        upper_joint_radians=upper_radians,
    )
    _save_plot(
        plot_path,
        collision_free_points,
        rest_tcp_m,
        max_plot_points=args.max_plot_points,
        seed=args.seed,
    )

    report = {
        "classification": (
            "simulation_candidate_workspace_not_real_hardware_certification"
        ),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "model": {
            "path": _portable_path(model_path),
            "sha256": _sha256(model_path),
            "support_plane_enabled": True,
            "support_plane_z_m": 0.0,
            "support_plane_represents_tabletop": True,
            "separate_table_mesh_enabled": False,
            "extra_physical_base_geometry_included": False,
            "extra_base_obstacle_required_by_user": False,
        },
        "sampling": {
            "method": "uniform_random_joint_space",
            "seed": args.seed,
            "requested_samples": args.samples,
            "processed_samples": processed,
            "chunk_size": args.chunk_size,
            "gripper_model_radians": DEFAULT_GRIPPER_MODEL_RADIANS,
        },
        "joint_ranges": _joint_range_report(
            lower_radians,
            upper_radians,
        ),
        "base_driver_degrees": {
            "minimum": (
                SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.minimum
            ),
            "maximum": (
                SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.maximum
            ),
        },
        "filtering": {
            "collision_free_samples": int(len(collision_free_points)),
            "rejected_samples": int(processed - len(collision_free_points)),
            "collision_free_ratio": float(
                len(collision_free_points) / processed
            ),
            "mujoco_collision_samples": mujoco_collision_samples,
            "tcp_below_floor_samples": below_floor_samples,
            "floor_contact_samples": floor_contact_samples,
            "self_contact_samples": self_contact_samples,
            "top_contact_pair_events": dict(contact_pairs.most_common(20)),
        },
        "tcp_workspace": {
            "coordinate_frame": "SO100 Plus kinematics base frame",
            "unit": "meter",
            "tcp_offset_m": list(SO100_PLUS_GRIPPER_TCP_OFFSET_M),
            "rest_tcp_m": rest_tcp_m.tolist(),
            "geometric_sample_aabb_m": _axis_bounds(
                all_geometric_points
            ),
            "collision_free_sample_aabb_m": _axis_bounds(
                collision_free_points
            ),
            "collision_free_central_99_percent_m": _quantile_bounds(
                collision_free_points,
                0.005,
                0.995,
            ),
            "warning": (
                "AABB corners and interior are not guaranteed reachable; "
                "the point union is orientation-dependent."
            ),
        },
        "voxelization": {
            "voxel_size_m": voxel_size_m,
            "origin_m": voxel_origin_m.tolist(),
            "occupied_voxel_count": int(len(occupied_voxels)),
            "approximate_occupied_volume_m3": float(
                len(occupied_voxels) * voxel_size_m**3
            ),
        },
        "artifacts": {
            "points": _portable_path(points_path),
            "plot": _portable_path(plot_path),
            "report": _portable_path(report_path),
        },
        "limitations": [
            "The tabletop is idealized as an infinite z=0 support plane.",
            "Table edges and thickness are intentionally outside the user-defined scope.",
            "The extra mount is represented only by the measured base rotation range.",
            "Only the measured base rotation range is a real-hardware limit.",
            "Other joint ranges come from MuJoCo, not physical certification.",
            "Cable routing, backlash, gravity sag, payload, and thermal effects are absent.",
            "The workspace is the union over sampled TCP orientations.",
            "Random sampling approximates boundaries and cannot prove completeness.",
            "The axis-aligned bounding box is not a safe WorkspaceLimits box.",
        ],
    }
    with report_path.open("w", encoding="utf-8") as report_file:
        json.dump(report, report_file, ensure_ascii=False, indent=2)
        report_file.write("\n")

    bounds = report["tcp_workspace"]["collision_free_sample_aabb_m"]
    print("仿真候选 TCP 外包围范围（不是完整可达长方体）：", flush=True)
    for axis in ("x", "y", "z"):
        axis_bounds = bounds[axis]
        print(
            f"  {axis.upper()}: "
            f"{axis_bounds['minimum']:.6f} .. "
            f"{axis_bounds['maximum']:.6f} m",
            flush=True,
        )
    print(
        f"无碰撞候选：{len(collision_free_points)}/{processed} "
        f"({len(collision_free_points) / processed:.1%})",
        flush=True,
    )
    print(f"报告：{_portable_path(report_path)}", flush=True)
    print(f"点云：{_portable_path(points_path)}", flush=True)
    print(f"图片：{_portable_path(plot_path)}", flush=True)
    print(
        "结论等级：仿真候选；尚未写入真机 WorkspaceLimits。",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
