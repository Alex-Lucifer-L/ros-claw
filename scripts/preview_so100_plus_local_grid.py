"""一次只读真实姿态，并离线筛选显式指定的局部笛卡尔网格。"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from rosclaw_mini.arm.kinematics import SO100PlusKinematics
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_readonly_robot,
)
from rosclaw_mini.arm.so100_plus_diagnostics import (
    MotionPreviewSafetyError,
    build_symmetric_local_grid_offsets,
    preview_local_motion_grid_once,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "只读一次 SO-100 Plus 当前姿态，离线计算显式局部网格；"
            "不会发送 Goal_Position。"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--half-extent-mm",
        required=True,
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="相对当前位置的 X/Y/Z 网格半宽；不会被当成已认证工作空间",
    )
    parser.add_argument(
        "--step-mm",
        required=True,
        type=float,
        help="离线网格步长；每个非零半宽必须是它的整数倍",
    )
    parser.add_argument(
        "--acknowledge-readonly-grid-connect-risk",
        action="store_true",
        help="确认脚本会打开真实串口读取扭矩和位置，但不会写任何寄存器",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_readonly_grid_connect_risk:
        raise SystemExit(
            "已停止：必须显式添加 "
            "--acknowledge-readonly-grid-connect-risk。"
        )

    offsets = build_symmetric_local_grid_offsets(
        half_extent_mm=args.half_extent_mm,
        step_mm=args.step_mm,
    )
    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_readonly_robot(config)
    print(
        f"即将只读一次当前位置并离线计算 {len(offsets)} 个点；"
        "Goal_Position 写入为 0。",
        flush=True,
    )
    try:
        result = preview_local_motion_grid_once(
            robot,
            args.follower_name,
            SO100PlusKinematics(),
            offsets_m=offsets,
        )
    except MotionPreviewSafetyError as error:
        print(f"网格预览已拒绝：{error}", flush=True)
        return 2
    finally:
        print("通信已关闭；脚本没有改变扭矩或校准。", flush=True)

    candidate_count = sum(point.is_candidate for point in result.points)
    print(
        "当前 XYZ (m): "
        + ", ".join(f"{value:.6f}" for value in result.current_position_m),
        flush=True,
    )
    print(
        f"候选 {candidate_count}，拒绝 {len(result.points) - candidate_count}；"
        "全部结果均未批准执行。",
        flush=True,
    )
    for point in result.points:
        offset_mm = tuple(value * 1000.0 for value in point.offset_m)
        prefix = "偏移(mm) " + ", ".join(
            f"{value:+.1f}" for value in offset_mm
        )
        if not point.is_candidate:
            print(f"  {prefix}: 拒绝，{point.rejection_reason}", flush=True)
            continue
        preview = point.preview
        print(
            f"  {prefix}: 计算候选，底座 "
            f"{preview.target_driver_degrees[0]:.3f}°，"
            "最大关节变化 "
            f"{math.degrees(preview.max_joint_delta_radians):.3f}°，"
            f"模型 0.1 rad 分段 {preview.model_step_count}",
            flush=True,
        )
    print("is_approved_for_execution=False", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
