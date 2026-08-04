"""Offline SO-100 Plus eye-in-hand solve; never access hardware."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from rosclaw_mini.vision.calibration import load_camera_intrinsic_calibration
from rosclaw_mini.vision.exceptions import CameraCalibrationError
from rosclaw_mini.vision.hand_eye import (
    DEFAULT_MINIMUM_HAND_EYE_SAMPLES,
    load_hand_eye_dataset,
    solve_hand_eye_calibration,
    write_hand_eye_calibration,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "从已有同步数据集离线求解 SO-100 Plus tcp_T_camera；"
            "不打开摄像头或机械臂。"
        )
    )
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--intrinsics", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--minimum-samples",
        type=int,
        default=DEFAULT_MINIMUM_HAND_EYE_SAMPLES,
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        intrinsics = load_camera_intrinsic_calibration(args.intrinsics)
        dataset = load_hand_eye_dataset(args.dataset)
        calibration = solve_hand_eye_calibration(
            dataset,
            intrinsics,
            minimum_samples=args.minimum_samples,
        )
        write_hand_eye_calibration(
            calibration,
            args.output,
            overwrite=args.overwrite,
        )
    except CameraCalibrationError as error:
        print(f"手眼标定失败：{error}")
        return 1

    print(
        "手眼标定求解完成："
        f"{calibration.sample_count} 组，"
        f"固定标定板平移 RMS="
        f"{calibration.translation_rms_m * 1000.0:.3f} mm，"
        f"最大={calibration.translation_max_m * 1000.0:.3f} mm，"
        f"旋转 RMS={calibration.rotation_rms_degrees:.4f}°，"
        f"最大={calibration.rotation_max_degrees:.4f}°。"
    )
    if calibration.checkerboard_flipped_sample_ids:
        print(
            "已统一对称棋盘180°坐标方向的样本："
            + ", ".join(calibration.checkerboard_flipped_sample_ids)
        )
    print(f"结果已写入：{args.output}")
    print(f"SHA-256={calibration.calibration_sha256}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
