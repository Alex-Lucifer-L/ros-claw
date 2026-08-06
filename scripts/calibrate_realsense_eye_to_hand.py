"""Solve fixed RealSense-to-base extrinsics from saved real point pairs."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import json
from pathlib import Path

from rosclaw_mini.vision.exceptions import EyeToHandCalibrationError
from rosclaw_mini.vision.eye_to_hand import (
    load_eye_to_hand_dataset,
    solve_eye_to_hand_calibration,
    write_eye_to_hand_calibration,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="离线求解 T_base_from_camera；不访问任何硬件。"
    )
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-fit-points", type=int, default=6)
    parser.add_argument("--minimum-spread-mm", type=float, default=80.0)
    parser.add_argument("--max-rmse-mm", type=float, default=20.0)
    parser.add_argument("--max-error-mm", type=float, default=40.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        dataset = load_eye_to_hand_dataset(args.dataset)
        calibration = solve_eye_to_hand_calibration(
            dataset,
            minimum_fit_points=args.minimum_fit_points,
            minimum_spread_m=args.minimum_spread_mm / 1000.0,
            activation_max_rmse_m=args.max_rmse_mm / 1000.0,
            activation_max_error_m=args.max_error_mm / 1000.0,
        )
        write_eye_to_hand_calibration(
            calibration,
            args.output,
            overwrite=args.overwrite,
        )
    except (ValueError, EyeToHandCalibrationError) as error:
        output_func(f"eye-to-hand 标定失败：{error}")
        return 1
    output_func(json.dumps(calibration.to_dict(), ensure_ascii=False, indent=2))
    return 0 if calibration.active else 2


if __name__ == "__main__":
    raise SystemExit(main())
