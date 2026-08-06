"""Append one manually verified camera/base point pair; no hardware access."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from pathlib import Path

from rosclaw_mini.vision.exceptions import EyeToHandCalibrationError
from rosclaw_mini.vision.eye_to_hand import (
    EyeToHandDataset,
    append_eye_to_hand_point,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "向本地 eye-to-hand 数据集追加一个人工确认点对；"
            "命令不访问相机或机械臂。"
        )
    )
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--camera-point", type=float, nargs=3, metavar=("X", "Y", "Z"), required=True
    )
    parser.add_argument(
        "--base-point", type=float, nargs=3, metavar=("X", "Y", "Z"), required=True
    )
    parser.add_argument("--split", choices=("fit", "validation"), default="fit")
    parser.add_argument("--point-id", default=None)
    return parser


def append_point(args: argparse.Namespace) -> EyeToHandDataset:
    return append_eye_to_hand_point(
        Path(args.dataset),
        camera_serial=args.serial,
        width=args.width,
        height=args.height,
        camera_point_m=args.camera_point,
        base_point_m=args.base_point,
        split=args.split,
        point_id=args.point_id,
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        dataset = append_point(args)
    except (ValueError, EyeToHandCalibrationError) as error:
        output_func(f"记录 eye-to-hand 点对失败：{error}")
        return 1
    output_func(
        f"已记录 {dataset.points[-1].point_id}；总点数={len(dataset.points)}，"
        f"dataset_sha256={dataset.dataset_sha256}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
