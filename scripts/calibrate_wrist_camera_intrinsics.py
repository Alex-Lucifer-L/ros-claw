"""Solve wrist-camera intrinsics from an existing checkerboard image directory."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from rosclaw_mini.vision.calibration import (
    CameraCalibrationIdentity,
    CheckerboardSpec,
    calibrate_camera_intrinsics,
    write_camera_intrinsic_calibration,
)
from rosclaw_mini.vision.exceptions import CameraCalibrationError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="从已有棋盘图片离线求腕部 RGB 相机内参；不访问机械臂。"
    )
    parser.add_argument("--images-dir", required=True, type=Path)
    parser.add_argument("--image-glob", default="wrist_view_*.jpg")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--device", required=True)
    parser.add_argument("--vendor-id", required=True)
    parser.add_argument("--product-id", required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--pixel-format", default="YUYV")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--inner-columns", type=int, default=7)
    parser.add_argument("--inner-rows", type=int, default=6)
    parser.add_argument("--square-size-mm", type=float, default=24.0)
    parser.add_argument("--minimum-views", type=int, default=10)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    paths = tuple(sorted(args.images_dir.glob(args.image_glob)))
    if not paths:
        print(
            f"标定失败：目录 {args.images_dir} 中没有匹配 {args.image_glob!r} 的图片。"
        )
        return 2
    try:
        identity = CameraCalibrationIdentity(
            device=args.device,
            vendor_id=args.vendor_id,
            product_id=args.product_id,
            serial=args.serial,
            width=args.width,
            height=args.height,
            pixel_format=args.pixel_format,
        )
        checkerboard = CheckerboardSpec(
            inner_columns=args.inner_columns,
            inner_rows=args.inner_rows,
            square_size_m=args.square_size_mm / 1000.0,
        )
        calibration = calibrate_camera_intrinsics(
            paths,
            camera_identity=identity,
            checkerboard=checkerboard,
            minimum_views=args.minimum_views,
        )
        write_camera_intrinsic_calibration(
            calibration,
            args.output,
            overwrite=args.overwrite,
        )
    except CameraCalibrationError as error:
        print(f"标定失败：{error}")
        return 1

    print(
        "标定完成："
        f"有效 {len(calibration.accepted_images)} 张，"
        f"拒绝 {len(calibration.rejected_images)} 张，"
        f"RMS={calibration.rms_reprojection_error_px:.6f} px，"
        f"SHA-256={calibration.calibration_sha256}。"
    )
    print(f"结果已写入：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
