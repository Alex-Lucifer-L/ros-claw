"""Read-only RealSense RGBD diagnostic; never imports the arm runtime."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import json
from pathlib import Path
from typing import Any

from rosclaw_mini.vision.exceptions import VisionError
from rosclaw_mini.vision.realsense import (
    RealSenseCameraAdapter,
    frame_number_gaps,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="只读验收指定序列号 RealSense 的同步 RGBD 数据流。"
    )
    parser.add_argument("--serial", required=True, help="RealSense 设备序列号。")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    return parser


def run_diagnostic(
    args: argparse.Namespace,
    *,
    adapter_factory: Callable[..., Any] = RealSenseCameraAdapter,
) -> dict[str, Any]:
    if args.frames <= 0:
        raise ValueError("--frames 必须大于 0。")
    if args.warmup_frames < 0:
        raise ValueError("--warmup-frames 不能为负数。")
    frame_numbers: list[int] = []
    timestamps_ms: list[float] = []
    last_frame = None
    with adapter_factory(
        args.serial,
        width=args.width,
        height=args.height,
        fps=args.fps,
        timeout_ms=args.timeout_ms,
    ) as camera:
        for _index in range(args.warmup_frames):
            camera.capture_frame()
        for _index in range(args.frames):
            last_frame = camera.capture_frame()
            frame_numbers.append(last_frame.frame_number)
            timestamps_ms.append(last_frame.timestamp_ms)
    if last_frame is None:
        raise RuntimeError("未读取到任何 RealSense 帧。")
    gaps = frame_number_gaps(frame_numbers)
    return {
        "serial": args.serial,
        "source": last_frame.source,
        "valid_frames": len(frame_numbers),
        "warmup_frames": args.warmup_frames,
        "frame_number_gaps": gaps,
        "accepted": gaps == 0,
        "first_frame_number": frame_numbers[0],
        "last_frame_number": frame_numbers[-1],
        "first_timestamp_ms": timestamps_ms[0],
        "last_timestamp_ms": timestamps_ms[-1],
        "depth_scale_m_per_unit": last_frame.depth_scale_m_per_unit,
        "color_intrinsics": last_frame.color_intrinsics.to_dict(),
    }


def main(
    argv: Sequence[str] | None = None,
    *,
    adapter_factory: Callable[..., Any] = RealSenseCameraAdapter,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = run_diagnostic(args, adapter_factory=adapter_factory)
    except (ValueError, RuntimeError, VisionError) as error:
        output_func(f"RealSense 只读验收失败：{error}")
        return 1
    output_func(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
