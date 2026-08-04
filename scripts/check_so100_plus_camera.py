"""只连接 SO-100 Plus OpenCV 摄像头并抓取一帧，绝不连接机械臂。"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusCameraConfig,
    create_so100_plus_cameras,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="打开一路 SO-100 Plus 摄像头，抓取一帧后立即断开。"
    )
    parser.add_argument(
        "--device",
        required=True,
        help="摄像头设备路径，例如 /dev/video2 或 /dev/v4l/by-id/...",
    )
    parser.add_argument("--name", default="right")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--acknowledge-camera-capture",
        action="store_true",
        help="确认允许摄像头抓取一帧；图像不会写入磁盘",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_camera_capture:
        raise SystemExit("已停止：必须显式确认抓取一帧摄像头图像。")

    config = SO100PlusCameraConfig(
        name=args.name,
        device=Path(args.device),
        fps=args.fps,
        width=args.width,
        height=args.height,
        color_mode="rgb",
    )
    camera = create_so100_plus_cameras((config,))[args.name]

    print(
        f"即将打开摄像头 {args.name!r} ({args.device})；"
        "不连接机械臂，不保存图像。",
        flush=True,
    )
    try:
        camera.connect()
        image = camera.read()
        shape = tuple(getattr(image, "shape", ()))
        expected_shape = (args.height, args.width, 3)
        if shape != expected_shape:
            raise RuntimeError(
                f"图像形状 {shape} 与期望 {expected_shape} 不一致。"
            )
        print(
            f"摄像头验证通过：shape={shape}，fps={camera.fps}，"
            f"color_mode={camera.color_mode}。",
            flush=True,
        )
        return 0
    finally:
        if getattr(camera, "is_connected", False):
            camera.disconnect()
        print("摄像头已断开。", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
