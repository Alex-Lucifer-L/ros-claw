"""Live, read-only RealSense RGB and aligned-depth preview."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from rosclaw_mini.vision.exceptions import VisionError
from rosclaw_mini.vision.realsense import RealSenseCameraAdapter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="只读预览 RealSense RGB 和对齐深度；Q/Esc 退出。"
    )
    parser.add_argument("--serial", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=10000)
    parser.add_argument(
        "--target-pixel",
        type=int,
        nargs=2,
        default=None,
        metavar=("U", "V"),
        help="可选：在 RGB 和对齐 Depth 上标出目标像素十字准星。",
    )
    parser.add_argument(
        "--target-depth-m",
        type=float,
        default=None,
        help="可选：在预览中显示目标深度和当前准星深度。",
    )
    parser.add_argument(
        "--acknowledge-camera-preview",
        action="store_true",
        help="确认只在本机实时显示 D435i 画面，不保存不上传。",
    )
    return parser


def _depth_visualization(depth, cv2_module):
    array = np.asarray(depth)
    valid = array[array > 0]
    upper = float(np.percentile(valid, 95.0)) if valid.size else 1.0
    gray = np.clip(
        array.astype(np.float32) / max(upper, 1.0) * 255.0,
        0.0,
        255.0,
    ).astype(np.uint8)
    colored = cv2_module.applyColorMap(gray, cv2_module.COLORMAP_TURBO)
    colored[array == 0] = 0
    return colored


def _draw_crosshair(image, pixel, *, color=(0, 255, 0), radius=12):
    """直接在 numpy BGR 图像上画准星，不依赖 OpenCV 绘图 API。"""

    u, v = pixel
    height, width = image.shape[:2]
    left = max(0, u - radius)
    right = min(width, u + radius + 1)
    top = max(0, v - radius)
    bottom = min(height, v + radius + 1)
    image[v, left:right] = color
    image[top:bottom, u] = color


def run_preview(
    args: argparse.Namespace,
    *,
    adapter_factory: Callable[..., Any] = RealSenseCameraAdapter,
    cv2_module,
) -> int:
    window_name = "RosClaw D435i - RGB | Aligned Depth (Q/Esc to exit)"
    cv2_module.namedWindow(window_name, cv2_module.WINDOW_NORMAL)
    try:
        with adapter_factory(
            args.serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            timeout_ms=args.timeout_ms,
        ) as camera:
            while True:
                frame = camera.capture_frame()
                bgr = np.ascontiguousarray(frame.rgb[..., ::-1])
                depth_view = _depth_visualization(
                    frame.aligned_depth,
                    cv2_module,
                )
                target_pixel = (
                    tuple(args.target_pixel)
                    if args.target_pixel is not None
                    else None
                )
                current_depth_m = None
                if target_pixel is not None:
                    _draw_crosshair(bgr, target_pixel)
                    _draw_crosshair(depth_view, target_pixel)
                    u, v = target_pixel
                    raw_depth = float(frame.aligned_depth[v, u])
                    if raw_depth > 0.0:
                        current_depth_m = (
                            raw_depth * frame.depth_scale_m_per_unit
                        )
                combined = np.hstack((bgr, depth_view))
                if target_pixel is not None and args.target_depth_m is not None:
                    current_text = (
                        f"{current_depth_m:.3f} m"
                        if current_depth_m is not None
                        else "no depth"
                    )
                    cv2_module.putText(
                        combined,
                        (
                            f"target pixel={target_pixel}, desired depth="
                            f"{args.target_depth_m:.3f} m, current={current_text}"
                        ),
                        (10, 28),
                        cv2_module.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        1,
                    )
                cv2_module.imshow(window_name, combined)
                key = cv2_module.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
    finally:
        try:
            cv2_module.destroyWindow(window_name)
        except Exception:
            pass
    return 0


def main(
    argv: Sequence[str] | None = None,
    *,
    adapter_factory: Callable[..., Any] = RealSenseCameraAdapter,
    cv2_module=None,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    if not args.acknowledge_camera_preview:
        output_func("已停止：请显式确认本地摄像头实时预览。")
        return 2
    if args.target_pixel is not None:
        u, v = args.target_pixel
        if not 0 <= u < args.width or not 0 <= v < args.height:
            output_func("RealSense 预览失败：--target-pixel 超出图像范围。")
            return 2
    if args.target_depth_m is not None and args.target_depth_m <= 0.0:
        output_func("RealSense 预览失败：--target-depth-m 必须大于 0。")
        return 2
    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except (ImportError, OSError) as error:
            output_func(f"RealSense 预览失败：OpenCV GUI 不可用：{error}")
            return 1
    try:
        return run_preview(
            args,
            adapter_factory=adapter_factory,
            cv2_module=cv2_module,
        )
    except (ValueError, RuntimeError, VisionError) as error:
        output_func(f"RealSense 预览失败：{error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
