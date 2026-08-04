"""Manually collect wrist-camera intrinsic calibration images; never move the arm."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from pathlib import Path

from rosclaw_mini.vision.camera import CameraAdapter
from rosclaw_mini.vision.calibration import (
    SO100_PLUS_WRIST_CHECKERBOARD,
    detect_checkerboard_corners,
)
from rosclaw_mini.vision.exceptions import (
    CheckerboardDetectionError,
    VisionError,
)
from rosclaw_mini.vision.image import OpenCVImageProcessor


InputFunction = Callable[[str], str]
OutputFunction = Callable[[str], None]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "手动逐张采集腕部相机内参图片；每次按 Enter 只读取一帧，"
            "不连接或移动机械臂。"
        )
    )
    parser.add_argument("--device", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--count", type=int, default=15)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--preview",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "默认打开实时预览，Space/C 保存完整检测到角点的帧，Q/Esc 退出；"
            "使用 --no-preview 退回终端逐张确认。"
        ),
    )
    parser.add_argument(
        "--acknowledge-camera-capture",
        action="store_true",
        help="确认只采集相机图片，不控制机械臂。",
    )
    return parser


def collect_images(
    *,
    device: Path,
    output_directory: Path,
    count: int,
    expected_width: int,
    expected_height: int,
    overwrite: bool,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
    camera_factory=CameraAdapter,
    image_processor: OpenCVImageProcessor | None = None,
) -> tuple[Path, ...]:
    if not device.is_absolute():
        raise ValueError("--device 必须是绝对路径。")
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("--count 必须是正整数。")
    if expected_width <= 0 or expected_height <= 0:
        raise ValueError("图像宽高必须是正整数。")
    processor = image_processor or OpenCVImageProcessor()
    output_directory.mkdir(parents=True, exist_ok=True)
    output_paths = tuple(
        output_directory / f"wrist_view_{index:03d}.jpg"
        for index in range(1, count + 1)
    )
    existing = tuple(path for path in output_paths if path.exists())
    if existing and not overwrite:
        raise ValueError(
            f"已有 {len(existing)} 张同名图片；请更换目录或显式 --overwrite。"
        )

    captured: list[Path] = []
    for index, output_path in enumerate(output_paths, start=1):
        answer = input_func(
            f"[{index}/{count}] 摆好棋盘并保证四周留白后按 Enter；输入 q 结束："
        ).strip().lower()
        if answer == "q":
            break
        with camera_factory(str(device)) as camera:
            frame = camera.capture_frame()
        width, height = processor.dimensions(frame)
        if (width, height) != (expected_width, expected_height):
            raise ValueError(
                f"实际图像为 {width}×{height}，期望 "
                f"{expected_width}×{expected_height}；已停止，禁止混用分辨率。"
            )
        processor.save(frame, output_path)
        captured.append(output_path)
        output_func(f"已保存 {output_path}")
    return tuple(captured)


def collect_images_with_preview(
    *,
    device: Path,
    output_directory: Path,
    count: int,
    expected_width: int,
    expected_height: int,
    overwrite: bool,
    output_func: OutputFunction = print,
    camera_factory=CameraAdapter,
    image_processor: OpenCVImageProcessor | None = None,
    cv2_module=None,
    corner_detector=detect_checkerboard_corners,
) -> tuple[Path, ...]:
    """Show live video and save only frames with all 7x6 corners detected."""

    if not device.is_absolute():
        raise ValueError("--device 必须是绝对路径。")
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise ValueError("--count 必须是正整数。")
    if expected_width <= 0 or expected_height <= 0:
        raise ValueError("图像宽高必须是正整数。")
    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except (ImportError, OSError) as error:
            raise ValueError("实时预览需要带 GUI 支持的 opencv-python。") from error

    processor = image_processor or OpenCVImageProcessor(cv2_module=cv2_module)
    output_directory.mkdir(parents=True, exist_ok=True)
    output_paths = tuple(
        output_directory / f"wrist_view_{index:03d}.jpg"
        for index in range(1, count + 1)
    )
    existing = tuple(path for path in output_paths if path.exists())
    if existing and not overwrite:
        raise ValueError(
            f"已有 {len(existing)} 张同名图片；请更换目录或显式 --overwrite。"
        )

    window_name = "RosClaw Wrist Camera Calibration"
    captured: list[Path] = []
    output_func(
        "实时预览已启动：绿色角点表示完整识别；按 Space 或 C 保存，"
        "按 Q 或 Esc 结束。"
    )
    try:
        cv2_module.namedWindow(window_name, cv2_module.WINDOW_NORMAL)
        with camera_factory(str(device)) as camera:
            while len(captured) < count:
                frame = camera.capture_frame()
                width, height = processor.dimensions(frame)
                if (width, height) != (expected_width, expected_height):
                    raise ValueError(
                        f"实际图像为 {width}×{height}，期望 "
                        f"{expected_width}×{expected_height}；"
                        "已停止，禁止混用分辨率。"
                    )

                display = frame.copy()
                corners = None
                detected = False
                try:
                    corners = corner_detector(
                        frame,
                        SO100_PLUS_WRIST_CHECKERBOARD,
                        cv2_module=cv2_module,
                    )
                    detected = True
                except CheckerboardDetectionError:
                    detected = False

                if detected:
                    cv2_module.drawChessboardCorners(
                        display,
                        SO100_PLUS_WRIST_CHECKERBOARD.pattern_size,
                        corners,
                        True,
                    )
                    state_text = "READY - Space/C to capture"
                    state_color = (0, 180, 0)
                else:
                    state_text = "NOT READY - show all 7x6 corners"
                    state_color = (0, 0, 255)
                cv2_module.putText(
                    display,
                    f"{state_text}  {len(captured)}/{count}",
                    (12, 28),
                    cv2_module.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    state_color,
                    2,
                    cv2_module.LINE_AA,
                )
                cv2_module.imshow(window_name, display)
                key = cv2_module.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("c"), ord("C"), 32):
                    if not detected:
                        output_func("未保存：当前帧没有完整检测到 7×6 内角点。")
                        continue
                    output_path = output_paths[len(captured)]
                    processor.save(frame, output_path)
                    captured.append(output_path)
                    output_func(f"已保存 {output_path}")
    except Exception as error:
        if isinstance(error, (ValueError, VisionError)):
            raise
        raise ValueError(f"实时预览失败：{error}") from error
    finally:
        try:
            cv2_module.destroyWindow(window_name)
        except Exception:
            pass
    return tuple(captured)


def main(
    argv: Sequence[str] | None = None,
    *,
    input_func: InputFunction = input,
    output_func: OutputFunction = print,
) -> int:
    args = build_parser().parse_args(argv)
    if not args.acknowledge_camera_capture:
        output_func("已停止：必须显式确认只采集相机图片。")
        return 2
    try:
        collector = (
            collect_images_with_preview if args.preview else collect_images
        )
        collector_kwargs = {
            "device": args.device,
            "output_directory": args.output_dir,
            "count": args.count,
            "expected_width": args.width,
            "expected_height": args.height,
            "overwrite": args.overwrite,
            "output_func": output_func,
        }
        if not args.preview:
            collector_kwargs["input_func"] = input_func
        paths = collector(**collector_kwargs)
    except (ValueError, VisionError) as error:
        output_func(f"采集失败：{error}")
        return 1
    output_func(
        f"采集结束：共 {len(paths)} 张；未连接或移动机械臂。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
