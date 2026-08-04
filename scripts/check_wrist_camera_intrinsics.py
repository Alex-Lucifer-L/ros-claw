"""Live, read-only validation of one wrist-camera intrinsic calibration."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from pathlib import Path

from rosclaw_mini.vision.calibration import (
    CameraIntrinsicCalibration,
    load_camera_intrinsic_calibration,
    measure_checkerboard_reprojection_error,
    undistort_camera_frame,
    validate_camera_intrinsic_binding,
)
from rosclaw_mini.vision.camera import CameraAdapter
from rosclaw_mini.vision.exceptions import (
    CameraCalibrationError,
    CheckerboardDetectionError,
    VisionError,
)
from rosclaw_mini.vision.image import OpenCVImageProcessor


OutputFunction = Callable[[str], None]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "实时对比腕部摄像头原图与去畸变图，"
            "只读取摄像头，不连接机械臂。"
        )
    )
    parser.add_argument("--device", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--count", type=int, default=3)
    parser.add_argument("--alpha", type=float, default=1.0)
    parser.add_argument("--pixel-format", default="YUYV")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--acknowledge-camera-capture",
        action="store_true",
        help="确认只读取摄像头画面，不控制机械臂。",
    )
    return parser


def _put_text(cv2_module, image, text: str, origin, color) -> None:
    cv2_module.putText(
        image,
        text,
        origin,
        cv2_module.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2_module.LINE_AA,
    )


def preview_intrinsic_calibration(
    *,
    device: Path,
    calibration: CameraIntrinsicCalibration,
    output_directory: Path,
    count: int,
    alpha: float,
    pixel_format: str,
    overwrite: bool,
    output_func: OutputFunction = print,
    camera_factory=CameraAdapter,
    image_processor: OpenCVImageProcessor | None = None,
    cv2_module=None,
    undistorter=undistort_camera_frame,
    reprojection_error_measure=measure_checkerboard_reprojection_error,
) -> tuple[tuple[Path, Path, float], ...]:
    """Capture new checkerboard views after live raw/corrected comparison."""

    if not device.is_absolute():
        raise CameraCalibrationError("--device 必须是绝对路径。")
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise CameraCalibrationError("--count 必须是正整数。")
    validate_camera_intrinsic_binding(
        calibration,
        device=device,
        width=calibration.camera_identity.width,
        height=calibration.camera_identity.height,
        pixel_format=pixel_format,
    )
    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except (ImportError, OSError) as error:
            raise CameraCalibrationError(
                "实时去畸变预览需要带 GUI 支持的 opencv-python。"
            ) from error

    processor = image_processor or OpenCVImageProcessor(cv2_module=cv2_module)
    output_directory.mkdir(parents=True, exist_ok=True)
    output_pairs = tuple(
        (
            output_directory / f"validation_view_{index:03d}_raw.jpg",
            output_directory
            / f"validation_view_{index:03d}_undistorted.jpg",
        )
        for index in range(1, count + 1)
    )
    existing = tuple(
        path
        for pair in output_pairs
        for path in pair
        if path.exists()
    )
    if existing and not overwrite:
        raise CameraCalibrationError(
            f"验收目录已有 {len(existing)} 个同名文件；"
            "请更换目录或显式 --overwrite。"
        )

    window_name = "RosClaw Wrist Camera Intrinsic Check"
    samples: list[tuple[Path, Path, float]] = []
    output_func(
        "内参验收预览已启动：左侧 RAW，右侧 UNDISTORTED；"
        "检测到完整棋盘后按 Space 或 C 保存新验收样本，"
        "按 Q 或 Esc 结束。"
    )
    try:
        cv2_module.namedWindow(window_name, cv2_module.WINDOW_NORMAL)
        with camera_factory(str(device)) as camera:
            while len(samples) < count:
                frame = camera.capture_frame()
                width, height = processor.dimensions(frame)
                validate_camera_intrinsic_binding(
                    calibration,
                    device=device,
                    width=width,
                    height=height,
                    pixel_format=pixel_format,
                )
                corrected = undistorter(
                    frame,
                    calibration,
                    alpha=alpha,
                    cv2_module=cv2_module,
                )

                checkerboard_error = None
                try:
                    checkerboard_error = reprojection_error_measure(
                        frame,
                        calibration,
                        cv2_module=cv2_module,
                    )
                except CheckerboardDetectionError:
                    checkerboard_error = None

                raw_display = frame.copy()
                corrected_display = corrected.copy()
                _put_text(cv2_module, raw_display, "RAW", (12, 28), (0, 220, 255))
                _put_text(
                    cv2_module,
                    corrected_display,
                    "UNDISTORTED",
                    (12, 28),
                    (0, 220, 0),
                )
                if checkerboard_error is None:
                    status = "NO BOARD - show all 7x6 corners"
                    status_color = (0, 0, 255)
                else:
                    status = f"BOARD ERROR {checkerboard_error:.3f} px"
                    status_color = (0, 220, 0)
                _put_text(
                    cv2_module,
                    raw_display,
                    status,
                    (12, height - 16),
                    status_color,
                )
                _put_text(
                    cv2_module,
                    corrected_display,
                    f"saved {len(samples)}/{count}",
                    (12, height - 16),
                    status_color,
                )
                display = cv2_module.hconcat([raw_display, corrected_display])
                cv2_module.imshow(window_name, display)
                key = cv2_module.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key in (ord("c"), ord("C"), 32):
                    if checkerboard_error is None:
                        output_func(
                            "未保存：当前帧没有完整检测到 7×6 内角点。"
                        )
                        continue
                    raw_path, corrected_path = output_pairs[len(samples)]
                    processor.save(frame, raw_path)
                    processor.save(corrected, corrected_path)
                    samples.append(
                        (raw_path, corrected_path, float(checkerboard_error))
                    )
                    output_func(
                        f"已保存验收样本 {len(samples)}/{count}："
                        f"重投影误差 {checkerboard_error:.6f} px。"
                    )
    except Exception as error:
        if isinstance(error, (CameraCalibrationError, VisionError)):
            raise
        raise CameraCalibrationError(f"实时内参验收失败：{error}") from error
    finally:
        try:
            cv2_module.destroyWindow(window_name)
        except Exception:
            pass
    return tuple(samples)


def main(
    argv: Sequence[str] | None = None,
    *,
    output_func: OutputFunction = print,
) -> int:
    args = build_parser().parse_args(argv)
    if not args.acknowledge_camera_capture:
        output_func("已停止：必须显式确认只读取摄像头画面。")
        return 2
    try:
        calibration = load_camera_intrinsic_calibration(args.calibration)
        output_func(
            "已验证内参文件："
            f"{calibration.camera_identity.width}×"
            f"{calibration.camera_identity.height}，"
            f"RMS={calibration.rms_reprojection_error_px:.6f} px，"
            f"SHA-256={calibration.calibration_sha256}。"
        )
        samples = preview_intrinsic_calibration(
            device=args.device,
            calibration=calibration,
            output_directory=args.output_dir,
            count=args.count,
            alpha=args.alpha,
            pixel_format=args.pixel_format,
            overwrite=args.overwrite,
            output_func=output_func,
        )
    except (CameraCalibrationError, VisionError) as error:
        output_func(f"内参验收失败：{error}")
        return 1

    if samples:
        errors = tuple(sample[2] for sample in samples)
        output_func(
            "内参验收采样结束："
            f"共 {len(samples)} 组，"
            f"平均重投影误差 {sum(errors) / len(errors):.6f} px，"
            f"最大 {max(errors):.6f} px。"
        )
    else:
        output_func("内参验收采样结束：没有保存新样本。")
    output_func("全程未连接或移动机械臂。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
