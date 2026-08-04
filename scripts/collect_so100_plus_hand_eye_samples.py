"""Collect synchronized eye-in-hand samples without writing robot registers."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path

from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_GRIPPER_TCP_OFFSET_M,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_readonly_robot,
)
from rosclaw_mini.arm.so100_plus_session import (
    SO100PlusPoseSnapshot,
    read_so100_plus_pose_snapshot,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
)
from rosclaw_mini.safety.limits import (
    SO100_PLUS_MODEL_JOINT_LIMITS,
    SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS,
)
from rosclaw_mini.vision.calibration import (
    CameraIntrinsicCalibration,
    load_camera_intrinsic_calibration,
    validate_camera_intrinsic_binding,
)
from rosclaw_mini.vision.camera import CameraAdapter
from rosclaw_mini.vision.exceptions import (
    CameraCalibrationError,
    CheckerboardDetectionError,
    HandEyeCalibrationError,
    VisionError,
)
from rosclaw_mini.vision.hand_eye import (
    HandEyeDataset,
    HandEyeSample,
    SO100_PLUS_HAND_EYE_KINEMATICS_MODEL,
    SO100_PLUS_HAND_EYE_REFERENCE_FRAME,
    estimate_checkerboard_pose,
    write_hand_eye_dataset,
)
from rosclaw_mini.vision.image import OpenCVImageProcessor


DEFAULT_HAND_EYE_SAMPLE_COUNT = 15
MAX_CAPTURE_JOINT_DRIFT_DEGREES = 1.0
OutputFunction = Callable[[str], None]
PoseReader = Callable[[], SO100PlusPoseSnapshot]
PoseValidator = Callable[[SO100PlusPoseSnapshot], None]
SampleCallback = Callable[[tuple[HandEyeSample, ...]], None]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "在力矩关闭时手动摆动 SO-100 Plus，同步采集关节反馈"
            "和腕部棋盘图像；程序不写任何电机寄存器。"
        )
    )
    parser.add_argument("--port", required=True, type=Path)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument("--camera-device", required=True, type=Path)
    parser.add_argument("--camera-intrinsics", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--count", type=int, default=DEFAULT_HAND_EYE_SAMPLE_COUNT)
    parser.add_argument("--pixel-format", default="YUYV")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--acknowledge-readonly-hand-eye-capture",
        action="store_true",
        help=(
            "确认脚本会打开真实串口和摄像头只读取，"
            "且不写 Goal_Position、力矩、PID 或校准。"
        ),
    )
    return parser


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as input_file:
            for block in iter(lambda: input_file.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise HandEyeCalibrationError(f"无法读取机械臂校准文件：{path}。") from error
    return digest.hexdigest()


def validate_readonly_hand_eye_snapshot(
    snapshot: SO100PlusPoseSnapshot,
    kinematics: SO100PlusKinematics,
    trajectory_validator: SO100PlusMuJoCoTrajectoryValidator,
) -> None:
    """Validate one manually placed pose without planning any motion."""

    validate_readonly_hand_eye_torque_state(snapshot)
    try:
        SO100_PLUS_MODEL_JOINT_LIMITS.validate_position(snapshot.joint_radians)
        SO100_PLUS_RIGHT_FOLLOWER_SHOULDER_ROTATION_DRIVER_LIMITS.validate(
            snapshot.driver_degrees[0],
            "手眼采集底座关节",
        )
    except ValueError as error:
        raise HandEyeCalibrationError(
            f"当前姿态不能作为手眼标定样本：{error}"
        ) from error
    if snapshot.gripper_driver_degrees is None:
        raise HandEyeCalibrationError("手眼采集缺少夹爪位置反馈。")
    gripper_qpos = trajectory_validator.gripper_driver_degrees_to_qpos(
        snapshot.gripper_driver_degrees
    )
    trajectory_validator.verify_collision_free_pose(
        snapshot.joint_radians,
        kinematics,
        gripper_qpos=gripper_qpos,
    )


def validate_readonly_hand_eye_torque_state(
    snapshot: SO100PlusPoseSnapshot,
) -> None:
    """Allow REST at startup while still requiring every motor torque off."""

    if not snapshot.torque_enabled:
        raise HandEyeCalibrationError("未读到力矩状态，已拒绝手眼样本。")
    if any(int(value) != 0 for value in snapshot.torque_enabled):
        raise HandEyeCalibrationError(
            f"手眼采集只允许在力矩全部关闭时执行："
            f"Torque_Enable={snapshot.torque_enabled}。"
        )


def capture_synchronized_hand_eye_sample(
    *,
    sample_index: int,
    camera,
    pose_reader: PoseReader,
    pose_validator: PoseValidator,
    kinematics: SO100PlusKinematics,
    intrinsics: CameraIntrinsicCalibration,
    image_path: Path,
    image_processor: OpenCVImageProcessor,
    max_joint_drift_degrees: float = MAX_CAPTURE_JOINT_DRIFT_DEGREES,
    pose_estimator=estimate_checkerboard_pose,
    cv2_module=None,
) -> HandEyeSample:
    """Read joints, capture one frame, then prove the arm remained still."""

    before = pose_reader()
    pose_validator(before)
    frame = camera.capture_frame()
    estimate = pose_estimator(
        frame,
        intrinsics,
        cv2_module=cv2_module,
    )
    after = pose_reader()
    pose_validator(after)
    if len(before.joint_radians) != len(after.joint_radians):
        raise HandEyeCalibrationError("采样前后关节数不一致。")
    drift_degrees = tuple(
        math.degrees(abs(end - start))
        for start, end in zip(
            before.joint_radians,
            after.joint_radians,
            strict=True,
        )
    )
    maximum_drift = max(drift_degrees)
    if maximum_drift > max_joint_drift_degrees:
        raise HandEyeCalibrationError(
            "拍照前后机械臂没有保持静止："
            f"最大关节漂移 {maximum_drift:.3f}°，"
            f"允许 {max_joint_drift_degrees:.1f}°。"
        )
    base_from_tcp = kinematics.forward_transform(before.joint_radians)
    image_processor.save(frame, image_path)
    return HandEyeSample(
        sample_id=f"sample_{sample_index:03d}",
        captured_at=datetime.now(timezone.utc).isoformat(),
        image_path=str(image_path),
        joint_radians=before.joint_radians,
        base_from_tcp=tuple(
            tuple(float(value) for value in row)
            for row in base_from_tcp
        ),
        camera_from_target=estimate.camera_from_target,
        reprojection_error_px=estimate.reprojection_error_px,
    )


def collect_hand_eye_samples_with_preview(
    *,
    camera_device: Path,
    intrinsics: CameraIntrinsicCalibration,
    output_directory: Path,
    count: int,
    pixel_format: str,
    overwrite: bool,
    pose_reader: PoseReader,
    pose_validator: PoseValidator,
    kinematics: SO100PlusKinematics,
    output_func: OutputFunction = print,
    camera_factory=CameraAdapter,
    image_processor: OpenCVImageProcessor | None = None,
    cv2_module=None,
    pose_estimator=estimate_checkerboard_pose,
    sample_callback: SampleCallback | None = None,
) -> tuple[HandEyeSample, ...]:
    if not camera_device.is_absolute():
        raise HandEyeCalibrationError("--camera-device 必须是绝对路径。")
    if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
        raise HandEyeCalibrationError("--count 必须是正整数。")
    validate_camera_intrinsic_binding(
        intrinsics,
        device=camera_device,
        width=intrinsics.camera_identity.width,
        height=intrinsics.camera_identity.height,
        pixel_format=pixel_format,
    )
    if cv2_module is None:
        try:
            import cv2 as cv2_module
        except (ImportError, OSError) as error:
            raise HandEyeCalibrationError(
                "手眼采集预览需要带 GUI 支持的 opencv-python。"
            ) from error
    processor = image_processor or OpenCVImageProcessor(cv2_module=cv2_module)
    output_directory.mkdir(parents=True, exist_ok=True)
    image_paths = tuple(
        output_directory / f"hand_eye_sample_{index:03d}.jpg"
        for index in range(1, count + 1)
    )
    existing = tuple(path for path in image_paths if path.exists())
    if existing and not overwrite:
        raise HandEyeCalibrationError(
            f"手眼采集目录已有 {len(existing)} 张同名图片。"
        )

    window_name = "RosClaw SO100 Plus Read-Only Hand-Eye Capture"
    samples: list[HandEyeSample] = []
    output_func(
        "只读手眼采集已启动：请托住已关闭力矩的机械臂并手动摆姿；"
        "绿色 READY 后按 Space/C 同步采样，Q/Esc 结束。"
    )
    try:
        cv2_module.namedWindow(window_name, cv2_module.WINDOW_NORMAL)
        with camera_factory(str(camera_device)) as camera:
            while len(samples) < count:
                frame = camera.capture_frame()
                width, height = processor.dimensions(frame)
                validate_camera_intrinsic_binding(
                    intrinsics,
                    device=camera_device,
                    width=width,
                    height=height,
                    pixel_format=pixel_format,
                )
                ready = False
                try:
                    estimate = pose_estimator(
                        frame,
                        intrinsics,
                        cv2_module=cv2_module,
                    )
                    ready = True
                    state_text = (
                        f"READY error={estimate.reprojection_error_px:.3f}px "
                        f"saved={len(samples)}/{count}"
                    )
                    color = (0, 200, 0)
                except CheckerboardDetectionError:
                    state_text = "NOT READY - show all 7x6 corners"
                    color = (0, 0, 255)
                display = frame.copy()
                cv2_module.putText(
                    display,
                    state_text,
                    (12, 28),
                    cv2_module.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2_module.LINE_AA,
                )
                cv2_module.imshow(window_name, display)
                key = cv2_module.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                if key not in (ord("c"), ord("C"), 32):
                    continue
                if not ready:
                    output_func("未保存：当前帧没有完整 7×6 棋盘内角点。")
                    continue
                try:
                    sample = capture_synchronized_hand_eye_sample(
                        sample_index=len(samples) + 1,
                        camera=camera,
                        pose_reader=pose_reader,
                        pose_validator=pose_validator,
                        kinematics=kinematics,
                        intrinsics=intrinsics,
                        image_path=image_paths[len(samples)],
                        image_processor=processor,
                        pose_estimator=pose_estimator,
                        cv2_module=cv2_module,
                    )
                except (CameraCalibrationError, VisionError, RuntimeError) as error:
                    output_func(f"未保存当前样本：{error}")
                    continue
                samples.append(sample)
                if sample_callback is not None:
                    sample_callback(tuple(samples))
                output_func(
                    f"已保存 {sample.sample_id}："
                    f"重投影误差 {sample.reprojection_error_px:.6f} px。"
                )
    finally:
        try:
            cv2_module.destroyWindow(window_name)
        except Exception:
            pass
    return tuple(samples)


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.acknowledge_readonly_hand_eye_capture:
        print(
            "已停止：必须显式确认串口和摄像头只读采集风险。"
        )
        return 2

    robot_config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    dataset_path = args.output_dir / "hand_eye_dataset.json"
    if dataset_path.exists() and not args.overwrite:
        print(f"手眼采集失败：数据集已存在：{dataset_path}。")
        return 1
    try:
        intrinsics = load_camera_intrinsic_calibration(args.camera_intrinsics)
        kinematics = SO100PlusKinematics()
        trajectory_validator = SO100PlusMuJoCoTrajectoryValidator()
        robot = create_so100_plus_readonly_robot(robot_config)
        follower_bus = robot.follower_arms[args.follower_name]
        robot_calibration_path = robot_config.calibration_path
        robot_calibration_sha256 = _sha256_file(robot_calibration_path)
    except (ValueError, RuntimeError, CameraCalibrationError) as error:
        print(f"手眼采集预检失败：{error}")
        return 1

    print(
        "即将只读打开机械臂串口和腕部摄像头；"
        "Goal_Position/力矩/PID/校准写入均为 0。",
        flush=True,
    )
    dataset_created_at = datetime.now(timezone.utc).isoformat()

    def build_dataset(samples: tuple[HandEyeSample, ...]) -> HandEyeDataset:
        return HandEyeDataset(
            camera_device=str(args.camera_device),
            intrinsics_sha256=intrinsics.calibration_sha256,
            robot_port=str(args.port),
            robot_calibration_filename=robot_calibration_path.name,
            robot_calibration_sha256=robot_calibration_sha256,
            reference_frame=SO100_PLUS_HAND_EYE_REFERENCE_FRAME,
            kinematics_model=SO100_PLUS_HAND_EYE_KINEMATICS_MODEL,
            tcp_offset_m=SO100_PLUS_GRIPPER_TCP_OFFSET_M,
            samples=samples,
            created_at=dataset_created_at,
        )

    def persist_samples(samples: tuple[HandEyeSample, ...]) -> None:
        write_hand_eye_dataset(
            build_dataset(samples),
            dataset_path,
            overwrite=dataset_path.exists(),
        )

    try:
        robot.connect()

        def pose_reader() -> SO100PlusPoseSnapshot:
            return read_so100_plus_pose_snapshot(
                follower_bus,
                kinematics,
                include_torque=True,
            )

        def pose_validator(snapshot: SO100PlusPoseSnapshot) -> None:
            validate_readonly_hand_eye_snapshot(
                snapshot,
                kinematics,
                trajectory_validator,
            )

        validate_readonly_hand_eye_torque_state(pose_reader())
        print(
            "力矩已确认全部关闭；初始姿态可以是 follower_rest。"
            "打开预览后，只在按 Space/C 时验证当前姿态是否可作为手眼样本。",
            flush=True,
        )
        samples = collect_hand_eye_samples_with_preview(
            camera_device=args.camera_device,
            intrinsics=intrinsics,
            output_directory=args.output_dir,
            count=args.count,
            pixel_format=args.pixel_format,
            overwrite=args.overwrite,
            pose_reader=pose_reader,
            pose_validator=pose_validator,
            kinematics=kinematics,
            sample_callback=persist_samples,
        )
        dataset = build_dataset(samples)
        if not dataset_path.exists():
            write_hand_eye_dataset(dataset, dataset_path, overwrite=False)
    except (ValueError, RuntimeError, CameraCalibrationError, VisionError) as error:
        print(f"手眼采集失败：{error}")
        return 1
    finally:
        if robot.is_connected:
            robot.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print(
            "只读通信已关闭；程序未改变力矩或发送运动命令。",
            flush=True,
        )

    print(
        f"手眼采集结束：{len(dataset.samples)} 组，"
        f"数据集 {dataset_path}，SHA-256={dataset.dataset_sha256}。"
    )
    if len(dataset.samples) < DEFAULT_HAND_EYE_SAMPLE_COUNT:
        print("样本数未达到默认 15 组，不建议进入正式求解。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
