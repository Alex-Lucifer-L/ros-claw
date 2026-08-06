"""Capture one fixed-camera/robot-base point pair without motor writes.

The operator is responsible for placing the same physical calibration mark at
the robot TCP.  This command only reads one synchronized RealSense target
position and the stationary SO-100 Plus joint feedback, then appends the pair
to the local, identity-bound eye-to-hand dataset.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any

from rosclaw_mini.arm.kinematics import SO100PlusKinematics
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_readonly_robot,
)
from rosclaw_mini.arm.so100_plus_session import (
    SO100PlusPoseSnapshot,
    read_so100_plus_pose_snapshot,
)
from rosclaw_mini.vision.exceptions import (
    EyeToHandCalibrationError,
    VisionError,
)
from rosclaw_mini.vision.eye_to_hand import append_eye_to_hand_point
from rosclaw_mini.vision.localization import RealSenseLocalizationService
from rosclaw_mini.vision.realsense import RealSenseCameraAdapter
from rosclaw_mini.vision.vlm_client import (
    DEFAULT_DASHSCOPE_BASE_URL,
    DEFAULT_QWEN_VL_MODEL,
    QwenVLMClient,
)


DEFAULT_MAX_JOINT_DRIFT_DEGREES = 1.0
PoseReader = Callable[[], SO100PlusPoseSnapshot]
PoseValidator = Callable[[SO100PlusPoseSnapshot], None]


@dataclass(frozen=True)
class CapturedEyeToHandPoint:
    camera_point_m: tuple[float, float, float]
    base_point_m: tuple[float, float, float]
    observation_id: str
    source_frame: int
    uncertainty_m: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "只读采集一组 D435i 目标点 / SO-100 Plus TCP 基座点对；"
            "不写力矩、目标位置、PID 或校准。"
        )
    )
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout-ms", type=int, default=10000)
    parser.add_argument("--question", required=True)
    parser.add_argument("--port", type=Path, required=True)
    parser.add_argument("--calibration-dir", type=Path, required=True)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument("--split", choices=("fit", "validation"), default="fit")
    parser.add_argument("--point-id", default=None)
    parser.add_argument(
        "--max-joint-drift-degrees",
        type=float,
        default=DEFAULT_MAX_JOINT_DRIFT_DEGREES,
    )
    parser.add_argument("--vlm-timeout", type=float, default=30.0)
    parser.add_argument("--vlm-model", default=None)
    parser.add_argument(
        "--acknowledge-readonly-arm-capture",
        action="store_true",
        help="确认只读打开真实机械臂串口，且当前力矩全部关闭。",
    )
    parser.add_argument(
        "--acknowledge-camera-cloud-upload",
        action="store_true",
        help="确认当前 D435i RGB 帧会发送给配置的千问视觉服务。",
    )
    return parser


def validate_readonly_point_snapshot(snapshot: SO100PlusPoseSnapshot) -> None:
    """Require finite stationary feedback with every motor torque disabled.

    This command never moves the arm.  A manually taught pose may lie outside
    the production motion envelope, so applying WORK motion limits here would
    conflate read-only calibration data collection with authorization to move.
    The production adapters retain all of their existing motion limits.
    """

    if not snapshot.torque_enabled:
        raise EyeToHandCalibrationError("未读到力矩状态，已拒绝点对采集。")
    if any(int(value) != 0 for value in snapshot.torque_enabled):
        raise EyeToHandCalibrationError(
            "点对采集只允许在力矩全部关闭时执行："
            f"Torque_Enable={snapshot.torque_enabled}。"
        )
    if len(snapshot.driver_degrees) != 6 or not all(
        math.isfinite(value) for value in snapshot.driver_degrees
    ):
        raise EyeToHandCalibrationError("当前驱动反馈必须是 6 个有限数值。")
    if len(snapshot.joint_radians) != 6 or not all(
        math.isfinite(value) for value in snapshot.joint_radians
    ):
        raise EyeToHandCalibrationError("当前模型关节反馈必须是 6 个有限数值。")
    if len(snapshot.tcp_position_m) != 3:
        raise EyeToHandCalibrationError("当前 TCP 基座坐标必须是 3 个有限数值。")
    if not all(math.isfinite(value) for value in snapshot.tcp_position_m):
        raise EyeToHandCalibrationError("当前 TCP 基座坐标不是有限数值。")


def capture_synchronized_eye_to_hand_point(
    *,
    question: str,
    localization_service,
    pose_reader: PoseReader,
    pose_validator: PoseValidator = validate_readonly_point_snapshot,
    max_joint_drift_degrees: float = DEFAULT_MAX_JOINT_DRIFT_DEGREES,
) -> CapturedEyeToHandPoint:
    if (
        isinstance(max_joint_drift_degrees, bool)
        or not isinstance(max_joint_drift_degrees, (int, float))
        or not math.isfinite(float(max_joint_drift_degrees))
        or float(max_joint_drift_degrees) <= 0.0
    ):
        raise EyeToHandCalibrationError(
            "max_joint_drift_degrees 必须是有限正数。"
        )
    before = pose_reader()
    pose_validator(before)
    localization = localization_service.locate(question)
    after = pose_reader()
    pose_validator(after)
    if len(before.joint_radians) != len(after.joint_radians):
        raise EyeToHandCalibrationError("采集前后关节数不一致。")
    drift = tuple(
        math.degrees(abs(end - start))
        for start, end in zip(
            before.joint_radians,
            after.joint_radians,
            strict=True,
        )
    )
    maximum_drift = max(drift, default=math.inf)
    if maximum_drift > float(max_joint_drift_degrees):
        raise EyeToHandCalibrationError(
            "D435i 取帧和机械臂反馈不同步："
            f"采集前后最大关节漂移 {maximum_drift:.3f}°，"
            f"允许 {float(max_joint_drift_degrees):.3f}°。"
        )
    return CapturedEyeToHandPoint(
        camera_point_m=tuple(localization.position.camera_point_m),
        base_point_m=tuple(before.tcp_position_m),
        observation_id=localization.observation.observation_id,
        source_frame=localization.position.source_frame,
        uncertainty_m=localization.position.uncertainty_m,
    )


def capture_real_point(args: argparse.Namespace, *, client) -> CapturedEyeToHandPoint:
    config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    kinematics = SO100PlusKinematics()
    robot = create_so100_plus_readonly_robot(config)
    follower_bus = robot.follower_arms[args.follower_name]
    service = RealSenseLocalizationService(
        client=client,
        camera_factory=lambda: RealSenseCameraAdapter(
            args.serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
            timeout_ms=args.timeout_ms,
        ),
    )
    try:
        robot.connect()

        def pose_reader() -> SO100PlusPoseSnapshot:
            return read_so100_plus_pose_snapshot(
                follower_bus,
                kinematics,
                include_torque=True,
            )

        return capture_synchronized_eye_to_hand_point(
            question=args.question,
            localization_service=service,
            pose_reader=pose_reader,
            max_joint_drift_degrees=args.max_joint_drift_degrees,
        )
    finally:
        if robot.is_connected:
            robot.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()


def main(
    argv: Sequence[str] | None = None,
    *,
    environ: Mapping[str, str] | None = None,
    client_builder: Callable[..., Any] = QwenVLMClient,
    point_capture: Callable[..., CapturedEyeToHandPoint] = capture_real_point,
    output_func: Callable[[str], None] = print,
) -> int:
    args = build_parser().parse_args(argv)
    if not args.acknowledge_readonly_arm_capture:
        output_func("已停止：需显式确认真实机械臂只读采集且力矩全部关闭。")
        return 2
    if not args.acknowledge_camera_cloud_upload:
        output_func("已停止：需显式确认 D435i RGB 画面上传给千问视觉服务。")
        return 2
    environment = os.environ if environ is None else environ
    api_key = (
        environment.get("ROSCLAW_LLM_API_KEY", "").strip()
        or environment.get("DASHSCOPE_API_KEY", "").strip()
    )
    if not api_key:
        output_func("点对采集配置错误：缺少百炼 API Key。")
        return 2
    model = (
        args.vlm_model.strip()
        if isinstance(args.vlm_model, str) and args.vlm_model.strip()
        else environment.get("DASHSCOPE_VL_MODEL", "").strip()
        or DEFAULT_QWEN_VL_MODEL
    )
    base_url = (
        environment.get("ROSCLAW_LLM_BASE_URL", "").strip()
        or DEFAULT_DASHSCOPE_BASE_URL
    )
    try:
        client = client_builder(
            base_url=base_url,
            model=model,
            api_key=api_key,
            timeout_seconds=args.vlm_timeout,
        )
        captured = point_capture(args, client=client)
        dataset = append_eye_to_hand_point(
            args.dataset,
            camera_serial=args.serial,
            width=args.width,
            height=args.height,
            camera_point_m=captured.camera_point_m,
            base_point_m=captured.base_point_m,
            split=args.split,
            point_id=args.point_id,
        )
    except (ValueError, RuntimeError, VisionError) as error:
        output_func(f"eye-to-hand 点对采集失败：{error}")
        return 1
    point = dataset.points[-1]
    output_func(
        f"已只读保存 {point.point_id}："
        f"camera={point.camera_point_m} m，base={point.base_point_m} m，"
        f"uncertainty={captured.uncertainty_m:.6f} m，"
        f"source_frame={captured.source_frame}，"
        f"dataset_sha256={dataset.dataset_sha256}。"
    )
    output_func("采集期间没有发送电机写入或运动命令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
