"""通过正式 SO100PlusAdapter 验证一次 Z 方向厘米级 move_to。"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time

from rosclaw_mini.arm.kinematics import SO100PlusKinematics
from rosclaw_mini.arm.so100_plus import (
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionConfig,
    SO100PlusTelemetry,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.safety.limits import (
    AxisLimits,
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
    WorkspaceLimits,
    build_so100_plus_right_follower_execution_joint_limits,
)


MAX_DELTA_Z_CM = 10.0
MAX_SEGMENT_Z_CM = 1.0
MAX_SEGMENT_CARTESIAN_DISTANCE_M = 0.015
SINGLE_PLAN_MAX_CARTESIAN_STEP_M = 0.005
SINGLE_PLAN_MAX_LATERAL_DEVIATION_M = 0.005
LOCAL_XY_MARGIN_M = 0.0005
LOCAL_Z_MARGIN_M = 0.0005
MAX_JOINT_STEP_RADIANS = math.radians(2.0)
OBSERVATION_SECONDS = 3.0
OBSERVATION_JOINT_ERROR_LIMIT_DEGREES = 8.0
OBSERVATION_CARTESIAN_ERROR_LIMIT_M = 0.015
DEFAULT_RECORD_OUTPUT = Path("/tmp/rosclaw_so100_move_to_steps.jsonl")


@dataclass(frozen=True)
class RecordedMoveStep:
    index: int
    requested_cumulative_cm: float
    start_position_m: tuple[float, float, float]
    target_position_m: tuple[float, float, float]
    actual_position_m: tuple[float, float, float]
    cartesian_error_m: float
    max_joint_error_name: str
    max_joint_error_degrees: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "保持当前 X/Y 和姿态，使用正式 Adapter 将夹爪 TCP 沿 +Z 移动。"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--delta-z-cm",
        type=float,
        default=1.0,
        help="Z 方向总移动厘米数，必须大于 0 且不超过 10.0，默认 1.0",
    )
    parser.add_argument(
        "--runtime-acceleration",
        type=int,
        default=SO100_PLUS_REAL_HARDWARE_PROFILE.runtime_acceleration,
        help="本次运行时加速度，默认使用已验证值 35",
    )
    parser.add_argument(
        "--acknowledge-move-to-risk",
        action="store_true",
        help="确认已清空工作区、托住机械臂并同意整条手臂移动及结束时关闭力矩",
    )
    parser.add_argument(
        "--record-errors-without-soft-stop",
        action="store_true",
        help=(
            "诊断模式：记录每段误差并继续；关节误差 8 度或 TCP 误差 1.5 cm "
            "仍会硬停止"
        ),
    )
    parser.add_argument(
        "--record-output",
        type=Path,
        default=DEFAULT_RECORD_OUTPUT,
        help=(
            "误差记录模式的实时 JSONL 文件；默认 "
            "/tmp/rosclaw_so100_move_to_steps.jsonl"
        ),
    )
    parser.add_argument(
        "--single-cartesian-plan",
        action="store_true",
        help=(
            "只求解一次完整 +Z 目标，但仍保留内部 2 度关节轨迹点和路径检查"
        ),
    )
    parser.add_argument(
        "--stream-frequency-hz",
        type=float,
        default=SO100_PLUS_REAL_HARDWARE_PROFILE.stream_frequency_hz,
        help="连续流式轨迹命令频率，默认使用已验证值 20 Hz",
    )
    parser.add_argument(
        "--stream-max-joint-speed-degrees-per-second",
        type=float,
        default=(
            SO100_PLUS_REAL_HARDWARE_PROFILE.stream_max_joint_speed_degrees_per_second
        ),
        help="流式轨迹最大关节速度，默认 20°/s",
    )
    return parser


def _arm_driver_degrees(follower_bus) -> tuple[float, ...]:
    names = tuple(follower_bus.motor_names)
    positions = tuple(
        float(value) for value in follower_bus.read("Present_Position")
    )
    if len(names) != len(positions):
        raise RuntimeError("电机名称与位置数量不一致。")
    position_by_name = dict(zip(names, positions, strict=True))
    missing = tuple(
        name for name in SO100_PLUS_ARM_JOINT_NAMES if name not in position_by_name
    )
    if missing:
        raise RuntimeError(f"缺少手臂关节：{', '.join(missing)}。")
    return tuple(position_by_name[name] for name in SO100_PLUS_ARM_JOINT_NAMES)


def _local_motion_limits(
    current_position_m: tuple[float, float, float],
    current_joint_radians: tuple[float, ...],
    target_position_m: tuple[float, float, float],
) -> MotionLimits:
    x, y, z = current_position_m
    target_x, target_y, target_z = target_position_m
    return MotionLimits(
        workspace=WorkspaceLimits(
            x=AxisLimits(
                min(x, target_x) - LOCAL_XY_MARGIN_M,
                max(x, target_x) + LOCAL_XY_MARGIN_M,
            ),
            y=AxisLimits(
                min(y, target_y) - LOCAL_XY_MARGIN_M,
                max(y, target_y) + LOCAL_XY_MARGIN_M,
            ),
            z=AxisLimits(
                min(z, target_z) - LOCAL_Z_MARGIN_M,
                max(z, target_z) + LOCAL_Z_MARGIN_M,
            ),
        ),
        joints=build_so100_plus_right_follower_execution_joint_limits(
            current_joint_radians,
            max_step_radians=MAX_JOINT_STEP_RADIANS,
        ),
    )


def print_telemetry(telemetry: SO100PlusTelemetry) -> None:
    print(
        f"telemetry phase={telemetry.phase} "
        f"max_load={max(telemetry.load_magnitude):.1f} "
        f"temperatures={tuple(int(value) for value in telemetry.temperature_raw)}",
        flush=True,
    )


def _validate_single_cartesian_plan(
    kinematics: SO100PlusKinematics,
    current_position_m: tuple[float, float, float],
    waypoints_radians: tuple[tuple[float, ...], ...],
) -> tuple[float, float]:
    """拒绝内部步长过大、横向偏移过大或出现反向 Z 的单次规划。"""

    if not waypoints_radians:
        raise RuntimeError("单次笛卡尔规划没有生成内部轨迹点。")
    positions = (current_position_m,) + tuple(
        kinematics.forward_position(waypoint)
        for waypoint in waypoints_radians
    )
    cartesian_steps = tuple(
        math.dist(before, after)
        for before, after in zip(positions, positions[1:])
    )
    lateral_deviations = tuple(
        math.hypot(
            position[0] - current_position_m[0],
            position[1] - current_position_m[1],
        )
        for position in positions
    )
    z_steps = tuple(
        after[2] - before[2]
        for before, after in zip(positions, positions[1:])
    )
    max_cartesian_step = max(cartesian_steps)
    max_lateral_deviation = max(lateral_deviations)
    if max_cartesian_step > SINGLE_PLAN_MAX_CARTESIAN_STEP_M:
        raise RuntimeError(
            f"单次规划内部 TCP 步长 {max_cartesian_step * 1000:.3f} mm "
            f"超过 {SINGLE_PLAN_MAX_CARTESIAN_STEP_M * 1000:.1f} mm。"
        )
    if max_lateral_deviation > SINGLE_PLAN_MAX_LATERAL_DEVIATION_M:
        raise RuntimeError(
            f"单次规划横向偏移 {max_lateral_deviation * 1000:.3f} mm "
            f"超过 {SINGLE_PLAN_MAX_LATERAL_DEVIATION_M * 1000:.1f} mm。"
        )
    if any(z_step <= 0 for z_step in z_steps):
        raise RuntimeError("单次规划包含不向上的内部轨迹点。")
    return max_cartesian_step, max_lateral_deviation


def _record_move_step(
    *,
    index: int,
    requested_cumulative_cm: float,
    start_position_m: tuple[float, float, float],
    target_position_m: tuple[float, float, float],
    plan,
    follower_bus,
    kinematics: SO100PlusKinematics,
) -> RecordedMoveStep:
    actual_driver = _arm_driver_degrees(follower_bus)
    actual_joints = kinematics.driver_degrees_to_model_radians(actual_driver)
    actual_position = kinematics.forward_position(actual_joints)
    planned_driver = kinematics.model_radians_to_driver_degrees(
        plan.target_joint_radians
    )
    joint_errors = tuple(
        (name, abs(actual - target))
        for name, actual, target in zip(
            SO100_PLUS_ARM_JOINT_NAMES,
            actual_driver,
            planned_driver,
            strict=True,
        )
    )
    max_error_name, max_joint_error = max(
        joint_errors,
        key=lambda item: item[1],
    )
    cartesian_error = math.sqrt(
        sum(
            (actual - target) ** 2
            for actual, target in zip(
                actual_position,
                target_position_m,
                strict=True,
            )
        )
    )
    return RecordedMoveStep(
        index=index,
        requested_cumulative_cm=requested_cumulative_cm,
        start_position_m=start_position_m,
        target_position_m=target_position_m,
        actual_position_m=actual_position,
        cartesian_error_m=cartesian_error,
        max_joint_error_name=max_error_name,
        max_joint_error_degrees=max_joint_error,
    )


def _print_recorded_summary(
    records: list[RecordedMoveStep],
    initial_position_m: tuple[float, float, float] | None,
) -> None:
    if not records or initial_position_m is None:
        return
    print("\n误差记录模式总结：", flush=True)
    for record in records:
        print(
            f"  第 {record.index} 段：请求累计 "
            f"{record.requested_cumulative_cm:.2f} cm，"
            f"目标 Z={record.target_position_m[2]:.6f} m，"
            f"实际 Z={record.actual_position_m[2]:.6f} m，"
            f"TCP 误差={record.cartesian_error_m * 1000:.3f} mm，"
            f"最大关节误差={record.max_joint_error_name} "
            f"{record.max_joint_error_degrees:.3f}°",
            flush=True,
        )
    final_position = records[-1].actual_position_m
    actual_rise_cm = (final_position[2] - initial_position_m[2]) * 100.0
    print(
        f"  实际累计 +Z：{actual_rise_cm:.3f} cm；"
        f"计划累计 +Z：{records[-1].requested_cumulative_cm:.3f} cm。",
        flush=True,
    )


def _append_record_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as output_file:
        output_file.write(json.dumps(payload, ensure_ascii=False) + "\n")


def main() -> int:
    args = build_parser().parse_args()
    if not args.acknowledge_move_to_risk:
        raise SystemExit("已停止：必须显式确认 move_to 真机风险。")
    if args.follower_name != "right":
        raise SystemExit("已停止：本次实测限制只适用于 follower_name=right。")
    if (
        not math.isfinite(args.delta_z_cm)
        or not 0 < args.delta_z_cm <= MAX_DELTA_Z_CM
    ):
        raise SystemExit("已停止：delta-z-cm 必须大于 0 且不超过 10.0。")
    if args.stream_frequency_hz is not None and (
        not math.isfinite(args.stream_frequency_hz)
        or not 5.0 <= args.stream_frequency_hz <= 50.0
    ):
        raise SystemExit("已停止：stream-frequency-hz 必须在 5 到 50 之间。")
    if (
        not math.isfinite(args.stream_max_joint_speed_degrees_per_second)
        or not 0 < args.stream_max_joint_speed_degrees_per_second <= 60.0
    ):
        raise SystemExit(
            "已停止：流式最大关节速度必须大于 0 且不超过 60°/s。"
        )

    robot_config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )
    robot = create_so100_plus_robot(robot_config)
    gripper_config = SO100PlusGripperConfig(
        follower_name=args.follower_name,
        open_degrees=60.0,
        close_degrees=-5.0,
        runtime_acceleration=args.runtime_acceleration,
    )
    connection_adapter = SO100PlusAdapter(
        robot,
        gripper_config,
        on_telemetry=print_telemetry,
    )
    execution_adapter = None
    follower_bus = robot.follower_arms[args.follower_name]
    exit_code = 0
    recorded_steps: list[RecordedMoveStep] = []
    initial_position = None
    failure_message = None
    if args.record_errors_without_soft_stop:
        args.record_output.parent.mkdir(parents=True, exist_ok=True)
        args.record_output.write_text("", encoding="utf-8")

    print(
        "即将连接机械臂；connect 会启用扭矩并设置运行时加速度。",
        flush=True,
    )
    try:
        connection_adapter.connect()
        kinematics = SO100PlusKinematics()
        initial_driver = _arm_driver_degrees(follower_bus)
        initial_joints = kinematics.driver_degrees_to_model_radians(
            initial_driver
        )
        initial_position = kinematics.forward_position(initial_joints)
        if args.record_errors_without_soft_stop:
            _append_record_json(
                args.record_output,
                {
                    "type": "start",
                    "initial_position_m": initial_position,
                    "requested_total_cm": args.delta_z_cm,
                },
            )
        segment_count = (
            1
            if args.single_cartesian_plan
            else math.ceil(args.delta_z_cm / MAX_SEGMENT_Z_CM)
        )
        print(
            "初始 XYZ(m): "
            f"{initial_position[0]:.6f}, {initial_position[1]:.6f}, "
            f"{initial_position[2]:.6f}",
            flush=True,
        )
        print(
            f"总目标 +Z {args.delta_z_cm:.2f} cm，"
            + (
                "使用一次笛卡尔规划。"
                if args.single_cartesian_plan
                else f"拆为 {segment_count} 段。"
            ),
            flush=True,
        )

        for segment_index in range(1, segment_count + 1):
            if args.single_cartesian_plan:
                checkpoint_cm = args.delta_z_cm
                segment_cm = args.delta_z_cm
            else:
                checkpoint_cm = min(
                    segment_index * MAX_SEGMENT_Z_CM,
                    args.delta_z_cm,
                )
                segment_cm = checkpoint_cm - (
                    (segment_index - 1) * MAX_SEGMENT_Z_CM
                )
            current_driver = _arm_driver_degrees(follower_bus)
            current_joints = kinematics.driver_degrees_to_model_radians(
                current_driver
            )
            current_position = kinematics.forward_position(current_joints)
            if args.record_errors_without_soft_stop:
                target_position = (
                    current_position[0],
                    current_position[1],
                    current_position[2] + segment_cm / 100.0,
                )
            else:
                target_position = (
                    initial_position[0],
                    initial_position[1],
                    initial_position[2] + checkpoint_cm / 100.0,
                )
            cartesian_distance = math.sqrt(
                sum(
                    (target - current) ** 2
                    for current, target in zip(
                        current_position,
                        target_position,
                        strict=True,
                    )
                )
            )
            if (
                not args.single_cartesian_plan
                and cartesian_distance > MAX_SEGMENT_CARTESIAN_DISTANCE_M
            ):
                raise RuntimeError(
                    f"第 {segment_index} 段需要移动 "
                    f"{cartesian_distance * 100:.3f} cm，超过 1.5 cm 限制。"
                )

            execution_adapter = SO100PlusAdapter(
                robot,
                gripper_config,
                on_telemetry=print_telemetry,
                kinematics=kinematics,
                motion_limits=_local_motion_limits(
                    current_position,
                    current_joints,
                    target_position,
                ),
                motion_config=SO100PlusMotionConfig(
                    joint_position_tolerance_degrees=(
                        OBSERVATION_JOINT_ERROR_LIMIT_DEGREES
                        if args.record_errors_without_soft_stop
                        else SO100_PLUS_REAL_HARDWARE_PROFILE.joint_position_tolerance_degrees
                    ),
                    cartesian_tolerance_m=(
                        OBSERVATION_CARTESIAN_ERROR_LIMIT_M
                        if args.record_errors_without_soft_stop
                        else SO100_PLUS_REAL_HARDWARE_PROFILE.cartesian_tolerance_m
                    ),
                    stream_frequency_hz=args.stream_frequency_hz,
                    stream_max_joint_speed_degrees_per_second=(
                        args.stream_max_joint_speed_degrees_per_second
                    ),
                ),
            )
            plan = execution_adapter.plan_move_to(*target_position)
            if args.single_cartesian_plan:
                (
                    max_internal_cartesian_step,
                    max_lateral_deviation,
                ) = _validate_single_cartesian_plan(
                    kinematics,
                    current_position,
                    plan.waypoints_radians,
                )
                print(
                    "单次规划路径检查通过：内部 TCP 最大步长 "
                    f"{max_internal_cartesian_step * 1000:.3f} mm，"
                    f"最大横向偏移 {max_lateral_deviation * 1000:.3f} mm。",
                    flush=True,
                )
            joint_deltas = tuple(
                target - current
                for current, target in zip(
                    plan.current_joint_radians,
                    plan.target_joint_radians,
                    strict=True,
                )
            )
            max_joint_delta = max(abs(value) for value in joint_deltas)
            print(
                f"第 {segment_index}/{segment_count} 段，目标累计 "
                f"+Z {checkpoint_cm:.2f} cm，当前到目标 "
                f"{cartesian_distance * 100:.3f} cm，最大关节变化 "
                f"{math.degrees(max_joint_delta):.3f}°，"
                f"内部轨迹点 {len(plan.waypoints_radians)} 个。",
                flush=True,
            )
            motion_error = None
            try:
                execution_adapter.move_to(*target_position)
            except Exception as error:
                motion_error = error
            finally:
                executed_plan = execution_adapter.last_motion_plan or plan
                recorded_step = _record_move_step(
                    index=segment_index,
                    requested_cumulative_cm=checkpoint_cm,
                    start_position_m=current_position,
                    target_position_m=target_position,
                    plan=executed_plan,
                    follower_bus=follower_bus,
                    kinematics=kinematics,
                )
                recorded_steps.append(recorded_step)
                if args.record_errors_without_soft_stop:
                    _append_record_json(
                        args.record_output,
                        {"type": "step", **asdict(recorded_step)},
                    )
            if motion_error is not None:
                raise motion_error
            print(f"第 {segment_index} 段通过。", flush=True)

        print(
            f"累计 +Z {args.delta_z_cm:.2f} cm 的 move_to 全部通过；"
            "保持目标位置 3 秒供观察。",
            flush=True,
        )
        time.sleep(OBSERVATION_SECONDS)
    except Exception as error:
        print(f"安全停止：{error}", flush=True)
        failure_message = str(error)
        exit_code = 2
    finally:
        _print_recorded_summary(recorded_steps, initial_position)
        if args.record_errors_without_soft_stop and initial_position is not None:
            final_position = (
                recorded_steps[-1].actual_position_m
                if recorded_steps
                else initial_position
            )
            _append_record_json(
                args.record_output,
                {
                    "type": "summary",
                    "completed_steps": len(recorded_steps),
                    "actual_rise_cm": (
                        final_position[2] - initial_position[2]
                    )
                    * 100.0,
                    "failure": failure_message,
                },
            )
            print(f"逐段记录已写入 {args.record_output}。", flush=True)
        active_adapter = execution_adapter or connection_adapter
        if getattr(robot, "is_connected", False):
            try:
                active_adapter.stop()
                active_adapter.disable_torque()
                print("已保持当前位置并关闭全部力矩。", flush=True)
            except Exception as error:
                print(
                    f"力矩关闭确认失败：{error}；请立即物理断电并托住机械臂。",
                    flush=True,
                )
                exit_code = 3
            finally:
                active_adapter.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print("串口已关闭。", flush=True)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
