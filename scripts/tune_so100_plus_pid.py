"""对 right_follower 做有限轮 PID 调参与一次残差补偿验证。

硬件模式固定最多执行五组 PID：

1. 当前 P，I=0，D=0
2. 当前 P，I=0，D=16
3. 当前 P，I=0，D=32
4. 当前 P，I=1，D=前三组中最好的 D
5. 当前 P，I=2，D=前三组中最好的 D

每组只走 JoyCon 初始姿态与 near_internal 之间的一个往返。若最佳
PID 的 TCP 误差仍大于 6 mm，脚本最多再执行一次、每关节不超过
2° 的实测关节残差补偿。它不会无限搜索，也不会自动改写源码中的
正式实机配置；完整结果写入 JSONL，确认后再人工保存最佳参数。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Sequence

from check_so100_plus_candidate_workspace import (
    DEFAULT_CANDIDATE_REPORT,
    MODEL_PATH,
    MAX_JOINT_STEP_RADIANS,
    PoseSnapshot,
    TelemetryRecorder,
    ValidationCheckpoint,
    _append_json_line,
    _execute_joint_checkpoint,
    _read_pose_snapshot,
    _record_settle_report,
    _transition_workspace_limits,
    build_validation_checkpoints,
    load_workspace_candidate,
    read_only_preflight,
    validate_collision_free_joint_path,
    validate_initial_pose,
    validate_storage_rest_start,
    validate_storage_to_initial_transition,
)
from rosclaw_mini.arm.kinematics import (
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus import (
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionConfig,
    SO100PlusPIDGains,
    SO100PlusSettleReport,
    SO100PlusTelemetry,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_robot,
)
from rosclaw_mini.safety.limits import (
    AxisLimits,
    JointLimits,
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
    WorkspaceLimits,
    build_so100_plus_right_follower_execution_joint_limits,
)


DEFAULT_RECORD_OUTPUT = Path("/tmp/rosclaw_so100_pid_tuning.jsonl")
PID_MOTOR_P = {
    "shoulder_rotation_joint": 16,
    "ellbow_joint": 64,
    "wrist_pitch_joint": 24,
}
PID_MOTOR_NAMES = tuple(PID_MOTOR_P)
MAX_PID_TRIALS = 5
MAX_RESIDUAL_CORRECTION_DEGREES = 2.0
MEASUREMENT_TCP_TOLERANCE_M = 0.015
ACCEPTED_TCP_ERROR_M = 0.006
ACCEPTED_TCP_JITTER_M = 0.002
PID_HOLD_SECONDS = 0.5
FIXED_AB_WORKSPACE_MARGIN_M = 0.015


@dataclass(frozen=True)
class PIDCandidate:
    name: str
    i: int
    d: int


@dataclass(frozen=True)
class PIDTrialResult:
    candidate: PIDCandidate
    tcp_error_m: float
    max_joint_span_degrees: float
    max_tcp_span_m: float
    max_load: float
    max_temperature_celsius: float
    nominal_target_driver_degrees: tuple[float, ...]
    actual_driver_degrees: tuple[float, ...]

    @property
    def score(self) -> float:
        """误差优先，稳定波动作为次级惩罚；数值单位近似为 mm。"""

        return (
            self.tcp_error_m * 1000
            + self.max_tcp_span_m * 1000
            + self.max_joint_span_degrees * 0.5
        )

    @property
    def accepted(self) -> bool:
        return (
            self.tcp_error_m <= ACCEPTED_TCP_ERROR_M
            and self.max_tcp_span_m <= ACCEPTED_TCP_JITTER_M
        )


class PIDTelemetryRecorder(TelemetryRecorder):
    def __init__(self, path: Path) -> None:
        super().__init__(path)
        self.history: list[SO100PlusTelemetry] = []

    def __call__(self, telemetry: SO100PlusTelemetry) -> None:
        self.history.append(telemetry)
        super().__call__(telemetry)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "最多五组 PID + 一次残差补偿，验证 near_internal 的 TCP 误差。"
        )
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--calibration-dir", required=True, type=Path)
    parser.add_argument("--follower-name", required=True)
    parser.add_argument(
        "--candidate-report",
        type=Path,
        default=DEFAULT_CANDIDATE_REPORT,
    )
    parser.add_argument(
        "--record-output",
        type=Path,
        default=DEFAULT_RECORD_OUTPUT,
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="只读真机姿态并做 MuJoCo 路径检查，不写 PID、不运动。",
    )
    parser.add_argument(
        "--final-ab-only",
        action="store_true",
        help=(
            "固定同一个六关节目标，只比较 I=2/D=16 和 I=2/D=32；"
            "最多两个往返"
        ),
    )
    parser.add_argument(
        "--acknowledge-bounded-pid-eprom-tuning-risk",
        action="store_true",
        help=(
            "确认最多五组 PID EEPROM 更新、五个验证往返、"
            "一次残差补偿往返和最终关闭力矩"
        ),
    )
    return parser


def derivative_candidates() -> tuple[PIDCandidate, ...]:
    return (
        PIDCandidate("baseline_i0_d0", i=0, d=0),
        PIDCandidate("damping_i0_d16", i=0, d=16),
        PIDCandidate("damping_i0_d32", i=0, d=32),
    )


def integral_candidates(best_d: int) -> tuple[PIDCandidate, ...]:
    if best_d not in {0, 16, 32}:
        raise ValueError("best_d 必须来自前三组候选。")
    return (
        PIDCandidate(f"integral_i1_d{best_d}", i=1, d=best_d),
        PIDCandidate(f"integral_i2_d{best_d}", i=2, d=best_d),
    )


def finalist_candidates() -> tuple[PIDCandidate, ...]:
    return (
        PIDCandidate("final_i2_d16", i=2, d=16),
        PIDCandidate("final_i2_d32", i=2, d=32),
    )


def gains_for_candidate(
    candidate: PIDCandidate,
) -> dict[str, SO100PlusPIDGains]:
    return {
        name: SO100PlusPIDGains(p=p, i=candidate.i, d=candidate.d)
        for name, p in PID_MOTOR_P.items()
    }


def select_best_trial(
    trials: Sequence[PIDTrialResult],
) -> PIDTrialResult:
    if not trials:
        raise ValueError("至少需要一个 PID 试验结果。")
    return min(
        trials,
        key=lambda trial: (
            trial.score,
            trial.candidate.i,
            trial.candidate.d,
        ),
    )


def build_fixed_validation_target(
    *,
    kinematics: SO100PlusKinematics,
    checkpoint: ValidationCheckpoint,
    joint_limits: JointLimits,
) -> tuple[float, ...]:
    """从理想 JoyCon 初始姿态求解一次，供所有最终候选共同使用。"""

    target = kinematics.solve_position(
        SO100_PLUS_JOYCON_INITIAL_RADIANS,
        checkpoint.position_m,
        joint_limits=joint_limits,
    )
    actual_position = kinematics.forward_position(target)
    error_m = math.dist(actual_position, checkpoint.position_m)
    if error_m > kinematics.position_tolerance_m:
        raise RuntimeError(
            f"固定 A/B 目标的 FK 复算误差 {error_m * 1000:.3f} mm "
            "超过运动学容差。"
        )
    return target


def build_fixed_ab_workspace(
    *,
    kinematics: SO100PlusKinematics,
    target_joint_radians: Sequence[float],
    margin_m: float = FIXED_AB_WORKSPACE_MARGIN_M,
) -> WorkspaceLimits:
    """只包住理想初始姿态到固定 A/B 目标的关节插值路径。"""

    if not math.isfinite(margin_m) or margin_m <= 0:
        raise ValueError("固定 A/B 工作空间余量必须是有限正数。")
    start = tuple(SO100_PLUS_JOYCON_INITIAL_RADIANS)
    target = tuple(float(value) for value in target_joint_radians)
    if len(target) != len(start):
        raise ValueError("固定 A/B 目标必须包含六个关节。")
    maximum_delta = max(
        abs(target_value - start_value)
        for start_value, target_value in zip(start, target, strict=True)
    )
    step_count = max(
        1,
        math.ceil(maximum_delta / math.radians(1.0)),
    )
    positions = tuple(
        kinematics.forward_position(
            tuple(
                start_value
                + (target_value - start_value) * step_index / step_count
                for start_value, target_value in zip(
                    start,
                    target,
                    strict=True,
                )
            )
        )
        for step_index in range(step_count + 1)
    )
    lower = tuple(
        min(position[axis] for position in positions) - margin_m
        for axis in range(3)
    )
    upper = tuple(
        max(position[axis] for position in positions) + margin_m
        for axis in range(3)
    )
    return WorkspaceLimits(
        x=AxisLimits(lower[0], upper[0]),
        y=AxisLimits(lower[1], upper[1]),
        z=AxisLimits(max(0.0, lower[2]), upper[2]),
    )


def calculate_residual_correction(
    nominal_target_driver_degrees: Sequence[float],
    actual_driver_degrees: Sequence[float],
    *,
    max_correction_degrees: float = MAX_RESIDUAL_CORRECTION_DEGREES,
) -> tuple[float, ...]:
    nominal = tuple(float(value) for value in nominal_target_driver_degrees)
    actual = tuple(float(value) for value in actual_driver_degrees)
    if len(nominal) != len(SO100_PLUS_ARM_JOINT_NAMES) or len(actual) != len(
        SO100_PLUS_ARM_JOINT_NAMES
    ):
        raise ValueError("残差补偿必须包含六个手臂关节。")
    if (
        not math.isfinite(max_correction_degrees)
        or max_correction_degrees <= 0
    ):
        raise ValueError("残差补偿上限必须是有限正数。")

    residual = tuple(
        actual_value - nominal_value
        for nominal_value, actual_value in zip(
            nominal,
            actual,
            strict=True,
        )
    )
    largest_index = max(range(len(residual)), key=lambda index: abs(residual[index]))
    if abs(residual[largest_index]) > max_correction_degrees:
        raise RuntimeError(
            f"关节 {SO100_PLUS_ARM_JOINT_NAMES[largest_index]} 的重复残差 "
            f"{residual[largest_index]:.3f}° 超过一次补偿上限 "
            f"{max_correction_degrees:.1f}°，拒绝补偿。"
        )
    return tuple(
        nominal_value - residual_value
        for nominal_value, residual_value in zip(
            nominal,
            residual,
            strict=True,
        )
    )


def _mean_arm_driver_degrees(
    report: SO100PlusSettleReport,
) -> tuple[float, ...]:
    if not report.position_samples_degrees:
        raise RuntimeError("稳定报告没有位置样本。")
    means_by_name = {
        name: sum(sample[index] for sample in report.position_samples_degrees)
        / len(report.position_samples_degrees)
        for index, name in enumerate(report.motor_names)
    }
    missing = tuple(
        name for name in SO100_PLUS_ARM_JOINT_NAMES if name not in means_by_name
    )
    if missing:
        raise RuntimeError(f"稳定报告缺少手臂关节：{missing}。")
    return tuple(means_by_name[name] for name in SO100_PLUS_ARM_JOINT_NAMES)


def _trial_telemetry_limits(
    telemetry: Sequence[SO100PlusTelemetry],
) -> tuple[float, float]:
    if not telemetry:
        raise RuntimeError("PID 试验没有产生遥测记录。")
    return (
        max(max(item.load_magnitude) for item in telemetry),
        max(max(item.temperature_raw) for item in telemetry),
    )


def _run_trial(
    *,
    candidate: PIDCandidate,
    adapter: SO100PlusAdapter,
    checkpoint: ValidationCheckpoint,
    initial_joint_radians: Sequence[float],
    fixed_target_joint_radians: Sequence[float] | None,
    kinematics: SO100PlusKinematics,
    recorder: PIDTelemetryRecorder,
    record_output: Path,
) -> PIDTrialResult:
    adapter.set_pid_gains(
        gains_for_candidate(candidate),
        acknowledge_eprom_write=True,
    )
    time.sleep(PID_HOLD_SECONDS)
    adapter.stop()

    if fixed_target_joint_radians is None:
        plan = adapter.plan_move_to(*checkpoint.position_m)
    else:
        plan = adapter.plan_joints(fixed_target_joint_radians)
    validate_collision_free_joint_path(
        plan.current_joint_radians,
        plan.target_joint_radians,
        kinematics,
        model_path=MODEL_PATH,
    )
    nominal_driver = kinematics.model_radians_to_driver_degrees(
        plan.target_joint_radians
    )
    telemetry_start = len(recorder.history)
    print(
        f"PID {candidate.name}: P={tuple(PID_MOTOR_P.values())}, "
        f"I={candidate.i}, D={candidate.d}。",
        flush=True,
    )
    completed = False
    try:
        if fixed_target_joint_radians is None:
            adapter.move_to(*checkpoint.position_m)
        else:
            adapter.move_joints(fixed_target_joint_radians)
        report = adapter.last_settle_report
        if report is None:
            raise RuntimeError("PID 试验结束后没有稳定报告。")
        _record_settle_report(
            path=record_output,
            checkpoint_name=f"pid_{candidate.name}",
            adapter=adapter,
        )
        actual_driver = _mean_arm_driver_degrees(report)
        actual_joint = kinematics.driver_degrees_to_model_radians(actual_driver)
        actual_tcp = kinematics.forward_position(actual_joint)
        tcp_error = math.dist(actual_tcp, checkpoint.position_m)
        max_joint_span = max(report.position_span_degrees)
        max_tcp_span = max(
            maximum - minimum
            for minimum, maximum in zip(
                report.tcp_min_m,
                report.tcp_max_m,
                strict=True,
            )
        )
        max_load, max_temperature = _trial_telemetry_limits(
            recorder.history[telemetry_start:]
        )
        result = PIDTrialResult(
            candidate=candidate,
            tcp_error_m=tcp_error,
            max_joint_span_degrees=max_joint_span,
            max_tcp_span_m=max_tcp_span,
            max_load=max_load,
            max_temperature_celsius=max_temperature,
            nominal_target_driver_degrees=tuple(nominal_driver),
            actual_driver_degrees=actual_driver,
        )
        _append_json_line(
            record_output,
            {
                "type": "pid_trial",
                "candidate": candidate.__dict__,
                "gains": {
                    name: gains.__dict__
                    for name, gains in gains_for_candidate(candidate).items()
                },
                "tcp_error_m": result.tcp_error_m,
                "max_joint_span_degrees": result.max_joint_span_degrees,
                "max_tcp_span_m": result.max_tcp_span_m,
                "max_load": result.max_load,
                "max_temperature_celsius": (
                    result.max_temperature_celsius
                ),
                "score": result.score,
                "accepted": result.accepted,
                "nominal_target_driver_degrees": nominal_driver,
                "actual_driver_degrees": actual_driver,
                "fixed_joint_target": (
                    fixed_target_joint_radians is not None
                ),
            },
        )
        print(
            f"  TCP 误差 {tcp_error * 1000:.3f} mm，"
            f"最大 TCP 波动 {max_tcp_span * 1000:.3f} mm，"
            f"最大关节波动 {max_joint_span:.3f}°，"
            f"负载 {max_load:.1f}，温度 {max_temperature:.0f}°C。",
            flush=True,
        )
        completed = True
        return result
    finally:
        if completed:
            adapter.move_joints(initial_joint_radians)


def _run_residual_correction(
    *,
    best: PIDTrialResult,
    adapter: SO100PlusAdapter,
    checkpoint: ValidationCheckpoint,
    kinematics: SO100PlusKinematics,
    record_output: Path,
) -> float:
    corrected_driver = calculate_residual_correction(
        best.nominal_target_driver_degrees,
        best.actual_driver_degrees,
    )
    corrected_joint = kinematics.driver_degrees_to_model_radians(
        corrected_driver
    )
    plan = adapter.plan_joints(corrected_joint)
    validate_collision_free_joint_path(
        plan.current_joint_radians,
        plan.target_joint_radians,
        kinematics,
        model_path=MODEL_PATH,
    )
    adapter.move_joints(corrected_joint)
    report = adapter.last_settle_report
    if report is None:
        raise RuntimeError("残差补偿后没有稳定报告。")
    actual_driver = _mean_arm_driver_degrees(report)
    actual_joint = kinematics.driver_degrees_to_model_radians(actual_driver)
    actual_tcp = kinematics.forward_position(actual_joint)
    error_m = math.dist(actual_tcp, checkpoint.position_m)
    _record_settle_report(
        path=record_output,
        checkpoint_name="residual_correction",
        adapter=adapter,
    )
    _append_json_line(
        record_output,
        {
            "type": "residual_correction",
            "source_candidate": best.candidate.__dict__,
            "corrected_target_driver_degrees": corrected_driver,
            "actual_driver_degrees": actual_driver,
            "target_tcp_m": checkpoint.position_m,
            "actual_tcp_m": actual_tcp,
            "tcp_error_m": error_m,
        },
    )
    if error_m >= best.tcp_error_m:
        raise RuntimeError(
            f"一次补偿未改善误差：补偿前 {best.tcp_error_m * 1000:.3f} mm，"
            f"补偿后 {error_m * 1000:.3f} mm；停止，不再迭代。"
        )
    if error_m > ACCEPTED_TCP_ERROR_M:
        raise RuntimeError(
            f"一次补偿后误差仍为 {error_m * 1000:.3f} mm，超过 "
            f"{ACCEPTED_TCP_ERROR_M * 1000:.1f} mm；停止，不再迭代。"
        )
    return error_m


def _record_preflight(
    path: Path,
    snapshot: PoseSnapshot,
) -> None:
    _append_json_line(
        path,
        {
            "type": "preflight",
            "driver_degrees": snapshot.driver_degrees,
            "joint_radians": snapshot.joint_radians,
            "tcp_position_m": snapshot.tcp_position_m,
            "torque_enabled": snapshot.torque_enabled,
        },
    )


def main() -> int:
    args = build_parser().parse_args()
    if args.follower_name != "right":
        raise SystemExit("已停止：自动 PID 脚本只适用于 follower_name=right。")
    if (
        not args.preflight_only
        and not args.acknowledge_bounded_pid_eprom_tuning_risk
    ):
        raise SystemExit(
            "已停止：必须显式确认有限轮 PID EEPROM 调参与真机运动风险。"
        )

    candidate_workspace = load_workspace_candidate(args.candidate_report)
    checkpoint = build_validation_checkpoints(candidate_workspace)[0]
    kinematics = SO100PlusKinematics()
    robot_config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )

    print("第一阶段只读预检：不会写 PID、力矩或目标位置。", flush=True)
    preflight = read_only_preflight(robot_config, kinematics)
    max_storage_error = validate_storage_rest_start(preflight)
    transition = validate_storage_to_initial_transition(
        preflight.joint_radians,
        kinematics,
    )
    print(
        f"预检通过：follower_rest 最大偏差 {max_storage_error:.3f}°；"
        f"展开路径 {transition.cartesian_path_length_m * 100:.2f} cm，"
        "MuJoCo 无新增接触。",
        flush=True,
    )
    if args.preflight_only:
        print("只读预检结束；没有写 PID，也没有运动。", flush=True)
        return 0

    args.record_output.parent.mkdir(parents=True, exist_ok=True)
    args.record_output.write_text("", encoding="utf-8")
    _record_preflight(args.record_output, preflight)

    joint_limits = build_so100_plus_right_follower_execution_joint_limits(
        preflight.joint_radians,
        max_step_radians=MAX_JOINT_STEP_RADIANS,
    )
    fixed_target_joint_radians = (
        build_fixed_validation_target(
            kinematics=kinematics,
            checkpoint=checkpoint,
            joint_limits=joint_limits,
        )
        if args.final_ab_only
        else None
    )
    if fixed_target_joint_radians is not None:
        fixed_path_validation = validate_collision_free_joint_path(
            SO100_PLUS_JOYCON_INITIAL_RADIANS,
            fixed_target_joint_radians,
            kinematics,
            model_path=MODEL_PATH,
        )
        print(
            "固定 A/B 目标离线路径通过："
            f"{fixed_path_validation.step_count} 个检查段，最低 Z="
            f"{fixed_path_validation.minimum_tcp_z_m:.6f} m。",
            flush=True,
        )
    transition_limits = MotionLimits(
        workspace=_transition_workspace_limits(transition),
        joints=joint_limits,
    )
    candidate_limits = MotionLimits(
        workspace=(
            build_fixed_ab_workspace(
                kinematics=kinematics,
                target_joint_radians=fixed_target_joint_radians,
            )
            if fixed_target_joint_radians is not None
            else candidate_workspace.workspace_limits()
        ),
        joints=joint_limits,
    )
    robot = create_so100_plus_robot(robot_config)
    follower_bus = robot.follower_arms[args.follower_name]
    recorder = PIDTelemetryRecorder(args.record_output)
    gripper_config = SO100PlusGripperConfig(
        follower_name=args.follower_name,
        open_degrees=60.0,
        close_degrees=-5.0,
        runtime_acceleration=(
            SO100_PLUS_REAL_HARDWARE_PROFILE.runtime_acceleration
        ),
    )
    transition_adapter = SO100PlusAdapter(
        robot,
        gripper_config,
        on_telemetry=recorder,
        kinematics=kinematics,
        motion_limits=transition_limits,
        motion_config=SO100PlusMotionConfig(
            joint_position_tolerance_degrees=3.0,
            cartesian_tolerance_m=0.012,
        ),
    )
    tuning_adapter = SO100PlusAdapter(
        robot,
        gripper_config,
        on_telemetry=recorder,
        kinematics=kinematics,
        motion_limits=candidate_limits,
        motion_config=SO100PlusMotionConfig(
            cartesian_tolerance_m=MEASUREMENT_TCP_TOLERANCE_M,
        ),
    )
    baseline_gains: dict[str, SO100PlusPIDGains] | None = None
    trials: list[PIDTrialResult] = []
    exit_code = 0

    try:
        print(
            "第二阶段启用力矩并展开；夹爪不动作。"
            + (
                "固定关节目标，只比较两个最终 PID。"
                if args.final_ab_only
                else "PID 最多五组。"
            ),
            flush=True,
        )
        transition_adapter.connect()
        baseline_gains = transition_adapter.read_pid_gains(PID_MOTOR_NAMES)
        connected = _read_pose_snapshot(follower_bus, kinematics)
        validate_storage_rest_start(connected, require_torque_disabled=False)
        _execute_joint_checkpoint(
            adapter=transition_adapter,
            name="storage_escape",
            target_joint_radians=transition.escape_joint_radians,
            path=args.record_output,
            follower_bus=follower_bus,
            kinematics=kinematics,
        )
        _execute_joint_checkpoint(
            adapter=transition_adapter,
            name="joycon_initial",
            target_joint_radians=transition.target_joint_radians,
            path=args.record_output,
            follower_bus=follower_bus,
            kinematics=kinematics,
        )
        validate_initial_pose(
            _read_pose_snapshot(follower_bus, kinematics),
            candidate_workspace,
        )

        if args.final_ab_only:
            for pid_candidate in finalist_candidates():
                result = _run_trial(
                    candidate=pid_candidate,
                    adapter=tuning_adapter,
                    checkpoint=checkpoint,
                    initial_joint_radians=SO100_PLUS_JOYCON_INITIAL_RADIANS,
                    fixed_target_joint_radians=(
                        fixed_target_joint_radians
                    ),
                    kinematics=kinematics,
                    recorder=recorder,
                    record_output=args.record_output,
                )
                trials.append(result)
        else:
            for pid_candidate in derivative_candidates():
                result = _run_trial(
                    candidate=pid_candidate,
                    adapter=tuning_adapter,
                    checkpoint=checkpoint,
                    initial_joint_radians=SO100_PLUS_JOYCON_INITIAL_RADIANS,
                    fixed_target_joint_radians=None,
                    kinematics=kinematics,
                    recorder=recorder,
                    record_output=args.record_output,
                )
                trials.append(result)
                if result.accepted:
                    break

            if not trials[-1].accepted:
                best_d = select_best_trial(trials).candidate.d
                for pid_candidate in integral_candidates(best_d):
                    if len(trials) >= MAX_PID_TRIALS:
                        break
                    result = _run_trial(
                        candidate=pid_candidate,
                        adapter=tuning_adapter,
                        checkpoint=checkpoint,
                        initial_joint_radians=(
                            SO100_PLUS_JOYCON_INITIAL_RADIANS
                        ),
                        fixed_target_joint_radians=None,
                        kinematics=kinematics,
                        recorder=recorder,
                        record_output=args.record_output,
                    )
                    trials.append(result)
                    if result.accepted:
                        break

        best = select_best_trial(trials)
        tuning_adapter.set_pid_gains(
            gains_for_candidate(best.candidate),
            acknowledge_eprom_write=True,
        )
        correction_error = None
        if not best.accepted and not args.final_ab_only:
            correction_error = _run_residual_correction(
                best=best,
                adapter=tuning_adapter,
                checkpoint=checkpoint,
                kinematics=kinematics,
                record_output=args.record_output,
            )
            tuning_adapter.move_joints(SO100_PLUS_JOYCON_INITIAL_RADIANS)

        _append_json_line(
            args.record_output,
            {
                "type": "tuning_result",
                "trial_count": len(trials),
                "best_candidate": best.candidate.__dict__,
                "best_gains": {
                    name: gains.__dict__
                    for name, gains in gains_for_candidate(
                        best.candidate
                    ).items()
                },
                "best_tcp_error_m": best.tcp_error_m,
                "correction_tcp_error_m": correction_error,
                "fixed_joint_target": args.final_ab_only,
                "fixed_target_joint_radians": (
                    fixed_target_joint_radians
                ),
            },
        )
        print(
            f"选择 {best.candidate.name}：PID 原始误差 "
            f"{best.tcp_error_m * 1000:.3f} mm"
            + (
                f"，一次补偿后 {correction_error * 1000:.3f} mm。"
                if correction_error is not None
                else "，不需要残差补偿。"
            ),
            flush=True,
        )

        _execute_joint_checkpoint(
            adapter=transition_adapter,
            name="storage_escape_return",
            target_joint_radians=transition.escape_joint_radians,
            path=args.record_output,
            follower_bus=follower_bus,
            kinematics=kinematics,
        )
        _execute_joint_checkpoint(
            adapter=transition_adapter,
            name="follower_rest_return",
            target_joint_radians=preflight.joint_radians,
            path=args.record_output,
            follower_bus=follower_bus,
            kinematics=kinematics,
        )
    except Exception as error:
        print(f"安全停止：{error}", flush=True)
        _append_json_line(
            args.record_output,
            {"type": "failure", "message": str(error)},
        )
        exit_code = 2
        if (
            getattr(robot, "is_connected", False)
            and baseline_gains is not None
        ):
            try:
                tuning_adapter.set_pid_gains(
                    baseline_gains,
                    acknowledge_eprom_write=True,
                )
                print("已恢复本次调参前的 PID 基线。", flush=True)
            except Exception as rollback_error:
                print(f"PID 基线恢复失败：{rollback_error}", flush=True)
                exit_code = 3
    finally:
        if getattr(robot, "is_connected", False):
            try:
                transition_adapter.stop()
                transition_adapter.disable_torque()
                print("已保持当前位置并关闭全部力矩。", flush=True)
            except Exception as error:
                print(
                    f"关闭力矩确认失败：{error}；请托住机械臂并物理断电。",
                    flush=True,
                )
                exit_code = 3
            finally:
                transition_adapter.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print(f"串口已关闭；完整记录：{args.record_output}", flush=True)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
