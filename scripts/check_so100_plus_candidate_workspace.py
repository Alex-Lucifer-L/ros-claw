"""从校准收纳 Rest 展开并验证仿真候选工作空间代表点。

脚本先通过只读连接确认机械臂位于本机已测的 follower_rest 附近，
并在 MuJoCo 中复核展开路径。只有显式确认后，才会启用力矩并执行：

默认流程验证内部往返点；完整边界套件会在同一次运行中继续验证距离
候选框六个面和八个角各一个网格步长的代表点，再安全收纳。
边界续测模式只执行已有日志中尚未运行的八个边界点，并返回初始姿态。

候选框只来自离线仿真报告；本脚本的一次通过也不等于整个工作空间
已经获得真机安全认证。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Mapping, Sequence

import numpy as np

from rosclaw_mini.arm.kinematics import (
    JointMotionPlan,
    SO100_PLUS_JOYCON_INITIAL_RADIANS,
    SO100PlusKinematics,
)
from rosclaw_mini.arm.so100_plus import (
    SO100_PLUS_REAL_HARDWARE_PROFILE,
    SO100PlusAdapter,
    SO100PlusGripperConfig,
    SO100PlusMotionConvergenceError,
    SO100PlusMotionConfig,
    SO100PlusTelemetry,
)
from rosclaw_mini.arm.so100_plus_factory import (
    SO100PlusRobotConfig,
    create_so100_plus_readonly_robot,
    create_so100_plus_robot,
)
from rosclaw_mini.arm.so100_plus_session import (
    SO100_PLUS_STORAGE_ESCAPE_FRACTION,
    SO100_PLUS_WORK_INITIAL_MAX_JOINT_ERROR_DEGREES,
    SO100_PLUS_WORK_INITIAL_MAX_TCP_ERROR_M,
    SO100PlusPoseSnapshot as PoseSnapshot,
    build_so100_plus_storage_transition,
    read_so100_plus_pose_snapshot as _read_pose_snapshot,
    validate_storage_rest_start,
    validate_work_initial_pose,
)
from rosclaw_mini.arm.so100_plus_trajectory_validation import (
    SO100PlusMuJoCoTrajectoryValidator,
    StorageTransitionDirection,
)
from rosclaw_mini.safety.limits import (
    AxisLimits,
    MotionLimits,
    SO100_PLUS_ARM_JOINT_NAMES,
    WorkspaceLimits,
    build_so100_plus_right_follower_execution_joint_limits,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CANDIDATE_REPORT = (
    ROOT
    / "artifacts"
    / "so100_plus_rest_workspace"
    / "rest_workspace_report.json"
)
DEFAULT_RECORD_OUTPUT = Path(
    "/tmp/rosclaw_so100_candidate_workspace_check.jsonl"
)
EXPECTED_REPORT_CLASSIFICATION = (
    "joycon_initial_orientation_grid_candidate_not_real_hardware_certification"
)
MODEL_PATH = (
    ROOT
    / "lerobot-joycon_plus"
    / "lerobot"
    / "common"
    / "robot_devices"
    / "controllers"
    / "scene_plus.xml"
)
# 仅供不读硬件的历史验收报告重现使用；真实转换会话
# 必须传入当次 Present_Position 中的 gripper_joint，不能把它
# 当成真机当前姿态。
CERTIFIED_OFFLINE_GRIPPER_REFERENCE_DRIVER_DEGREES = math.degrees(-0.157)
# 2026-07-18 对 right_follower 的只读实测。它描述这台机械臂的
# README follower_rest 收纳姿态，不是通用 SO-100 Plus 出厂常量。
EXPECTED_STORAGE_REST_DRIVER_DEGREES = (
    SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_driver_degrees
)
STORAGE_REST_DRIVER_TOLERANCES_DEGREES = (
    SO100_PLUS_REAL_HARDWARE_PROFILE.storage_rest_tolerances_degrees
)
MAX_INITIAL_JOINT_ERROR_DEGREES = (
    SO100_PLUS_WORK_INITIAL_MAX_JOINT_ERROR_DEGREES
)
MAX_INITIAL_TCP_ERROR_M = SO100_PLUS_WORK_INITIAL_MAX_TCP_ERROR_M
MAX_JOINT_STEP_RADIANS = math.radians(2.0)
SIMULATION_STEP_RADIANS = math.radians(1.0)
ESCAPE_FRACTION = SO100_PLUS_STORAGE_ESCAPE_FRACTION
TRANSITION_WORKSPACE_MARGIN_M = 0.005
TRANSITION_FINAL_JOINT_TOLERANCE_DEGREES = 3.0
TRANSITION_FINAL_TCP_TOLERANCE_M = 0.012
CANDIDATE_FINAL_JOINT_TOLERANCE_DEGREES = 3.0
CANDIDATE_FINAL_TCP_TOLERANCE_M = 0.012
OBSERVATION_SECONDS = 1.0
VALIDATION_OFFSETS_M = (
    ("near_internal", (0.03, 0.0, 0.03)),
    ("middle_internal", (0.07, -0.01, 0.06)),
    ("near_internal_return", (0.03, 0.0, 0.03)),
    ("initial_return", (0.0, 0.0, 0.0)),
)
BOUNDARY_MARGIN_GRID_STEPS = 1
# 14 个边界代表点已经全部得到真机结果。索引 14 指向最后的
# boundary_initial_return，不再有未执行的边界代表点。
BOUNDARY_RESUME_START_INDEX = 14


@dataclass(frozen=True)
class WorkspaceCandidate:
    initial_position_m: tuple[float, float, float]
    lower_m: tuple[float, float, float]
    upper_m: tuple[float, float, float]
    grid_step_m: float

    def workspace_limits(self) -> WorkspaceLimits:
        return WorkspaceLimits(
            x=AxisLimits(self.lower_m[0], self.upper_m[0]),
            y=AxisLimits(self.lower_m[1], self.upper_m[1]),
            z=AxisLimits(self.lower_m[2], self.upper_m[2]),
        )


@dataclass(frozen=True)
class ValidationCheckpoint:
    name: str
    position_m: tuple[float, float, float]


@dataclass(frozen=True)
class TransitionValidation:
    escape_joint_radians: tuple[float, ...]
    target_joint_radians: tuple[float, ...]
    path_positions_m: tuple[tuple[float, float, float], ...]
    max_joint_change_degrees: float
    cartesian_distance_m: float
    cartesian_path_length_m: float
    initial_contact_pair_count: int
    last_contact_step: int


@dataclass(frozen=True)
class CollisionFreePathValidation:
    step_count: int
    max_cartesian_step_m: float
    minimum_tcp_z_m: float


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "从 README follower_rest 展开到 JoyCon 初始工作姿态，"
            "按所选模式验证进出通道或候选框内部点，"
            "最后回到 follower_rest。"
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
    validation_scope = parser.add_mutually_exclusive_group()
    validation_scope.add_argument(
        "--transition-only",
        action="store_true",
        help=(
            "只验证 follower_rest 与 JoyCon 初始姿态之间的进出通道；"
            "跳过已经通过的候选框内部点"
        ),
    )
    validation_scope.add_argument(
        "--boundary-suite",
        action="store_true",
        help=(
            "一次运行内部往返点，以及距候选框六面和八角各一个"
            "网格步长的全部边界代表点"
        ),
    )
    validation_scope.add_argument(
        "--boundary-resume",
        action="store_true",
        help=(
            "跳过已有日志结果，只连续验证尚未执行的边界代表点，"
            "最后返回JoyCon初始姿态；全部完成后拒绝重复执行"
        ),
    )
    parser.add_argument(
        "--continue-on-convergence-error",
        action="store_true",
        help=(
            "候选点轨迹安全完成但最终关节/TCP超差时记录失败并继续；"
            "不会忽略碰撞、越界、过载、过温、运动中跟踪或通信异常"
        ),
    )
    parser.add_argument(
        "--acknowledge-candidate-workspace-motion-risk",
        action="store_true",
        help=(
            "确认已托住机械臂、清空路径，并同意约 21 cm 的展开/收回、"
            "所选验证范围内的运动以及结束时关闭力矩"
        ),
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help=(
            "只读当前位置并在 MuJoCo 复核展开路径；"
            "不启用力矩、不执行运动"
        ),
    )
    return parser


def build_validation_checkpoints_for_mode(
    candidate: WorkspaceCandidate,
    *,
    transition_only: bool,
    boundary_suite: bool = False,
    boundary_resume: bool = False,
) -> tuple[ValidationCheckpoint, ...]:
    """按本次验证范围选择检查点。"""

    if transition_only:
        return ()
    if boundary_resume:
        boundary = build_boundary_validation_checkpoints(candidate)
        remaining = boundary[BOUNDARY_RESUME_START_INDEX:-1]
        if not remaining:
            return ()
        return remaining + boundary[-1:]
    internal = build_validation_checkpoints(candidate)
    if boundary_suite:
        return internal + build_boundary_validation_checkpoints(candidate)
    return internal


def can_continue_after_checkpoint_error(
    error: Exception,
    *,
    enabled: bool,
) -> bool:
    """只有显式启用时，才允许最终到位误差不终止后续候选点。"""

    return enabled and isinstance(error, SO100PlusMotionConvergenceError)


def _finite_tuple(
    values: Sequence[float],
    *,
    length: int,
    label: str,
) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length or any(not math.isfinite(value) for value in result):
        raise ValueError(f"{label} 必须包含 {length} 个有限数值。")
    return result


def load_workspace_candidate(path: Path) -> WorkspaceCandidate:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"无法读取候选工作空间报告：{path}") from error

    if payload.get("classification") != EXPECTED_REPORT_CLASSIFICATION:
        raise ValueError("候选报告类型不匹配，拒绝用于真机验证。")
    neighbor_check = payload.get("box_directed_neighbor_check", {})
    if (
        neighbor_check.get("all_valid") is not True
        or int(neighbor_check.get("directed_edge_count", 0)) <= 0
    ):
        raise ValueError("候选报告没有通过全部相邻网格路径检查。")

    try:
        initial = _finite_tuple(
            payload["inputs"]["rest_tcp_m"],
            length=3,
            label="JoyCon 初始工作姿态 TCP",
        )
        box = payload["largest_all_valid_grid_box_containing_rest"]
        lower = _finite_tuple(
            (box[axis]["minimum"] for axis in ("x", "y", "z")),
            length=3,
            label="候选框下限",
        )
        upper = _finite_tuple(
            (box[axis]["maximum"] for axis in ("x", "y", "z")),
            length=3,
            label="候选框上限",
        )
        grid_step = float(box["grid_step_m"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            "候选报告缺少初始工作姿态、候选框或网格步长。"
        ) from error

    if not math.isfinite(grid_step) or grid_step <= 0:
        raise ValueError("候选报告的网格步长必须是有限正数。")
    candidate = WorkspaceCandidate(initial, lower, upper, grid_step)
    candidate.workspace_limits().validate_position(*initial)
    return candidate


def build_validation_checkpoints(
    candidate: WorkspaceCandidate,
) -> tuple[ValidationCheckpoint, ...]:
    workspace = candidate.workspace_limits()
    checkpoints = []
    for name, offset in VALIDATION_OFFSETS_M:
        position = tuple(
            initial_value + offset_value
            for initial_value, offset_value in zip(
                candidate.initial_position_m,
                offset,
                strict=True,
            )
        )
        workspace.validate_position(*position)
        for offset_value in offset:
            grid_steps = offset_value / candidate.grid_step_m
            if not math.isclose(grid_steps, round(grid_steps), abs_tol=1e-9):
                raise ValueError(f"验证点 {name} 不在报告的网格上。")
        checkpoints.append(ValidationCheckpoint(name, position))
    return tuple(checkpoints)


def build_boundary_validation_checkpoints(
    candidate: WorkspaceCandidate,
) -> tuple[ValidationCheckpoint, ...]:
    """生成距六面和八角一个仿真网格步长的代表点。"""

    margin = candidate.grid_step_m * BOUNDARY_MARGIN_GRID_STEPS
    low = tuple(value + margin for value in candidate.lower_m)
    high = tuple(value - margin for value in candidate.upper_m)
    center = tuple(
        (lower + upper) / 2.0
        for lower, upper in zip(
            candidate.lower_m,
            candidate.upper_m,
            strict=True,
        )
    )
    if any(lower >= upper for lower, upper in zip(low, high, strict=True)):
        raise ValueError("候选框太小，无法生成内缩一个网格的边界代表点。")

    x_low, y_low, z_low = low
    x_high, y_high, z_high = high
    x_center, y_center, z_center = center
    points = (
        ("boundary_face_z_min", (x_center, y_center, z_low)),
        (
            "boundary_corner_x_min_y_min_z_min",
            (x_low, y_low, z_low),
        ),
        (
            "boundary_corner_x_min_y_max_z_min",
            (x_low, y_high, z_low),
        ),
        (
            "boundary_corner_x_max_y_max_z_min",
            (x_high, y_high, z_low),
        ),
        (
            "boundary_corner_x_max_y_min_z_min",
            (x_high, y_low, z_low),
        ),
        ("boundary_face_x_max", (x_high, y_center, z_center)),
        (
            "boundary_corner_x_max_y_min_z_max",
            (x_high, y_low, z_high),
        ),
        (
            "boundary_corner_x_max_y_max_z_max",
            (x_high, y_high, z_high),
        ),
        (
            "boundary_corner_x_min_y_max_z_max",
            (x_low, y_high, z_high),
        ),
        (
            "boundary_corner_x_min_y_min_z_max",
            (x_low, y_low, z_high),
        ),
        ("boundary_face_z_max", (x_center, y_center, z_high)),
        ("boundary_face_x_min", (x_low, y_center, z_center)),
        ("boundary_face_y_min", (x_center, y_low, z_center)),
        ("boundary_face_y_max", (x_center, y_high, z_center)),
        ("boundary_initial_return", candidate.initial_position_m),
    )

    workspace = candidate.workspace_limits()
    checkpoints = []
    for name, position in points:
        workspace.validate_position(*position)
        for axis, value in enumerate(position):
            grid_steps = (
                value - candidate.lower_m[axis]
            ) / candidate.grid_step_m
            if not math.isclose(grid_steps, round(grid_steps), abs_tol=1e-9):
                raise ValueError(f"边界验证点 {name} 不在报告的网格上。")
        checkpoints.append(ValidationCheckpoint(name, position))
    return tuple(checkpoints)


def validate_checkpoint_suite_offline(
    checkpoints: Sequence[ValidationCheckpoint],
    *,
    initial_joint_radians: Sequence[float],
    candidate: WorkspaceCandidate,
    kinematics: SO100PlusKinematics,
    joint_limits,
) -> tuple[int, int, float, float]:
    """依序求解全部代表点，并用 MuJoCo 检查每段关节路径。"""

    current = tuple(float(value) for value in initial_joint_radians)
    motion_limits = MotionLimits(
        workspace=candidate.workspace_limits(),
        joints=joint_limits,
    )
    total_path_steps = 0
    maximum_joint_change_degrees = 0.0
    minimum_tcp_z_m = kinematics.forward_position(current)[2]
    for checkpoint in checkpoints:
        plan = kinematics.plan_position(
            current,
            checkpoint.position_m,
            motion_limits,
        )
        path_validation = validate_collision_free_joint_path(
            plan.current_joint_radians,
            plan.target_joint_radians,
            kinematics,
        )
        total_path_steps += path_validation.step_count
        minimum_tcp_z_m = min(
            minimum_tcp_z_m,
            path_validation.minimum_tcp_z_m,
        )
        maximum_joint_change_degrees = max(
            maximum_joint_change_degrees,
            *(
                abs(math.degrees(target - start))
                for start, target in zip(
                    plan.current_joint_radians,
                    plan.target_joint_radians,
                    strict=True,
                )
            ),
        )
        current = plan.target_joint_radians
    return (
        len(checkpoints),
        total_path_steps,
        maximum_joint_change_degrees,
        minimum_tcp_z_m,
    )


def validate_initial_pose(
    snapshot: PoseSnapshot,
    candidate: WorkspaceCandidate,
    *,
    max_joint_error_degrees: float = MAX_INITIAL_JOINT_ERROR_DEGREES,
    max_tcp_error_m: float = MAX_INITIAL_TCP_ERROR_M,
) -> tuple[float, float]:
    return validate_work_initial_pose(
        snapshot,
        candidate.initial_position_m,
        max_joint_error_degrees=max_joint_error_degrees,
        max_tcp_error_m=max_tcp_error_m,
    )


def read_only_preflight(
    robot_config: SO100PlusRobotConfig,
    kinematics: SO100PlusKinematics,
) -> PoseSnapshot:
    robot = create_so100_plus_readonly_robot(robot_config)
    follower_bus = robot.follower_arms[robot_config.follower_name]
    try:
        robot.connect()
        return _read_pose_snapshot(follower_bus, kinematics)
    finally:
        if robot.is_connected:
            robot.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()


def _linear_joint_plan(
    start_joint_radians: Sequence[float],
    target_joint_radians: Sequence[float],
    kinematics: SO100PlusKinematics,
    *,
    gripper_driver_degrees: float = (
        CERTIFIED_OFFLINE_GRIPPER_REFERENCE_DRIVER_DEGREES
    ),
) -> JointMotionPlan:
    """构造验收脚本原有的 1° 直线关节计划。"""

    start = _finite_tuple(
        start_joint_radians,
        length=len(SO100_PLUS_ARM_JOINT_NAMES),
        label="路径起点关节角",
    )
    target = _finite_tuple(
        target_joint_radians,
        length=len(SO100_PLUS_ARM_JOINT_NAMES),
        label="路径终点关节角",
    )
    delta = tuple(
        end - begin
        for begin, end in zip(start, target, strict=True)
    )
    step_count = max(
        1,
        math.ceil(
            max(abs(value) for value in delta)
            / SIMULATION_STEP_RADIANS
        ),
    )
    waypoints = tuple(
        tuple(
            begin + change * (step_index / step_count)
            for begin, change in zip(start, delta, strict=True)
        )
        for step_index in range(1, step_count + 1)
    )
    return JointMotionPlan(
        target_position_m=kinematics.forward_position(target),
        current_joint_radians=start,
        target_joint_radians=target,
        waypoints_radians=waypoints,
        is_final_execution_plan=True,
        held_gripper_driver_degrees=float(gripper_driver_degrees),
    )


def validate_storage_to_initial_transition(
    start_joint_radians: Sequence[float],
    kinematics: SO100PlusKinematics,
    *,
    model_path: Path = MODEL_PATH,
    gripper_driver_degrees: float = (
        CERTIFIED_OFFLINE_GRIPPER_REFERENCE_DRIVER_DEGREES
    ),
) -> TransitionValidation:
    """确认收纳姿态只脱离已有贴靠、TCP 单调上升且最终无接触。"""

    shared_transition = build_so100_plus_storage_transition(
        start_joint_radians,
        kinematics,
    )
    plans = (
        _linear_joint_plan(
            shared_transition.storage_joint_radians,
            shared_transition.escape_joint_radians,
            kinematics,
            gripper_driver_degrees=gripper_driver_degrees,
        ),
        _linear_joint_plan(
            shared_transition.escape_joint_radians,
            shared_transition.work_joint_radians,
            kinematics,
            gripper_driver_degrees=gripper_driver_degrees,
        ),
    )
    validator = SO100PlusMuJoCoTrajectoryValidator(
        model_path=model_path
    )
    verified = validator.verify_storage_transition(
        plans,
        escape_joint_radians=(
            shared_transition.escape_joint_radians
        ),
        kinematics=kinematics,
        direction=StorageTransitionDirection.UNFOLD,
        gripper_qpos=validator.gripper_driver_degrees_to_qpos(
            gripper_driver_degrees
        ),
    )
    path = np.asarray(verified.sampled_joint_radians, dtype=float)
    positions = np.asarray(
        [kinematics.forward_position(joints) for joints in path],
        dtype=float,
    )
    start = path[0]
    target = path[-1]
    max_joint_change = float(np.max(np.abs(target - start)))
    segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    return TransitionValidation(
        escape_joint_radians=shared_transition.escape_joint_radians,
        target_joint_radians=tuple(float(value) for value in target),
        path_positions_m=tuple(
            tuple(float(value) for value in position)
            for position in positions
        ),
        max_joint_change_degrees=math.degrees(max_joint_change),
        cartesian_distance_m=float(
            np.linalg.norm(positions[-1] - positions[0])
        ),
        cartesian_path_length_m=float(np.sum(segment_lengths)),
        initial_contact_pair_count=len(
            verified.report.initial_contact_pairs
        ),
        last_contact_step=verified.report.last_contact_sample,
    )


def _transition_workspace_limits(
    transition: TransitionValidation,
) -> WorkspaceLimits:
    positions = np.asarray(transition.path_positions_m, dtype=float)
    lower = np.min(positions, axis=0) - TRANSITION_WORKSPACE_MARGIN_M
    upper = np.max(positions, axis=0) + TRANSITION_WORKSPACE_MARGIN_M
    return WorkspaceLimits(
        x=AxisLimits(float(lower[0]), float(upper[0])),
        y=AxisLimits(float(lower[1]), float(upper[1])),
        z=AxisLimits(float(lower[2]), float(upper[2])),
    )


def validate_collision_free_joint_path(
    start_joint_radians: Sequence[float],
    target_joint_radians: Sequence[float],
    kinematics: SO100PlusKinematics,
    *,
    model_path: Path = MODEL_PATH,
    gripper_driver_degrees: float = (
        CERTIFIED_OFFLINE_GRIPPER_REFERENCE_DRIVER_DEGREES
    ),
) -> CollisionFreePathValidation:
    """按当次实测起点检查无接触、且 TCP 不低于 Z=0 的关节路径。"""

    plan = _linear_joint_plan(
        start_joint_radians,
        target_joint_radians,
        kinematics,
        gripper_driver_degrees=gripper_driver_degrees,
    )
    validator = SO100PlusMuJoCoTrajectoryValidator(
        model_path=model_path
    )
    verified = validator.verify_collision_free_sequence(
        (plan,),
        kinematics,
        gripper_qpos=validator.gripper_driver_degrees_to_qpos(
            gripper_driver_degrees
        ),
    )
    path = np.asarray(verified.sampled_joint_radians, dtype=float)
    positions = np.asarray(
        [kinematics.forward_position(joints) for joints in path],
        dtype=float,
    )
    minimum_z = float(np.min(positions[:, 2]))

    cartesian_steps = np.linalg.norm(
        np.diff(positions, axis=0),
        axis=1,
    )
    return CollisionFreePathValidation(
        step_count=max(len(path) - 1, 0),
        max_cartesian_step_m=float(np.max(cartesian_steps)),
        minimum_tcp_z_m=minimum_z,
    )


class TelemetryRecorder:
    def __init__(self, path: Path) -> None:
        self.path = path

    def __call__(self, telemetry: SO100PlusTelemetry) -> None:
        payload = {
            "type": "telemetry",
            "monotonic_seconds": time.monotonic(),
            "phase": telemetry.phase,
            "motor_names": telemetry.motor_names,
            "voltage_raw": telemetry.voltage_raw,
            "current_raw": telemetry.current_raw,
            "load_magnitude": telemetry.load_magnitude,
            "temperature_raw": telemetry.temperature_raw,
        }
        _append_json_line(self.path, payload)
        print(
            f"遥测 {telemetry.phase}: "
            f"电压={tuple(round(value, 1) for value in telemetry.voltage_raw)}，"
            f"最大电流={max(telemetry.current_raw):.1f}，"
            f"最大负载={max(telemetry.load_magnitude):.1f}，"
            f"最高温度={max(telemetry.temperature_raw):.0f}°C",
            flush=True,
        )


def _append_json_line(path: Path, payload: Mapping[str, object]) -> None:
    with path.open("a", encoding="utf-8") as output_file:
        output_file.write(json.dumps(payload, ensure_ascii=False) + "\n")


def _record_checkpoint(
    *,
    path: Path,
    checkpoint: ValidationCheckpoint,
    follower_bus,
    kinematics: SO100PlusKinematics,
) -> float:
    driver_degrees = _arm_driver_degrees(follower_bus)
    joint_radians = kinematics.driver_degrees_to_model_radians(driver_degrees)
    actual_position = kinematics.forward_position(joint_radians)
    error_m = math.dist(actual_position, checkpoint.position_m)
    _append_json_line(
        path,
        {
            "type": "checkpoint",
            "name": checkpoint.name,
            "target_position_m": checkpoint.position_m,
            "actual_position_m": actual_position,
            "cartesian_error_m": error_m,
            "driver_degrees": driver_degrees,
        },
    )
    return error_m


def _record_settle_report(
    *,
    path: Path,
    checkpoint_name: str,
    adapter: SO100PlusAdapter,
) -> None:
    report = adapter.last_settle_report
    if report is None:
        return
    _append_json_line(
        path,
        {
            "type": "settle_summary",
            "checkpoint": checkpoint_name,
            "duration_seconds": report.duration_seconds,
            "motor_names": report.motor_names,
            "position_samples_degrees": (
                report.position_samples_degrees
            ),
            "position_span_degrees": report.position_span_degrees,
            "tcp_samples_m": report.tcp_samples_m,
            "tcp_min_m": report.tcp_min_m,
            "tcp_max_m": report.tcp_max_m,
            "tcp_mean_m": report.tcp_mean_m,
        },
    )
    max_span_index = max(
        range(len(report.position_span_degrees)),
        key=report.position_span_degrees.__getitem__,
    )
    tcp_span_mm = tuple(
        (maximum - minimum) * 1000
        for minimum, maximum in zip(
            report.tcp_min_m,
            report.tcp_max_m,
            strict=True,
        )
    )
    print(
        f"  稳定观察 {report.duration_seconds:.2f} 秒："
        f"最大关节波动 {report.motor_names[max_span_index]}="
        f"{report.position_span_degrees[max_span_index]:.3f}°；"
        f"TCP XYZ 波动范围=({tcp_span_mm[0]:.3f}, "
        f"{tcp_span_mm[1]:.3f}, {tcp_span_mm[2]:.3f}) mm，"
        f"平均位置=({report.tcp_mean_m[0]:.6f}, "
        f"{report.tcp_mean_m[1]:.6f}, "
        f"{report.tcp_mean_m[2]:.6f}) m。",
        flush=True,
    )


def _execute_joint_checkpoint(
    *,
    adapter: SO100PlusAdapter,
    name: str,
    target_joint_radians: Sequence[float],
    path: Path,
    follower_bus,
    kinematics: SO100PlusKinematics,
) -> None:
    target = tuple(float(value) for value in target_joint_radians)
    target_position = kinematics.forward_position(target)
    plan = adapter.plan_joints(target)
    max_joint_change = max(
        abs(math.degrees(target_value - current_value))
        for current_value, target_value in zip(
            plan.current_joint_radians,
            plan.target_joint_radians,
            strict=True,
        )
    )
    print(
        f"{name}: 目标 TCP XYZ=({target_position[0]:.6f}, "
        f"{target_position[1]:.6f}, {target_position[2]:.6f}) m，"
        f"最大关节变化 {max_joint_change:.3f}°。",
        flush=True,
    )
    motion_error: Exception | None = None
    try:
        adapter.move_joints(target)
    except Exception as error:
        motion_error = error
    finally:
        _record_settle_report(
            path=path,
            checkpoint_name=name,
            adapter=adapter,
        )
        error_m = _record_checkpoint(
            path=path,
            checkpoint=ValidationCheckpoint(name, target_position),
            follower_bus=follower_bus,
            kinematics=kinematics,
        )
    print(
        f"  到位，实测 TCP 误差 {error_m * 1000:.3f} mm。",
        flush=True,
    )
    if motion_error is not None:
        raise motion_error


def _return_to_storage_rest(
    *,
    candidate_adapter: SO100PlusAdapter,
    transition_adapter: SO100PlusAdapter,
    candidate: WorkspaceCandidate,
    transition: TransitionValidation,
    storage_joint_radians: Sequence[float],
    path: Path,
    follower_bus,
    kinematics: SO100PlusKinematics,
    return_from_candidate: bool,
) -> float:
    """沿已检查路径收纳，并在返回后验证真实 follower_rest。"""

    if return_from_candidate:
        checkpoint = ValidationCheckpoint(
            "initial_recovery",
            candidate.initial_position_m,
        )
        plan = candidate_adapter.plan_move_to(*checkpoint.position_m)
        validate_collision_free_joint_path(
            plan.current_joint_radians,
            plan.target_joint_radians,
            kinematics,
        )
        candidate_adapter.move_to(*checkpoint.position_m)
        _record_settle_report(
            path=path,
            checkpoint_name=checkpoint.name,
            adapter=candidate_adapter,
        )
        _record_checkpoint(
            path=path,
            checkpoint=checkpoint,
            follower_bus=follower_bus,
            kinematics=kinematics,
        )

    _execute_joint_checkpoint(
        adapter=transition_adapter,
        name="storage_escape_return",
        target_joint_radians=transition.escape_joint_radians,
        path=path,
        follower_bus=follower_bus,
        kinematics=kinematics,
    )
    _execute_joint_checkpoint(
        adapter=transition_adapter,
        name="follower_rest_return",
        target_joint_radians=storage_joint_radians,
        path=path,
        follower_bus=follower_bus,
        kinematics=kinematics,
    )
    final_snapshot = _read_pose_snapshot(follower_bus, kinematics)
    return validate_storage_rest_start(
        final_snapshot,
        require_torque_disabled=False,
    )


def main() -> int:
    args = build_parser().parse_args()
    if (
        not args.preflight_only
        and not args.acknowledge_candidate_workspace_motion_risk
    ):
        raise SystemExit("已停止：必须显式确认候选工作空间真机运动风险。")
    if args.follower_name != "right":
        raise SystemExit("已停止：本次候选框只适用于 follower_name=right。")

    candidate = load_workspace_candidate(args.candidate_report)
    checkpoints = build_validation_checkpoints_for_mode(
        candidate,
        transition_only=args.transition_only,
        boundary_suite=args.boundary_suite,
        boundary_resume=args.boundary_resume,
    )
    if args.boundary_resume and not checkpoints:
        raise SystemExit(
            "边界续测已完成：14 个边界代表点均已有真机结果；"
            "未访问串口、未启用力矩。"
        )
    kinematics = SO100PlusKinematics()
    robot_config = SO100PlusRobotConfig(
        port=args.port,
        calibration_dir=args.calibration_dir,
        follower_name=args.follower_name,
    )

    print("第一阶段只读预检：不会写力矩、加速度或目标位置。", flush=True)
    preflight = read_only_preflight(robot_config, kinematics)
    print(
        "只读实测驱动角(°): "
        + ", ".join(
            f"{name}={value:.3f}"
            for name, value in zip(
                SO100_PLUS_ARM_JOINT_NAMES,
                preflight.driver_degrees,
                strict=True,
            )
        ),
        flush=True,
    )
    print(
        "只读实测 TCP XYZ(m): "
        f"{preflight.tcp_position_m[0]:.6f}, "
        f"{preflight.tcp_position_m[1]:.6f}, "
        f"{preflight.tcp_position_m[2]:.6f}",
        flush=True,
    )
    max_storage_error = validate_storage_rest_start(preflight)
    transition = validate_storage_to_initial_transition(
        preflight.joint_radians,
        kinematics,
    )
    joint_limits = build_so100_plus_right_follower_execution_joint_limits(
        preflight.joint_radians,
        max_step_radians=MAX_JOINT_STEP_RADIANS,
    )
    (
        suite_checkpoint_count,
        suite_path_step_count,
        suite_max_joint_change_degrees,
        suite_minimum_tcp_z_m,
    ) = validate_checkpoint_suite_offline(
        checkpoints,
        initial_joint_radians=transition.target_joint_radians,
        candidate=candidate,
        kinematics=kinematics,
        joint_limits=joint_limits,
    )
    print(
        "只读与 MuJoCo 预检通过：最大 follower_rest 偏差 "
        f"{max_storage_error:.3f}°，展开 TCP 直线位移 "
        f"{transition.cartesian_distance_m * 100:.3f} cm，"
        f"路径长度 {transition.cartesian_path_length_m * 100:.3f} cm，"
        f"最大关节变化 {transition.max_joint_change_degrees:.3f}°；"
        f"已有 {transition.initial_contact_pair_count} 组收纳贴靠，"
        "路径无新增接触并最终全部脱离。",
        flush=True,
    )
    if suite_checkpoint_count:
        print(
            f"候选点离线预检通过：{suite_checkpoint_count} 个检查点，"
            f"{suite_path_step_count} 个 1° MuJoCo 路径段，"
            f"单段最大关节变化 {suite_max_joint_change_degrees:.3f}°，"
            f"全程最低 TCP Z={suite_minimum_tcp_z_m:.6f} m。",
            flush=True,
        )
    if args.preflight_only:
        print("只读预检模式结束；未启用力矩、未发送目标位置。", flush=True)
        return 0

    args.record_output.parent.mkdir(parents=True, exist_ok=True)
    args.record_output.write_text("", encoding="utf-8")
    _append_json_line(
        args.record_output,
        {
            "type": "preflight",
            "validation_mode": (
                "transition_only"
                if args.transition_only
                else (
                    "boundary_suite"
                    if args.boundary_suite
                    else (
                        "boundary_resume"
                        if args.boundary_resume
                        else "candidate_internal"
                    )
                )
            ),
            "driver_degrees": preflight.driver_degrees,
            "joint_radians": preflight.joint_radians,
            "tcp_position_m": preflight.tcp_position_m,
            "torque_enabled": preflight.torque_enabled,
            "transition": {
                "escape_joint_radians": transition.escape_joint_radians,
                "target_joint_radians": transition.target_joint_radians,
                "max_joint_change_degrees": (
                    transition.max_joint_change_degrees
                ),
                "cartesian_distance_m": transition.cartesian_distance_m,
                "cartesian_path_length_m": (
                    transition.cartesian_path_length_m
                ),
                "initial_contact_pair_count": (
                    transition.initial_contact_pair_count
                ),
                "last_contact_step": transition.last_contact_step,
            },
        },
    )

    transition_motion_limits = MotionLimits(
        workspace=_transition_workspace_limits(transition),
        joints=joint_limits,
    )
    candidate_motion_limits = MotionLimits(
        workspace=candidate.workspace_limits(),
        joints=joint_limits,
    )
    robot = create_so100_plus_robot(robot_config)
    follower_bus = robot.follower_arms[args.follower_name]
    recorder = TelemetryRecorder(args.record_output)
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
        motion_limits=transition_motion_limits,
        motion_config=SO100PlusMotionConfig(
            joint_position_tolerance_degrees=(
                TRANSITION_FINAL_JOINT_TOLERANCE_DEGREES
            ),
            cartesian_tolerance_m=TRANSITION_FINAL_TCP_TOLERANCE_M,
        ),
    )
    candidate_adapter = SO100PlusAdapter(
        robot,
        gripper_config,
        on_telemetry=recorder,
        kinematics=kinematics,
        motion_limits=candidate_motion_limits,
        motion_config=SO100PlusMotionConfig(
            # 当前教学版存在方向相关的回差；内部点往返曾实测
            # 2.321° / 10.035 mm。工作空间验收使用 3° / 12 mm，
            # 后续已由用户确认保存为正式运行默认值；脚本仍记录
            # 每一步的精确误差。
            joint_position_tolerance_degrees=(
                CANDIDATE_FINAL_JOINT_TOLERANCE_DEGREES
            ),
            cartesian_tolerance_m=CANDIDATE_FINAL_TCP_TOLERANCE_M,
        ),
    )
    exit_code = 0
    recovery_stage = "before_connect"
    at_storage_rest = False
    convergence_failures: list[dict[str, object]] = []

    try:
        print(
            "第二阶段将启用力矩；夹爪不动作，先展开到初始工作姿态。",
            flush=True,
        )
        transition_adapter.connect()
        recovery_stage = "transition"
        connected_snapshot = _read_pose_snapshot(follower_bus, kinematics)
        validate_storage_rest_start(
            connected_snapshot,
            require_torque_disabled=False,
        )
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
        initial_snapshot = _read_pose_snapshot(follower_bus, kinematics)
        validate_initial_pose(initial_snapshot, candidate)
        if args.transition_only:
            print(
                "已到达 JoyCon 初始工作姿态；进出通道模式跳过"
                "候选框内部点，现在沿已检查路径返回 follower_rest。",
                flush=True,
            )
        else:
            recovery_stage = "candidate"
            print(
                (
                    "已到达 JoyCon 初始工作姿态，开始一次性验证"
                    "内部点、六面和八角代表点。"
                    if args.boundary_suite
                    else (
                        "已到达 JoyCon 初始工作姿态，开始连续验证"
                        f"剩余{max(len(checkpoints) - 1, 0)}个"
                        "边界代表点。"
                        if args.boundary_resume
                        else "已到达 JoyCon 初始工作姿态，"
                        "开始验证候选框内部点。"
                    )
                ),
                flush=True,
            )

        for index, checkpoint in enumerate(checkpoints, start=1):
            plan = candidate_adapter.plan_move_to(*checkpoint.position_m)
            path_validation = validate_collision_free_joint_path(
                plan.current_joint_radians,
                plan.target_joint_radians,
                kinematics,
            )
            max_joint_change = max(
                abs(math.degrees(target - current))
                for current, target in zip(
                    plan.current_joint_radians,
                    plan.target_joint_radians,
                    strict=True,
                )
            )
            print(
                f"{index}/{len(checkpoints)} {checkpoint.name}: "
                f"目标 XYZ=({checkpoint.position_m[0]:.6f}, "
                f"{checkpoint.position_m[1]:.6f}, "
                f"{checkpoint.position_m[2]:.6f}) m，"
                f"最大关节变化 {max_joint_change:.3f}°；"
                f"当次实测路径 {path_validation.step_count} 个 1°检查段，"
                f"最低 Z={path_validation.minimum_tcp_z_m:.6f} m。",
                flush=True,
            )
            motion_error: Exception | None = None
            try:
                candidate_adapter.move_to(*checkpoint.position_m)
            except Exception as error:
                motion_error = error
            finally:
                _record_settle_report(
                    path=args.record_output,
                    checkpoint_name=checkpoint.name,
                    adapter=candidate_adapter,
                )
                error_m = _record_checkpoint(
                    path=args.record_output,
                    checkpoint=checkpoint,
                    follower_bus=follower_bus,
                    kinematics=kinematics,
                )
            print(
                f"  到位，实测 TCP 误差 {error_m * 1000:.3f} mm。",
                flush=True,
            )
            if motion_error is not None:
                if not can_continue_after_checkpoint_error(
                    motion_error,
                    enabled=args.continue_on_convergence_error,
                ):
                    raise motion_error
                failure = {
                    "checkpoint": checkpoint.name,
                    "cartesian_error_m": error_m,
                    "message": str(motion_error),
                }
                convergence_failures.append(failure)
                _append_json_line(
                    args.record_output,
                    {
                        "type": "checkpoint_convergence_failure",
                        **failure,
                    },
                )
                print(
                    "  最终到位误差已记录；运动中安全检查均未触发，"
                    "从当前实测位置重新规划下一个点。",
                    flush=True,
                )

        if not args.transition_only:
            if convergence_failures:
                print(
                    f"全部检查点已执行；{len(convergence_failures)}个"
                    "最终到位误差已记录，现在开始收回 follower_rest。",
                    flush=True,
                )
                _append_json_line(
                    args.record_output,
                    {
                        "type": "checkpoint_suite_summary",
                        "checkpoint_count": len(checkpoints),
                        "convergence_failure_count": len(
                            convergence_failures
                        ),
                        "convergence_failures": convergence_failures,
                    },
                )
                exit_code = 2
            else:
                print(
                    (
                        "内部点和全部边界代表点通过，开始收回 "
                        "follower_rest。"
                        if args.boundary_suite
                        else (
                            f"剩余{max(len(checkpoints) - 1, 0)}个"
                            "边界代表点通过，开始收回 follower_rest。"
                            if args.boundary_resume
                            else "内部点通过，开始收回 follower_rest。"
                        )
                    ),
                    flush=True,
                )
        recovery_stage = "transition"
        final_storage_error = _return_to_storage_rest(
            candidate_adapter=candidate_adapter,
            transition_adapter=transition_adapter,
            candidate=candidate,
            transition=transition,
            storage_joint_radians=preflight.joint_radians,
            path=args.record_output,
            follower_bus=follower_bus,
            kinematics=kinematics,
            return_from_candidate=False,
        )
        at_storage_rest = True
        recovery_stage = "at_rest"
        print(
            (
                "进出通道验证通过并回到 follower_rest；"
                if args.transition_only
                else (
                    "全部检查点执行完成并回到 follower_rest；"
                    if convergence_failures
                    else (
                        "完整边界套件通过并回到 follower_rest；"
                        if args.boundary_suite
                        else (
                            "剩余边界点续测通过并回到 follower_rest；"
                            if args.boundary_resume
                            else "完整验证通过并回到 follower_rest；"
                        )
                    )
                )
            )
            + "最大收回关节误差 "
            f"{final_storage_error:.3f}°，保持 "
            f"{OBSERVATION_SECONDS:.0f} 秒。",
            flush=True,
        )
        time.sleep(OBSERVATION_SECONDS)
    except Exception as error:
        print(f"安全停止：{error}", flush=True)
        _append_json_line(
            args.record_output,
            {"type": "failure", "message": str(error)},
        )
        exit_code = 2
        if (
            getattr(robot, "is_connected", False)
            and isinstance(error, SO100PlusMotionConvergenceError)
        ):
            try:
                print(
                    "普通到位误差：先沿已检查路径受控返回 "
                    "follower_rest，再允许关闭力矩。",
                    flush=True,
                )
                final_storage_error = _return_to_storage_rest(
                    candidate_adapter=candidate_adapter,
                    transition_adapter=transition_adapter,
                    candidate=candidate,
                    transition=transition,
                    storage_joint_radians=preflight.joint_radians,
                    path=args.record_output,
                    follower_bus=follower_bus,
                    kinematics=kinematics,
                    return_from_candidate=(recovery_stage == "candidate"),
                )
                at_storage_rest = True
                recovery_stage = "at_rest"
                print(
                    "误差后已受控返回 follower_rest；最大关节偏差 "
                    f"{final_storage_error:.3f}°。",
                    flush=True,
                )
            except Exception as recovery_error:
                _append_json_line(
                    args.record_output,
                    {
                        "type": "recovery_failure",
                        "message": str(recovery_error),
                    },
                )
                print(
                    f"受控收纳失败：{recovery_error}",
                    flush=True,
                )
    finally:
        if getattr(robot, "is_connected", False):
            try:
                transition_adapter.stop()
                if at_storage_rest:
                    transition_adapter.disable_torque()
                    print(
                        "已验证处于 follower_rest，并正常关闭全部力矩。",
                        flush=True,
                    )
                else:
                    print(
                        "紧急异常或受控收纳失败：请托住机械臂，"
                        "现在执行显式紧急力矩释放。",
                        flush=True,
                    )
                    transition_adapter.disable_torque(emergency=True)
            except Exception as error:
                print(
                    f"关闭力矩确认失败：{error}；请立即托住机械臂并物理断电。",
                    flush=True,
                )
                exit_code = 3
            finally:
                transition_adapter.disconnect()
        elif getattr(follower_bus, "is_connected", False):
            follower_bus.disconnect()
        print(
            f"串口已关闭；完整记录：{args.record_output}",
            flush=True,
        )

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
